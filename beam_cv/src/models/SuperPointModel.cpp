#include <beam_cv/models/SuperPointModel.h>
#include <beam_utils/log.hpp>

namespace beam_cv {

const int c1 = 64;
const int c2 = 64;
const int c3 = 128;
const int c4 = 128;
const int c5 = 256;
const int d1 = 256;

SuperPoint::SuperPoint()
    : conv1a(torch::nn::Conv2dOptions(1, c1, 3).stride(1).padding(1)),
      conv1b(torch::nn::Conv2dOptions(c1, c1, 3).stride(1).padding(1)),

      conv2a(torch::nn::Conv2dOptions(c1, c2, 3).stride(1).padding(1)),
      conv2b(torch::nn::Conv2dOptions(c2, c2, 3).stride(1).padding(1)),

      conv3a(torch::nn::Conv2dOptions(c2, c3, 3).stride(1).padding(1)),
      conv3b(torch::nn::Conv2dOptions(c3, c3, 3).stride(1).padding(1)),

      conv4a(torch::nn::Conv2dOptions(c3, c4, 3).stride(1).padding(1)),
      conv4b(torch::nn::Conv2dOptions(c4, c4, 3).stride(1).padding(1)),

      convPa(torch::nn::Conv2dOptions(c4, c5, 3).stride(1).padding(1)),
      convPb(torch::nn::Conv2dOptions(c5, 65, 1).stride(1).padding(0)),

      convDa(torch::nn::Conv2dOptions(c4, c5, 3).stride(1).padding(1)),
      convDb(torch::nn::Conv2dOptions(c5, d1, 1).stride(1).padding(0))

{
  register_module("conv1a", conv1a);
  register_module("conv1b", conv1b);

  register_module("conv2a", conv2a);
  register_module("conv2b", conv2b);

  register_module("conv3a", conv3a);
  register_module("conv3b", conv3b);

  register_module("conv4a", conv4a);
  register_module("conv4b", conv4b);

  register_module("convPa", convPa);
  register_module("convPb", convPb);

  register_module("convDa", convDa);
  register_module("convDb", convDb);
}

std::vector<torch::Tensor> SuperPoint::forward(torch::Tensor x) {
  x = torch::relu(conv1a->forward(x));
  x = torch::relu(conv1b->forward(x));
  x = torch::max_pool2d(x, 2, 2);

  x = torch::relu(conv2a->forward(x));
  x = torch::relu(conv2b->forward(x));
  x = torch::max_pool2d(x, 2, 2);

  x = torch::relu(conv3a->forward(x));
  x = torch::relu(conv3b->forward(x));
  x = torch::max_pool2d(x, 2, 2);

  x = torch::relu(conv4a->forward(x));
  x = torch::relu(conv4b->forward(x));

  auto cPa = torch::relu(convPa->forward(x));
  auto semi = convPb->forward(cPa); // [B, 65, H/8, W/8]

  auto cDa = torch::relu(convDa->forward(x));
  auto desc = convDb->forward(cDa); // [B, d1, H/8, W/8]

  auto dn = torch::norm(desc, 2, 1);
  desc = desc.div(torch::unsqueeze(dn, 1));

  semi = torch::softmax(semi, 1);
  semi = semi.slice(1, 0, 64);
  semi = semi.permute({0, 2, 3, 1}); // [B, H/8, W/8, 64]

  int Hc = semi.size(1);
  int Wc = semi.size(2);
  semi = semi.contiguous().view({-1, Hc, Wc, 8, 8});
  semi = semi.permute({0, 1, 3, 2, 4});
  semi = semi.contiguous().view({-1, Hc * 8, Wc * 8}); // [B, H, W]

  std::vector<torch::Tensor> ret;
  ret.push_back(semi);
  ret.push_back(desc);

  return ret;
}

SuperPointModel::SuperPointModel(const std::string& model_file, bool use_cuda) {
  BEAM_INFO("Loading model: {}", model_file);
  if (use_cuda){
    BEAM_INFO("Using CUDA with superpoint model");
    use_cuda_ = true;
  } else {
    BEAM_INFO("Not using CUDA with superpoint model");
  }
  model_ = std::make_shared<SuperPoint>();
  torch::load(model_, model_file);
  BEAM_INFO("Model loaded successfully.");
}

void SuperPointModel::Detect(cv::Mat img) {
  auto x = torch::from_blob(img.data, {1, 1, img.rows, img.cols},
                            torch::kByte);
  x = x.to(torch::kFloat) / 255;

  torch::DeviceType device_type;
  if (use_cuda_ && torch::cuda::is_available()){
    device_type = torch::kCUDA;
  } else {
    device_type = torch::kCPU;
  }
  torch::Device device(device_type);

  model_->to(device);
  x = x.set_requires_grad(false);
  auto out = model_->forward(x.to(device));
  mProb_ = out[0].squeeze(0); // [H, W]
  mDesc_ = out[1];            // [1, 256, H/8, W/8]
  has_been_initialized = true;
}

void SuperPointModel::GetKeyPoints(std::vector<cv::KeyPoint>& keypoints,
                                   float conf_threshold, int border,
                                   int nms_dist_threshold, int max_features,
                                   int grid_size) {
  
  auto kpts = (mProb_ > conf_threshold);
  
  kpts = torch::nonzero(kpts); // [n_keypoints, 2]  (y, x)
  
  std::vector<cv::KeyPoint> keypoints_no_nms;
  
  for (int i = 0; i < kpts.size(0); i++) {
    float response = mProb_[kpts[i][0]][kpts[i][1]].item<float>();
    keypoints_no_nms.push_back(cv::KeyPoint(
        kpts[i][1].item<float>(), kpts[i][0].item<float>(), 8, -1, response));
  }

  
  cv::Mat conf(keypoints_no_nms.size(), 1, CV_32F);
  for (size_t i = 0; i < keypoints_no_nms.size(); i++) {
    int x = keypoints_no_nms[i].pt.x;
    int y = keypoints_no_nms[i].pt.y;
    conf.at<float>(i, 0) = mProb_[y][x].item<float>();
  }

  
  int height = mProb_.size(0);
  int width = mProb_.size(1);
  NMS(keypoints_no_nms, conf, keypoints, border, nms_dist_threshold, width,
      height);
  
  if (!max_features == 0) {
    SelectKBestFromGrid(keypoints, keypoints, max_features, grid_size, border);
  }
}

void SuperPointModel::ComputeDescriptors(
    const std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
  cv::Mat kpt_mat(keypoints.size(), 2, CV_32F); // [n_keypoints, 2]  (y, x)

  for (size_t i = 0; i < keypoints.size(); i++) {
    kpt_mat.at<float>(i, 0) = (float)keypoints[i].pt.y;
    kpt_mat.at<float>(i, 1) = (float)keypoints[i].pt.x;
  }

  auto fkpts = torch::from_blob(kpt_mat.data,
                                {static_cast<long int>(keypoints.size()), 2},
                                torch::kFloat);

  auto grid = torch::zeros({1, 1, fkpts.size(0), 2}); // [1, 1, n_keypoints, 2]
  grid[0][0].slice(1, 0, 1) =
      2.0 * fkpts.slice(1, 1, 2) / mProb_.size(1) - 1; // x
  grid[0][0].slice(1, 1, 2) =
      2.0 * fkpts.slice(1, 0, 1) / mProb_.size(0) - 1; // y

  if (use_cuda_) {
    grid = grid.to(torch::Device(torch::kCUDA));
  }

  auto desc = torch::grid_sampler(mDesc_, grid, 0, 0, false);
  desc = desc.squeeze(0).squeeze(1);           // [256, n_keypoints]

  // normalize to 1
  auto dn = torch::norm(desc, 2, 1);
  desc = desc.div(torch::unsqueeze(dn, 1));

  desc = desc.transpose(0, 1).contiguous(); // [n_keypoints, 256]
  desc = desc.to(torch::kCPU);

  cv::Mat desc_mat(cv::Size(desc.size(1), desc.size(0)), CV_32FC1,
                   desc.data_ptr<float>());

  descriptors = desc_mat.clone();
}

void SuperPointModel::NMS(const std::vector<cv::KeyPoint>& det,
                          const cv::Mat& conf, std::vector<cv::KeyPoint>& pts,
                          int border, int dist_thresh, int img_width,
                          int img_height) {

  std::vector<cv::Point2f> pts_raw;

  for (uint32_t i = 0; i < det.size(); i++) {
    int u = (int)det[i].pt.x;
    int v = (int)det[i].pt.y;

    pts_raw.push_back(cv::Point2f(u, v));
  }

  cv::Mat grid = cv::Mat(cv::Size(img_width, img_height), CV_8UC1);
  cv::Mat inds = cv::Mat(cv::Size(img_width, img_height), CV_16UC1);

  cv::Mat confidence = cv::Mat(cv::Size(img_width, img_height), CV_32FC1);

  grid.setTo(0);
  inds.setTo(0);
  confidence.setTo(0);

  for (uint32_t i = 0; i < pts_raw.size(); i++) {
    int uu = (int)pts_raw[i].x;
    int vv = (int)pts_raw[i].y;

    grid.at<char>(vv, uu) = 1;
    inds.at<unsigned short>(vv, uu) = i;

    confidence.at<float>(vv, uu) = conf.at<float>(i, 0);
  }

  cv::copyMakeBorder(grid, grid, dist_thresh, dist_thresh, dist_thresh,
                     dist_thresh, cv::BORDER_CONSTANT, 0);

  for (uint32_t i = 0; i < pts_raw.size(); i++) {
    int uu = (int)pts_raw[i].x + dist_thresh;
    int vv = (int)pts_raw[i].y + dist_thresh;

    if (grid.at<char>(vv, uu) != 1) continue;

    for (int k = -dist_thresh; k < (dist_thresh + 1); k++)
      for (int j = -dist_thresh; j < (dist_thresh + 1); j++) {
        if (j == 0 && k == 0) continue;

        if (confidence.at<float>(vv + k, uu + j) < confidence.at<float>(vv, uu))
          grid.at<char>(vv + k, uu + j) = 0;
      }
    grid.at<char>(vv, uu) = 2;
  }

  size_t valid_cnt = 0;
  std::vector<int> select_indice;

  for (int v = 0; v < (img_height + dist_thresh); v++) {
    for (int u = 0; u < (img_width + dist_thresh); u++) {
      if (u - dist_thresh >= (img_width - border) || u - dist_thresh < border ||
          v - dist_thresh >= (img_height - border) || v - dist_thresh < border)
        continue;

      if (grid.at<char>(v, u) == 2) {
        int select_ind =
            (int)inds.at<unsigned short>(v - dist_thresh, u - dist_thresh);
        cv::Point2f p = pts_raw[select_ind];
        float response = conf.at<float>(select_ind, 0);
        pts.push_back(cv::KeyPoint(p, 8.0f, -1, response));

        select_indice.push_back(select_ind);
        valid_cnt++;
      }
    }
  }

}

bool GridKeypointSortDecreasing(const std::pair<float, cv::KeyPoint>& i,
                                const std::pair<float, cv::KeyPoint>& j) {
  return (i.first > j.first);
}

void SuperPointModel::SelectKBestFromGrid(
    const std::vector<cv::KeyPoint>& keypoints_in,
    std::vector<cv::KeyPoint>& keypoints_out, int max_features, int grid_size,
    int border) {
      
  int features_per_grid;
  if (grid_size == 0) {
    features_per_grid = max_features;
    grid_size = std::max(mProb_.size(0), mProb_.size(1));
  } else {
    int num_grids_x =
        static_cast<int>(std::ceil((mProb_.size(1) - 2 * border) / grid_size));
    int num_grids_y =
        static_cast<int>(std::ceil((mProb_.size(0) - 2 * border) / grid_size));
    features_per_grid = static_cast<int>(
        std::floor(max_features / (num_grids_x * num_grids_y)));
  }

  if (features_per_grid == 0) {
    BEAM_WARN(
        "Cannot select top features. Grid too fine for number of features.");
    keypoints_out = keypoints_in;
    return;
  }

  std::vector<cv::KeyPoint> keypoints_new = std::vector<cv::KeyPoint>{};
  for (int y = border; y < mProb_.size(0) - border; y += grid_size) {
    for (int x = border; x < mProb_.size(1) - border; x += grid_size) {
      // find all points in grid
      std::vector<std::pair<float, cv::KeyPoint>> grid_keypoints;
      for (cv::KeyPoint k : keypoints_in) {
        if (k.pt.x >= x && k.pt.x <= x + grid_size && k.pt.y >= y &&
            k.pt.y <= y + grid_size) {
          float conf = mProb_[k.pt.y][k.pt.x].item<float>();
          grid_keypoints.push_back(std::make_pair(conf, k));
        }
      }

      // sort from highest to lowest and take top
      std::sort(grid_keypoints.begin(), grid_keypoints.end(),
                GridKeypointSortDecreasing);
      int stop =
          std::min(static_cast<int>(grid_keypoints.size()), features_per_grid);
      for (int i = 0; i < stop; i++) {
        keypoints_new.push_back(grid_keypoints[i].second);
      }
    }
  }
  keypoints_out = keypoints_new;
}

} // namespace beam_cv
