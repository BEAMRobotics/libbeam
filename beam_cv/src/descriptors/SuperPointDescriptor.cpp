#include <beam_cv/descriptors/SuperPointDescriptor.h>
#include <beam_utils/log.hpp>

namespace beam_cv {

SuperPointDescriptor::SuperPointDescriptor(const std::string& model_file,
                                           int max_features,
                                           float conf_threshold, int border,
                                           int nms_dist_threshold,
                                           int grid_size, bool use_cuda)
    : conf_threshold_(conf_threshold),
      border_(border),
      nms_dist_threshold_(nms_dist_threshold),
      max_features_(max_features),
      grid_size_(grid_size),
      use_cuda_(use_cuda),
      model_file_(model_file) {}

cv::Mat SuperPointDescriptor::ExtractDescriptors(
    const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  ComputeDescriptors(image, false, keypoints, descriptors);
  return descriptors;
}

void SuperPointDescriptor::ComputeDescriptors(
    const cv::Mat& img, bool cuda, std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors) {
  ////////////////////////////
  // copied from SuperPointModel::Detect
  keypoints_.clear(); // NEW

  std::shared_ptr<SuperPoint> model = std::make_shared<SuperPoint>();
  torch::Tensor mProb;
  torch::Tensor mDesc;
  torch::load(model, model_file_);

  auto x = torch::from_blob(img.clone().data, {1, 1, img.rows, img.cols},
                            torch::kByte);
  x = x.to(torch::kFloat) / 255;

  bool use_cuda = cuda && torch::cuda::is_available();
  torch::DeviceType device_type;
  if (use_cuda)
    device_type = torch::kCUDA;
  else
    device_type = torch::kCPU;
  torch::Device device(device_type);

  model->to(device);
  x = x.set_requires_grad(false);
  x = x.to(device);
  auto out = model->forward(x);
  mProb = out[0].squeeze(0); // [H, W]
  mDesc = out[1];            // [1, 256, H/8, W/8]

  ////////////////////////////
  // copied from SuperPointModel::GetKeyPoints

  auto kpts = (mProb > conf_threshold_);
  kpts = torch::nonzero(kpts); // [n_keypoints, 2]  (y, x)
  std::vector<cv::KeyPoint> keypoints_no_nms;
  for (int i = 0; i < kpts.size(0); i++) {
    float response = mProb[kpts[i][0]][kpts[i][1]].item<float>();
    keypoints_no_nms.push_back(cv::KeyPoint(
        kpts[i][1].item<float>(), kpts[i][0].item<float>(), 8, -1, response));
  }

  cv::Mat conf(keypoints_no_nms.size(), 1, CV_32F);
  for (size_t i = 0; i < keypoints_no_nms.size(); i++) {
    int x = keypoints_no_nms[i].pt.x;
    int y = keypoints_no_nms[i].pt.y;
    conf.at<float>(i, 0) = mProb[y][x].item<float>();
  }

  int height = mProb.size(0);
  int width = mProb.size(1);
  NMS(keypoints_no_nms, conf, keypoints_, border_, nms_dist_threshold_, width,
      height);

  if (!max_features_ == 0) {
    SelectKBestFromGrid(keypoints_, keypoints_, max_features_, grid_size_,
                        border_, mProb);
  }
  keypoints = keypoints_;

  ////////////////////////////
  // copied from SuperPointModel::ComputeDescriptors
  const std::vector<cv::KeyPoint> keypoints_copy = keypoints;
  cv::Mat kpt_mat(keypoints_copy.size(), 2, CV_32F); // [n_keypoints, 2]  (y, x)

  for (size_t i = 0; i < keypoints_copy.size(); i++) {
    kpt_mat.at<float>(i, 0) = (float)keypoints_copy[i].pt.y;
    kpt_mat.at<float>(i, 1) = (float)keypoints_copy[i].pt.x;
  }

  auto fkpts = torch::from_blob(
      kpt_mat.data, {static_cast<long int>(keypoints_copy.size()), 2},
      torch::kFloat);

  auto grid = torch::zeros({1, 1, fkpts.size(0), 2}); // [1, 1, n_keypoints, 2]
  grid[0][0].slice(1, 0, 1) =
      2.0 * fkpts.slice(1, 1, 2) / mProb.size(1) - 1; // x
  grid[0][0].slice(1, 1, 2) =
      2.0 * fkpts.slice(1, 0, 1) / mProb.size(0) - 1; // y

  auto desc =
      torch::grid_sampler(mDesc, grid, 0, 0, false); // [1, 256, 1, n_keypoints]
  desc = desc.squeeze(0).squeeze(1);          // [256, n_keypoints]

  // normalize to 1
  auto dn = torch::norm(desc, 2, 1);
  desc = desc.div(torch::unsqueeze(dn, 1));

  desc = desc.transpose(0, 1).contiguous(); // [n_keypoints, 256]
  desc = desc.to(torch::kCPU);

  cv::Mat desc_mat(cv::Size(desc.size(1), desc.size(0)), CV_32FC1,
                   desc.data_ptr<float>());
  descriptors = desc_mat.clone();
}

bool GridKeypointSortDecreasingDes(const std::pair<float, cv::KeyPoint>& i,
                                   const std::pair<float, cv::KeyPoint>& j) {
  return (i.first > j.first);
}

void SuperPointDescriptor::SelectKBestFromGrid(
    const std::vector<cv::KeyPoint>& keypoints_in,
    std::vector<cv::KeyPoint>& keypoints_out, int max_features, int grid_size,
    int border, const torch::Tensor& mProb_) {
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
                GridKeypointSortDecreasingDes);
      int stop =
          std::min(static_cast<int>(grid_keypoints.size()), features_per_grid);
      for (int i = 0; i < stop; i++) {
        keypoints_new.push_back(grid_keypoints[i].second);
      }
    }
  }
  keypoints_out = keypoints_new;
}

void SuperPointDescriptor::NMS(const std::vector<cv::KeyPoint>& det,
                               const cv::Mat& conf,
                               std::vector<cv::KeyPoint>& pts, int border,
                               int dist_thresh, int img_width, int img_height) {
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

} // namespace beam_cv
