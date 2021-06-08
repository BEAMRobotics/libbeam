#include <beam_cv/Utils.h>

#include <algorithm>

#include <beam_cv/geometry/Triangulation.h>

namespace beam_cv {

cv::Mat AdaptiveHistogram(const cv::Mat& input) {
  cv::Mat lab_image;
  cv::cvtColor(input, lab_image, CV_BGR2Lab);
  std::vector<cv::Mat> lab_planes(6);
  cv::split(lab_image, lab_planes);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(3);
  cv::Mat dst;
  clahe->apply(lab_planes[0], dst);
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);
  cv::Mat new_image;
  cv::cvtColor(lab_image, new_image, CV_Lab2BGR);
  return new_image;
}

cv::Mat KMeans(const cv::Mat& input, int K) {
  cv::Mat image = input.clone();
  cv::Size og(image.cols, image.rows);
  cv::Size half(image.cols / 2, image.rows / 2);
  cv::resize(image, image, half);
  cv::Mat data;
  image.convertTo(data, CV_32F);
  data = data.reshape(1, data.total());
  // do kmeans
  cv::Mat labels, centers;
  cv::kmeans(data, K, labels, cv::TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 3,
             cv::KMEANS_PP_CENTERS, centers);
  // reshape both to a single row of Vec3f pixels:
  centers = centers.reshape(3, centers.rows);
  data = data.reshape(3, data.rows);
  // replace pixel values with their center value:
  cv::Vec3f* p = data.ptr<cv::Vec3f>();
  for (int i = 0; i < (int)data.rows; i++) {
    int center_id = labels.at<int>(i);
    p[i] = centers.at<cv::Vec3f>(center_id);
  }
  // back to 2d, and uchar:
  image = data.reshape(3, image.rows);
  image.convertTo(image, CV_8UC1);
  cv::Mat grey;
  cv::cvtColor(image, grey, CV_BGR2GRAY);
  cv::morphologyEx(grey, grey, cv::MORPH_CLOSE, cv::Mat::ones(5, 5, CV_8U));
  cv::resize(grey, grey, og);
  return grey;
}

cv::Mat ExtractSkeleton(const cv::Mat& input_image) {
  cv::Mat output_image = input_image.clone();
  cv::Mat skel(output_image.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat temp(output_image.size(), CV_8UC1);
  // Declare structuring element for open function
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  bool done = false;
  while (!done) {
    cv::morphologyEx(output_image, temp, cv::MORPH_OPEN, element);
    cv::bitwise_not(temp, temp);
    cv::bitwise_and(output_image, temp, temp);
    cv::bitwise_or(skel, temp, skel);
    cv::erode(output_image, output_image, element);

    double max;
    cv::minMaxLoc(output_image, 0, &max);
    done = (max == 0);
  }
  return skel;
}

cv::Mat RemoveClusters(const cv::Mat& input_image, int threshold) {
  // Remove small pixel clusters
  cv::Mat output_image = input_image.clone();
  cv::Mat category(output_image.size(), CV_16UC1);
  cv::Mat stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = cv::connectedComponentsWithStats(
      output_image, category, stats, my_centroids, connectivity, itype);
  BEAM_INFO("Number of Components in image: {}", num_comp);

  category.convertTo(category, CV_8UC1);
  for (int x = 0; x < output_image.rows; ++x) // iterate over skeleton image
  {
    for (int y = 0; y < output_image.cols; ++y) // iterate over skeleton image
    {
      int seg_category = int(category.at<uchar>(x, y));
      if (seg_category > 0) {
        if (stats.at<int>(seg_category, 4) < threshold) {
          output_image.at<uchar>(x, y) = 0;
        }
      }
    }
  }
  return output_image;
}

std::vector<cv::Mat> SegmentComponents(const cv::Mat& image) {
  // Create a copy of the binary image
  cv::Mat im_category(image.size(), CV_16UC1);
  cv::Mat my_stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = cv::connectedComponentsWithStats(
      image, im_category, my_stats, my_centroids, connectivity, itype);
  im_category.convertTo(im_category, CV_8UC1);
  // initialize smaller window matrices
  std::vector<cv::Mat> cracks;
  for (int i = 0; i < num_comp; i++) {
    cv::Mat crack(image.size(), CV_8UC1);
    cracks.push_back(crack);
  }
  for (int x = 0; x < image.rows; ++x) // iterate over skeleton image
  {
    for (int y = 0; y < image.cols; ++y) // iterate over skeleton image
    {
      int crack_num = int(im_category.at<uchar>(x, y));
      cracks[crack_num].at<uchar>(x, y) = 255;
    }
  }
  cracks.erase(cracks.begin());
  return cracks;
}

std::map<int, std::vector<cv::Point2i>>
    ConnectedComponents(const cv::Mat& image) {
  auto UnionCoords = [](cv::Mat image, int x, int y, int x2, int y2,
                        beam_cv::UnionFind& uf) {
    if (y2 < image.cols && x2 < image.rows &&
        image.at<uchar>(x, y) == image.at<uchar>(x2, y2)) {
      uf.UnionSets(x * image.cols + y, x2 * image.cols + y2);
    }
  };
  beam_cv::UnionFind uf;
  uf.Initialize(image.rows * image.cols);
  for (int x = 0; x < image.rows; x++) {
    for (int y = 0; y < image.cols; y++) {
      UnionCoords(image, x, y, x + 1, y, uf);
      UnionCoords(image, x, y, x, y + 1, uf);
    }
  }
  std::map<int, std::vector<cv::Point2i>> sets;
  for (int i = 0; i < image.rows * image.cols; i++) {
    int seti = uf.FindSet(i);
    if (sets.count(seti) != 0) {
      cv::Point2i p(i / image.cols, i % image.cols);
      sets[seti].push_back(p);
    } else {
      std::vector<cv::Point2i> points;
      sets.insert({seti, points});
    }
  }
  return sets;
}

Eigen::Vector2d ConvertKeypoint(const cv::KeyPoint& keypoint) {
  Eigen::Vector2d vec_keypoint(keypoint.pt.x, keypoint.pt.y);
  return vec_keypoint;
}

Eigen::Vector2d ConvertKeypoint(const cv::Point2f& keypoint) {
  Eigen::Vector2d vec_keypoint(keypoint.x, keypoint.y);
  return vec_keypoint;
}

cv::Point2f ConvertKeypoint(const Eigen::Vector2d& keypoint) {
  cv::Point2f cv_keypoint((float)keypoint(0), (float)keypoint(1));
  return cv_keypoint;
}

std::vector<Eigen::Vector2d>
    ConvertKeypoints(const std::vector<cv::KeyPoint>& keypoints) {
  std::vector<Eigen::Vector2d> vec_keypoints;
  for (const auto& k : keypoints) {
    vec_keypoints.emplace_back(k.pt.x, k.pt.y);
  }
  return vec_keypoints;
}

std::vector<Eigen::Vector2d>
    ConvertKeypoints(const std::vector<cv::Point2f>& keypoints) {
  std::vector<Eigen::Vector2d> vec_keypoints;
  for (const auto& k : keypoints) { vec_keypoints.emplace_back(k.x, k.y); }
  return vec_keypoints;
}

std::vector<cv::Point2f>
    ConvertKeypoints(const std::vector<Eigen::Vector2d>& keypoints) {
  std::vector<cv::Point2f> cv_keypoints;
  for (const auto& k : keypoints) {
    cv_keypoints.emplace_back((float)k(0), (float)k(1));
  }
  return cv_keypoints;
}

int CheckInliers(std::shared_ptr<beam_calibration::CameraModel> cam1,
                 std::shared_ptr<beam_calibration::CameraModel> cam2,
                 std::vector<Eigen::Vector2i> p1_v,
                 std::vector<Eigen::Vector2i> p2_v,
                 Eigen::Matrix4d T_cam1_world, Eigen::Matrix4d T_cam2_world,
                 double inlier_threshold) {
  if (p1_v.size() != p2_v.size()) {
    BEAM_WARN("Invalid input, number of pixels must match.");
    return -1;
  }
  std::vector<beam::opt<Eigen::Vector3d>> points =
      Triangulation::TriangulatePoints(cam1, cam2, T_cam1_world, T_cam2_world,
                                       p1_v, p2_v);
  return CheckInliers(cam1, cam2, p1_v, p2_v, points, T_cam1_world,
                      T_cam2_world, inlier_threshold);
}

int CheckInliers(std::shared_ptr<beam_calibration::CameraModel> cam1,
                 std::shared_ptr<beam_calibration::CameraModel> cam2,
                 std::vector<Eigen::Vector2i> p1_v,
                 std::vector<Eigen::Vector2i> p2_v,
                 std::vector<beam::opt<Eigen::Vector3d>> points,
                 Eigen::Matrix4d T_cam1_world, Eigen::Matrix4d T_cam2_world,
                 double inlier_threshold) {
  if (p1_v.size() != p2_v.size()) {
    BEAM_WARN("Invalid input, number of pixels must match.");
    return -1;
  }
  int inliers = 0;
  // reproject triangulated points and find their error
  for (size_t i = 0; i < points.size(); i++) {
    if (points[i].has_value()) {
      // transform points into each camera frame
      Eigen::Vector4d pt_h;
      pt_h << points[i].value()[0], points[i].value()[1], points[i].value()[2],
          1;
      Eigen::Vector4d pt_h_1 = T_cam1_world * pt_h,
                      pt_h_2 = T_cam2_world * pt_h;
      Eigen::Vector3d pt1 = pt_h_1.head(3) / pt_h_1(3);
      Eigen::Vector3d pt2 = pt_h_2.head(3) / pt_h_2(3);
      // reproject triangulated points into each frame
      bool in_image1 = false, in_image2 = false;
      Eigen::Vector2d p1_rep, p2_rep;
      if (!cam1->ProjectPoint(pt1, p1_rep, in_image1) ||
          !cam2->ProjectPoint(pt2, p2_rep, in_image2)) {
        continue;
      } else if (!in_image1 || !in_image2) {
        continue;
      }
      // compute distance to actual pixel
      Eigen::Vector2d p1_d{p1_v[i][0], p1_v[i][1]};
      Eigen::Vector2d p2_d{p2_v[i][0], p2_v[i][1]};
      double dist_1 = beam::distance(p1_rep, p1_d);
      double dist_2 = beam::distance(p2_rep, p2_d);
      if (dist_1 < inlier_threshold && dist_2 < inlier_threshold) { inliers++; }
    }
  }
  return inliers;
}

int CheckInliers(std::shared_ptr<beam_calibration::CameraModel> cam,
                 std::vector<Eigen::Vector3d> points,
                 std::vector<Eigen::Vector2i> pixels,
                 Eigen::Matrix4d T_cam_world, double inlier_threshold) {
  if (points.size() != pixels.size()) {
    BEAM_WARN("Invalid input, number of points and pixels must match.");
    return -1;
  }
  int inliers = 0;
  // reproject points and find their error
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector4d pt_h;
    pt_h << points[i][0], points[i][1], points[i][2], 1;
    pt_h = T_cam_world * pt_h;
    Eigen::Vector3d ptc = pt_h.head(3) / pt_h(3);

    bool in_image = false;
    Eigen::Vector2d p;
    if (!cam->ProjectPoint(ptc, p, in_image)) {
      continue;
    } else if (!in_image) {
      continue;
    }

    Eigen::Vector2d pd{pixels[i][0], pixels[i][1]};
    double dist = beam::distance(p, pd);
    if (dist < inlier_threshold) { inliers++; }
  }
  return inliers;
}

void DetectComputeAndMatch(
    cv::Mat imL, cv::Mat imR,
    const std::shared_ptr<beam_cv::Descriptor>& descriptor,
    const std::shared_ptr<beam_cv::Detector>& detector,
    const std::shared_ptr<beam_cv::Matcher>& matcher,
    std::vector<Eigen::Vector2i>& pL_v, std::vector<Eigen::Vector2i>& pR_v) {
  std::vector<cv::KeyPoint> kpL = detector->DetectFeatures(imL);
  cv::Mat descL = descriptor->ExtractDescriptors(imL, kpL);

  std::vector<cv::KeyPoint> kpR = detector->DetectFeatures(imR);
  cv::Mat descR = descriptor->ExtractDescriptors(imR, kpR);

  std::vector<cv::DMatch> matches =
      matcher->MatchDescriptors(descL, descR, kpL, kpR);

  for (auto& match : matches) {
    cv::KeyPoint imL_p = kpL[match.queryIdx];
    cv::KeyPoint imR_p = kpR[match.trainIdx];
    Eigen::Vector2i pL = beam_cv::ConvertKeypoint(imL_p).cast<int>();
    Eigen::Vector2i pR = beam_cv::ConvertKeypoint(imR_p).cast<int>();
    pL_v.push_back(pL);
    pR_v.push_back(pR);
  }
}

double
    ComputeMedianMatchDistance(std::vector<cv::DMatch> matches,
                               const std::vector<cv::KeyPoint>& keypoints_1,
                               const std::vector<cv::KeyPoint>& keypoints_2) {
  std::vector<double> distances;
  for (auto& mat : matches) {
    cv::KeyPoint kp1 = keypoints_1[mat.queryIdx];
    cv::KeyPoint kp2 = keypoints_2[mat.trainIdx];
    double d = sqrt((kp1.pt.x - kp2.pt.x) * (kp1.pt.x - kp2.pt.x) +
                    (kp1.pt.y - kp2.pt.y) * (kp1.pt.y - kp2.pt.y));
    distances.push_back(d);
  }
  std::sort(distances.begin(), distances.end());
  if (distances.size() > 0) {
    std::sort(distances.begin(), distances.end());
    return distances[distances.size() / 2];
  } else {
    return -1.0;
  }
}

} // namespace beam_cv
