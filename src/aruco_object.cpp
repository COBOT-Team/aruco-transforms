#include "aruco_transforms/aruco_object.hpp"

#include <iostream>

using namespace cv;
using namespace std;

ArucoDefinedObject::ArucoDefinedObject(SolvePnPMethod method,
                                       const vector<ArucoMarkerObjectPoints>& markers,
                                       const array<ArucoMarkerWarpedImagePoint, 4>& object_corners)
  : method(method), has_previous_warp_matrix_(false)
{
  set_markers(markers, object_corners);
}

void ArucoDefinedObject::set_markers(const vector<ArucoMarkerObjectPoints>& markers,
                                     const array<ArucoMarkerWarpedImagePoint, 4>& object_corners)
{
  marker_object_points_ = markers;

  // Find the rectangular bounding box of the object.
  Point2f warp_min = object_corners[0].warped_image_point;
  Point2f warp_max = object_corners[0].warped_image_point;
  for (const auto& corner : object_corners) {
    if (corner.warped_image_point.x < warp_min.x) warp_min.x = corner.warped_image_point.x;
    if (corner.warped_image_point.y < warp_min.y) warp_min.y = corner.warped_image_point.y;
    if (corner.warped_image_point.x > warp_max.x) warp_max.x = corner.warped_image_point.x;
    if (corner.warped_image_point.y > warp_max.y) warp_max.y = corner.warped_image_point.y;
  }

  // Determine the scale of the warp. This ensures that the warp is in the range [0, 1], which can
  // be scaled to the desired size of the image later.
  float warp_scale_x = 1.0 / (warp_max.x - warp_min.x);
  float warp_scale_y = 1.0 / (warp_max.y - warp_min.y);
  warp_aspect_ratio_ = (warp_max.x - warp_min.x) / (warp_max.y - warp_min.y);

  // Re-scale the warp points to the range [0, 1] and store them.
  for (size_t i = 0; i < object_corners.size(); ++i) {
    const ArucoMarkerWarpedImagePoint& corner = object_corners[i];
    ArucoMarkerWarpedImagePoint& scaled_corner = object_corners_[i];
    scaled_corner.id = corner.id;
    scaled_corner.marker_corner = corner.marker_corner;
    scaled_corner.warped_image_point.x = corner.warped_image_point.x * warp_scale_x;
    scaled_corner.warped_image_point.y = corner.warped_image_point.y * warp_scale_y;
  }
}

bool ArucoDefinedObject::solve_transform(const vector<int>& img_marker_ids,
                                         const vector<vector<Point2f>> img_marker_corners,
                                         cv::InputArray camera_matrix, cv::InputArray dist_coeffs,
                                         tf2::Transform& transform, bool invert) const
{
  // Find the points that correspond to the object.
  vector<Point2f> img_points;
  vector<Point3d> obj_points;
  for (const auto& marker : marker_object_points_) {
    auto it = find(img_marker_ids.begin(), img_marker_ids.end(), marker.id);
    if (it != img_marker_ids.end()) {
      size_t index = distance(img_marker_ids.begin(), it);
      for (size_t i = 0; i < 4; i++) {
        img_points.emplace_back(img_marker_corners[index][i]);
        obj_points.emplace_back(marker.object_points[i]);
      }
    }
  }

  // Try to solve the PnP problem.
  Vec3d rvec, tvec;
  Mat rmat(3, 3, CV_64F);
  if (!solvePnP(obj_points, img_points, camera_matrix, dist_coeffs, rvec, tvec, false, method)) {
    return false;
  }
  Rodrigues(rvec, rmat);

  // Convert the rotation and translation vectors to a tf2 transform.
  tf2::Vector3 tvec_tf(tvec[0], tvec[1], tvec[2]);
  tf2::Matrix3x3 rmat_tf(rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
                         rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
                         rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2));
  transform = tf2::Transform(rmat_tf, tvec_tf);
  if (invert) transform = transform.inverse();

  return true;
}

bool ArucoDefinedObject::warp_perspective(
  const Mat& input, Mat& output, const vector<int>& img_marker_ids,
  const vector<vector<Point2f>> img_marker_corners, const Size& size, bool fallback,
  bool outline_on_fallback)
{
  // Find the points that correspond to the object.
  vector<Point2f> img_points;
  vector<Point2f> obj_points;
  for (const auto& corner : object_corners_) {
    auto it = find(img_marker_ids.begin(), img_marker_ids.end(), corner.id);
    if (it != img_marker_ids.end()) {
      size_t index = distance(img_marker_ids.begin(), it);
      img_points.emplace_back(img_marker_corners[index][corner.marker_corner]);
      obj_points.emplace_back(corner.warped_image_point.x * size.width,
                              corner.warped_image_point.y * size.height);
    }
  }

  // Generate a warp matrix, either from the detected points or the previous matrix.
  Mat transform;
  bool has_4_points = img_points.size() == 4;
  if (has_4_points) {
    transform = getPerspectiveTransform(img_points, obj_points);
    previous_warp_matrix_ = transform.clone();
    has_previous_warp_matrix_ = true;
  } else if (fallback && has_previous_warp_matrix_) {
    transform = previous_warp_matrix_;
  } else {
    return false;
  }

  // Warp the input image to isolate the object.
  warpPerspective(input, output, transform, size);

  // Draw a red outline on the output image if markers are not detected.
  if (outline_on_fallback && !has_4_points) {
    cv::rectangle(output, cv::Point(0, 0), size, cv::Scalar(255, 0, 0), 16);
  }

  return true;
}

float ArucoDefinedObject::get_warp_aspect_ratio() const
{
  return warp_aspect_ratio_;
}
