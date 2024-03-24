#ifndef ARUCO_TRANSFORMS__ARUCO_OBJECT_HPP_
#define ARUCO_TRANSFORMS__ARUCO_OBJECT_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

/**
 * Mapping of Aruco marker corners to object points.
 *
 * Corners are in the order:
 *  - top-left
 *  - top-right
 *  - bottom-right
 *  - bottom-left
 */
struct ArucoMarkerObjectPoints {
  // The ID of the marker.
  int id;

  // The center of the marker in object space.
  cv::Point3d center;
};

/**
 * Mapping of a corner of an Aruco marker to a point in the warped image space.
 *
 * Corners are numbered in the order:
 * - top-left
 * - top-right
 * - bottom-right
 * - bottom-left
 *
 * Points can be in any unit; they will be automatically normalized when used.
 */
struct ArucoMarkerWarpedImagePoint {
  // The ID of the marker.
  int id;

  // The corner of the marker in the warped image space.
  size_t marker_corner;

  // The point in the warped image space.
  cv::Point2f warped_image_point;
};

/**
 * Parameters used to define an object using Aruco markers.
 */
struct ArucoDefinedObjectParams {
  // The method used to solve the PnP problem.
  cv::SolvePnPMethod method;

  // The markers that define the object.
  std::vector<ArucoMarkerObjectPoints> markers;

  // The 4 corners of the object in object space.
  std::array<ArucoMarkerWarpedImagePoint, 4> object_corners;
};

/**
 * An object defined by Aruco markers. This may optionally include four corners of the object,
 * which can be used to create a warp perspective transform.
 */
class ArucoDefinedObject
{
public:
  // The method used to solve the PnP problem.
  cv::SolvePnPMethod method;

  /**
   * Construct a new Aruco-Defined Object.
   *
   * @param[in] params The parameters used to define the object.
   */
  ArucoDefinedObject(const ArucoDefinedObjectParams& params);

  /**
   * Set the markers and bounding box that define the object.
   *
   * @param[in] markers The markers that define the object.
   * @param[in] object_corners The 4 corners of the object in object space.
   */
  void set_markers(const std::vector<ArucoMarkerObjectPoints>& markers,
                   const std::array<ArucoMarkerWarpedImagePoint, 4>& object_corners);

  /**
   * Solve the transform between the camera and the object.
   *
   * @param[in] img_marker_ids The IDs of the markers that were detected.
   * @param[in] img_marker_corners The corners of the markers that were detected.
   * @param[in] camera_matrix The camera matrix.
   * @param[in] dist_coeffs The distortion coefficients of the camera.
   * @param[out] transform The transform between the camera and the object.
   * @param[in] invert If true, the transform is inverted.
   * @return True if the transform was solved, false otherwise.
   */
  bool solve_transform(const std::vector<int>& img_marker_ids,
                       const std::vector<std::vector<cv::Point2f>> img_marker_corners,
                       cv::InputArray camera_matrix, cv::InputArray dist_coeffs,
                       tf2::Transform& transform, bool invert = true) const;

  /**
   * Warp the input image to isolate the object.
   *
   * @param[in] input The input image to warp.
   * @param[out] output The output image.
   * @param[in] img_marker_ids The IDs of the markers that were detected.
   * @param[in] img_marker_corners The corners of the markers that were detected.
   * @param[in] size The size of the output image.
   * @param[in] fallback If true, use the previous warp matrix if markers are not detected.
   * @param[in] outline_on_fallback If true, draw a red outline on the output image if markers are
   *                                not detected.
   * @return
   */
  bool warp_perspective(const cv::Mat& input, cv::Mat& output,
                        const std::vector<int>& img_marker_ids,
                        const std::vector<std::vector<cv::Point2f>> img_marker_corners,
                        const cv::Size& size, bool fallback = true,
                        bool outline_on_fallback = true);

  /**
   * Get the correct aspect ratio of the warped image.
   *
   * @return The aspect ratio of the warped image (width / height);
   */
  float get_warp_aspect_ratio() const;

private:
  // List of markers that define the object.
  std::vector<ArucoMarkerObjectPoints> marker_object_points_;

  // The 4 corners of the object in object space.
  std::array<ArucoMarkerWarpedImagePoint, 4> object_corners_;

  // The aspect ratio of the warped image.
  float warp_aspect_ratio_;

  // The most recent warp matrix. Can be used when markers are not detected.
  bool has_previous_warp_matrix_;
  cv::Mat previous_warp_matrix_;
};

#endif