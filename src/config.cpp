#include "aruco_transforms/config.hpp"

#include <opencv2/opencv.hpp>

#include "aruco_transforms/aruco_object.hpp"

//                                                                                                //
// ========================================= Chessboard ========================================= //
//                                                                                                //

const double CHESSBOARD_SIZE = 374.65 / 1000.0;
const double ARUCO_SIZE = 18.5 / 1000.0;

const double HALF_CB = CHESSBOARD_SIZE / 2.0;  // Half the size of the chessboard in meters.

const ArucoDefineObjectParams CHESSBOARD_PARAMS = {

  // SolvePnP method.
  cv::SOLVEPNP_IPPE,

  // Object points.
  {
    {
      0,
      {
        cv::Point3d(-HALF_CB, HALF_CB, 0.0),
        cv::Point3d(-HALF_CB + ARUCO_SIZE, HALF_CB, 0.0),
        cv::Point3d(-HALF_CB + ARUCO_SIZE, HALF_CB - ARUCO_SIZE, 0.0),
        cv::Point3d(-HALF_CB, HALF_CB - ARUCO_SIZE, 0.0),
      },
    },
    {
      1,
      {
        cv::Point3d(HALF_CB - ARUCO_SIZE, HALF_CB, 0.0),
        cv::Point3d(HALF_CB, HALF_CB, 0.0),
        cv::Point3d(HALF_CB, HALF_CB - ARUCO_SIZE, 0.0),
        cv::Point3d(HALF_CB - ARUCO_SIZE, HALF_CB - ARUCO_SIZE, 0.0),
      },
    },
    {
      2,
      {
        cv::Point3d(HALF_CB - ARUCO_SIZE, -HALF_CB + ARUCO_SIZE, 0.0),
        cv::Point3d(HALF_CB, -HALF_CB + ARUCO_SIZE, 0.0),
        cv::Point3d(HALF_CB, -HALF_CB, 0.0),
        cv::Point3d(HALF_CB - ARUCO_SIZE, -HALF_CB, 0.0),
      },
    },
    {
      3,
      {
        cv::Point3d(-HALF_CB, -HALF_CB + ARUCO_SIZE, 0.0),
        cv::Point3d(-HALF_CB + ARUCO_SIZE, -HALF_CB + ARUCO_SIZE, 0.0),
        cv::Point3d(-HALF_CB + ARUCO_SIZE, -HALF_CB, 0.0),
        cv::Point3d(-HALF_CB, -HALF_CB, 0.0),
      },
    },
  },

  //  Warped image points.
  { {
    {
      0,
      0,
      cv::Point2f(0.0, 0.0),
    },
    {
      1,
      1,
      cv::Point2f(1.0, 0.0),
    },
    {
      2,
      2,
      cv::Point2f(1.0, 1.0),
    },
    {
      3,
      3,
      cv::Point2f(0.0, 1.0),
    },
  } }
};

//                                                                                                //
// =========================================== Table ============================================
// //
//                                                                                                //

// TODO: Define the Aruco-defined table.