#include "aruco_transforms/config.hpp"

#include <opencv2/opencv.hpp>

#include "aruco_transforms/aruco_object.hpp"

//                                                                                                //
// ========================================= Chessboard ========================================= //
//                                                                                                //

const double CHESSBOARD_SIZE = 374.65 / 1000.0;
const double HALF_CB = CHESSBOARD_SIZE / 2.0;
const double HALF_CB_ARUCO_CENTER = HALF_CB - (18.5 / 2000.0);

const ArucoDefinedObjectParams CHESSBOARD_PARAMS = {

  // SolvePnP method.
  cv::SOLVEPNP_IPPE,

  // Object points.
  {
    {
      0,
      cv::Point3d(-HALF_CB_ARUCO_CENTER, HALF_CB_ARUCO_CENTER, 0.0),
    },
    {
      1,
      cv::Point3d(HALF_CB_ARUCO_CENTER, HALF_CB_ARUCO_CENTER, 0.0),
    },
    {
      2,
      cv::Point3d(HALF_CB_ARUCO_CENTER, -HALF_CB_ARUCO_CENTER, 0.0),
    },
    {
      3,
      cv::Point3d(-HALF_CB_ARUCO_CENTER, -HALF_CB_ARUCO_CENTER, 0.0),
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
// =========================================== Table ============================================ //
//                                                                                                //

const double TABLE_ARUCO_SIZE = 30.0 / 1000.0;
const double TABLE_WIDTH = 1171.575 / 1000.0;
const double TABLE_HEIGHT = 682.625 / 1000.0;
const double TABLE_ARUCO_OFFSET = 2.5 / 1000.0;
const double TABLE_OUTER_ARUCO_X =
  (TABLE_WIDTH / 2.0) - TABLE_ARUCO_OFFSET - (TABLE_ARUCO_SIZE / 2.0);
const double TABLE_INNER_ARUCO_X = TABLE_OUTER_ARUCO_X - (400.0 / 1000.0);
const double TABLE_ARUCO_Y = (TABLE_HEIGHT / 2.0) - TABLE_ARUCO_OFFSET - (TABLE_ARUCO_SIZE / 2.0);

const ArucoDefinedObjectParams TABLE_PARAMS = {

  // SolvePnP method.
  cv::SOLVEPNP_IPPE,

  // Object points.
  {
    {
      4,
      cv::Point3d(-TABLE_OUTER_ARUCO_X, TABLE_ARUCO_Y, 0.0),
    },
    {
      5,
      cv::Point3d(-TABLE_INNER_ARUCO_X, TABLE_ARUCO_Y, 0.0),
    },
    {
      6,
      cv::Point3d(TABLE_INNER_ARUCO_X, TABLE_ARUCO_Y, 0.0),
    },
    {
      7,
      cv::Point3d(TABLE_OUTER_ARUCO_X, TABLE_ARUCO_Y, 0.0),
    },
    {
      8,
      cv::Point3d(TABLE_OUTER_ARUCO_X, -TABLE_ARUCO_Y, 0.0),
    },
    {
      9,
      cv::Point3d(TABLE_INNER_ARUCO_X, -TABLE_ARUCO_Y, 0.0),
    },
    {
      10,
      cv::Point3d(-TABLE_INNER_ARUCO_X, -TABLE_ARUCO_Y, 0.0),
    },
    {
      11,
      cv::Point3d(-TABLE_OUTER_ARUCO_X, -TABLE_ARUCO_Y, 0.0),
    },
  },

  // Warped image points.
  { {
    {
      4,
      0,
      cv::Point2f(0, 0),
    },
    {
      7,
      1,
      cv::Point2f(TABLE_WIDTH, 0),
    },
    {
      8,
      2,
      cv::Point2f(TABLE_WIDTH, TABLE_HEIGHT),
    },
    {
      11,
      3,
      cv::Point2f(0, TABLE_HEIGHT),
    },
  } }
};