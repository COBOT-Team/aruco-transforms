#include "aruco_transforms/config.hpp"

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace aruco_object_manager
{

//                                                                                                //
// ========================================= Chessboard ========================================= //
//                                                                                                //

const double CHESSBOARD_SIZE = 374.65 / 1000.0;
const double CHESSBOARD_ARUCO_SIZE = 18.5 / 1000.0;
const double HALF_CB = CHESSBOARD_SIZE / 2.0;

const ArucoObjectManager::Params CHESSBOARD_PARAMS = {
  // SolvePnP method.
  SOLVEPNP_IPPE,

  // Minimum number of markers.
  4,

  // 3D object markers.
  {
    Marker3d(0, make_shared<Marker3d::SinglePoint>(
                  Corner::TOP_LEFT, Point3d(-HALF_CB, HALF_CB, 0.0), CHESSBOARD_ARUCO_SIZE)),
    Marker3d(1, make_shared<Marker3d::SinglePoint>(
                  Corner::TOP_RIGHT, Point3d(HALF_CB, HALF_CB, 0.0), CHESSBOARD_ARUCO_SIZE)),
    Marker3d(2, make_shared<Marker3d::SinglePoint>(
                  Corner::BOTTOM_RIGHT, Point3d(HALF_CB, -HALF_CB, 0.0), CHESSBOARD_ARUCO_SIZE)),
    Marker3d(3, make_shared<Marker3d::SinglePoint>(
                  Corner::BOTTOM_LEFT, Point3d(-HALF_CB, -HALF_CB, 0.0), CHESSBOARD_ARUCO_SIZE)),
  },

  // 2D Warped corners.
  { {
    Marker2d(0, Corner::TOP_LEFT, Point2f(0, 0)),
    Marker2d(1, Corner::TOP_RIGHT, Point2f(CHESSBOARD_SIZE, 0)),
    Marker2d(2, Corner::BOTTOM_RIGHT, Point2f(CHESSBOARD_SIZE, CHESSBOARD_SIZE)),
    Marker2d(3, Corner::BOTTOM_LEFT, Point2f(0, CHESSBOARD_SIZE)),
  } },
};

//                                                                                                //
// =========================================== Table ============================================
// //
//                                                                                                //

// TODO: Define the Aruco-defined table.

};  // namespace aruco_object_manager