#ifndef ARUCO_TRANSFORMS__CONFIG_HPP_
#define ARUCO_TRANSFORMS__CONFIG_HPP_

#include "aruco_transforms/aruco_object.hpp"

extern const double ARUCO_SIZE;  // Size of the Aruco marker in meters.

/**
 * Return an ArucoDefinedObject with chessboard data.
 */
ArucoDefinedObject aruco_chessboard();

#endif