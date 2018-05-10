//
// Created by tom on 24/02/18.
//
#include "OccupancyUtils.h"
#include <ros/ros.h>
#define HEIGHT_THRESH 0.20
#define SLOPE_THRESH 0.30

// DANGER ZONE
#define DANGER_ZONE_X_DIST 1.25 // times robotWidth
#define DANGER_ZONE_Y_DIST 0.40 // meters

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

OccupancyUtils::OccupancyUtils(occupancy_grid::OccupancyGrid::ConstPtr &pGrid,
                               float robotLength, float robotWidth,
                               float timestep)
    : m_pGrid(pGrid), m_robotLength(robotLength), m_robotWidth(robotWidth),
      m_timestep(timestep) {
  m_maxW = 1.0;
  ros::param::get("/roverParams_maxW", m_maxW);
}

// accessor for Occupancy Grid Message
float OccupancyUtils::GridDataAccessor(unsigned int i, unsigned int j,
                                       unsigned int k) {
  return m_pGrid->data[i * m_pGrid->dataDimension[0].stride +
                       j * m_pGrid->dataDimension[1].stride +
                       k * m_pGrid->dataDimension[2].stride];
}

void OccupancyUtils::PointForCoord(double y, double x, int &zOut, int &xOut) {
  double coordZ = (y) / m_pGrid->header.gridResolution;
  double coordX = (x) / m_pGrid->header.gridResolution +
                  (m_pGrid->dataDimension[1].size / 2.0);
  zOut = (int)std::round(coordZ);
  xOut = (int)std::round(coordX);
}

static void odbg(double x, double y, int xi, int yi,
                 occupancy_grid::OccupancyGrid::ConstPtr &grid) {
  // ROS_INFO("checking (%f,%f) --> (%d,%d) --
  // height=%f",x,y,xi,yi,GridDataAccessor(grid,yi,xi,1));
}

OccupancyUtils::eTraversableResult
OccupancyUtils::IsPointTraversable(double x, double y) {
  int zInGrid = 0, xInGrid = 0;
  PointForCoord(y, x, zInGrid, xInGrid);
  // TODO:
  if (zInGrid >= (int)m_pGrid->dataDimension[0].size) {
    return BEYOND_GRID;
  }
  if (zInGrid < 0 || xInGrid < 0 ||
      xInGrid >= (int)m_pGrid->dataDimension[1].size) {
    return INVALID_POINT;
  }
  odbg(x, y, xInGrid, zInGrid, m_pGrid);
  if (GridDataAccessor(zInGrid, xInGrid, 4) > HEIGHT_THRESH /*||
            GridDataAccessor(zInGrid, xInGrid, 5) > SLOPE_THRESH*/) {
    return NOT_TRAVERSABLE;
  } else {
    return TRAVERSABLE;
  }
}

double OccupancyUtils::CalcDistance(float v, float w, bool &foundDanger) {
  ROS_INFO("v=%f,w=%f", v, w);
  //        double radius = ((double) v) / w;
  double retDist = 0;
  double safetyBubble = 0.2 + 0.2 * v;
  double bufferFromCenter = safetyBubble + m_robotWidth / 2;
  double halfRobotWidth = m_robotWidth / 2;

  // not moving
  if (std::round(v * 1000) == 0) {

    double theta = M_PI / 2 * w / m_maxW; // w * m_timestep * 2;
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    double dist = 0;
    bool done = false;
    double centreX = 0;
    double centreY = -m_robotLength / 2;
    while (!done) {
      dist += m_pGrid->header.gridResolution / 2.0;

      double frontCenterX = centreX - (dist + m_robotLength / 2) * sinTheta;
      double frontCenterY = centreY + (dist + m_robotLength / 2) * cosTheta;
      double frontRightX = centreX - (dist + m_robotLength / 2) * sinTheta +
                           bufferFromCenter * cosTheta;
      double frontRightY = centreY + (dist + m_robotLength / 2) * cosTheta +
                           bufferFromCenter * sinTheta;
      double frontLeftX = centreX - (dist + m_robotLength / 2) * sinTheta -
                          bufferFromCenter * cosTheta;
      double frontLeftY = centreY + (dist + m_robotLength / 2) * cosTheta -
                          bufferFromCenter * sinTheta;

      eTraversableResult centerTraversability =
          IsPointTraversable(frontCenterX, frontCenterY);
      eTraversableResult rightTraversability =
          IsPointTraversable(frontRightX, frontRightY);
      eTraversableResult leftTraversability =
          IsPointTraversable(frontLeftX, frontLeftY);
      // ROS_INFO("left: (%f,%f), result=%d",frontLeftX, frontLeftY,
      // leftTraversability);
      // ROS_INFO("center: (%f,%f), result=%d",frontCenterX, frontCenterY,
      // centerTraversability);
      ROS_INFO("right: (%f,%f), result=%d", frontRightX, frontRightY,
               rightTraversability);

      if ((centerTraversability == BEYOND_GRID) ||
          (leftTraversability == BEYOND_GRID) ||
          (rightTraversability == BEYOND_GRID)) {
        done = true;
        retDist = DISTANCE_INF;
      } else if ((centerTraversability != TRAVERSABLE && frontCenterY > 0) ||
                 (rightTraversability != TRAVERSABLE && frontRightY > 0) ||
                 (leftTraversability != TRAVERSABLE && frontLeftY > 0)) {
        done = true;
        retDist = dist;
      } else {
      }
    }

  }

  else if (round(w * 1000) == 0) {
    // Going straight
    double xCenter = 0, xLeft = -bufferFromCenter, xRight = bufferFromCenter;
    double y = 0;

    bool continueChecking = true;

    while (continueChecking) {
      eTraversableResult leftTraversability = IsPointTraversable(xLeft, y);
      eTraversableResult centerTraversability = IsPointTraversable(xCenter, y);
      eTraversableResult rightTraversability = IsPointTraversable(xRight, y);

      if (centerTraversability == BEYOND_GRID) {
        continueChecking = false;
        retDist = DISTANCE_INF;
      } else if ((centerTraversability != TRAVERSABLE) ||
                 (rightTraversability != TRAVERSABLE) ||
                 (leftTraversability != TRAVERSABLE)) {
        continueChecking = false;
        retDist = y - m_timestep * v;
      } else {
        y += m_pGrid->header.gridResolution;
      }
    }
  } else {
    double radius = v / w;

    // sample at regular intervals along the arc
    double distTravelled = 0.0;
    double ds = m_pGrid->header.gridResolution / 2.0;
    double dTheta = ds / radius;
    //
    bool done = false;
    //
    double centreX = -radius;
    double centreY = -m_robotLength / 2;
    double theta = 0; // asin(robotLength/2/(radius - bufferFromCenter));//0;//
                      // + robotLength/2/radius;//Start checking from the front
                      // of the robot
    // ROS_INFO("rad: %f, ds: %f, dtheta: %f",radius, ds,dTheta);
    while (!done) {
      theta = theta + dTheta;
      //
      double cosTheta = cos(theta);
      double sinTheta = sin(theta);
      //
      double frontCenterX =
          centreX + radius * cosTheta - m_robotLength / 2 * sinTheta;
      double frontCenterY =
          centreY + radius * sinTheta + m_robotLength / 2 * cosTheta;
      double frontRightX = centreX + (radius + bufferFromCenter) * cosTheta -
                           m_robotLength / 2 * sinTheta;
      double frontRightY = centreY + (radius + bufferFromCenter) * sinTheta +
                           m_robotLength / 2 * cosTheta;
      double frontLeftX = centreX + (radius - bufferFromCenter) * cosTheta -
                          m_robotLength / 2 * sinTheta;
      double frontLeftY = centreY + (radius - bufferFromCenter) * sinTheta +
                          m_robotLength / 2 * cosTheta;
      distTravelled += ds;

      eTraversableResult leftResult =
          IsPointTraversable(frontLeftX, frontLeftY);
      eTraversableResult rightResult =
          IsPointTraversable(frontRightX, frontRightY);
      eTraversableResult centerResult =
          IsPointTraversable(frontCenterX, frontCenterY);

      if (leftResult == INVALID_POINT || rightResult == INVALID_POINT ||
          centerResult == INVALID_POINT) {
        retDist = distTravelled - m_timestep * v;
        done = true;
      } else if (leftResult == BEYOND_GRID || rightResult == BEYOND_GRID ||
                 centerResult == BEYOND_GRID) {
        retDist = DISTANCE_INF;
        done = true;
      } else if (leftResult == NOT_TRAVERSABLE ||
                 rightResult == NOT_TRAVERSABLE ||
                 centerResult == NOT_TRAVERSABLE) {
        // can't go here
        retDist = distTravelled - m_timestep * v;
        done = true;

        // check for danger
        if (leftResult == NOT_TRAVERSABLE &&
            (sgn(w) == -sgn(frontLeftX) &&
             abs(frontLeftX) < DANGER_ZONE_X_DIST * halfRobotWidth &&
             frontLeftY < DANGER_ZONE_Y_DIST)) {
          ROS_INFO("Found danger at %f,%f for %f,%f", frontLeftX, frontLeftY, v,
                   w);
          foundDanger = true;
        }
        if (rightResult == NOT_TRAVERSABLE &&
            (sgn(w) == -sgn(frontRightX) &&
             abs(frontRightX) < DANGER_ZONE_X_DIST * halfRobotWidth &&
             frontRightY < DANGER_ZONE_Y_DIST)) {
          ROS_INFO("Found danger at %f,%f for %f,%f", frontRightX, frontRightY,
                   v, w);
          foundDanger = true;
        }
        if (centerResult == NOT_TRAVERSABLE &&
            (sgn(w) == -sgn(frontCenterX) &&
             abs(frontCenterX) < DANGER_ZONE_X_DIST * halfRobotWidth &&
             frontCenterY < DANGER_ZONE_Y_DIST)) {
          ROS_INFO("Found danger at %f,%f for %f,%f", frontCenterX,
                   frontCenterY, v, w);
          foundDanger = true;
        }
      }
    }
  }

  // ROS_INFO("Traveled %f",retDist);
  return retDist;
}
