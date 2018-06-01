#include "DynamicWindow.h"
#include "OccupancyUtils.h"
#include <algorithm>
#include <cmath>

CDynamicWindow::CDynamicWindow(float curV, float curW,
                               const RobotParams_t &robotParams,
                               bool bDangerOnLeft, bool bDangerOnRight)
    : m_robotParams(robotParams), m_vIncrement(0.1), m_wIncrement(0.1),
      m_timestep(robotParams.timestep), m_curV(curV), m_curW(curW),
      m_maxDist(0), m_bFoundDangerOnRight(false), m_bFoundDangerOnLeft(false) {
  // determine the bounds on velocities
  m_lowV = std::max(curV - m_robotParams.maxLinDecel * m_timestep,
                    m_robotParams.minV);
  m_highV = std::min(curV + m_robotParams.maxLinAccel * m_timestep,
                     m_robotParams.maxV);
  m_lowW = std::max(curW - m_robotParams.maxAngAccel * m_timestep,
                    -m_robotParams.maxW);
  m_highW = std::min(curW + m_robotParams.maxAngAccel * m_timestep,
                     m_robotParams.maxW);

  // clip the dynamic window if a side is to be avoided
  if (bDangerOnRight) {
    m_lowW = std::max(m_lowW, 0.0f);
  }
  if (bDangerOnLeft) {
    m_highW = std::min(m_highW, 0.0f);
  }
  if (bDangerOnLeft && bDangerOnRight) {
    m_lowW = -m_wIncrement;
    m_highW = m_wIncrement;
  }
  if (m_lowW > m_highW) {
    double temp = m_lowW;
    m_lowW = m_highW;
    m_highW = temp;
  }

  // construct the dynamic window
  m_dynamicWindowGrid.resize(std::round((m_highV - m_lowV) / m_vIncrement) + 1);
  int row = 0;
  for (auto &velocityRow : m_dynamicWindowGrid) {
    double velocity = m_lowV + row * m_vIncrement;
    // if this row is for approximately zero velocity, populate using radial
    // velocities
    if (std::round(velocity * 1000) == 0) {
      unsigned long rowSize = std::round((m_highW - m_lowW) / m_wIncrement) + 1;
      velocityRow.reserve(rowSize);
      for (int col = 0; col < rowSize; col++) {
        velocityRow.emplace_back(m_lowV + row * m_vIncrement,
                                 m_lowW + col * m_wIncrement);
      }

    } else {
      // if the speed is non-zero, then instead populate based on turning radii
      // for a better selection of trajectories
      // note: rad = v/w
      double maxRad = 10;
      double radIncrement = 0.5;

      unsigned long rowSize = 2 * std::round(maxRad / radIncrement) + 1;
      velocityRow.reserve(rowSize);

      double curRad = -radIncrement;
      for (double curRad = -radIncrement; curRad > -maxRad;
           curRad -= radIncrement) {
        velocityRow.emplace_back(velocity, velocity / curRad);
      }

      // add a straight trajectory
      velocityRow.emplace_back(velocity, 0);

      for (double curRad = radIncrement; curRad < maxRad;
           curRad += radIncrement) {
        velocityRow.emplace_back(velocity, velocity / curRad);
      }
    }
    row++;
  }
}

// assess the options in the dynamic window
// first, go through the window and determine the distance to colision for each
// trajectory
// reject any trajectories with unacceptable distances
// then go back through the admissible trajectories and score them
geometry_msgs::Twist CDynamicWindow::AssessOccupancyGrid(
    occupancy_grid::OccupancyGrid::ConstPtr &pGrid, double orientationToGoal) {

  m_pOccupancyGrid = pGrid;
  OccupancyUtils OccupancyGridCalculator(pGrid, m_robotParams.robotLength,
                                         m_robotParams.robotWidth, m_timestep);
  // first loop through:
  // calculate distances and reject unacceptable trajectories
  for (auto &velocityRow : m_dynamicWindowGrid) {
    for (auto &dynWndPnt : velocityRow) {
      // reject points with infeasible radial velocities
      if (dynWndPnt.w < m_lowW || dynWndPnt.w > m_highW) {
        dynWndPnt.feasible = false;
        continue;
      }
      bool foundDanger = false;
      // Find the distanace to collision
      double distance = OccupancyGridCalculator.CalcDistance(
          dynWndPnt.v, dynWndPnt.w, foundDanger);
      if (/*m_curV*m_curV*/ std::max(m_curV * m_curV,
                                     dynWndPnt.v * dynWndPnt.v) >=
          2 * distance * m_robotParams.maxLinDecel) {
        // distance is too short, reject it
        dynWndPnt.feasible = false;
        ROS_INFO("Rejecting v=%f, w=%f", dynWndPnt.v, dynWndPnt.w);
      } else {
        // accept the trajectory and keep track of the best distance
        dynWndPnt.feasible = true;
        dynWndPnt.dist = distance;

        if (distance != DISTANCE_INF && distance > m_maxDist) {
          m_maxDist = distance;
        }
      }

      if (foundDanger) {
        // Note the danger found by the trajectory, and which side its on.
        if (dynWndPnt.w > 0) {
          ROS_INFO("Left danger for v:%f, w=%f", dynWndPnt.v, dynWndPnt.w);
          m_bFoundDangerOnLeft = true;
        } else if (dynWndPnt.w < 0) {
          ROS_INFO("Right danger for v:%f, w=%f", dynWndPnt.v, dynWndPnt.w);
          m_bFoundDangerOnRight = true;
        }
      }
    }
  }

  double highestScore = -1;
  DynamicWindowPoint *pBestPoint = nullptr;
  // Second loop through:
  // Calculate each trajectory's score and pick the best
  for (auto &velocityRow : m_dynamicWindowGrid) {
    for (auto &dynWndPnt : velocityRow) {
      if (!dynWndPnt.feasible) {
        continue;
      }
      double score = 0;

      // Assess distance score
      double distanceScore = 0;
      if (dynWndPnt.dist == DISTANCE_INF) {
        distanceScore = 1;
      } else {
        // distance score is normalized from 0 to 1,
        // where the furthest finite distance gets 1
        distanceScore = dynWndPnt.dist / m_maxDist;
      }

      // Assess heading score
      // need current gps heading, heading to gps goal
      double headingScore = 0;
      double headingChange = dynWndPnt.w * m_timestep;
      double newHeadingToGoal = /*fabs*/ (orientationToGoal - headingChange);

      if (newHeadingToGoal > M_PI) {
        newHeadingToGoal -= 2 * M_PI;
      }
      if (newHeadingToGoal < -M_PI) {
        newHeadingToGoal += 2 * M_PI;
      }
      // score is 0 to 1, where 1 is directly towards target
      // and 0 is directly away
      headingScore = fabs(M_PI - fabs(newHeadingToGoal)) / M_PI;

      // velocityScore
      // score is 0 to 1, where 0 is for the minimum velocity
      // and 1 is for the max velocity
      double velocityScore = (dynWndPnt.v - m_lowV) / (m_highV - m_lowV);

      // compute the total weighted score, and keep track of the best trajectory
      score = m_robotParams.headingWeight * headingScore +
              m_robotParams.distanceWeight * distanceScore +
              m_robotParams.velocityWeight * velocityScore;
      ROS_INFO(
          "v=%f, w=%f :  dist=%f, dScore=%f, hScore=%f, vScore=%f, score=%f",
          dynWndPnt.v, dynWndPnt.w, dynWndPnt.dist, distanceScore, headingScore,
          velocityScore, score);
      if (score > highestScore) {
        highestScore = score;
        pBestPoint = &dynWndPnt;
        ROS_INFO("new best vel: v=%f, w=%f, score=%f", dynWndPnt.v, dynWndPnt.w,
                 score);
      }
    }
  }

  // save the best velocity
  geometry_msgs::Twist ret;
  ret.linear.y = 0;
  ret.linear.z = 0;
  ret.angular.x = 0;
  ret.angular.y = 0;
  if (!pBestPoint) {
    ret.linear.x = 0;
    ret.angular.z = 0;
  } else {
    ret.linear.x = pBestPoint->v;
    ret.angular.z = pBestPoint->w;
  }
  return ret;
}
