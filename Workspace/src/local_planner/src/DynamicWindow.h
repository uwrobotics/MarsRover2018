#ifndef DYNAMICWINDOW_H
#define DYNAMICWINDOW_H
#include "geometry_msgs/Twist.h"
#include "occupancy_grid/OccupancyGrid.h"
#include "ros/ros.h"

typedef struct RobotParams {
  double maxV;
  double minV;
  double maxW;
  double maxLinAccel;
  double maxLinDecel;
  double maxAngAccel;
  double robotWidth;
  double robotLength;
  double timestep;

  double headingWeight;
  double distanceWeight;
  double velocityWeight;

} RobotParams_t;

// This class implements the logic for a single cycle's
// Dynamic window, including creating the window,
// assessing the points for validity, and scoring them
class CDynamicWindow {
public:
  CDynamicWindow(float curV, float curW, const RobotParams_t &robotParams,
                 bool bDangerOnLeft, bool bDangerOnRight);
  geometry_msgs::Twist
  AssessOccupancyGrid(occupancy_grid::OccupancyGrid::ConstPtr &pGrid,
                      double orentationToGoal);

  // Take note of any obstacles on the sides that may leave the camera cone on
  // the next
  bool FoundDangerOnRight() { return m_bFoundDangerOnRight; }
  bool FoundDangerOnLeft() { return m_bFoundDangerOnLeft; }

private:
  class DynamicWindowPoint {
  public:
    DynamicWindowPoint(float vel, float angVel)
        : v(vel), w(angVel), dist(0), score(0), feasible(false) {}
    float v;
    float w;
    double dist;
    double score;
    bool feasible;
  };

  // The dynamic window grid
  typedef std::vector<DynamicWindowPoint> VelocityRow;
  std::vector<VelocityRow> m_dynamicWindowGrid;

  // Parameters
  const RobotParams_t &m_robotParams;
  float m_vIncrement;
  float m_wIncrement;
  float m_lowV;
  float m_highV;
  float m_lowW;
  float m_highW;
  float m_timestep;

  // Occupancy grid
  occupancy_grid::OccupancyGrid::ConstPtr m_pOccupancyGrid;

  // Status
  float m_curV;
  float m_curW;

  // Keeping track
  double m_maxDist;

  // Take note of any obstacles on the sides that may leave the camera FOV on
  // the next iteration
  bool m_bFoundDangerOnRight;
  bool m_bFoundDangerOnLeft;
};

#endif // DYNAMICWINDOW_H