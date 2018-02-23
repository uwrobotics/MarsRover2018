#ifndef DYNAMICWINDOW_H
#define DYNAMICWINDOW_H
#include "ros/ros.h"
#include "local_planner/OccupancyGrid.h"


typedef struct RobotParams
{
    float maxV;
    float minV;
    float maxW;
    float maxLinAccel;
    float maxLinDecel;
    float maxAngAccel;
    float robotWidth;
    float robotLength;

} RobotParams_t;

//This class implements the logic for a single cycle's
//Dynamic window, including creating the window,
//assessing the points for validity, and scoring them
class CDynamicWindow
{
public:
    CDynamicWindow(float curV, float curW, const RobotParams_t& robotParams);
    void AssessOccupancyGrid(local_planner::OccupancyGrid::ConstPtr pGrid);
private:
    class DynamicWindowPoint
    {
    public:

        DynamicWindowPoint(float vel, float angVel)
        : v(vel),
          w(angVel),
          dist(0),
          score(0),
          feasible(false)
        {
        }
        float v;
        float w;
        float dist;
        float score;
        bool feasible;
    };


    double CalcDistance(float v, float w);

    //The dynamic window grid
    typedef std::vector< DynamicWindowPoint > VelocityRow;
    std::vector < VelocityRow > m_dynamicWindowGrid;


    //Parameters
    const RobotParams_t& m_robotParams;


};

#endif //DYNAMICWINDOW_H