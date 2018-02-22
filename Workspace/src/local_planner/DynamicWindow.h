#ifndef DYNAMICWINDOW_H
#define DYNAMICWINDOW_H
#include "ros/ros.h"



//This class implements the logic for a single cycle's
//Dynamic window, including creating the window,
//assessing the points for validity, and scoring them
class CDynamicWindow
{
public:
    CDynamicWindow();

private:
    class DynamicWindowPoint
    {
        DynamicWindowPoint(float vel, float angvel)
        : v(vel),
          w(angvel),
          dist(0),
          score(0)
        {
        }
        float v;
        float w;
        float dist;
        float score;
    };

    //The dynamic window grid
    typedef std::vector< DynamicWindowPoint > VelocityRow;
    std::vector < VelocityRow > m_dynamicWindowGrid;


};

#endif //DYNAMICWINDOW_H