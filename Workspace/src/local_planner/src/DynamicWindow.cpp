#include "DynamicWindow.h"
#include <cmath>
#include <algorithm>
#include "OccupancyUtils.h"

#define INF_DIST 100.0

CDynamicWindow::CDynamicWindow(float curV, float curW, const RobotParams_t& robotParams, bool bDangerOnLeft, bool bDangerOnRight)
 : m_robotParams(robotParams),
   m_vIncrement(0.1),
   m_wIncrement(0.1),
   m_timestep(robotParams.timestep),
   m_curV(curV),
   m_curW(curW),
   m_maxDist(0),
   m_bFoundDangerOnRight(false),
   m_bFoundDangerOnLeft(false)
{
    m_lowV = std::max(curV - m_robotParams.maxLinDecel * m_timestep, m_robotParams.minV);
    m_highV = std::min(curV + m_robotParams.maxLinAccel * m_timestep, m_robotParams.maxV);
    m_lowW = std::max(curW - m_robotParams.maxAngAccel*m_timestep, -m_robotParams.maxW);
    m_highW = std::min(curW + m_robotParams.maxAngAccel*m_timestep, m_robotParams.maxW);

    if (bDangerOnRight)
    {
        m_lowW = std::max(m_lowW, 0.0f);
    }
    if (bDangerOnLeft)
    {
        m_highW = std::min(m_highW, 0.0f);
    }

    m_dynamicWindowGrid.resize(std::round((m_highV - m_lowV)/m_vIncrement) + 1);
    int row = 0;
    for (auto& velocityRow : m_dynamicWindowGrid)
    {
        unsigned long rowSize = std::round((m_highW - m_lowW) / m_wIncrement) + 1;
        velocityRow.reserve(rowSize);
        for (int col = 0; col < rowSize; col++) {
            velocityRow.emplace_back(m_lowV + row * m_vIncrement,
                                     m_lowW + col * m_wIncrement);
        }


        row++;
    }
}

geometry_msgs::Twist CDynamicWindow::AssessOccupancyGrid(occupancy_grid::OccupancyGrid::ConstPtr& pGrid,
                                                         double orientationToGoal)
{
    m_pOccupancyGrid = pGrid;
    for (auto& velocityRow : m_dynamicWindowGrid)
    {
        for (auto& dynWndPnt : velocityRow)
        {
            bool foundDanger = false;
            double distance = OccupancyUtils::CalcDistance(pGrid,
                                                           dynWndPnt.v,
                                                           dynWndPnt.w,
                                                           m_robotParams.robotLength,
                                                           m_robotParams.robotWidth,
                                                           m_timestep,
                                                           foundDanger);
            if (/*m_curV*m_curV*/std::max(m_curV*m_curV, dynWndPnt.v*dynWndPnt.v)  >= 2*distance*m_robotParams.maxLinDecel)
            {
                dynWndPnt.feasible = false;
                ROS_INFO("Rejecting v=%f, w=%f",dynWndPnt.v,dynWndPnt.w);
            }
            else
            {
                dynWndPnt.feasible = true;
                dynWndPnt.dist = distance;

                if (distance != DISTANCE_MAX && distance > m_maxDist)
                {
                    m_maxDist = distance;
                }

            }

            if (foundDanger)
            {
                //Note the danger of the point.
                if (dynWndPnt.w > 0)
                {
                    ROS_INFO("Left danger for v:%f, w=%f",dynWndPnt.v,dynWndPnt.w);
                    m_bFoundDangerOnLeft = true;
                }
                else if (dynWndPnt.w < 0)
                {
                    ROS_INFO("Right danger for v:%f, w=%f",dynWndPnt.v,dynWndPnt.w);
                    m_bFoundDangerOnRight = true;
                }
            }
        }
    }

    double highestScore = -1;
    DynamicWindowPoint* pBestPoint = nullptr;
    for (auto& velocityRow : m_dynamicWindowGrid)
    {
        for (auto &dynWndPnt : velocityRow) {
            if (!dynWndPnt.feasible)
            {
                continue;
            }
            double score = 0;

            //Assess distance score
            double distanceScore = 0;
            if (dynWndPnt.dist == DISTANCE_MAX)
            {
                distanceScore = 1;
            }
            else
            {
                distanceScore = dynWndPnt.dist/m_maxDist;
            }

            //Assess heading score
            //need current gps heading, heading to gps goal
            double headingScore = 0;
            double headingChange = dynWndPnt.w*m_timestep;
            double newHeadingToGoal = /*fabs*/(orientationToGoal - headingChange);

//ROS_INFO("v=%f, w=%f, orientToGoal=%f", dynWndPnt.v, dynWndPnt.w, orientationToGoal);
            if (newHeadingToGoal > M_PI)
            {
                newHeadingToGoal -= 2*M_PI;
            }
            if (newHeadingToGoal < -M_PI)
            {
                newHeadingToGoal += 2*M_PI;
            }
//ROS_INFO("change=%f, newHeading=%f", headingChange, newHeadingToGoal);
            headingScore = fabs(M_PI - fabs(newHeadingToGoal))/M_PI;

            //velocityScore
            double velocityScore = (dynWndPnt.v - m_lowV)/(m_highV - m_lowV);

            score = 1.5*headingScore + 0.9*distanceScore + 0.4*velocityScore;
            ROS_INFO("v=%f, w=%f :  dist=%f, dScore=%f, hScore=%f, vScore=%f, score=%f", dynWndPnt.v, dynWndPnt.w,
                        dynWndPnt.dist, distanceScore, headingScore, velocityScore, score);
            if (score > highestScore)
            {
                highestScore = score;
                pBestPoint = &dynWndPnt;
                ROS_INFO("new best vel: v=%f, w=%f, score=%f", dynWndPnt.v, dynWndPnt.w, score);
            }
        }
    }
    geometry_msgs::Twist ret;
    ret.linear.y = 0;
    ret.linear.z = 0;
    ret.angular.x = 0;
    ret.angular.y = 0;
    if (!pBestPoint)
    {
        ret.linear.x = 0;
        ret.angular.z = 0;
    } else
    {
        ret.linear.x = pBestPoint->v;
        ret.angular.z = pBestPoint->w;
    }
    return ret;
}

