//
// Created by tom on 24/02/18.
//
#include "OccupancyUtils.h"
#include <ros/ros.h>
#define HEIGHT_THRESH 0.20
#define SLOPE_THRESH 0.30

//DANGER ZONE
#define DANGER_ZONE_X_DIST 1.25 //times robotWidth
#define DANGER_ZONE_Y_DIST 0.40 //meters

namespace OccupancyUtils {
    typedef enum {
        NOT_TRAVERSABLE = 0,
        TRAVERSABLE,
        BEYOND_GRID,
        INVALID_POINT
    } eTraversableResult;


    template <typename T> int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

//accessor for Occupancy Grid Message
    float oGridDataAccessor (occupancy_grid::OccupancyGrid::ConstPtr& oGrid, unsigned int i, unsigned int j, unsigned int  k) {
        return oGrid->data[i*oGrid->dataDimension[0].stride + j*oGrid->dataDimension[1].stride + k*oGrid->dataDimension[2].stride];
    }

    void PointForCoord(occupancy_grid::OccupancyGrid::ConstPtr& pGrid, double y, double x, int& zOut, int& xOut)
    {
        float coordZ = (y)/pGrid->header.gridResolution;
        float coordX = (x)/pGrid->header.gridResolution + (pGrid->dataDimension[1].size/2.0);
        zOut = (int)(coordZ + 0.5);
        xOut = (int)(coordX + 0.5);
    }


void odbg(double x, double y, int xi, int yi, occupancy_grid::OccupancyGrid::ConstPtr& grid)
{
//ROS_INFO("checking (%f,%f) --> (%d,%d) -- height=%f",x,y,xi,yi,oGridDataAccessor(grid,yi,xi,1));
}  

    eTraversableResult IsPointTraversable(occupancy_grid::OccupancyGrid::ConstPtr& pGrid, double y, double x)
    {
        int zInGrid = 0, xInGrid = 0;
        PointForCoord(pGrid,y,x, zInGrid, xInGrid);
        //TODO:
        if (zInGrid >= (int)pGrid->dataDimension[0].size)
        {
            return BEYOND_GRID;
        }
        if (zInGrid < 0 || xInGrid < 0 || xInGrid >= (int)pGrid->dataDimension[1].size)
        {
            return INVALID_POINT;
        }
        odbg(x,y,xInGrid,zInGrid,pGrid);
        if (oGridDataAccessor(pGrid, zInGrid, xInGrid, 4) > HEIGHT_THRESH /*||
            oGridDataAccessor(pGrid, zInGrid, xInGrid, 5) > SLOPE_THRESH*/ )
        {
            return NOT_TRAVERSABLE;
        }
        else
        {
            return TRAVERSABLE;
        }
    }


    double CalcDistance(occupancy_grid::OccupancyGrid::ConstPtr& pGrid,
                        float v, float w,
                        float robotLength, float robotWidth,
                        float timestep,
                        bool& foundDanger)
    {
//ROS_INFO("v=%f,w=%f",v,w);
//        double radius = ((double) v) / w;
        double retDist = 0;
        ///////////
//        function distance = GetDistScore(OccupancyGrid, velocity, robotDims, resolution, timestep)
//        x = 0;%m
//                y = 0;%m
//                distanceMax=10;
//
        double safetyBubble = 0.2 + 0.2 * v;
        double bufferFromCenter = safetyBubble + robotWidth / 2;
        double halfRobotWidth = robotWidth/2;

        //not moving
        if (std::round(v * 1000) == 0) {
//             int yIndex = 0;//std::floor(1 + robotDims(1)/2/resolution);
//             int xIndexCenter = 0;//size(OccupancyGrid,2)/2;
//             int xIndexLeftCorn = 0;
//             int xIndexRightCorn = 0;
//             PointForCoord(pGrid, 0/*robotLength / 2*/, 0, yIndex, xIndexCenter);
//             PointForCoord(pGrid, 0/*robotLength / 2*/, bufferFromCenter, yIndex, xIndexLeftCorn);
//             PointForCoord(pGrid, 0/*robotLength / 2*/, -bufferFromCenter, yIndex, xIndexRightCorn);

// //        xIndexRightCorn = ceil(xIndexCenter + bufferFromCenter/resolution);
// //        xIndexLeftCorn = floor(xIndexCenter - bufferFromCenter/resolution);
// //        if ((OccupancyGrid(yIndex, xIndexCenter) == 0) || (OccupancyGrid(yIndex, xIndexRightCorn) == 0) || (OccupancyGrid(yIndex, xIndexLeftCorn) == 0))
// //            distance = 0;
// //        else
// //            distance = distanceMax;
// //        end
// odbg(0,0,xIndexCenter,yIndex,pGrid);
// odbg(-bufferFromCenter,0,xIndexLeftCorn,0,pGrid);
// odbg(bufferFromCenter,0,xIndexRightCorn,0,pGrid);
//             if (oGridDataAccessor(pGrid,yIndex, xIndexCenter, 1) > HEIGHT_THRESH ||
//                     oGridDataAccessor(pGrid,yIndex, xIndexRightCorn, 1) > HEIGHT_THRESH ||
//                     oGridDataAccessor(pGrid,yIndex, xIndexLeftCorn, 1) > HEIGHT_THRESH)
//             {
//                 retDist = 0;
//             }
//             else
//             {
//                 retDist = DISTANCE_MAX;
//             }


            double theta = w * timestep;
            double dist = 0;
            bool done = false;
            double centreX = 0;
            double centreY = -robotLength/2;
            while (!done)
            {
                dist += pGrid->header.gridResolution/2.0;
                double cosTheta = cos(theta);
                double sinTheta = sin(theta);
//
                double frontCenterX = centreX - (dist+robotLength/2)*sinTheta;
                double frontCenterY = centreY + (dist+robotLength/2)*cosTheta;
                double frontRightX = centreX - (dist+robotLength/2)*sinTheta + bufferFromCenter*cosTheta;
                double frontRightY = centreY + (dist+robotLength/2)*cosTheta + bufferFromCenter*sinTheta;
                double frontLeftX = centreX - (dist+robotLength/2)*sinTheta - bufferFromCenter*cosTheta;
                double frontLeftY = centreY + (dist+robotLength/2)*cosTheta - bufferFromCenter*sinTheta;


                int xIndexCenter = 0;//size(OccupancyGrid,2)/2;
                int xIndexLeftCorn = 0;
                int xIndexRightCorn = 0;
                int yIndexCenter = 0;//size(OccupancyGrid,2)/2;
                int yIndexLeftCorn = 0;
                int yIndexRightCorn = 0;


                PointForCoord(pGrid, frontCenterY,frontCenterX, yIndexCenter, xIndexCenter);
                PointForCoord(pGrid, frontRightY,frontRightX, yIndexRightCorn, xIndexRightCorn);
                PointForCoord(pGrid, frontLeftY,frontLeftX, yIndexLeftCorn, xIndexLeftCorn);


                if ((yIndexCenter >= (int)pGrid->dataDimension[0].size) ||
                        (yIndexRightCorn >= (int)pGrid->dataDimension[0].size) ||
                                (yIndexLeftCorn >= (int)pGrid->dataDimension[0].size))
                {
                    done = true;
                    retDist = DISTANCE_MAX;
//ROS_INFO("%d,%d,%d",yIndexCenter,yIndexRightCorn,yIndexLeftCorn);
                }
                else if ((xIndexLeftCorn < 0) ||
                        xIndexRightCorn < 0 ||
                        (xIndexRightCorn > (int)pGrid->dataDimension[1].size) ||
                        (xIndexLeftCorn > (int)pGrid->dataDimension[1].size))
                {
                    retDist = dist;
                    done = true;
                }
                else
                {
//if (yIndexCenter >= 0){ odbg(frontCenterX,frontCenterY, xIndexCenter,yIndexCenter,pGrid);} else {ROS_INFO("skip");}
//if (yIndexRightCorn >= 0){ odbg(frontRightX,frontRightY, xIndexRightCorn,yIndexRightCorn,pGrid);} else {ROS_INFO("skip");}
//if (yIndexLeftCorn >= 0){ odbg(frontLeftX,frontLeftY, xIndexLeftCorn,yIndexLeftCorn,pGrid);} else {ROS_INFO("skip");}
                    if ((yIndexCenter >= 0 && oGridDataAccessor(pGrid,yIndexCenter,xIndexCenter,1) > HEIGHT_THRESH) ||
                            (yIndexRightCorn >= 0 && oGridDataAccessor(pGrid,yIndexRightCorn,xIndexRightCorn,1) > HEIGHT_THRESH) ||
                            (yIndexLeftCorn >= 0 && oGridDataAccessor(pGrid,yIndexLeftCorn,xIndexLeftCorn,1) > HEIGHT_THRESH))
                    {
                        retDist = dist;
                        done = true;
//        end
                    }

                }


            }


        }

        else if (round(w*1000) == 0)
        {
            //Going straight
            int yIndex = 0;//std::floor(1 + robotDims(1)/2/resolution);
            int xIndexCenter = 0;//size(OccupancyGrid,2)/2;
            int xIndexLeftCorn = 0;
            int xIndexRightCorn = 0;
            PointForCoord(pGrid, 0/*robotLength / 2*/, 0, yIndex, xIndexCenter);
            PointForCoord(pGrid, 0/*robotLength / 2*/, bufferFromCenter, yIndex, xIndexLeftCorn);
            PointForCoord(pGrid, 0/*robotLength / 2*/, -bufferFromCenter, yIndex, xIndexRightCorn);
            double distance = -1.0;
            bool continueChecking = true;

//        yIndex = ceil(1 + robotDims(1)/2/resolution);
//        xIndexCenter = size(OccupancyGrid,2)/2;
//        xIndexRightCorn = ceil(xIndexCenter + bufferFromCenter/resolution);
//        xIndexLeftCorn = floor(xIndexCenter - bufferFromCenter/resolution);
//        distance = -1.0;
//        continueChecking = true;
//
            while (continueChecking)
            {
                if (yIndex >= pGrid->dataDimension[0].size)
                {
                    retDist = DISTANCE_MAX;// -velocity(1) * timestep;
                    continueChecking=false;
//        end
//                end
//        end
                }
        		else
                {

        odbg(0,yIndex*pGrid->header.gridResolution, xIndexCenter,yIndex,pGrid);
        odbg(-bufferFromCenter,yIndex*pGrid->header.gridResolution,xIndexLeftCorn,yIndex,pGrid);
        odbg(bufferFromCenter,yIndex*pGrid->header.gridResolution,xIndexRightCorn,yIndex,pGrid);
                            if (oGridDataAccessor(pGrid,yIndex, xIndexCenter, 1) > HEIGHT_THRESH ||
                                oGridDataAccessor(pGrid,yIndex, xIndexRightCorn, 1) > HEIGHT_THRESH ||
                                oGridDataAccessor(pGrid,yIndex, xIndexLeftCorn, 1) > HEIGHT_THRESH)
                            {
                                continueChecking = false;
                                retDist = yIndex * pGrid->header.gridResolution;// - timestep * v;
                            }
                            else
                            {
                                yIndex = yIndex + 1;
                            }
        		}
            }
        }
        else
        {
//        else
            double radius = v/w;
//
            //sample at regular intervals along the arc
            double distTravelled = 0.0;
            double ds = pGrid->header.gridResolution/2.0;
            double dTheta = ds/radius;
//
            bool done = false;
//
            double centreX = -radius;
            double centreY = -robotLength/2;
            double theta = 0;//asin(robotLength/2/(radius - bufferFromCenter));//0;// + robotLength/2/radius;//Start checking from the front of the robot
//ROS_INFO("rad: %f, ds: %f, dtheta: %f",radius, ds,dTheta);
            while (!done)
            {
                theta = theta + dTheta;
//
                double cosTheta = cos(theta);
                double sinTheta = sin(theta);
//
                double frontCenterX = centreX + radius*cosTheta - robotLength/2*sinTheta;
                double frontCenterY = centreY + radius*sinTheta + robotLength/2*cosTheta;
                double frontRightX = centreX + (radius + bufferFromCenter)*cosTheta - robotLength/2*sinTheta;
                double frontRightY = centreY + (radius + bufferFromCenter)*sinTheta + robotLength/2*cosTheta;
                double frontLeftX = centreX + (radius - bufferFromCenter)*cosTheta - robotLength/2*sinTheta;
                double frontLeftY = centreY + (radius - bufferFromCenter)*sinTheta + robotLength/2*cosTheta;
//
                distTravelled += ds;
//
                /*
//ROS_INFO("distTravelled=%f",distTravelled);
                int centerXIndex = 0;//round(frontCenterX/resolution + size(OccupancyGrid,2)/2);
                int centerYIndex = 0;//round(frontCenterY/resolution + 1);
                int rightXIndex = 0;//round(frontRightX/resolution + size(OccupancyGrid,2)/2);
                int rightYIndex = 0;//round(frontRightY/resolution + 1);
                int leftXIndex = 0;//round(frontLeftX/resolution + size(OccupancyGrid,2)/2);
                int leftYIndex = 0;//round(frontLeftY/resolution + 1);

                PointForCoord(pGrid, frontCenterY,frontCenterX, centerYIndex, centerXIndex);
                PointForCoord(pGrid, frontRightY,frontRightX, rightYIndex, rightXIndex);
                PointForCoord(pGrid, frontLeftY,frontLeftX, leftYIndex, leftXIndex);
//
                if ((centerYIndex >= (int)pGrid->dataDimension[0].size) ||
                        (rightYIndex >= (int)pGrid->dataDimension[0].size) ||
                                (leftYIndex >= (int)pGrid->dataDimension[0].size))
                {
                    done = true;
                    retDist = DISTANCE_MAX;
//ROS_INFO("%d,%d,%d",centerYIndex,rightYIndex,leftYIndex);
                }
                else if ((leftXIndex < 0) ||
                        rightXIndex < 0 ||
                        (rightXIndex > (int)pGrid->dataDimension[1].size) ||
                        (leftXIndex > (int)pGrid->dataDimension[1].size) ||
                        centerYIndex < 0 || rightYIndex < 0 || leftYIndex < 0)
                {
                    retDist = distTravelled - timestep * v;
                    done = true;
                }
                else
{
odbg(frontCenterX,frontCenterY, centerXIndex,centerYIndex,pGrid);
odbg(frontRightX,frontRightY, rightXIndex,rightYIndex,pGrid);
odbg(frontLeftX,frontLeftY, leftXIndex,leftYIndex,pGrid);
 if ((oGridDataAccessor(pGrid,centerYIndex,centerXIndex,1) > HEIGHT_THRESH) ||
                        (oGridDataAccessor(pGrid,rightYIndex,rightXIndex,1) > HEIGHT_THRESH) ||
                        (oGridDataAccessor(pGrid,leftYIndex,leftXIndex,1) > HEIGHT_THRESH))
                {
                    retDist = distTravelled - timestep * v;
                    done = true;
//        end
                }

*/

                eTraversableResult leftResult = IsPointTraversable(pGrid, frontLeftY,frontLeftX);
                eTraversableResult rightResult = IsPointTraversable(pGrid, frontRightY,frontRightX);
                eTraversableResult centerResult = IsPointTraversable(pGrid, frontCenterY,frontCenterX);

                if (leftResult == INVALID_POINT || rightResult == INVALID_POINT || centerResult == INVALID_POINT)
                {
                    retDist = distTravelled - timestep * v;
                    done = true;
                }
                else if (leftResult == BEYOND_GRID || rightResult == BEYOND_GRID || centerResult == BEYOND_GRID)
                {
                    retDist = DISTANCE_MAX;
                    done = true;
                }
                else if (leftResult == NOT_TRAVERSABLE || rightResult == NOT_TRAVERSABLE || centerResult == NOT_TRAVERSABLE)
                {
                    //can't go here
                    retDist = distTravelled - timestep * v;
                    done = true;

                    //check for danger
                    if (leftResult == NOT_TRAVERSABLE && 
                        (sgn(w) == -sgn(frontLeftX) && abs(frontLeftX) < DANGER_ZONE_X_DIST*halfRobotWidth && frontLeftY < DANGER_ZONE_Y_DIST))
                    {
                        ROS_INFO("Found danger at %f,%f for %f,%f", frontLeftX,frontLeftY,v,w);
                        foundDanger = true;
                    }
                    if (rightResult == NOT_TRAVERSABLE && 
                        (sgn(w) == -sgn(frontRightX) && abs(frontRightX) < DANGER_ZONE_X_DIST*halfRobotWidth && frontRightY < DANGER_ZONE_Y_DIST))
                    {
                        ROS_INFO("Found danger at %f,%f for %f,%f", frontRightX,frontRightY,v,w);
                        foundDanger = true;
                    }
                    if (centerResult == NOT_TRAVERSABLE && 
                        (sgn(w) == -sgn(frontCenterX) && abs(frontCenterX) < DANGER_ZONE_X_DIST*halfRobotWidth && frontCenterY < DANGER_ZONE_Y_DIST))
                    {
                        ROS_INFO("Found danger at %f,%f for %f,%f", frontCenterX,frontCenterY,v,w);
                        foundDanger = true;
                    }

                }
//                end
           }
//

        }
//        end
//
//
//                end
//

        /////////////


//ROS_INFO("Traveled %f",retDist);
        return retDist;
    }

}
