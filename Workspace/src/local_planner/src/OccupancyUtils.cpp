//
// Created by tom on 24/02/18.
//
#include "OccupancyUtils.h"
#include <ros/ros.h>
#define HEIGHT_THRESH 0.02

namespace OccupancyUtils {


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

    double CalcDistance(occupancy_grid::OccupancyGrid::ConstPtr& pGrid,
                        float v, float w,
                        float robotLength, float robotWidth,
                        float timestep)
    {
ROS_INFO("v=%f,w=%f",v,w);
        double radius = ((double) v) / w;
        double retDist = 0;
        ///////////
//        function distance = GetDistScore(OccupancyGrid, velocity, robotDims, resolution, timestep)
//        x = 0;%m
//                y = 0;%m
//                distanceMax=10;
//
        double safetyBubble = 0.1;// * v;
        double bufferFromCenter = safetyBubble + robotWidth / 2;

        //not moving
        if (std::round(v * 1000) == 0) {
            int yIndex = 0;//std::floor(1 + robotDims(1)/2/resolution);
            int xIndexCenter = 0;//size(OccupancyGrid,2)/2;
            int xIndexLeftCorn = 0;
            int xIndexRightCorn = 0;
            PointForCoord(pGrid, 0/*robotLength / 2*/, 0, yIndex, xIndexCenter);
            PointForCoord(pGrid, 0/*robotLength / 2*/, bufferFromCenter, yIndex, xIndexLeftCorn);
            PointForCoord(pGrid, 0/*robotLength / 2*/, -bufferFromCenter, yIndex, xIndexRightCorn);

//        xIndexRightCorn = ceil(xIndexCenter + bufferFromCenter/resolution);
//        xIndexLeftCorn = floor(xIndexCenter - bufferFromCenter/resolution);
//        if ((OccupancyGrid(yIndex, xIndexCenter) == 0) || (OccupancyGrid(yIndex, xIndexRightCorn) == 0) || (OccupancyGrid(yIndex, xIndexLeftCorn) == 0))
//            distance = 0;
//        else
//            distance = distanceMax;
//        end
odbg(0,0,xIndexCenter,yIndex,pGrid);
odbg(-bufferFromCenter,0,xIndexLeftCorn,0,pGrid);
odbg(bufferFromCenter,0,xIndexRightCorn,0,pGrid);
            if (oGridDataAccessor(pGrid,yIndex, xIndexCenter, 1) > HEIGHT_THRESH ||
                    oGridDataAccessor(pGrid,yIndex, xIndexRightCorn, 1) > HEIGHT_THRESH ||
                    oGridDataAccessor(pGrid,yIndex, xIndexLeftCorn, 1) > HEIGHT_THRESH)
            {
                retDist = 0;
            }
            else
            {
                retDist = DISTANCE_MAX;
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
                    if (yIndex > pGrid->dataDimension[0].size)
                    {
                        retDist = DISTANCE_MAX;// -velocity(1) * timestep;
                        continueChecking=false;
//        end
//                end
//        end
                    }
                }
            }
        } else {
//        else
            double radius = v/w;
//
            //sample at regular intervals along the arc
            double distTravelled = 0.0;
            double ds = pGrid->header.gridResolution/2.0;
            double dTheta = ds/radius;
//
            bool done = false;
            double theta = 0;// + robotLength/2/radius;//Start checking from the front of the robot
//
            double centreX = -radius;
            double centreY = 0;
            while (!done)
            {
                theta = theta + dTheta;
//
                double cosTheta = cos(theta);
                double sinTheta = sin(theta);
//
                double frontCenterX = centreX + radius*cosTheta;
                double frontCenterY = centreY + radius*sinTheta;
                double frontRightX = centreX + (radius + bufferFromCenter)*cosTheta;
                double frontRightY = centreY + (radius + bufferFromCenter)*sinTheta;
                double frontLeftX = centreX + (radius - bufferFromCenter)*cosTheta;
                double frontLeftY = centreY + (radius - bufferFromCenter)*sinTheta;
//
                distTravelled += ds;
//
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
                if ((centerYIndex > pGrid->dataDimension[0].size) ||
                        (rightYIndex > pGrid->dataDimension[0].size) ||
                                (leftYIndex > pGrid->dataDimension[0].size))
                {
                    done = true;
                    retDist = DISTANCE_MAX;
                }
                else if ((leftXIndex < 0) ||
                        rightXIndex < 0 ||
                        (rightXIndex > pGrid->dataDimension[1].size) ||
                        (leftXIndex > pGrid->dataDimension[1].size) ||
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
//                end
} 
           }
//

        }
//        end
//
//
//                end
//

        /////////////


ROS_INFO("Traveled %f",retDist);
        return retDist;
    }

}
