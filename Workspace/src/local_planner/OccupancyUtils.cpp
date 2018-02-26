//
// Created by tom on 24/02/18.
//
#include "OccupancyUtils.h"

namespace OccupancyUtils {


//accessor for Occupancy Grid Message
    float oGridDataAccessor (occupancy_grid::OccupancyGrid::ConstPtr oGrid, unsigned int i, unsigned int j, unsigned int  k) {
        return oGrid->data[i*oGrid->dataDimension[0].stride + j*oGrid->dataDimension[1].stride + k*oGrid->dataDimension[2].stride];
    }

    void PointForCoord(occupancy_grid::OccupancyGrid::ConstPtr& pGrid, double z, double x, int& zOut, int& xOut)
    {
        float coordZ = (z)/pGrid->header.gridResolution;
        float coordX = (x)/pGrid->header.gridResolution + (pGrid->dataDimension[0].stride/2.0);
        zOut = (int)(coordZ + 0.5);
        xOut = (int)(coordX + 0.5);
    }

    double CalcDistance(occupancy_grid::OccupancyGrid::ConstPtr& pGrid,
                        float v, float w,
                        float robotLength, float robotWidth,
                        float timestep)
    {
        double radius = ((double) v) / w;
        double retDist = 0;
        ///////////
#define DISTANCE_MAX 20;
//        function distance = GetDistScore(OccupancyGrid, velocity, robotDims, resolution, timestep)
//        x = 0;%m
//                y = 0;%m
//                distanceMax=10;
//
        double safetyBubble = 0.3 * v;
        double bufferFromCenter = safetyBubble + robotWidth / 2;

        //not moving
        if (std::round(v * 1000) == 0) {
            int yIndex = 0;//std::floor(1 + robotDims(1)/2/resolution);
            int xIndexCenter = 0;//size(OccupancyGrid,2)/2;
            int xIndexLeftCorn = 0;
            int xIndexRightCorn = 0;
            PointForCoord(pGrid, robotLength / 2, 0, yIndex, xIndexCenter);
            PointForCoord(pGrid, robotLength / 2, bufferFromCenter, yIndex, xIndexLeftCorn);
            PointForCoord(pGrid, robotLength / 2, -bufferFromCenter, yIndex, xIndexRightCorn);

//        xIndexRightCorn = ceil(xIndexCenter + bufferFromCenter/resolution);
//        xIndexLeftCorn = floor(xIndexCenter - bufferFromCenter/resolution);
//        if ((OccupancyGrid(yIndex, xIndexCenter) == 0) || (OccupancyGrid(yIndex, xIndexRightCorn) == 0) || (OccupancyGrid(yIndex, xIndexLeftCorn) == 0))
//            distance = 0;
//        else
//            distance = distanceMax;
//        end
            if (oGridDataAccessor(pGrid,yIndex, xIndexCenter, 0) != 0 ||
                    oGridDataAccessor(pGrid,yIndex, xIndexRightCorn, 0) != 0 ||
                    oGridDataAccessor(pGrid,yIndex, xIndexLeftCorn, 0) != 0)
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
            PointForCoord(pGrid, robotLength / 2, 0, yIndex, xIndexCenter);
            PointForCoord(pGrid, robotLength / 2, bufferFromCenter, yIndex, xIndexLeftCorn);
            PointForCoord(pGrid, robotLength / 2, -bufferFromCenter, yIndex, xIndexRightCorn);
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
                if (oGridDataAccessor(pGrid,yIndex, xIndexCenter, 0) != 0 ||
                    oGridDataAccessor(pGrid,yIndex, xIndexRightCorn, 0) != 0 ||
                    oGridDataAccessor(pGrid,yIndex, xIndexLeftCorn, 0) != 0)
                {
                    continueChecking = false;
                    retDist = yIndex * pGrid->header.gridResolution - timestep * v;
                }
                else
                {
                    yIndex = yIndex + 1;
                    if (yIndex > pGrid->dataDimension[0].size)
                    {
                        distance = DISTANCE_MAX;// -velocity(1) * timestep;
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
            double ds = pGrid->header.gridResolution * 2;
            double dTheta = ds/radius;
//
            bool done = false;
            double theta = 0 + robotLength/2/radius;//Start checking from the front of the robot
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
                else if ((oGridDataAccessor(pGrid,centerYIndex,centerXIndex,0) != 0) ||
                        (oGridDataAccessor(pGrid,rightYIndex,rightXIndex,0) != 0) ||
                        (oGridDataAccessor(pGrid,leftYIndex,leftXIndex,0) != 0))
                {
                    retDist = distTravelled - timestep * v;
                    done = true;
//        end
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



        return retDist;
    }

}
