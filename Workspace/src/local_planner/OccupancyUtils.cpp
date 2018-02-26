//
// Created by tom on 24/02/18.
//
#include "OccupancyUtils.h"

namespace OccupancyUtils {
    double CalcDistance(occupancy_grid::OccupancyGrid::ConstPtr& pGrid,
                        float v, float w,
                        float robotLength, float robotWidth)
    {

        return 0;
    }

    void PointForCoord(occupancy_grid::OccupancyGrid::ConstPtr& pGrid, double z, double x, int& zOut, int& xOut)
    {
        float coordZ = (z)/pGrid->header.gridResolution;
        float coordX = (x)/pGrid->header.gridResolution + (pGrid->dataDimension[0].stride/2.0);
        zOut = (int)(coordZ + 0.5);
        xOut = (int)(coordX + 0.5);
    }


}
