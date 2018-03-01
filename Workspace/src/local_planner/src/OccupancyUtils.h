//
// Created by tom on 24/02/18.
//

#ifndef PROJECT_OCCUPANCYUTILS_H
#define PROJECT_OCCUPANCYUTILS_H

#include "occupancy_grid/OccupancyGrid.h"

#define DISTANCE_MAX 20.0

namespace OccupancyUtils {

double CalcDistance(occupancy_grid::OccupancyGrid::ConstPtr& pGrid,
                    float v, float w,
                    float robotLength, float robotWidth,
                    float timestep);
void PointForCoord(occupancy_grid::OccupancyGrid::ConstPtr& pGrid,
                   double z, double x, int& zOut, int& xOut);



}
#endif //PROJECT_OCCUPANCYUTILS_H
