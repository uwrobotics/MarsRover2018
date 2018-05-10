//
// Created by tom on 24/02/18.
//

#ifndef PROJECT_OCCUPANCYUTILS_H
#define PROJECT_OCCUPANCYUTILS_H

#include "occupancy_grid/OccupancyGrid.h"

#define DISTANCE_INF 20.0

class OccupancyUtils {
public:
  OccupancyUtils(occupancy_grid::OccupancyGrid::ConstPtr& pGrid, float robotLength, float robotWidth,
                 float timestep);
  double CalcDistance(float v, float w, bool &foundDanger);

private:
  typedef enum {
    NOT_TRAVERSABLE = 0,
    TRAVERSABLE,
    BEYOND_GRID,
    INVALID_POINT
  } eTraversableResult;

  void PointForCoord(double z, double x, int &zOut, int &xOut);
  eTraversableResult IsPointTraversable(double y, double x);
  float GridDataAccessor(unsigned int i, unsigned int j, unsigned int k);


  occupancy_grid::OccupancyGrid::ConstPtr m_pGrid;
  double m_robotWidth, m_robotLength;
  double m_timestep;
  double m_maxW;


};
#endif // PROJECT_OCCUPANCYUTILS_H
