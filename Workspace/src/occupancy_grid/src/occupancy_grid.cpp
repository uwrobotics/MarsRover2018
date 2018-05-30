/*
NOTE:
Duo Pointcloud2 data follows (z = forward, x = left, y = down)

Optimized for ~1500 points detected per grid square
With median filter of size 5, resolution 0.05, frequency should be around 5Hz,
reduce filter size or remove the weighted average if need more performance
*/

#include "ros/ros.h"
#include <math.h>
#include <ros/console.h>

// std and ext. lib msg types
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// custom msg types
#include "occupancy_grid/GridDataDimension.h"
#include "occupancy_grid/OccupancyGrid.h"
#include "occupancy_grid/OccupancyGridHeader.h"

// accessor for Occupancy Grid Message
float &oGridDataAccessor(occupancy_grid::OccupancyGrid &oGrid, unsigned int i,
                         unsigned int j, unsigned int k) {
  return oGrid.data[i * oGrid.dataDimension[0].stride +
                    j * oGrid.dataDimension[1].stride +
                    k * oGrid.dataDimension[2].stride];
}

// resolution is in terms of side length of a grid cell
// z, x max is the max range of the camera (in one direction)
typedef struct {
  float zMax;
  float xMax;
  float yOffset;
  float brightness_lower_lim;
  float brightness_upper_lim;
  float resolution;
  float expo_weighted_avg_var;
  int num_points_thres_ratio;
  int median_filter_size;
  int rate;
  int queue_size;
  float mappingScalar;
  float mappingNormalizer;
} gridParams;

class OccupancyGrid {
public:
  class Kernel {
  public:
    Kernel() {}

    void setSize(int size) {
      std::vector<float> temp;
      temp.reserve(size * size);
      data = std::move(temp);
      m_size = size;
    }

    unsigned int size() { return m_size; }

    float *operator[](unsigned int i) { return &data[i * m_size]; }

    void push_back(float f) { data.push_back(f); }

  private:
    std::vector<float> data;
    unsigned int m_size;
  };

  OccupancyGrid() {
    // get params from yaml file
    ROS_ASSERT(ros::param::get("zMax", m_gridParams.zMax));
    ROS_ASSERT(ros::param::get("xMax", m_gridParams.xMax));
    ROS_ASSERT(ros::param::get("yOffset", m_gridParams.yOffset));
    ROS_ASSERT(ros::param::get("brightness_lower_lim",
                               m_gridParams.brightness_lower_lim));
    ROS_ASSERT(ros::param::get("brightness_upper_lim",
                               m_gridParams.brightness_upper_lim));
    ROS_ASSERT(ros::param::get("resolution", m_gridParams.resolution));
    ROS_ASSERT(ros::param::get("num_points_thres_ratio",
                               m_gridParams.num_points_thres_ratio));
    ROS_ASSERT(ros::param::get("expo_weighted_avg_var",
                               m_gridParams.expo_weighted_avg_var));
    ROS_ASSERT(
        ros::param::get("median_filter_size", m_gridParams.median_filter_size));
    ROS_ASSERT(ros::param::get("rate", m_gridParams.rate));
    ROS_ASSERT(ros::param::get("queue_size", m_gridParams.queue_size));
    ROS_ASSERT(ros::param::get("logging", m_log));
    ROS_ASSERT(ros::param::get("Scalar", m_gridParams.mappingScalar));
    ROS_ASSERT(ros::param::get("Normalizer", m_gridParams.mappingNormalizer));

    std::string pcl2TopicName;
    ROS_ASSERT(ros::param::get("PCL2TopicName", pcl2TopicName));

    std::vector<float> param;
    ROS_ASSERT(ros::param::get("gaussian_blur_kernel", param));
    m_gaussian_blur_kernel.setSize(sqrt(param.size()));
    for (int i = 0;
         i < m_gaussian_blur_kernel.size() * m_gaussian_blur_kernel.size();
         i++) {
      m_gaussian_blur_kernel.push_back(param[i]);
    }

    ROS_ASSERT(ros::param::get("gaussian_hor_kernel", param));
    m_gaussian_hor_kernel.setSize(sqrt(param.size()));
    for (int i = 0;
         i < m_gaussian_hor_kernel.size() * m_gaussian_hor_kernel.size(); i++) {
      m_gaussian_hor_kernel.push_back(param[i]);
    }
    ROS_ASSERT(ros::param::get("gaussian_ver_kernel", param));
    m_gaussian_ver_kernel.setSize(sqrt(param.size()));
    for (int i = 0;
         i < m_gaussian_ver_kernel.size() * m_gaussian_ver_kernel.size(); i++) {
      m_gaussian_ver_kernel.push_back(param[i]);
    }
    ROS_ASSERT(ros::param::get("gaussian_hor_norm_kernel", param));
    m_gaussian_hor_norm_kernel.setSize(sqrt(param.size()));
    for (int i = 0; i < m_gaussian_hor_norm_kernel.size() *
                            m_gaussian_hor_norm_kernel.size();
         i++) {
      m_gaussian_hor_norm_kernel.push_back(param[i]);
    }
    ROS_ASSERT(ros::param::get("gaussian_ver_norm_kernel", param));
    m_gaussian_ver_norm_kernel.setSize(sqrt(param.size()));
    for (int i = 0; i < m_gaussian_ver_norm_kernel.size() *
                            m_gaussian_ver_norm_kernel.size();
         i++) {
      m_gaussian_ver_norm_kernel.push_back(param[i]);
    }

    // set grid params
    m_gridZSize = m_gridParams.zMax / m_gridParams.resolution + 1;
    m_gridXSize = m_gridParams.xMax * 2 / m_gridParams.resolution + 1;

    // make sure camera is in centre of a cell
    if (m_gridXSize % 2 == 0) {
      m_gridXSize++;
    }
    m_gridCameraZ = 0;
    m_gridCameraX = m_gridXSize / 2;

    // sub & pub
    m_sub = m_n.subscribe(pcl2TopicName, m_gridParams.queue_size,
                          &OccupancyGrid::callback, this);
    m_pub = m_n.advertise<occupancy_grid::OccupancyGrid>(
        "/OccupancyGrid", m_gridParams.queue_size);

    // Since rviz will flash back and forth between multiple messages published
    // by the same topic, using an array of topics
    m_pub_rviz[0] = m_n.advertise<nav_msgs::OccupancyGrid>(
        "/OccupancyGridCellsPointsDetected", m_gridParams.queue_size);
    m_pub_rviz[1] = m_n.advertise<nav_msgs::OccupancyGrid>(
        "/OccupancyGridCellsAvg", m_gridParams.queue_size);
    m_pub_rviz[2] = m_n.advertise<nav_msgs::OccupancyGrid>(
        "/OccupancyGridCellsBlur", m_gridParams.queue_size);
    m_pub_rviz[3] = m_n.advertise<nav_msgs::OccupancyGrid>(
        "/OccupancyGridCellsBlurSlope", m_gridParams.queue_size);

    /*
    m_pub_rviz[0] =
    m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsPointsDetected",
                                                           m_gridParams.queue_size);
    m_pub_rviz[1] =
    m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsAvg",
    m_gridParams.queue_size);
    m_pub_rviz[2] =
    m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsMax",
    m_gridParams.queue_size);
    m_pub_rviz[3] =
    m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsMin",
    m_gridParams.queue_size);
    m_pub_rviz[4] =
    m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsBlur",
    m_gridParams.queue_size);
    m_pub_rviz[5] =
    m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsBlurSlope",
    m_gridParams.queue_size);
    m_pub_rviz[6] =
    m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsBlurSlopeNorm",
                                                           m_gridParams.queue_size);
    */

    medium_filter_counter = 0;
    //occupancy_grid::OccupancyGrid temp;
    m_outputGrid.header.cameraZMax = m_gridParams.zMax;
    m_outputGrid.header.cameraXMax = m_gridParams.xMax;
    m_outputGrid.header.gridResolution = m_gridParams.resolution;
    m_outputGrid.header.gridCameraZ = m_gridCameraZ;
    m_outputGrid.header.gridCameraX = m_gridCameraX;
    m_outputGrid.header.cameraYOffset = m_gridParams.yOffset;

    m_outputGrid.dataDimension.emplace_back(
        std::move(occupancy_grid::GridDataDimension()));
    m_outputGrid.dataDimension.emplace_back(
        std::move(occupancy_grid::GridDataDimension()));
    m_outputGrid.dataDimension.emplace_back(
        std::move(occupancy_grid::GridDataDimension()));

    m_outputGrid.dataDimension[0].label = "Z(Forward)";
    m_outputGrid.dataDimension[1].label = "X(Left)";
    m_outputGrid.dataDimension[2].label = "Points Detected // Avg. Height // Gaussian "
                                  "Blur // Gaussian Blur * Slope";
    //"Points Detected // Avg. Height // Max Height // Min Height // Gaussian
    //Blur // Gaussian Blur * Slope // Gaussian Blur * Slope (Normalized)";

    m_outputGrid.dataDimension[0].size = m_gridZSize;
    m_outputGrid.dataDimension[1].size = m_gridXSize;
    m_outputGrid.dataDimension[2].size = 4; // 7;

    m_outputGrid.dataDimension[0].stride =
        m_outputGrid.dataDimension[1].size * m_outputGrid.dataDimension[2].size;
    m_outputGrid.dataDimension[1].stride = m_outputGrid.dataDimension[2].size;
    m_outputGrid.dataDimension[2].stride = 1;

    m_outputGrid.data.resize(m_outputGrid.dataDimension[0].size * m_outputGrid.dataDimension[1].size *
                         m_outputGrid.dataDimension[2].size,
                     0);

    //output.resize(m_gridParams.median_filter_size + 1, temp);


    m_pMedianResult = PopulateNewGrid();
  }

  void callback(const sensor_msgs::PointCloud2 input);

  gridParams getGridParams() const { return m_gridParams; }

private:
  std::shared_ptr<occupancy_grid::OccupancyGrid> PopulateNewGrid();
  ros::NodeHandle m_n;
  ros::Subscriber m_sub;
  ros::Publisher m_pub;
  ros::Publisher m_pub_rviz[4]; //[7];

  int m_gridZSize;
  int m_gridXSize;
  int m_gridCameraZ;
  int m_gridCameraX;

  bool m_log;

  gridParams m_gridParams;

  Kernel m_gaussian_blur_kernel;
  Kernel m_gaussian_ver_kernel;
  Kernel m_gaussian_hor_kernel;
  Kernel m_gaussian_ver_norm_kernel;
  Kernel m_gaussian_hor_norm_kernel;

  int medium_filter_counter;
  // last grid (output[5]) is the actual one to be published, the other ones are
  // for median filter
  //std::vector<occupancy_grid::OccupancyGrid> output;
  std::shared_ptr<occupancy_grid::OccupancyGrid> m_pMedianResult;
  occupancy_grid::OccupancyGrid m_outputGrid;



  std::list<std::shared_ptr<occupancy_grid::OccupancyGrid>> m_medianList;
};





// function to populate new frame
std::shared_ptr<occupancy_grid::OccupancyGrid> OccupancyGrid::PopulateNewGrid()
{
  std::shared_ptr<occupancy_grid::OccupancyGrid> pGrid = std::make_shared<occupancy_grid::OccupancyGrid>();
  pGrid->header.cameraZMax = m_gridParams.zMax;
  pGrid->header.cameraXMax = m_gridParams.xMax;
  pGrid->header.gridResolution = m_gridParams.resolution;
  pGrid->header.gridCameraZ = m_gridCameraZ;
  pGrid->header.gridCameraX = m_gridCameraX;
  pGrid->header.cameraYOffset = m_gridParams.yOffset;

  pGrid->dataDimension.emplace_back(
      std::move(occupancy_grid::GridDataDimension()));
  pGrid->dataDimension.emplace_back(
      std::move(occupancy_grid::GridDataDimension()));
  pGrid->dataDimension.emplace_back(
      std::move(occupancy_grid::GridDataDimension()));

  pGrid->dataDimension[0].label = "Z(Forward)";
  pGrid->dataDimension[1].label = "X(Left)";
  pGrid->dataDimension[2].label = "Points Detected // Avg. Height // Gaussian "
                                "Blur // Gaussian Blur * Slope";
  //"Points Detected // Avg. Height // Max Height // Min Height // Gaussian
  //Blur // Gaussian Blur * Slope // Gaussian Blur * Slope (Normalized)";

  pGrid->dataDimension[0].size = m_gridZSize;
  pGrid->dataDimension[1].size = m_gridXSize;
  pGrid->dataDimension[2].size = 4; // 7;

  pGrid->dataDimension[0].stride =
      pGrid->dataDimension[1].size * pGrid->dataDimension[2].size;
  pGrid->dataDimension[1].stride = pGrid->dataDimension[2].size;
  pGrid->dataDimension[2].stride = 1;

  pGrid->data.resize(pGrid->dataDimension[0].size * pGrid->dataDimension[1].size *
                   pGrid->dataDimension[2].size,
                   0);
  return pGrid;
}





/*
 * 1) populate new grid (only average, numpoints)
 * 2) median last 5 for average, points
 * 3) exp weight filter
 * 4) gauss the results
 *
 *
 *
 */
void OccupancyGrid::callback(const sensor_msgs::PointCloud2 input) {

  ROS_INFO_STREAM_COND(m_log, std::endl << "New Frame Detected" << std::endl);
  ROS_DEBUG_STREAM_COND(m_log, std::endl
                                   << "Input Data & Conversion" << std::endl);



  // 1) populate a new grid
  std::shared_ptr<occupancy_grid::OccupancyGrid> pNewGrid = PopulateNewGrid();

  // get xyz and rgb data from the pointcloud message and store in a oGridPoints
  // array
  std::vector<std::vector<float>> oGridPoints;
  oGridPoints.resize(m_gridZSize * m_gridXSize, std::vector<float>());

  sensor_msgs::PointCloud2ConstIterator<float> iterX(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iterY(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iterZ(input, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iterRGB(input, "rgb");

  for (; iterZ != iterZ.end(); ++iterX, ++iterY, ++iterZ, ++iterRGB) {
    float height = (-1 * (*iterY) + m_gridParams.yOffset);
    uint8_t r = iterRGB[0], g = iterRGB[1], b = iterRGB[2];
    float brightness = 0.2126 * r + 0.7152 * g + 0.0722 * b;
    int convZ = (*iterZ) / m_gridParams.resolution;
    int convX = 1 * (*iterX) / m_gridParams.resolution + (m_gridXSize / 2.0);

    /* ROS_DEBUG_STREAM_COND(m_log, std::fixed << std::setprecision(3) << "Z: "
       << *iterZ << "\tX: " << *iterX << "\tY: "
                                     << *iterY << "\tconvZ: " << convZ <<
       "\tconvX: " << convX << "\tHeight: "
                                     << height << std::endl);*/

    // invalid bounds error check, and remove points too black or two white
    // since their depth is not accurate
    if (convZ < m_gridZSize && convX < m_gridXSize && convZ >= 0 &&
        convX >= 0 && brightness > m_gridParams.brightness_lower_lim &&
        brightness < m_gridParams.brightness_upper_lim) {
      oGridPoints[convZ * m_gridXSize + convX].reserve(1500);
      oGridPoints[convZ * m_gridXSize + convX].emplace_back(height);
    } else {
      // ROS_DEBUG_STREAM_COND(m_log, "Point Detected Out of Occupancy Grid
      // Limits" << std::endl);
    }
  }


  // fill in the data to the output occupancy grid message with the oGridPoints
  // array, without weighted average
  for (int z = 0; z < m_gridZSize; z++) {
    for (int x = 0; x < m_gridXSize; x++) {
      if (oGridPoints[z * m_gridXSize + x].size() >
          m_gridParams.num_points_thres_ratio * m_gridParams.resolution ||
          oGridPoints[z * m_gridXSize + x].size() == 0) {

        //TODO: is this still needed??
        std::sort(oGridPoints[z * m_gridXSize + x].begin(),
                  oGridPoints[z * m_gridXSize + x].end(),
                  std::greater<float>());

        // point count
        //oGridDataAccessor(output[medium_filter_counter], z, x, 0) =
        oGridDataAccessor(*pNewGrid, z, x, 0) =
            oGridPoints[z * m_gridXSize + x].size();

        // if no points detected, set number of points to high value to avoid
        // divide by 0 error (while the sum is 0 if the number of points is zero
        // for sure
        float sum = 0;
        float size = oGridPoints[z * m_gridXSize + x].size();
        if (size == 0)
          size = 1;

        // avg height
        for (float a : oGridPoints[z * m_gridXSize + x]) {
          sum += a;
        }

        //oGridDataAccessor(output[medium_filter_counter], z, x, 1) = sum / size;
        oGridDataAccessor(*pNewGrid, z, x, 1) = sum / size;

        /*
        //max height
        sum = 0;
        for (int i = 0; i < oGridPoints[z * m_gridXSize + x].size() * 0.05; i++)
        {
        sum += oGridPoints[z * m_gridXSize + x][i];
        }
        oGridDataAccessor(output, z, x, 2) =
                (m_gridParams.expo_weighted_avg_var * oGridDataAccessor(output,
        z, x, 2) +
                 sum / (unsigned int) (size * 0.05 + 1) * (1 -
        m_gridParams.expo_weighted_avg_var));

        //min height
        sum = 0;
        for (int i = 0, index = oGridPoints[z * m_gridXSize + x].size() - 1;
        i < oGridPoints[z * m_gridXSize + x].size() * 0.05; i++, index--) {
        sum += oGridPoints[z * m_gridXSize + x][index];
        }
        oGridDataAccessor(output, z, x, 3) =
                (m_gridParams.expo_weighted_avg_var * oGridDataAccessor(output,
        z, x, 3) +
                 sum / (unsigned int) (size * 0.05 + 1)* (1 -
        m_gridParams.expo_weighted_avg_var) );
        */
      }
    }
  }




  // 2) Add to the median queue and do the median filter

  if (m_medianList.size() < m_gridParams.median_filter_size)
  {
    m_medianList.push_front(pNewGrid);
    return;
  }


  m_medianList.pop_back();
  m_medianList.push_front(pNewGrid);





  //medium_filter_counter++;
  // the median filter
  if (m_medianList.size() == m_gridParams.median_filter_size) {
    //medium_filter_counter = 0;

    // do the median on the last 5 values
    for (int z = 0; z < m_gridZSize; z++) {
      for (int x = 0; x < m_gridXSize; x++) {
        for (int channel = 0; channel < pNewGrid->dataDimension[2].size;
             channel++) {
          float median_val = 0;
          std::vector<float> median_val_array;
          median_val_array.reserve(m_medianList.size());
          //for (int i = 0; i < m_gridParams.median_filter_size; i++) {
          for (auto& grid : m_medianList) {
            median_val_array.push_back(
                oGridDataAccessor(*grid, z, x, channel));
          }

          // find the median and populate the grid that will be published
          // if (i == m_gridParams.median_filter_size - 1) {
          std::sort(median_val_array.begin(), median_val_array.end(),
                    std::greater<float>());
          median_val =
              median_val_array.at(m_gridParams.median_filter_size / 2);// + 1);

//          oGridDataAccessor(output[m_gridParams.median_filter_size], z, x,
//                            channel) =
//              (m_gridParams.expo_weighted_avg_var *
//                   oGridDataAccessor(
//                       output[m_gridParams.median_filter_size], z, x,
//                       channel) +
//               median_val * (1 - m_gridParams.expo_weighted_avg_var));

          oGridDataAccessor(*m_pMedianResult, z, x,
                            channel) =
              (m_gridParams.expo_weighted_avg_var *
               oGridDataAccessor(
                   *m_pMedianResult, z, x,
                   channel) +
               median_val * (1 - m_gridParams.expo_weighted_avg_var));


              // change to this if necessary, slight improvement
              // oGridDataAccessor(output[m_gridParams.median_filter_size],z,x,channel)
              // = median_val;
           // }
          //}
        }
      }
    }

  }



  // 4) Blur the results
  for (int z = 0; z < m_gridZSize; z++) {
    for (int x = 0; x < m_gridXSize; x++) {
      int zExtended = 0;
      int xExtended = 0;
      float weightedSum[3]{0}; //[5]{0}
      // ROS_ERROR_STREAM(z<< " "<<x<<std::endl);
      // ASSUMES ALL KERNELS ARE SAME SIZE (5)
      for (int row = 0; row < m_gaussian_blur_kernel.size(); row++) {
        zExtended = z - m_gaussian_blur_kernel.size() / 2 + row;
        if (zExtended < 0) {
          zExtended = 0;
        } else if (zExtended >= m_gridZSize) {
          zExtended = m_gridZSize - 1;
        }
        for (int col = 0; col < m_gaussian_blur_kernel.size(); col++) {
          xExtended = x - m_gaussian_blur_kernel.size() / 2 + col;
          if (xExtended < 0) {
            xExtended = 0;
          } else if (xExtended >= m_gridXSize) {
            xExtended = m_gridXSize - 1;
          }

          weightedSum[0] += oGridDataAccessor(*m_pMedianResult,
                                              zExtended, xExtended, 1) *
                            m_gaussian_blur_kernel[row][col];
          weightedSum[1] += oGridDataAccessor(*m_pMedianResult,
                                              zExtended, xExtended, 1) *
                            m_gaussian_hor_kernel[row][col];
          weightedSum[2] += oGridDataAccessor(*m_pMedianResult,
                                              zExtended, xExtended, 1) *
                            m_gaussian_ver_kernel[row][col];

//          weightedSum[0] += oGridDataAccessor(output[medium_filter_counter],
//                                              zExtended, xExtended, 1) *
//                            m_gaussian_blur_kernel[row][col];
//          weightedSum[1] += oGridDataAccessor(output[medium_filter_counter],
//                                              zExtended, xExtended, 1) *
//                            m_gaussian_hor_kernel[row][col];
//          weightedSum[2] += oGridDataAccessor(output[medium_filter_counter],
//                                              zExtended, xExtended, 1) *
//                            m_gaussian_ver_kernel[row][col];
          /*
           weightedSum[3] +=
                   oGridDataAccessor(output, zExtended, xExtended, 1) *
           m_gaussian_hor_norm_kernel[row][col];
           weightedSum[4] +=
                   oGridDataAccessor(output, zExtended, xExtended, 1) *
           m_gaussian_ver_norm_kernel[row][col];
          */
          // ROS_ERROR_STREAM(row<<" "<<col<<" "<<
          // m_gaussian_blur_kernel[row][col]<<" "<<(output, zExtended,
          // xExtended, 1) << " " <<weightedSum[0]<<std::endl);
        }
      }
      // ROS_ERROR_STREAM(weightedSum[0]<<std::endl);
      // gaussian blur
//      oGridDataAccessor(output[medium_filter_counter], z, x, 2) =
//          weightedSum[0];
//      // slope + gaussian blur
//      oGridDataAccessor(output[medium_filter_counter], z, x, 3) = sqrt(
//          weightedSum[1] * weightedSum[1] + weightedSum[2] * weightedSum[2]);

      // gaussian blur
      oGridDataAccessor(m_outputGrid, z, x, 2) =
          weightedSum[0];
      // slope + gaussian blur
      oGridDataAccessor(m_outputGrid, z, x, 3) = sqrt(
          weightedSum[1] * weightedSum[1] + weightedSum[2] * weightedSum[2]);


      //fill in other info
      oGridDataAccessor(m_outputGrid, z, x, 0) = oGridDataAccessor(*m_pMedianResult, z,x, 0);
      oGridDataAccessor(m_outputGrid, z, x, 1) = oGridDataAccessor(*m_pMedianResult, z,x, 1);

      // normalized slope + gaussian blur
      // oGridDataAccessor(output, z, x, 4) = sqrt(
      //        weightedSum[3] * weightedSum[3] + weightedSum[4] *
      //        weightedSum[4]);
    }
  }


  //m_pub.publish(output[m_gridParams.median_filter_size]);
  m_pub.publish(m_outputGrid);

//  for (int i = 0; i < m_gridParams.median_filter_size; i++) {
//    output[i].data.resize(0, 0);
//    output[i].data.resize(output[i].dataDimension[0].size *
//                          output[i].dataDimension[1].size *
//                          output[i].dataDimension[2].size,
//                          0);
//  }

  // m_pub.publish(output);

  // ouput to rviz to visualize, publishes 7 messages, each corresponds to one
  // element of the third dimension of the occupancy grid message(ie. point
  // count, avg, max, min heights)
//  for (int i = 0; i < output[0].dataDimension[2].size; i++) {
  for (int i = 0; i < m_outputGrid.dataDimension[2].size; i++) {
    nav_msgs::OccupancyGrid gridcells;
    gridcells.header.frame_id = "/base_link";
    gridcells.header.stamp = ros::Time::now();
    gridcells.info.resolution = 1.0;
    gridcells.info.width = m_gridXSize;
    gridcells.info.height = m_gridZSize;
    // set the view to topdownortho in rviz, the display will be in the correct
    // orientation. Bottom left: point count, bottom right: avg, top left: max,
    // top right: min
    gridcells.info.origin.position.x = i % 2 * m_gridXSize + i % 2 * 10.0;
    gridcells.info.origin.position.y = i / 2 * m_gridZSize + i / 2 * 10.0;
    gridcells.info.origin.position.z = 0.0;
    gridcells.info.origin.orientation.x = 0.0;
    gridcells.info.origin.orientation.y = 0.0;
    gridcells.info.origin.orientation.z = 0.0;
    gridcells.info.origin.orientation.w = 1.0;

    gridcells.data.resize(m_gridXSize * m_gridZSize, 0.0);


    // map the occupancy grid values to standard 0-100 value for display
    for (int x = 0; x < m_gridXSize; x++) {
      for (int z = 0; z < m_gridZSize; z++) {

        float cost =
//            oGridDataAccessor(output[m_gridParams.median_filter_size], z, x, i);
            oGridDataAccessor(m_outputGrid, z, x, i);
        // ROS_ERROR_STREAM ( "1normalizer: "<<
        // (float)m_gridParams.mappingNormalizer << " scalar:
        // "<<m_gridParams.mappingScalar << " mpas to:"<<
        // (cost/m_gridParams.mappingNormalizer/2.0*100*m_gridParams.mappingScalar)
        // );

        // since number of points is typically very large
        if (i == 0) {
          if (cost / m_gridParams.num_points_thres_ratio *
                  m_gridParams.resolution >
              100)
            cost = 100;
          else if (cost > 100)
            cost /=
                (m_gridParams.num_points_thres_ratio * m_gridParams.resolution);

          gridcells.data[z * m_gridXSize + x] = int(cost);
        } else {
          if (cost <= 0)
            cost = 0;
          else {
            cost = int(cost / m_gridParams.mappingNormalizer * 100 *
                       m_gridParams.mappingScalar);
            if (cost >= 100)
              cost = 100;
            gridcells.data[z * m_gridXSize + x] = cost;
          }
        }
      }
    }
    m_pub_rviz[i].publish(gridcells);
  }

  if (m_log) {
    std::stringstream debugString;
    debugString << std::fixed << std::setprecision(3);

    debugString << std::endl << "Points Detected" << std::endl;
    for (int z = m_gridZSize - 1; z >= 0; z--) {
      for (int x = 0; x < m_gridXSize; x++) {
        debugString << std::setw(8)
                    << (unsigned int)oGridDataAccessor(
//                           output[medium_filter_counter], z, x, 0);
                           m_outputGrid, z, x, 0);
      }
      debugString << std::endl;
    }
    ROS_DEBUG_STREAM(debugString.str());

    debugString.str("");
    debugString << std::endl << "Average Height" << std::endl;
    for (int z = m_gridZSize - 1; z >= 0; z--) {
      for (int x = 0; x < m_gridXSize; x++) {
//        debugString << oGridDataAccessor(output[medium_filter_counter], z, x, 1)
        debugString << oGridDataAccessor(m_outputGrid, z, x, 1)
                    << "\t";
      }
      debugString << std::endl;
    }
    ROS_DEBUG_STREAM(debugString.str());

    /*
    debugString.str("");
    debugString << std::endl << "Average Max Height (Highest 5%)" << std::endl;
    for (int z = m_gridZSize - 1; z >= 0; z--) {
        for (int x = 0; x < m_gridXSize; x++) {
            debugString << oGridDataAccessor(output[medium_filter_counter], z,
    x, 2) << "\t";
        }
        debugString << std::endl;
    }
    ROS_DEBUG_STREAM(debugString.str());

    debugString.str("");
    debugString << std::endl << "Average Min Height (Lowest 5%)" << std::endl;
    for (int z = m_gridZSize - 1; z >= 0; z--) {
        for (int x = 0; x < m_gridXSize; x++) {
            debugString << oGridDataAccessor(output, z, x, 3) << "\t";
        }
        debugString << std::endl;
    }
    ROS_DEBUG_STREAM(debugString.str());
    */

    debugString.str("");
    debugString << std::endl << "Blurred Avg" << std::endl;
    for (int z = m_gridZSize - 1; z >= 0; z--) {
      for (int x = 0; x < m_gridXSize; x++) {
//        debugString << oGridDataAccessor(output[medium_filter_counter], z, x, 2)
        debugString << oGridDataAccessor(m_outputGrid, z, x, 2)
                    << "\t";
      }
      debugString << std::endl;
    }
    ROS_DEBUG_STREAM(debugString.str());

    debugString.str("");
    debugString << std::endl << "Slopes" << std::endl;
    for (int z = m_gridZSize - 1; z >= 0; z--) {
      for (int x = 0; x < m_gridXSize; x++) {
//        debugString << oGridDataAccessor(output[medium_filter_counter], z, x, 3)
        debugString << oGridDataAccessor(m_outputGrid, z, x, 3)
                    << "\t";
      }
      debugString << std::endl;
    }
    ROS_DEBUG_STREAM(debugString.str());
    /*
    debugString.str("");
    debugString << std::endl << "Norm Slopes" << std::endl;
    for (int z = m_gridZSize - 1; z >= 0; z--) {
        for (int x = 0; x < m_gridXSize; x++) {
            debugString << oGridDataAccessor(output, z, x, 6) << "\t";
        }
        debugString << std::endl;
    }
    ROS_DEBUG_STREAM(debugString.str());
    */
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "OccupancyGrid");

  OccupancyGrid o;

  ros::Rate rate(o.getGridParams().rate);

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  while (ros::ok()) {
    ros::spinOnce();
    ros::Rate(rate).sleep();
  }
}
