/*
NOTE:
Duo Pointcloud2 data follows (z = forward, x = left, y = down)

Optimized for ~1500 points detected per grid square
*/

#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

//std and ext. lib msg types
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>

//custom msg types
#include "occupancy_grid/OccupancyGrid.h"
#include "occupancy_grid/OccupancyGridHeader.h"
#include "occupancy_grid/GridDataDimension.h"

//accessor for Occupancy Grid Message
float &oGridDataAccessor(occupancy_grid::OccupancyGrid &oGrid, unsigned int i, unsigned int j, unsigned int k) {
    return oGrid.data[i * oGrid.dataDimension[0].stride + j * oGrid.dataDimension[1].stride +
                      k * oGrid.dataDimension[2].stride];
}

//resolution is in terms of side length of a grid cell
//z, x max is the max range of the camera (in one direction)
typedef struct {
    float zMax;
    float xMax;
    float yOffset;
    std::vector<float>rgb_lower_lim;
    std::vector<float>rgb_upper_lim;
    float resolution;
    int rate;
    int queue_size;
    float mappingScalar;
    float mappingNormalizer;
} gridParams;


class OccupancyGrid {
public:
    class Kernel {
    public:
        Kernel() {
        }

        void setSize(int size) {
            std::vector<float> temp;
            temp.reserve(size * size);
            data = std::move(temp);
            m_size = size;
        }

        unsigned int size() {
            return m_size;
        }

        float *operator[](unsigned int i) {
            return &data[i * m_size];
        }

        void push_back(float f) {
            data.push_back(f);
        }

    private:
        std::vector<float> data;
        unsigned int m_size;
    };

    OccupancyGrid() {
        //get params from yaml file
        ROS_ASSERT(ros::param::get("zMax", m_gridParams.zMax));
        ROS_ASSERT(ros::param::get("xMax", m_gridParams.xMax));
        ROS_ASSERT(ros::param::get("yOffset", m_gridParams.yOffset));
	ROS_ASSERT(ros::param::get("rgb_lower_lim", m_gridParams.rgb_lower_lim));
	ROS_ASSERT(ros::param::get("rgb_upper_lim", m_gridParams.rgb_upper_lim));
        ROS_ASSERT(ros::param::get("resolution", m_gridParams.resolution));
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
        for (int i = 0; i < m_gaussian_blur_kernel.size() * m_gaussian_blur_kernel.size(); i++) {
            m_gaussian_blur_kernel.push_back(param[i]);
        }

        ROS_ASSERT(ros::param::get("gaussian_hor_kernel", param));
        m_gaussian_hor_kernel.setSize(sqrt(param.size()));
        for (int i = 0; i < m_gaussian_hor_kernel.size() * m_gaussian_hor_kernel.size(); i++) {
            m_gaussian_hor_kernel.push_back(param[i]);
        }
        ROS_ASSERT(ros::param::get("gaussian_ver_kernel", param));
        m_gaussian_ver_kernel.setSize(sqrt(param.size()));
        for (int i = 0; i < m_gaussian_ver_kernel.size() * m_gaussian_ver_kernel.size(); i++) {
            m_gaussian_ver_kernel.push_back(param[i]);
        }
        ROS_ASSERT(ros::param::get("gaussian_hor_norm_kernel", param));
        m_gaussian_hor_norm_kernel.setSize(sqrt(param.size()));
        for (int i = 0; i < m_gaussian_hor_norm_kernel.size() * m_gaussian_hor_norm_kernel.size(); i++) {
            m_gaussian_hor_norm_kernel.push_back(param[i]);
        }
        ROS_ASSERT(ros::param::get("gaussian_ver_norm_kernel", param));
        m_gaussian_ver_norm_kernel.setSize(sqrt(param.size()));
        for (int i = 0; i < m_gaussian_ver_norm_kernel.size() * m_gaussian_ver_norm_kernel.size(); i++) {
            m_gaussian_ver_norm_kernel.push_back(param[i]);
        }

        //set grid params
        m_gridZSize = m_gridParams.zMax / m_gridParams.resolution + 1;
        m_gridXSize = m_gridParams.xMax * 2 / m_gridParams.resolution + 1;

        //make sure camera is in centre of a cell
        if (m_gridXSize % 2 == 0) {
            m_gridXSize++;
        }
        m_gridCameraZ = 0;
        m_gridCameraX = m_gridXSize / 2;

        //sub & pub
        m_sub = m_n.subscribe(pcl2TopicName, m_gridParams.queue_size, &OccupancyGrid::callback, this);
        m_pub = m_n.advertise<occupancy_grid::OccupancyGrid>("/OccupancyGrid", m_gridParams.queue_size);

        //Since rviz will flash back and forth between multiple messages published by the same topic, using an array of topics
        m_pub_rviz[0] = m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsPointsDetected",
                                                               m_gridParams.queue_size);
        m_pub_rviz[1] = m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsAvg", m_gridParams.queue_size);
        m_pub_rviz[2] = m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsMax", m_gridParams.queue_size);
        m_pub_rviz[3] = m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsMin", m_gridParams.queue_size);
        m_pub_rviz[4] = m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsBlur", m_gridParams.queue_size);
        m_pub_rviz[5] = m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsBlurSlope", m_gridParams.queue_size);
        m_pub_rviz[6] = m_n.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridCellsBlurSlopeNorm",
                                                               m_gridParams.queue_size);
    }

    void callback(const sensor_msgs::PointCloud2 input);

    gridParams getGridParams() const {
        return m_gridParams;
    }

private:
    ros::NodeHandle m_n;
    ros::Subscriber m_sub;
    ros::Publisher m_pub;
    ros::Publisher m_pub_rviz[7];

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
};

void OccupancyGrid::callback(const sensor_msgs::PointCloud2 input) {
    occupancy_grid::OccupancyGrid output;

    output.header.cameraZMax = m_gridParams.zMax;
    output.header.cameraXMax = m_gridParams.xMax;
    output.header.gridResolution = m_gridParams.resolution;
    output.header.gridCameraZ = m_gridCameraZ;
    output.header.gridCameraX = m_gridCameraX;
    output.header.cameraYOffset = m_gridParams.yOffset;

    output.dataDimension.emplace_back(std::move(occupancy_grid::GridDataDimension()));
    output.dataDimension.emplace_back(std::move(occupancy_grid::GridDataDimension()));
    output.dataDimension.emplace_back(std::move(occupancy_grid::GridDataDimension()));

    output.dataDimension[0].label = "Z(Forward)";
    output.dataDimension[1].label = "X(Left)";
    output.dataDimension[2].label = "Points Detected // Avg. Height // Max Height // Min Height // Gaussian Blur // Gaussian Blur * Slope // Gaussian Blur * Slope (Normalized)";

    output.dataDimension[0].size = m_gridZSize;
    output.dataDimension[1].size = m_gridXSize;
    output.dataDimension[2].size = 7;

    output.dataDimension[0].stride = output.dataDimension[1].size * output.dataDimension[2].size;
    output.dataDimension[1].stride = output.dataDimension[2].size;
    output.dataDimension[2].stride = 1;

    output.data.resize(output.dataDimension[0].size * output.dataDimension[1].size * output.dataDimension[2].size, 0);

    std::vector<std::vector<float>> oGridPoints;
    oGridPoints.resize(m_gridZSize * m_gridXSize, std::vector<float>());

    sensor_msgs::PointCloud2ConstIterator<float> iterX(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterY(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iterZ(input, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iterRGB(input, "rgb");

    ROS_INFO_STREAM_COND(m_log, std::endl << "New Frame Detected" << std::endl);
    ROS_DEBUG_STREAM_COND(m_log, std::endl << "Input Data & Conversion" << std::endl);

    for (; iterZ != iterZ.end(); ++iterX, ++iterY, ++iterZ, ++iterRGB) {
        float height = (-1 * (*iterY) + m_gridParams.yOffset);
	uint8_t r = iterRGB[0], g = iterRGB[1], b = iterRGB[2];
        int convZ = (*iterZ) / m_gridParams.resolution;
        int convX = 1 * (*iterX) / m_gridParams.resolution + (m_gridXSize / 2.0);

        /* ROS_DEBUG_STREAM_COND(m_log, std::fixed << std::setprecision(3) << "Z: " << *iterZ << "\tX: " << *iterX << "\tY: "
                                         << *iterY << "\tconvZ: " << convZ << "\tconvX: " << convX << "\tHeight: "
                                         << height << std::endl);*/

	//invalid bounds error check, and remove points too black or two white since their depth is not accurate
        if (convZ < m_gridZSize && convX < m_gridXSize && convZ >= 0 && convX >= 0 && r > m_gridParams.rgb_lower_lim[0] && g > m_gridParams.rgb_lower_lim[1] && b > m_gridParams.rgb_lower_lim[2]
	     && r < m_gridParams.rgb_upper_lim[0] && g < m_gridParams.rgb_upper_lim[1] && b < m_gridParams.rgb_upper_lim[2]) {     
            oGridPoints[convZ * m_gridXSize + convX].reserve(1500);
            oGridPoints[convZ * m_gridXSize + convX].emplace_back(height);
        } else {
            //ROS_DEBUG_STREAM_COND(m_log, "Point Detected Out of Occupancy Grid Limits" << std::endl);
        }
    }

    for (int z = 0; z < m_gridZSize; z++) {
        for (int x = 0; x < m_gridXSize; x++) {
            std::sort(oGridPoints[z * m_gridXSize + x].begin(), oGridPoints[z * m_gridXSize + x].end(),
                      std::greater<float>());

            //point count
            oGridDataAccessor(output, z, x, 0) = oGridPoints[z * m_gridXSize + x].size();

            float sum = 0;
            if (oGridDataAccessor(output, z, x, 0) != 0) {
                //avg height
                for (float a : oGridPoints[z * m_gridXSize + x]) {
                    sum += a;
                }
                oGridDataAccessor(output, z, x, 1) = sum / oGridDataAccessor(output, z, x, 0);

                //max height
                sum = 0;
                for (int i = 0; i < oGridPoints[z * m_gridXSize + x].size() * 0.05; i++) {
                    sum += oGridPoints[z * m_gridXSize + x][i];
                }
                oGridDataAccessor(output, z, x, 2) =
                        sum / (unsigned int) (oGridPoints[z * m_gridXSize + x].size() * 0.05 + 1);

                //min height
                sum = 0;
                for (int i = 0, index = oGridPoints[z * m_gridXSize + x].size() - 1;
                     i < oGridPoints[z * m_gridXSize + x].size() * 0.05; i++, index--) {
                    sum += oGridPoints[z * m_gridXSize + x][index];
                }
                oGridDataAccessor(output, z, x, 3) =
                        sum / (unsigned int) (oGridPoints[z * m_gridXSize + x].size() * 0.05 + 1);
            }
        }
    }

    for (int z = 0; z < m_gridZSize; z++) {
        for (int x = 0; x < m_gridXSize; x++) {
            int zExtended = 0;
            int xExtended = 0;
            float weightedSum[5]{0};
            //ROS_ERROR_STREAM(z<< " "<<x<<std::endl);
            //ASSUMES ALL KERNELS ARE SAME SIZE (5)
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

                    weightedSum[0] +=
                            oGridDataAccessor(output, zExtended, xExtended, 1) * m_gaussian_blur_kernel[row][col];
                    weightedSum[1] +=
                            oGridDataAccessor(output, zExtended, xExtended, 1) * m_gaussian_hor_kernel[row][col];
                    weightedSum[2] +=
                            oGridDataAccessor(output, zExtended, xExtended, 1) * m_gaussian_ver_kernel[row][col];
                    weightedSum[3] +=
                            oGridDataAccessor(output, zExtended, xExtended, 1) * m_gaussian_hor_norm_kernel[row][col];
                    weightedSum[4] +=
                            oGridDataAccessor(output, zExtended, xExtended, 1) * m_gaussian_ver_norm_kernel[row][col];
                    //ROS_ERROR_STREAM(row<<" "<<col<<" "<< m_gaussian_blur_kernel[row][col]<<" "<<(output, zExtended, xExtended, 1) << " " <<weightedSum[0]<<std::endl);
                }
            }
            //ROS_ERROR_STREAM(weightedSum[0]<<std::endl);
            //gaussian blur
            oGridDataAccessor(output, z, x, 4) = weightedSum[0];
            //normalized slope + gaussian blur
            oGridDataAccessor(output, z, x, 5) = sqrt(
                    weightedSum[1] * weightedSum[1] + weightedSum[2] * weightedSum[2]);
            //normalized slope + gaussian blur
            oGridDataAccessor(output, z, x, 6) = sqrt(
                    weightedSum[3] * weightedSum[3] + weightedSum[4] * weightedSum[4]);
        }
    }

    m_pub.publish(output);

    //ouput to rviz to visualize, publishes 7 messages, each corresponds to one element of the third dimension of the occupancy grid message(ie. point count, avg, max, min heights)
    for (int i = 0; i < output.dataDimension[2].size; i++) {
        nav_msgs::OccupancyGrid gridcells;
        gridcells.header.frame_id = "/base_link";
        gridcells.header.stamp = ros::Time::now();
        gridcells.info.resolution = 1.0;
        gridcells.info.width = m_gridXSize;
        gridcells.info.height = m_gridZSize;
        //set the view to topdownortho in rviz, the display will be in the correct orientation. Bottom left: point count, bottom right: avg, top left: max, top right: min
        gridcells.info.origin.position.x = i % 2 * m_gridXSize + i % 2 * 10.0;
        gridcells.info.origin.position.y = i / 2 * m_gridZSize + i / 2 * 10.0;
        gridcells.info.origin.position.z = 0.0;
        gridcells.info.origin.orientation.x = 0.0;
        gridcells.info.origin.orientation.y = 0.0;
        gridcells.info.origin.orientation.z = 0.0;
        gridcells.info.origin.orientation.w = 1.0;

        gridcells.data.resize(m_gridXSize * m_gridZSize, 0.0);
        //map the occupancy grid values to standard 0-100 value for display
        for (int x = 0; x < m_gridXSize; x++) {
            for (int z = 0; z < m_gridZSize; z++) {

                float cost = oGridDataAccessor(output, z, x, i);
                //ROS_ERROR_STREAM ( "1normalizer: "<< (float)m_gridParams.mappingNormalizer << " scalar: "<<m_gridParams.mappingScalar << " mpas to:"<< (cost/m_gridParams.mappingNormalizer/2.0*100*m_gridParams.mappingScalar) );


                //since number of points is typically very large
                if (i == 0) {
                    int map_value = (int) cost;

                    if (map_value > 100)
                        map_value = 100;

                    gridcells.data[z * m_gridXSize + x] = map_value;
                } else {
                    if (cost <= 0)
                        cost = 0;
                    else {
                        cost = int(cost / m_gridParams.mappingNormalizer * 100 * m_gridParams.mappingScalar);

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
                debugString << std::setw(8) << (unsigned int) oGridDataAccessor(output, z, x, 0);
            }
            debugString << std::endl;
        }
        ROS_DEBUG_STREAM(debugString.str());

        debugString.str("");
        debugString << std::endl << "Average Height" << std::endl;
        for (int z = m_gridZSize - 1; z >= 0; z--) {
            for (int x = 0; x < m_gridXSize; x++) {
                debugString << oGridDataAccessor(output, z, x, 1) << "\t";
            }
            debugString << std::endl;
        }
        ROS_DEBUG_STREAM(debugString.str());

        debugString.str("");
        debugString << std::endl << "Average Max Height (Highest 5%)" << std::endl;
        for (int z = m_gridZSize - 1; z >= 0; z--) {
            for (int x = 0; x < m_gridXSize; x++) {
                debugString << oGridDataAccessor(output, z, x, 2) << "\t";
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

        debugString.str("");
        debugString << std::endl << "Blurred Avg" << std::endl;
        for (int z = m_gridZSize - 1; z >= 0; z--) {
            for (int x = 0; x < m_gridXSize; x++) {
                debugString << oGridDataAccessor(output, z, x, 4) << "\t";
            }
            debugString << std::endl;
        }
        ROS_DEBUG_STREAM(debugString.str());

        debugString.str("");
        debugString << std::endl << "Slopes" << std::endl;
        for (int z = m_gridZSize - 1; z >= 0; z--) {
            for (int x = 0; x < m_gridXSize; x++) {
                debugString << oGridDataAccessor(output, z, x, 5) << "\t";
            }
            debugString << std::endl;
        }
        ROS_DEBUG_STREAM(debugString.str());

        debugString.str("");
        debugString << std::endl << "Norm Slopes" << std::endl;
        for (int z = m_gridZSize - 1; z >= 0; z--) {
            for (int x = 0; x < m_gridXSize; x++) {
                debugString << oGridDataAccessor(output, z, x, 6) << "\t";
            }
            debugString << std::endl;
        }
        ROS_DEBUG_STREAM(debugString.str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "OccupancyGrid");

    OccupancyGrid o;

    ros::Rate rate(o.getGridParams().rate);

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    while (ros::ok()) {
        ros::spinOnce();
        ros::Rate(rate).sleep();
    }
}
