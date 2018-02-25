/*
NOTE:
Duo Pointcloud2 data follows (z = forward, x = left, y = down)

*/

#include "ros/ros.h"
#include <ros/console.h>

//std and ext. lib msg types
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//custom msg types
#include "occupancy_grid/OccupancyGrid.h"
#include "occupancy_grid/OccupancyGridHeader.h"
#include "occupancy_grid/GridDataDimension.h"


#define DEBUG_OUTPUT 1

//debug
#ifdef DEBUG_OUTPUT
#include <fstream>
#endif

//accessor for Occupancy Grid Message
float& oGridDataAcessor (occupancy_grid::OccupancyGrid& oGrid, unsigned int i, unsigned int j, unsigned int  k) {
	return oGrid.data[i*oGrid.dataDimension[0].stride + j*oGrid.dataDimension[1].stride + k*oGrid.dataDimension[2].stride];
}

//resolution is in terms of side length of a grid cell
//z, x max is the max range of the camera (in one direction)
typedef struct 
{
   float zMax;
   float xMax;
   float yOffset;
   float yThreshold;
   float resolution;
   int rate;
   int queue_size;
} gridParams;

class OccupancyGrid {
	public:
		OccupancyGrid(){
			//get params from yaml file
			ros::param::get("zMax", m_gridParams.zMax);
			ros::param::get("xMax", m_gridParams.xMax);
			ros::param::get("yOffset", m_gridParams.yOffset);
			ros::param::get("yThreshold", m_gridParams.yOffset);
			ros::param::get("resolution", m_gridParams.resolution);
			ros::param::get("rate", m_gridParams.rate);
			ros::param::get("queue_size", m_gridParams.queue_size);

			//set grid params
			m_gridZSize = m_gridParams.zMax/m_gridParams.resolution + 1;
			m_gridXSize = m_gridParams.xMax*2/m_gridParams.resolution + 1;
			//make sure camera is in centre of a cell
			if (m_gridXSize % 2 == 0) {
				m_gridXSize++;	
			}
			m_gridCameraZ = 0;
			m_gridCameraX = m_gridXSize/2;
			
			//sub & pub
			m_sub = m_n.subscribe("/duo3d/point_cloud/image_raw", m_gridParams.queue_size, &OccupancyGrid::callback, this);
			m_pub = m_n.advertise<occupancy_grid::OccupancyGrid>("/OccupancyGrid", m_gridParams.queue_size);
		}

		void callback(const sensor_msgs::PointCloud2 input);
		
		gridParams getGridParams() const{
			return m_gridParams;
		}
		
	private:
		ros::NodeHandle m_n;	
		ros::Subscriber m_sub;
		ros::Publisher m_pub;
				
		int m_gridZSize; 
		int m_gridXSize;
		int m_gridCameraZ;
		int m_gridCameraX;

		gridParams m_gridParams;

		#ifdef DEBUG_OUTPUT
		std::ofstream fout;
		std::ofstream fout2;
		#endif
};

void OccupancyGrid::callback(const sensor_msgs::PointCloud2 input){
	occupancy_grid::OccupancyGrid output;

	output.header.cameraZMax = m_gridParams.zMax;
	output.header.cameraXMax = m_gridParams.xMax;
	output.header.gridResolution = m_gridParams.resolution;
	output.header.gridCameraZ = m_gridCameraZ;
	output.header.gridCameraX = m_gridCameraX;
	output.header.cameraYOffset = m_gridParams.yOffset;

	output.dataDimension.push_back(occupancy_grid::GridDataDimension());
	output.dataDimension.push_back(occupancy_grid::GridDataDimension());
	output.dataDimension.push_back(occupancy_grid::GridDataDimension());

	output.dataDimension[0].label = "Z(Forward)";
	output.dataDimension[1].label = "X(Left)";
	output.dataDimension[2].label = "Points Detected // Avg. Height // Max Height";

	output.dataDimension[0].size = m_gridZSize;
	output.dataDimension[1].size = m_gridXSize;
	output.dataDimension[2].size = 3;

	output.dataDimension[0].stride = output.dataDimension[1].size * output.dataDimension[2].size;
	output.dataDimension[1].stride = output.dataDimension[2].size;
	output.dataDimension[2].stride = 1;

	output.data.resize(output.dataDimension[0].size * output.dataDimension[1].size * output.dataDimension[2].size, 0);

	#ifdef DEBUG_OUTPUT
	fout.open("/home/wmmc/Documents/OCCUPANCY_GRID_INPUT.txt", std::fstream::app);
	fout2.open("/home/wmmc/Documents/OCCUPANCY_GRID_OUTPUT.txt", std::fstream::app);
	fout << std::endl << std::endl << "NEW FRAME" << std::endl;
	#endif

	sensor_msgs::PointCloud2ConstIterator<float> iterX (input, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iterY (input, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iterZ (input, "z");

	for (;iterX != iterX.end(); ++iterX, ++iterY, ++iterZ){
		float height = -1 * (*iterY) + m_gridParams.yOffset;
		if(height > m_gridParams.yThreshold) {
			int convZ = (*iterZ)/m_gridParams.resolution;
			int convX = (*iterX)/m_gridParams.resolution + (m_gridXSize/2.0);
			#ifdef DEBUG_OUTPUT
			fout<<"X: "<<*iterX<<"\tY: "<<*iterY<<"\tZ: "<< *iterZ<<"\tconvZ: "<<convZ<<"\tconvX: "<<convX<<std::endl;
			#endif	
			ROS_DEBUG_STREAM("X: " << *iterX << "\tY: " << *iterY << "\tZ: " << *iterZ << "\tconvZ: "<< convZ << "\tconvX: " << convX<< std::endl);			
			if (convZ >= m_gridZSize || convX >= m_gridXSize || convZ < 0 || convX < 0 )//invalid bounds error check
			{
				ROS_ERROR_STREAM("X: " << *iterX << "\tY: " << *iterY << "\tZ: " << *iterZ << "\tconvZ: "<< convZ << "\tconvX: " << convX<< std::endl);			
				ROS_ERROR_STREAM("POINT OUTSIDE OF CONFIGURED GRID SIZE\tMaxZGridCoord: " << m_gridZSize << "\tMaxXGridCoord: " << m_gridXSize << std::endl);

				#ifdef DEBUG_OUTPUT
				fout<<"MaxZGridCoord: " << m_gridZSize << "\tMaxXGridCoord: " << m_gridXSize << std::endl;
				fout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^CONVERSION OUT OF BOUNDS ERROR ABOVE^^^^^^^^^^^^^^^^^^^^^^^^^^"<<std::endl;
				#endif	
			}
			else
			{
			oGridDataAccessor(output, convZ, convX, 0) ++;
			oGridDataAccessor(output, convZ, convX. 1) += height;
			}
		}
	}

	for(int z = 0; z < m_gridZSize; z++){
		for (int x = 0; x < m_gridXSize; x++){
			if (count[z][x]!=0) {
				oGridDataAccesor(output, z, x, 1) /= oGridDataAccessor(output, z, x, 0);
			}
		}
	}

	m_pub.publish(output);
	
	#ifdef DEBUG_OUTPUT
	fout2 << std::endl << std::endl << "OUTPUT" << std::endl << "" << std::endl; ///////////////////////////////////////////gbjbsljsbdlkj
	for (int z = m_gridZSize-1; z >= 0; z--){
		for (int x = 0; x < m_gridXSize; x++){
			fout2 << oGridDataAccesor(output, z, x, 1) << "\t";
		}
		fout2 << std::endl << std::endl;
	}

	for (int z = m_gridZSize-1; z >= 0; z--){
		for (int x = 0; x < m_gridXSize; x++){
			fout2 << oGridDataAccesor(output, z, x, 1) << "\t";
		}
		fout2 << std::endl << std::endl;
	}

	for (int z = m_gridZSize-1; z >= 0; z--){
		for (int x = 0; x < m_gridXSize; x++){
			fout2 << oGridDataAccesor(output, z, x, 1) << "\t";
		}
		fout2 << std::endl << std::endl;
	}
	fout.close();
	fout2.close();	
	#endif
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "OccupancyGrid");
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error) ) {
   		ros::console::notifyLoggerLevelsChanged();
	}

	OccupancyGrid o;

	ros::Rate rate(o.getGridParams().rate);

	while(ros::ok()) {
		ros::spinOnce();
		ros::Rate(rate).sleep();
	}
}
