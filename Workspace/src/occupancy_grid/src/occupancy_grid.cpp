#include "ros/ros.h"
#include <ros/console.h>

//msg types
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

//debug
#define DEBUG 
#ifdef DEBUG
#include <fstream>
#endif



//resolution is in terms of side length of a grid cell
//x, y max is the max range of the camera (in one direction)
typedef struct 
{
   float xMax;
   float yMax;
   float resolution;
   int rate;
   int queue_size;
} gridParams;

class OccupancyGrid {
	public:
		OccupancyGrid(){

			ros::param::get("xMax", m_gridParams.xMax);
			ros::param::get("yMax", m_gridParams.yMax);
			ros::param::get("resolution", m_gridParams.resolution);
			ros::param::get("rate", m_gridParams.rate);
			ros::param::get("queue_size", m_gridParams.queue_size);

			//account for centreeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
			m_gridXSize = m_gridParams.xMax*2/m_gridParams.resolution + 1;
			m_gridYSize = m_gridParams.yMax*2/m_gridParams.resolution + 1;

			if (m_gridXSize % 2 == 0) {
				m_gridXSize++;	
			}

			m_sub = m_n.subscribe("/duo3d/point_cloud/image_raw", m_gridParams.queue_size, &OccupancyGrid::callback, this);
			m_pub = m_n.advertise<std_msgs::Float32MultiArray>("OccupancyGrid", m_gridParams.queue_size);
		}

		void callback(const sensor_msgs::PointCloud2 input);
		
		gridParams getGridParams() const{
			return m_gridParams;
		}
		
	private:
		ros::NodeHandle m_n;	
		ros::Subscriber m_sub;
		ros::Publisher m_pub;
				
		int m_gridXSize; 
		int m_gridYSize;
   		
		gridParams m_gridParams;

		#ifdef DEBUG
		std::ofstream fout;
		#endif
};

void OccupancyGrid::callback(const sensor_msgs::PointCloud2 input){
	
	std_msgs::Float32MultiArray output;

	output.layout.dim.push_back(std_msgs::MultiArrayDimension());
	output.layout.dim.push_back(std_msgs::MultiArrayDimension());
	output.layout.dim.push_back(std_msgs::MultiArrayDimension());
	output.layout.dim[0].label  = "x";
	output.layout.dim[1].label  = "y";
	output.layout.dim[2].label  = "infoChannel";

	output.layout.dim[0].size   = m_gridXSize;
	output.layout.dim[1].size   = m_gridYSize;
	output.layout.dim[2].size   = 1;

	output.layout.dim[0].stride = output.layout.dim[0].size * output.layout.dim[1].size * output.layout.dim[2].size;
	output.layout.dim[1].stride = output.layout.dim[1].size * output.layout.dim[2].size;
	output.layout.dim[2].stride = output.layout.dim[2].size;
	
	output.data.resize(output.layout.dim[0].stride);
			
	output.data.clear();	

	std::vector<std::vector<int>> count;
	count.resize(m_gridXSize, std::vector<int>(m_gridYSize, 0));

	for (int i =0; i<input.row_step/input.point_step; i++){
		float x = input.data[i]<<24 | input.data[i+1]<<16 | input.data[i+2]<<8 | input.data[i+3];
		float y = input.data[i+4]<<24 | input.data[i+5]<<16 | input.data[i+6]<<8 | input.data[i+7];
		float z = input.data[i+8]<<24 | input.data[i+9]<<16 | input.data[i+10]<<8 | input.data[i+11];
		
		int convX = (x + m_gridParams.xMax) / m_gridParams.resolution;
		int convY = (y + m_gridParams.yMax) / m_gridParams.resolution;
//we can leave it as an average for now but we might want to look into other ways of calculating the cost of a grid, as tom said
//if (z > threshold)
		if (convX >= m_gridXSize || convY >= m_gridYSize || convX < 0 || convY < 0 )//invalid bounds error check
		{
			ROS_ERROR_STREAM("CONVERSION BOUND ERROR\tconvX: "<< convX << "\tconvY: " << convY << "\tMaxXGridCoord: " << m_gridXSize << "\tMaxYGridCoord: " << m_gridYSize << std::endl);			
			return;
		}
		output.data[convX*output.layout.dim[1].stride + convY*output.layout.dim[2].stride + 0] = z; 
		count[convX][convY] ++;
	} 

	for(int x = 0; x < m_gridXSize; x++){
		for (int y = 0; y < m_gridYSize; y++){
			output.data[x*output.layout.dim[1].stride + y*output.layout.dim[2].stride + 0]/=count[x][y];
		}
	}
	m_pub.publish(output);

	#ifdef DEBUG
	fout.open("/home/wmmc/Documents/OCCUPANCY_GRID_OUTPUT.txt", std::ofstream::app);
	fout << std::endl << "INPUT" << std::endl;
	for (int i =0; i<input.row_step/input.point_step; i++){
		float x = input.data[i]<<24 | input.data[i+1]<<16 | input.data[i+2]<<8 | input.data[i+3];
		float y = input.data[i+4]<<24 | input.data[i+5]<<16 | input.data[i+6]<<8 | input.data[i+7];
		float z = input.data[i+8]<<24 | input.data[i+9]<<16 | input.data[i+10]<<8 | input.data[i+11];
		
		fout << x << " " << y << " " << z << std::endl;
	}


	fout << std::endl << "OUTPUT" << std::endl;
	for(int row =0; row<output.layout.dim[0].size; row++){
		for(int col = 0; col<output.layout.dim[1].size; col++){
			fout << output.data[row*output.layout.dim[1].stride + col*output.layout.dim[2].stride + 0] << " ";
		}
		fout << std::endl;
	}
	fout.close();
	#endif
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "OccupancyGrid");
	#ifdef DEBUG
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
			ros::console::notifyLoggerLevelsChanged();
		}
	#endif	

	OccupancyGrid o;

	ros::Rate rate(o.getGridParams().rate);

	while(ros::ok()) {
		ros::spinOnce();
		ros::Rate(rate).sleep();
	}
}
