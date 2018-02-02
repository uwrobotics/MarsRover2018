#include "ros/ros.h"

//msg types
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

class OccupancyGrid {
	public:
		OccupancyGrid(float resolution){
			
			m_gridXSize = 10;//do some conversion w/ res///////////////////////////////////////////////////////////////// Bob implement conversion
			m_gridYSize = 10;//do some conversion w/ res///////////////////////////////////////////////////////////////// Bob implement conversion
			
			m_sub = m_n.subscribe("/duo3d/point_cloud/image_raw", 1, &OccupancyGrid::callback, this);
			m_pub = m_n.advertise<std_msgs::Float32MultiArray>("OccupancyGrid", 1);
		}

		void callback(const sensor_msgs::PointCloud2 input);
		
	private:
		ros::NodeHandle m_n;	
		ros::Subscriber m_sub;
		ros::Publisher m_pub;
				
		int m_gridXSize; 
		int m_gridYSize;
};

void OccupancyGrid::callback(const sensor_msgs::PointCloud2 input){
	
	std_msgs::Float32MultiArray output;

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
		
		int convX = x / m_gridXSize;
		int convY = y / m_gridYSize;
		output.data[convX*output.layout.dim[1].stride + convY*output.layout.dim[2].stride + 0] = z;
		count[convX][convY] ++;
	}
	
	for(int x = 0; x < m_gridXSize; x++){
		for (int y = 0; y < m_gridYSize; y++){
			output.data[x*output.layout.dim[1].stride + y*output.layout.dim[2].stride + 0]/=count[x][y];
		}
	}


	m_pub.publish(output);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "OccupancyGrid");

	OccupancyGrid o(10);//change rez///////////////////////////////////////////////////////////////// BOB implement param

	ros::Rate rate(5);//hz////////////////////////////////////////////////////////////////////////// BOB implement param

	while(ros::ok()) {
		ros::spinOnce();
		ros::Rate(rate).sleep();
	}
}
