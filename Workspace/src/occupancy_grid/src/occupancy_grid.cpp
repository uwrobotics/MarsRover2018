#include "ros/ros.h"

//msg types
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

//resolution is in terms of side length of a grid cell
//x, y max is the max range of the camera (in one direction)
typedef struct 
{
   int xMax;
   int yMax;
   int resolution;
   int rate;
   int queue_size;
} gridParams;

class OccupancyGrid {
	public:
		OccupancyGrid(){

			ros::param::get("xMax", myGrid.xMax);
			ros::param::get("yMax", myGrid.yMax);
			ros::param::get("resolution", myGrid.resolution);
			ros::param::get("rate", myGrid.rate);
			ros::param::get("queue_size", myGrid.queue_size);
			
			m_gridXSize = (myGrid.xMax * 2 % myGrid.resolution == 0)? myGrid.xMax*2/myGrid.resolution : myGrid.xMax*2/myGrid.resolution + 1;
			m_gridYSize = (myGrid.yMax * 2 % myGrid.resolution == 0)? myGrid.yMax*2/myGrid.resolution : myGrid.yMax*2/myGrid.resolution + 1;
			
			m_sub = m_n.subscribe("/duo3d/point_cloud/image_raw", myGrid.queue_size, &OccupancyGrid::callback, this);
			m_pub = m_n.advertise<std_msgs::Float32MultiArray>("OccupancyGrid", myGrid.queue_size);
		}

		void callback(const sensor_msgs::PointCloud2 input);
		
		gridParams getGridParams() const{
			return myGrid;
		}

		
	private:
		ros::NodeHandle m_n;	
		ros::Subscriber m_sub;
		ros::Publisher m_pub;
				
		int m_gridXSize; 
		int m_gridYSize;
   		
		gridParams myGrid;
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
//we can leave it as an average for now but we might want to look into other ways of calculating the cost of a grid, as tom said
//if (z > threshold)
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

	OccupancyGrid o;

	ros::Rate rate(o.getGridParams().rate);

	while(ros::ok()) {
		ros::spinOnce();
		ros::Rate(rate).sleep();
	}
}
