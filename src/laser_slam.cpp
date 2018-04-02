#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"

#include "nav_msgs/OccupancyGrid.h"
//#include "laser_slam.h"

#include <iostream>
#include <string>


class LaserSlam
{
	public:
		LaserSlam();
		~LaserSlam();
		void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	private:
	    ros::NodeHandle m_n;
	    ros::Publisher m_mapp;
	    ros::Subscriber m_scans;
};

LaserSlam::LaserSlam()
{
	m_mapp=m_n.advertise<nav_msgs::OccupancyGrid>("map", 1);
	m_scans=m_n.subscribe<sensor_msgs::LaserScan>("scan",1000,&LaserSlam::LaserCallback,this);

	
}
LaserSlam::~LaserSlam()
{

}
void LaserSlam::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("laser callback.");
}

int main(int argc,char **argv)
{
	/*init ros node*/
	ros::init(argc,argv,"laser_slam");
	LaserSlam ls;
	ros::spin();
    return 0;
}
