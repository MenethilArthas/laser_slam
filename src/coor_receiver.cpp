#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "laser_slam/coor.h"


union unionCoor
{
	float data[6];
	uint8_t buffer[24];
};

int main(int argc,char **argv)
{

	std_msgs::String port;
	int baudrate=0;
	unionCoor coorRec;
	uint8_t checkByte=0;
	static int count=0;
	bool updateFlag=false;
	laser_slam::coor coorMsg;

	/*init ros node*/
	ros::init(argc,argv,"coor_receiver");
	ros::NodeHandle n;
	
	/*get the serial parameters*/
	ros::param::get("~port",port.data);
	ros::param::get("~baudRate",baudrate);
	
	ROS_INFO("port=%s",port.data.c_str());
	ROS_INFO("baudRate=%d",baudrate);
	/*declare a publisher*/
	ros::Publisher coor_pub=n.advertise<laser_slam::coor>("coor",1000);

	serial::Serial coorSerial(port.data,(uint32_t)baudrate,serial::Timeout::simpleTimeout(1000));
	ROS_INFO("is the serial port open?");
	if(coorSerial.isOpen())
		ROS_INFO("Yes.");
	else 
	{
		ROS_INFO("No.");
		return 0;
	}
	while(ros::ok())
	{
		coorSerial.read(&checkByte,1);
		switch(count)
		{
			case 0:
				if(checkByte==0x0d)
					count++;
				else
					count=0;
				break;
			case 1:			
				if(checkByte==0x0a)
				{
					count++;
				}
				else if(checkByte==0x0d)
					break;
				else
				{
					count=0;
					break;
				}
					 
			case 2:
				coorSerial.read(coorRec.buffer,24);
				count++;
				break;
			case 3:
				if(checkByte==0x0a)
					count++;
				else
					count=0;
				break;
			case 4:
				if(checkByte==0x0d)
				{
					updateFlag=true;
					coorMsg.angle=coorRec.data[0];
					coorMsg.x=coorRec.data[3];
					coorMsg.y=coorRec.data[4];
				}
				count=0;
				memset(coorRec.data,0,sizeof(uint8_t)*24);
				break;
			default:
				count=0;
				break;
		}
		if(updateFlag==true)
		{
			updateFlag=false;
			ROS_INFO("x=%f,y=%f,angle=%f",coorMsg.x,coorMsg.y,coorMsg.angle);
			coor_pub.publish(coorMsg);
		}
		
	}
	return 1;
		
}
