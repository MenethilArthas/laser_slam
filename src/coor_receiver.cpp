#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "laser_slam/coor.h"
#include "tf/transform_broadcaster.h"

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
	ros::init(argc,argv,"pub_odom");
	ros::NodeHandle n;
	
    tf::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped laser2BaseLinkTrans;
    geometry_msgs::TransformStamped baseLink2WorldTrans;
    
	baseLink2WorldTrans.header.frame_id = "world";
    baseLink2WorldTrans.child_frame_id = "base_link";
    laser2BaseLinkTrans.header.frame_id="base_link";
    laser2BaseLinkTrans.child_frame_id="laser";
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
            /*set the transform of base_link */
			baseLink2WorldTrans.header.stamp = ros::Time::now();
	        baseLink2WorldTrans.transform.translation.x = coorMsg.x/1000.0;
	        baseLink2WorldTrans.transform.translation.y = coorMsg.y/1000.0;
	        baseLink2WorldTrans.transform.translation.z = 0.0;
	        baseLink2WorldTrans.transform.rotation = tf::createQuaternionMsgFromYaw(coorMsg.angle/360.0*2*3.1415);
	        /*set the transform of laser*/
			laser2BaseLinkTrans.header.stamp = ros::Time::now();
	        laser2BaseLinkTrans.transform.translation.x = 0.0;
	        laser2BaseLinkTrans.transform.translation.y = 0.0;
	        laser2BaseLinkTrans.transform.translation.z = 0.1;
	        laser2BaseLinkTrans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
            tfb.sendTransform(laser2BaseLinkTrans);
            tfb.sendTransform(baseLink2WorldTrans);
			//ROS_INFO("x=%f,y=%f,angle=%f",coorMsg.x,coorMsg.y,coorMsg.angle);
			coor_pub.publish(coorMsg);
		}
		
	}
	return 1;
		
}
