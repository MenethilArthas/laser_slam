#include "ros/ros.h"
#include "action_driver.h"
#include "std_msgs/String.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <string>


class MotorController
{
	public:
		MotorController();
		bool InitDriver();
		void KeyCmdCallback(const std_msgs::String::ConstPtr& msg);
	private:
		ros::NodeHandle n;
		ros::Subscriber sub;
		int s;
		ActionDriver driver1;
		ActionDriver driver2;
		ActionDriver driver3; 
		
};

MotorController::MotorController()
{
	struct sockaddr_can addr;
	struct ifreq ifr;
	/*subscribe the topic of cmd_key*/
	sub = n.subscribe("cmd_key",10,&MotorController::KeyCmdCallback,this);
	/*config can0 interface*/
	s = socket(PF_CAN,SOCK_RAW,CAN_RAW);
	strcpy(ifr.ifr_name,"can0");
	ioctl(s,SIOCGIFINDEX,&ifr);
	addr.can_family=AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	bind(s, (struct sockaddr *)&addr, sizeof(addr));
	/*ban the function of filter*/
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
	/*create driver instances*/
	driver1.SetParam(1,s);
	driver2.SetParam(2,s);
	driver3.SetParam(3,s);
}

bool MotorController::InitDriver()
{
	if((driver1.SetOptMode()&&driver2.SetOptMode()&&driver3.SetOptMode())==false)
	{
		ROS_ERROR("motor driver init failed.");
		return false;
	}
	if((driver1.CfgVel(100000,100000)
		&&driver2.CfgVel(100000,100000)
		&&driver3.CfgVel(100000,100000))==false)
	{
		ROS_ERROR("motor driver cfg failed.");
		return false;
	}
    return true;

}

void MotorController::KeyCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	
	char cmd=msg->data.c_str()[0];
	static char preCmd = 0;
	if(cmd!=preCmd)
	{
       // ROS_INFO("cmd=%c",cmd);
		switch(cmd)
		{
			case 'w':
				driver1.SetVel(0);
				driver2.SetVel(-30000);
				driver3.SetVel(30000);
				break;
			case 'a':
				driver1.SetVel(-30000);
				driver2.SetVel(15000);
				driver3.SetVel(15000);
				break;
			case 's':
				driver1.SetVel(0);
				driver2.SetVel(30000);
				driver3.SetVel(-30000);
				break;
			case 'd':
				driver1.SetVel(30000);
				driver2.SetVel(-15000);
				driver3.SetVel(-15000);
				break;
			case 'c':
				driver1.SetVel(0);
				driver2.SetVel(0);
				driver3.SetVel(0);
				break;
			case 'q':
				driver1.SetVel(-15000);
				driver2.SetVel(-15000);
				driver3.SetVel(-15000);
				break;
			case 'e':
				driver1.SetVel(15000);
				driver2.SetVel(15000);
				driver3.SetVel(15000);
				break;
		}
		preCmd=cmd;
	}


}

int main(int argc,char** argv)
{
	/*init ros node*/
	ros::init(argc,argv,"motor_control");
	MotorController motorController;	
	if(motorController.InitDriver()==false)
		return 0;
	else
		ROS_INFO("Init Driver success.");
	ros::spin();
    close(s);
    return 1;


}
