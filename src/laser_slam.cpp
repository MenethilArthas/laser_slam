#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
//#include "laser_slam.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "types.h"
#include "mapper.h"
#include <iostream>
#include <string>


int cellBlock::size=5;
float LocalizedScan::m_angleIncrement=0.01745;
float LocalizedScan::m_maxRange=8.0;
float LocalizedScan::m_minRange=0.2;
class LaserSlam
{
	public:
		LaserSlam();
		~LaserSlam();
		void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
		bool AddScan(const sensor_msgs::LaserScan::ConstPtr& scan, LaserParamManager laserParamManager,
                   Pose2& odomPose);
		void GetLaserParam(LaserParamManager &laserParamManager,const sensor_msgs::LaserScan::ConstPtr& scan);
		bool GetOdomPose(Pose2& robotPose, const ros::Time& t);
		float GetResolution();
		void PublishLoop();
		void PublishTransform();
	private:
		LaserParamManager m_laserParamManager;
	    ros::NodeHandle m_node;
		tf::TransformBroadcaster	tfb;
// 		geometry_msgs::TransformStamped correctedPose2WorldTrans;
// 		geometry_msgs::TransformStamped odom2WorldTrans;
		message_filters::Subscriber<sensor_msgs::LaserScan>* m_scanFilterSub;
		tf::MessageFilter<sensor_msgs::LaserScan>* m_scanFilter;
		tf::TransformListener m_tfl;
	    ros::Publisher m_mapp;
		Mapper m_mapper;
		std::string m_baseFrame;
		std::string m_odomFrame;
		std::string m_worldFrame;
		float m_resolution;
		ros::Duration m_mapUpdateInterval;
		
		boost::thread* transform_thread_;
		boost::mutex map_to_odom_mutex_;
		tf::Transform map_to_odom_;
};

LaserSlam::LaserSlam()
{
	m_baseFrame="base_link";
	m_odomFrame="odom";
	m_worldFrame="map";
	m_mapp=m_node.advertise<nav_msgs::OccupancyGrid>("map", 1);
	m_scanFilterSub=new message_filters::Subscriber<sensor_msgs::LaserScan>(m_node,"scan",5);
	m_scanFilter = new tf::MessageFilter<sensor_msgs::LaserScan>(*m_scanFilterSub, m_tfl, m_odomFrame, 5);
	m_scanFilter->registerCallback(boost::bind(&LaserSlam::LaserCallback,this,_1));
	m_resolution=0.05;
	m_mapUpdateInterval.fromSec(25.0);
	//transform_thread_=new boost::thread(boost::bind(&LaserSlam::PublishLoop,this));
}

void LaserSlam::PublishLoop()
{
	ros::Rate r(20);
	while(ros::ok())
	{
		PublishTransform();
		r.sleep();
	}
}
void LaserSlam::PublishTransform()
{
	//boost::mutex::scoped_lock lock(map_to_odom_mutex_);
	
}


LaserSlam::~LaserSlam()
{
	
}
void LaserSlam::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	nav_msgs::OccupancyGrid m_occGridMsg;
	static ros::Time lastUpdateTime(0,0);
	static bool gotMap=false;
	/*get the parameter of laser device*/	
	if(!m_laserParamManager.IsUpdate())
	{
		GetLaserParam(m_laserParamManager,scan);
	}	
	Pose2 odomPose;
	if(AddScan(scan,m_laserParamManager,odomPose))
	{
		ROS_INFO("added scan at pose: %.3f %.3f %.3f", 
              odomPose.GetX(),
              odomPose.GetY(),
              odomPose.GetHeading());
		if(!gotMap/*||scan->header.stamp - lastUpdateTime > m_mapUpdateInterval*/)
		{
			gotMap=true;
			OccupancyGrid occ(GetResolution());
			m_mapper.CreateMap(occ);
			m_occGridMsg.header.frame_id=m_worldFrame;
			m_occGridMsg.info.origin.position.z=0.0;
			m_occGridMsg.info.origin.orientation.x=0.0;
			m_occGridMsg.info.origin.orientation.y=0.0;
			m_occGridMsg.info.origin.orientation.z=0.0;
			m_occGridMsg.info.origin.orientation.w=1.0;
			m_occGridMsg.info.resolution=m_resolution;
			m_occGridMsg.info.height=occ.GetHeight();
			m_occGridMsg.info.width=occ.GetWidth();
			m_occGridMsg.info.origin.position.x=occ.GetOriginPose().GetX();
			m_occGridMsg.info.origin.position.y=occ.GetOriginPose().GetY();
			for(int y=0;y<occ.GetHeight();y++)
			{
				for(int x=0;x<occ.GetWidth();x++)
				{
					int8_t value=occ.GetMapValue(y*occ.GetWidth()+x);
					m_occGridMsg.data.push_back(value);
				}
			}
			m_occGridMsg.header.stamp=ros::Time::now();
			m_mapp.publish(m_occGridMsg);
			lastUpdateTime=scan->header.stamp;
		}
	}					
}


void LaserSlam::GetLaserParam(LaserParamManager &laserParamManager,const sensor_msgs::LaserScan::ConstPtr& scan)
{
	laserParamManager.SetAngleIncrement(scan->angle_increment);
	laserParamManager.SetMaxAngle(scan->angle_max);
	laserParamManager.SetMinAngle(scan->angle_min);
	laserParamManager.SetMinRange(scan->range_min);
	laserParamManager.SetMaxRange(scan->range_max);
	laserParamManager.SetUpdateFlag(true);
}
bool LaserSlam::AddScan(const sensor_msgs::LaserScan::ConstPtr& scan, LaserParamManager laserParamManager,
                   Pose2& odomPose)
{
	/*get the robot pose at the given scan*/
	if(!GetOdomPose(odomPose,scan->header.stamp))
		return false;
	VectorFloat		readings;
	float angle=0.0;
	for(std::vector<float>::const_iterator it=scan->ranges.begin(); it != scan->ranges.end();++it)
	{			
		angle+=laserParamManager.GetAngleIncrement();
		readings.push_back(*it);		
	}
	LocalizedScan *newScan=new LocalizedScan(readings,odomPose);

	if(m_mapper.Process(newScan)==true)
	{
		ROS_INFO("process success.");
		Pose2 corrected_pose=newScan->GetCorrectedPose();
		Pose2 odom_pose=newScan->GetOdomPose();
		
		tf::Stamped<tf::Pose> odom_to_map;
		try
		{
			m_tfl.transformPose(m_odomFrame,tf::Stamped<tf::Pose> (tf::Transform(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
                                                                    tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)).inverse(),
                                                                    scan->header.stamp, m_baseFrame),odom_to_map);
		}
		catch(tf::TransformException e)
		{
			ROS_ERROR("Transform from base_link to odom failed\n");
			odom_to_map.setIdentity();
		}

// 		Pose2 offsetPose=corrected_pose-odom_pose;
// 		double offsetAngle=corrected_pose.GetHeading()-odom_pose.GetHeading();

//		boost::mutex::scoped_lock lock(map_to_odom_mutex_);  
		map_to_odom_=tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
                                 tf::Point(      odom_to_map.getOrigin() ) ).inverse();
// 		map_to_odom_=tf::Transform(tf::createQuaternionFromRPY(0,0,offsetAngle),tf::Vector3(offsetPose.GetX(),offsetPose.GetY(),0));
		tfb.sendTransform(tf::StampedTransform (map_to_odom_, ros::Time::now(), m_worldFrame, m_odomFrame));
		return true;
	}
	else
	{
		tfb.sendTransform(tf::StampedTransform (map_to_odom_, ros::Time::now(), m_worldFrame, m_odomFrame));
		delete newScan;
		return false;
	}
	
}

bool LaserSlam::GetOdomPose(Pose2& robotPose, const ros::Time& t)
{
	
	// Get the robot's pose
	tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
											tf::Vector3(0,0,0)), t, m_baseFrame);
	tf::Stamped<tf::Transform> odom_pose;
	try
	{
		m_tfl.transformPose(m_odomFrame, ident, odom_pose);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	double yaw = tf::getYaw(odom_pose.getRotation());

	robotPose = Pose2(odom_pose.getOrigin().x(),
							odom_pose.getOrigin().y(),
							yaw);
	return true;
}
float LaserSlam::GetResolution()
{
	return m_resolution;
}
int main(int argc,char **argv)
{
	/*init ros node*/
	ros::init(argc,argv,"laser_slam");
	LaserSlam ls;
	ros::spin();
    return 0;
}
