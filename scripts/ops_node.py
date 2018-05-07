#!/usr/bin/env python

import	struct
import	threading
import	serial
import	rospy
import	roslib
from	tf.transformations	import	quaternion_from_euler
from	std_msgs.msg	import	Int16,Int32,Int64,Float32,Float64,String,UInt64
#from	nav_msgs.msg	import	Odometry
from	geometry_msgs.msg	import	Point
from	geometry_msgs.msg	import	Pose

updateflag=False
coorX=0.0
coorY=0.0
coorAngle=0.0

def Receiver():
	print('rec thread start')
	global updateflag
	global coorX
	global coorY
	global coorAngle
	ser=serial.Serial('/dev/serialPort',115200,timeout=1)
	
	while not rospy.is_shutdown():
		data=ser.read(1)
		while(ord(data[0])!=0x0d):
			data=ser.read(1)
		data=ser.read(1)
		if(ord(data[0])==0x0a):
			data=ser.read(24)
			if(len(data)==24):			
				coorAngle,=struct.unpack('f',data[0]+data[1]+data[2]+data[3])
				coorX,=struct.unpack('f',data[12]+data[13]+data[14]+data[15])
				coorY,=struct.unpack('f',data[16]+data[17]+data[18]+data[19])	
				updateflag=True

					
			else:
				pass
		elif(ord(data[0])==0x0d):
			data=ser.read(25)
			if(len(data)==25):			
				coorAngle,=struct.unpack('f',data[1]+data[2]+data[3]+data[4])
				coorX,=struct.unpack('f',data[13]+data[14]+data[15]+data[16])
				coorY,=struct.unpack('f',data[17]+data[18]+data[19]+data[20])
				updateflag=True
				#print("angle=%s" %coorAngle)
			else:
				pass
		else:
			pass
		

if __name__ =='__main__':
	
	#print("main thread start")
	
	rospy.init_node("ops_node",log_level=rospy.INFO)
	#pub=rospy.Publisher('odometry',Odometry,queue_size=100)
	pub=rospy.Publisher('www',Pose,queue_size=100)
	rec=threading.Thread(target=Receiver)
	rec.setDaemon(True)
	rec.start()
	rate=rospy.Rate(10)
	poseMsg=Pose()
	#odometryMsg=Odometry()
	#odometryMsg.header.frame_id='world'
	#odometryMsg.child_frame_id='ops'
	while not rospy.is_shutdown():
		
		if(updateflag):
			print("111")
			#odometryMsg.header.stamp=rospy.Time.now()
			#odometryMsg.pose.pose.position=Point(coorX/1000,coorY/1000,0.0)
			#odometryMsg.pose.pose.orientation=quaternion_from_euler(0.0,0.0,coorAngle/180.0*3.1415)
			poseMsg.position=Point(coorX/1000,coorY/1000,0.0)
			poseMsg.orientation=quaternion_from_euler(0.0,0.0,coorAngle/180.0*3.1415)
			pub.publish(poseMsg)
			updateflag=False
			
			rate.sleep()
				
