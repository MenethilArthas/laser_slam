#!/usr/bin/env python

import struct
import threading
import serial
import rospy
from std_msgs.msg import Int16,Int32,Int64,Float32,Float64,String,UInt64
from laser_slam.msg import coor

updateflag=False
coor_x=0
coor_y=0
coor_angle=0

def Receiver():
	print('rec thread start')
	global updateflag
	global coor_x
	global coor_y
	global coor_angle
	ser=serial.Serial('/dev/ttyUSB0',115200,timeout=1)
	
	while not rospy.is_shutdown():
		data=ser.read(1)
		while(ord(data[0])!=0x0d):
			data=ser.read(1)
		data=ser.read(1)
		if(ord(data[0])==0x0a):
			data=ser.read(24)
			if(len(data)==24):			
				coor_angle,=struct.unpack('f',data[0]+data[1]+data[2]+data[3])
				coor_x,=struct.unpack('f',data[12]+data[13]+data[14]+data[15])
				coor_y,=struct.unpack('f',data[16]+data[17]+data[18]+data[19])
	
				updateflag=True

					
			else:
				pass
		elif(ord(data[0])==0x0d):
			data=ser.read(25)
			if(len(data)==25):			
				coor_angle,=struct.unpack('f',data[1]+data[2]+data[3]+data[4])
				coor_x,=struct.unpack('f',data[13]+data[14]+data[15]+data[16])
				coor_y,=struct.unpack('f',data[17]+data[18]+data[19]+data[20])

				updateflag=True

					
			else:
				pass
		else:
			pass
		

if __name__ =='__main__':
	
	#print("main thread start")
	
	rospy.init_node("serial_receiver",log_level=rospy.INFO)
	pub=rospy.Publisher('coor_data',coor,queue_size=1000)
	rec=threading.Thread(target=Receiver)
	rec.setDaemon(True)
	rec.start()
	rate=rospy.Rate(10)
	
	while not rospy.is_shutdown():
		
		if(updateflag):
			
			coor_data=coor()
			coor_data.x=coor_x
			coor_data.y=coor_y
			coor_data.angle=coor_angle

			rospy.loginfo("x=%f y=%f angle=%f" %(coor_data.x,coor_data.y,coor_data.angle))
			print(coor_data.angle)
			#print(coord_angle)
			pub.publish(coor_data)
			updateflag=False
			
			rate.sleep()
				
