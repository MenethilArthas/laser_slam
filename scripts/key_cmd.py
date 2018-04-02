#!/usr/bin/env python
#moniter keyboard input and transmit it to PC
import rospy
import struct

from std_msgs.msg import String

import sys, select, termios, tty





def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = 'x'

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('transmit')
    rate=rospy.Rate(10)
    
    pub=rospy.Publisher('cmd_key',String,queue_size=100)

    try:
        while not rospy.is_shutdown():
            
            key=getKey()
            if(key=='\x03'):
                break    
            pub.publish(key)
            rate.sleep()
    except BaseException as e:
        print (e)
		

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

