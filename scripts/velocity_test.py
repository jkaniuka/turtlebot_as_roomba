#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
import time

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/simple_navigation_goals/roomba_commands', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    msg = String()
    target_room = raw_input("Room to clean: ")
    msg.data = target_room
    send = False
    while not send:
        if pub.get_num_connections() > 0:
            pub.publish(msg)
            send = True
        else:
            rate.sleep()
    time.sleep(5)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
