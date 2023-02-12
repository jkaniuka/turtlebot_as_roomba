#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import math
import tf
import tf2_ros
import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import BatteryState
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

cmd_publisher = rospy.Publisher('/simple_navigation_goals/roomba_commands', String, queue_size=10)

global docking_pos
global docking_ori
docking_pos = [0, 0, 0]
docking_ori = [0, 0, 0, 0]

global done_once
done_once = 0

global stop_now
stop_now = 0

global cleaning_finished
cleaning_finished = 0

global obstacle_detected
obstacle_detected = 0


class Search_for_docking_station(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.listener = tf.TransformListener()
        self.parent_frame = "odom"
        self.child_frame = "symbol_power"

    def execute(self, userdata):
        time.sleep(5) # for smach viewer
        global docking_pos
        global docking_ori

        time.sleep(5)
        try:
            self.listener.waitForTransform(self.parent_frame, self.child_frame, rospy.Time(0), rospy.Duration(30.0))
            (trans,rot) = self.listener.lookupTransform(self.parent_frame, self.child_frame, rospy.Time(0))
            rospy.loginfo('Getting docking station localization')
            docking_pos = [trans[0] - 0.5, trans[1], 0]
            docking_ori = [0, 0, 1, 0]
            return 'succeeded'
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
            rospy.loginfo('Timeout - there is no docking station sign on the wall')
            return 'aborted'


class Charge_batteries(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        global docking_pos
        global docking_ori
        rospy.loginfo('Docking in order to charge batteries')
        time.sleep(5) # for smach viewer

        client.wait_for_server()
        goal = create_goal_msg(docking_pos[0], docking_pos[1], docking_ori[2], docking_ori[3])
        client.send_goal(goal)

        finished_within_time = client.wait_for_result(rospy.Duration(20)) 
        print(finished_within_time)
        
        if not finished_within_time:
            client.cancel_goal()
            rospy.loginfo("Timeout achieving docking station")
            return 'aborted'
        else:
            state = client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!") 
        counter = 0
        while counter<15:
            print("Charging in progress...")
            time.sleep(1)
            counter = counter + 1
        return 'succeeded'

class Undock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UNDOCK')
        time.sleep(5) # for smach viewer
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        msg = Twist()
        msg.linear.x = 0.3
        publish_msg_once(msg, pub)
        time.sleep(5)
        msg = Twist()
        msg.linear.x = 0.0
        publish_msg_once(msg, pub)
        return 'succeeded'


class Clean_rooms(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

    def execute(self, userdata):
        global done_once
        global cleaning_finished
        if done_once == 0:
            rospy.loginfo('Executing state CLEAN_ROOMS')
            time.sleep(5) # for smach viewer
            print("Select room to clean from: office, cafeteria, dumpster, kitchen, dressing_room, bathroom")
            target_room = raw_input("Room to clean: ")
            msg = String()
            msg.data = target_room
            publish_msg_once(msg, cmd_publisher )
            done_once = 1
        while True:
            if (stop_now == 1):
                global docking_pos
                global docking_ori
                rospy.loginfo('Critical LOW BATTERY')
                rospy.loginfo('Returning to base')
                time.sleep(5) # for smach viewer

                client.wait_for_server()
                goal = create_goal_msg(docking_pos[0], docking_pos[1], docking_ori[2], docking_ori[3])
                client.send_goal(goal)

                finished_within_time = client.wait_for_result(rospy.Duration(300)) 
                print(finished_within_time)
                
                if not finished_within_time:
                    client.cancel_goal()
                    rospy.loginfo("Timeout")
                else:
                    state = client.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!") 
                return 'aborted'

            if (cleaning_finished == 1):
                print("Select room to clean from: office, cafeteria, dumpster, kitchen, dressing_room, bathroom or BREAK")
                target_room = raw_input("Room to clean: ")
                if (target_room == "BREAK"):
                    rospy.loginfo('No more rooms to clean')
                    return 'succeeded'
                else:
                    rospy.loginfo('Next room will be cleaned')
                    msg = String()
                    msg.data = target_room
                    publish_msg_once(msg, cmd_publisher )
                    cleaning_finished = 0
            if (obstacle_detected == 1):
                return 'preempted'
            

class Avoid_obstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        global obstacle_detected
        rospy.loginfo('Executing state AVOID_OBSTACLE')
        time.sleep(5) # for smach viewer
        rospy.loginfo('******************************')
        rospy.loginfo('Obstacle detected, local plan will be modified to avoid collision')
        rospy.loginfo('******************************')
        obstacle_detected = 0
        return 'succeeded'

class Docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        global docking_pos
        global docking_ori
        rospy.loginfo('Moving back to base')
        time.sleep(5) # for smach viewer

        client.wait_for_server()
        goal = create_goal_msg(docking_pos[0], docking_pos[1], docking_ori[2], docking_ori[3])
        client.send_goal(goal)

        finished_within_time = client.wait_for_result(rospy.Duration(300)) 
        print(finished_within_time)
        
        if not finished_within_time:
            client.cancel_goal()
            rospy.loginfo("Path is blocked, can't reach")
            return 'aborted'
        else:
            state = client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!") 
        return 'succeeded'       
              

def battery_callback(data):
    global stop_now
    rospy.loginfo(data.voltage)
    if data.voltage < 5.0:
        client.cancel_all_goals()
        msg = String()
        msg.data = "STOP"
        stop_now = 1
        publish_msg_once(msg, cmd_publisher)

def roomba_status_callback(msg):
    global cleaning_finished
    global obstacle_detected
    rospy.loginfo(msg.data)
    received = msg.data
    if(received == "FINISHED"):
        cleaning_finished = 1
    if(received == "OBSTACLE"):
        obstacle_detected = 1

# main
def main():
    rospy.init_node('ima_smach_scenario')
    rospy.Subscriber("/roomba_battery", BatteryState, battery_callback)
    rospy.Subscriber("/simple_navigation_goals/roomba_cleaning_status", String, roomba_status_callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['missing_docking_sign', 'path_is_blocked', 'low_battery', 'task_done'])

    # Open the container
    with sm:
        smach.StateMachine.add('SEARCH_FOR_DOCKING_STATION', Search_for_docking_station(), 
                               transitions={'succeeded':'CHARGE_BATTERIES', 
                                            'aborted':'missing_docking_sign'})

        smach.StateMachine.add('CHARGE_BATTERIES', Charge_batteries(), 
                               transitions={'succeeded':'UNDOCK',
                                            'aborted':'path_is_blocked'})

        smach.StateMachine.add('UNDOCK', Undock(), 
                               transitions={'succeeded':'CLEAN_ROOMS'})

        smach.StateMachine.add('CLEAN_ROOMS', Clean_rooms(), 
                               transitions={'succeeded':'DOCKING', 
                                            'aborted':'low_battery',
                                            'preempted':'AVOID_OBSTACLE'})

        smach.StateMachine.add('AVOID_OBSTACLE', Avoid_obstacle(), 
                               transitions={'succeeded':'CLEAN_ROOMS'})

        smach.StateMachine.add('DOCKING', Docking(), 
                               transitions={'succeeded':'task_done', 
                                            'aborted':'path_is_blocked'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

def create_goal_msg(x, y, z, w):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w
    return goal

def publish_msg_once(message, publisher):
    rate = rospy.Rate(10) # 10hz
    send = False
    while not send:
        if publisher.get_num_connections() > 0:
            publisher.publish(message)
            send = True
        else:
            rate.sleep()



if __name__ == '__main__':
    main()