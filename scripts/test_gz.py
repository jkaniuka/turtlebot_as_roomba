#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def add_two_ints_client():
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        add_two_ints = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp1 = add_two_ints("test_cube", open("/home/jkaniuka/piar_ws/object.urdf",'r').read(), "/rover", Pose(position= Point(-1-2,3,-1),orientation=Quaternion(0,0,0,0)),"map")
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print(add_two_ints_client())


# import rospy
# from gazebo_msgs.srv import DeleteModel
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Quaternion

# def add_two_ints_client():
#     rospy.wait_for_service('/gazebo/delete_model')
#     try:
#         add_two_ints = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
#         resp1 = add_two_ints("test_cube")
#         return resp1.success
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)


# if __name__ == "__main__":
#     print(add_two_ints_client())
