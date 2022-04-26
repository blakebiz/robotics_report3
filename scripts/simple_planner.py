#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from tf.transformations import *

from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, UInt8


sphere_params = SphereParams()
toolpose = None
received_sphere_params = False
rqt_toggle = False
pause_toggle = False


# /sphere_params subscriber
def get_sphere_params(data: SphereParams):
	global sphere_params
	global received_sphere_params
	sphere_params = data
	received_sphere_params = True
	
# /ur5e/toolpose subscriber
def get_toolpose(data):
	global toolpose
	toolpose = data

# /rqt_toggle
def rqt_listener(data):
	global rqt
	rqt_toggle = data.data

# /pause_toggle
def pause_listener(data):
	global pause
	pause_toggle = data.data


def setup_point(*data):
	# adds given lx, ly, lz, ax, ay, az to a given plan and sets the mode to value given
	# params: (lx, ly, lz, ax, ay, az, plan, mode)
	plan_point = Twist()
	point_mode = UInt8()
	plan_point.linear.x, plan_point.linear.y, plan_point.linear.z, plan_point.angular.x, plan_point.angular.y, plan_point.angular.z, plan, point_mode.data = data
	plan.points.append(plan_point)
	plan.modes.append(point_mode)
	



if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a subscriber to listen for the sphere parameters
	sphere_sub = rospy.Subscriber("/sphere_params", SphereParams, get_sphere_params)
	tool_sub = rospy.Subscriber("/ur5e/toolpose", Twist, get_toolpose)
	# add subscribers for cancelling plan (rqt) or pausing movement (pause)
	rqt_sub = rospy.Subscriber("/rqt_toggle", Bool, rqt_listener)
	pause_sub = rospy.Subscriber("/pause_toggle", Bool, rqt_listener)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	

	initial_toolpose = None
	q_rot = Quaternion()
	while not rospy.is_shutdown():
		if initial_toolpose is None:
			initial_toolpose = toolpose
		# ensure we have received parameters to go off of and toolpose has been received
		if received_sphere_params and initial_toolpose is not None:
			
			# try getting the most update transformation between the tool frame and the base frame
			try:
				trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Frames not available!!!')
				loop_rate.sleep()
				continue
			# extract the xyz coordinates
			x = trans.transform.translation.x
			y = trans.transform.translation.y
			z = trans.transform.translation.z
			
			# define a testpoint in the tool frame
			pt_in_camera = PointStamped()
			pt_in_camera.header.frame_id = 'camera_color_optical_frame'
			pt_in_camera.header.stamp = rospy.get_rostime()
			
			pt_in_camera.point.x = sphere_params.xc
			pt_in_camera.point.y = sphere_params.yc
			pt_in_camera.point.z = sphere_params.zc
			
			# convert the 3D point to the base frame coordinates
			pt_in_base = tfBuffer.transform(pt_in_camera,'base', rospy.Duration(1.0))
			x, y, z, rad = pt_in_base.point.x, pt_in_base.point.y, pt_in_base.point.z, sphere_params.radius
			plan = Plan()
			# set singular rpy for keeping the arm in the same position
			roll, pitch, yaw = initial_toolpose.angular.x, initial_toolpose.angular.y, initial_toolpose.angular.z
			# add robot starting position as initial point
			setup_point(initial_toolpose.linear.x, initial_toolpose.linear.y, initial_toolpose.linear.z, roll, pitch, yaw, plan, 0)
			# add rest of points in plan
			y_offset = -0.01
			z_offset = 0.02
			setup_point(x, y+y_offset, z + (rad*2), roll, pitch, yaw, plan, 0)
			setup_point(x, y+y_offset, z + z_offset, roll, pitch, yaw, plan, 0)
			setup_point(x, y+y_offset, z + z_offset, roll, pitch, yaw, plan, 2)
			setup_point(x, y, z + (rad*2), roll, pitch, yaw, plan, 0)
			setup_point(x + (rad*4), y, z + (rad*2), roll, pitch, yaw, plan, 0)
			setup_point(x + (rad*4), y, z + z_offset, roll, pitch, yaw, plan, 0)
			setup_point(x + (rad*4), y, z + z_offset, roll, pitch, yaw, plan, 1)
			setup_point(x + (rad*4), y, z + (rad*2), roll, pitch, yaw, plan, 0)
			# publish the plan
			if not rqt_toggle:
				plan_pub.publish(plan)
			if pause_toggle:
				plan_pub.publish(Plan())
			# wait for 0.1 seconds until the next loop and repeat
			loop_rate.sleep()
