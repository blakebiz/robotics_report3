#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('manual_unpause', anonymous = True)
	toggle_pub = rospy.Publisher('/pause_toggle', Bool, queue_size=1)
	toggle_pub.publish(Bool(False))
