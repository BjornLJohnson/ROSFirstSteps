#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publisher():
	pub = rospy.Publisher('random_messages', String, queue_size = 10)
	rospy.init_node('publisher', anonymous=True)
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		message = "Random message %s" % rospy.get_time()
		rospy.loginfo(message)
		pub.publish(message)
		rate.sleep()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
