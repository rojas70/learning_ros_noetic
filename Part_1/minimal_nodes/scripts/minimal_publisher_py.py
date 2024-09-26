#!/usr/bin/python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def talker():
    # Init node
    rospy.init_node('minimal_publisher', anonymous=True)

    # Create pub object
    pub = rospy.Publisher('topic1', Float64, queue_size=10)

    # Create a float data variable
    count = Float64()
    count = 0.0

    # Set a rate for your while loop in ROS at 10 Hz
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        # Change your message
        count = count + 0.001

        # Publish your object
        pub.publish(count)

        # Sets sleep time for 1/10 secs
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
