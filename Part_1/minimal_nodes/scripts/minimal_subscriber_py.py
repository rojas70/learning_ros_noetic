#!/usr/bin/env python3

import rospy
import ipdb
from std_msgs.msg import Float64

# Global variable to store the useful value
useful_var = None

class FloatProcessor:
    def __init__(self):
        global useful_var  # Access the global variable
        rospy.init_node('float_processor', anonymous=True)

        # Create a publisher for the output topic
        self.pub = rospy.Publisher('output_topic', Float64, queue_size=10)

        # Subscribe to the input topic
        rospy.Subscriber('topic1', Float64, self.callback)

    def callback(self, msg):
        ipdb.set_trace()

        # Declare that we're using the global variable
        global useful_var
        ctr = 1

        # Print received message
        rospy.loginfo(f"Received: {msg.data}")

        # Store the received value in the global variable
        useful_var = msg.data/msg.data + ctr

        # Process the value and publish it
        self.publish_processed_value()

        # Increase ctr
        ctr += 1

    def publish_processed_value(self):

        # Declare that we're using the global variable
        global useful_var

        # Process once per subsription
        if useful_var is not None:

            # As if doing something useful with this variable
            processed_value = useful_var * 2  # Multiply by 2
            rospy.loginfo(f"Publishing processed value: {processed_value}")

            # Create a Float64 message and publish it
            self.pub.publish(processed_value)

    def run(self):
        # Spin to keep the node running and processing callbacks
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = FloatProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
