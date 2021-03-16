#!/usr/bin/python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64


class _minimal_simulator:
    def __init__(self):
        self.g_force=0.0
    def myCallback(self,msg):
        rospy.loginfo("received force value is: %f", msg.data)
        self.g_force=msg.data
def main():
    _mi=_minimal_simulator()
    vel_pub = rospy.Publisher('velocity', Float64, queue_size=10)
    force_sub = rospy.Subscriber("force_cmd", Float64, _mi.myCallback)
    rospy.init_node('minimal_simulator_py', anonymous=True)
    mass = 1.0
    dt = 0.01 #10ms integration time step 
    sample_rate = 1.0 / dt # compute the corresponding update frequency 
    rate = rospy.Rate(sample_rate) # 1hz
    g_velocity=0.0
    while not rospy.is_shutdown():
        g_velocity = g_velocity + (_mi.g_force / mass) * dt# Euler integration of 
        rospy.loginfo("velocity = %f", g_velocity)
        vel_pub.publish(g_velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass