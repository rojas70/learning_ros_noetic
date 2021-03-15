#!/usr/bin/python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64


class _minimal_controller:
    def __init__(self):
        self.g_force=0.0
        self.g_velocity=0.0
        self.g_vel_cmd=0.0
    def g_vel_cmd_Callback(self,msg):#target
        rospy.loginfo("received velocity command(vel_cmd) value is: %f", msg.data)
        self.g_vel_cmd=msg.data
    def g_velocity__Callback(self,msg):#real time
        rospy.loginfo("received velocity value is: %f", msg.data)
        self.g_velocity=msg.data
def main():
    _mi=_minimal_controller()
    force_cmd__pub = rospy.Publisher('force_cmd', Float64, queue_size=10)
    g_vel_cmd_sub = rospy.Subscriber("vel_cmd", Float64, _mi.g_vel_cmd_Callback)
    velocity_cmd_sub = rospy.Subscriber("velocity", Float64, _mi.g_velocity__Callback)
    rospy.init_node('minimal_controller_py', anonymous=True)
    Kv = 1
    dt_controller = 0.1 #10ms integration time step 
    sample_rate = 1.0 / dt_controller # compute the corresponding update frequency 
    rate = rospy.Rate(sample_rate) # 1hz
    vel_err=0.0
    while not rospy.is_shutdown():
        vel_err = _mi.g_vel_cmd - _mi.g_velocity#compute error btwn desired and actual 
        #velocities 
        _mi.g_force = Kv*vel_err #proportional-only velocity-error feedback defines commanded 
        #force 
        force_cmd__pub.publish(_mi.g_force) #publish the control effort computed by this 
        #controller 
        rospy.loginfo("force command = %f", _mi.g_force)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass