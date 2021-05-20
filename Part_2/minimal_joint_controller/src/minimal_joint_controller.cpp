#include <ros/ros.h> //ALWAYS need to include this

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>

#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>

//some "magic number" global params:
const double Kp = 10.0; //controller gains
const double Kv = 3;
const double dt = 0.01;

// saturation function; provide saturation threshold, sat_val, and arg to be saturated, val
double sat(double val, double sat_val) {
    if (val > sat_val)
        return (sat_val);
    if (val< -sat_val)
        return (-sat_val);
    return val;

}
// I.e. if angle is 181 set to -179
double min_periodicity(double theta_val) {
    double periodic_val = theta_val;

    while (periodic_val > M_PI) {
        periodic_val -= 2 * M_PI;
    }

    while (periodic_val< -M_PI) {
        periodic_val += 2 * M_PI;
    }
    return periodic_val;
}

double g_pos_cmd = 0.0; //position command input-- global var

void posCmdCB(const std_msgs::Float64& pos_cmd_msg) {
    ROS_INFO("received value of pos_cmd is: %f", pos_cmd_msg.data);
    g_pos_cmd = pos_cmd_msg.data;
}

// Check if 2 gazebo services exist before calling them
bool test_services() {

  bool service_ready = false;

  if (!ros::service::exists("/gazebo/apply_joint_effort", true)) {
    ROS_WARN("waiting for apply_joint_effort service");
    return false;
  }

  if (!ros::service::exists("/gazebo/get_joint_properties", true)) {
    ROS_WARN("waiting for /gazebo/get_joint_properties service");
    return false;
  }
  
  ROS_INFO("services are ready");
  return true;
}

int main(int argc, char **argv) {

    //initializations:
    ros::init(argc, argv, "minimal_joint_controller");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);

    // Test services 0.5 secs before attempting to proceed, else node will crash
    while (!test_services()) {
        ros::spinOnce();
        half_sec.sleep();
    }
    
    // Create clients to apply (i) joint torques and (ii) get joint data
    ros::ServiceClient set_trq_client =
            nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    ros::ServiceClient get_jnt_state_client =
            nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg;

    // Publish all info to facilitate plotting and analysis
    ros::Publisher trq_publisher         = nh.advertise<std_msgs::Float64>("jnt_trq", 1);
    ros::Publisher vel_publisher         = nh.advertise<std_msgs::Float64>("jnt_vel", 1);
    ros::Publisher pos_publisher         = nh.advertise<std_msgs::Float64>("jnt_pos", 1);
    ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    // Subscribe to get joint angles, which will be saved to g_pos_cmmd
    ros::Subscriber pos_cmd_subscriber = nh.subscribe("pos_cmd", 1, posCmdCB);

    // Get data from service then publish
    std_msgs::Float64 trq_msg, q1_msg, q1dot_msg;
    double q1, q1dot, q1_err, trq_cmd;

    // JointState msg: rosmsg show sensor_msgs/JointState
    sensor_msgs::JointState joint_state_msg;

    ros::Duration duration(dt);
    ros::Rate rate_timer(1 / dt);

    // Prepare gazebo apply_joint_effort request
    effort_cmd_srv_msg.request.joint_name = "joint1";
    effort_cmd_srv_msg.request.effort = 0.0;
    effort_cmd_srv_msg.request.duration = duration;

    // prepare gazebo get_joint_state request
    // Check srv msg: rossrv show gazebo_msgs/GetJointProperties)
    get_joint_state_srv_msg.request.joint_name = "joint1";


    // set up the joint_state_msg fields to define a single joint,
    // called joint1, and initial position and vel values of 0
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.push_back("joint1");
    joint_state_msg.position.push_back(0.0);
    joint_state_msg.velocity.push_back(0.0);
    
    // here is the main controller loop:
    while (ros::ok()) {

	// Get Position & publish
        get_jnt_state_client.call(get_joint_state_srv_msg);
        q1 = get_joint_state_srv_msg.response.position[0];
        q1_msg.data = q1;
        pos_publisher.publish(q1_msg); //republish his val on topic jnt_pos

	// Get Velocity & publish
        q1dot = get_joint_state_srv_msg.response.rate[0];
        q1dot_msg.data = q1dot;
        vel_publisher.publish(q1dot_msg);

	// Fill in the joint state message [0] & publish
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.position[0] = q1;
        joint_state_msg.velocity[0] = q1dot;
        joint_state_publisher.publish(joint_state_msg);

	// Compute error between command and actual value. Command set by subscribe
        q1_err = min_periodicity(g_pos_cmd - q1); //jnt angle err; watch for periodicity

	// Compute Controller Update and publish
        trq_cmd = Kp * (q1_err) - Kv*q1dot;
        trq_msg.data = trq_cmd;
        trq_publisher.publish(trq_msg);

	// Apply torque to actuator in gazebo
        effort_cmd_srv_msg.request.effort = trq_cmd; 
        set_trq_client.call(effort_cmd_srv_msg);
	
        // make sure service call was successful
        bool result = effort_cmd_srv_msg.response.success;
        if (!result)
            ROS_WARN("service call to apply_joint_effort failed!");

	// Repeat all these commands once per time step
	ros::spinOnce();
        rate_timer.sleep();
    }
}
