// example_action_server: a simple action server
// Wyatt Newman

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<example_action_server/demoAction.h>

int g_count = 0;
bool g_count_failure = false;

class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    actionlib::SimpleActionServer<example_action_server::demoAction> as_;
    
    // here are some message types to communicate with our client(s)
    example_action_server::demoGoal goal_; // goal message, received from client
    example_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    example_action_server::demoFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<example_action_server::demoAction>::GoalConstPtr& goal);
};

ExampleActionServer::ExampleActionServer() :
   as_(nh_, "example_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 

{
    ROS_INFO("in constructor of exampleActionServer...");
    as_.start(); //start the server running
}

void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<example_action_server::demoAction>::GoalConstPtr& goal) {

    g_count++; // keep track of total number of goals serviced since this server was started
    result_.output = g_count; // we'll use the member variable result_, defined in our class
    result_.goal_stamp = goal->input;
    
    if (g_count != goal->input) {
        ROS_WARN("hey--mismatch!");
        ROS_INFO("g_count = %d; goal_stamp = %d", g_count, result_.goal_stamp);
        g_count_failure = true; //set a flag to commit suicide
        ROS_WARN("informing client of aborted goal");
        as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
    }
    else {
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_server_node"); // name this node 

    ROS_INFO("instantiating the demo action server: ");

    ExampleActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");

    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (!g_count_failure && ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
    }

    return 0;
}

