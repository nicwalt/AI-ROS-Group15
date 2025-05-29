// gazebo_interface.cpp
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

ros::Publisher state_pub;
ros::Publisher move_pub;

void actionCallback(const std_msgs::Int32::ConstPtr& msg) {
    int action = msg->data;
    std_msgs::Int32MultiArray state_msg;
    
    switch(action) {
        case 0: // Collect survivor
            // Perform collection in Gazebo
            state_msg.data = {current_x, current_y, 1, 0};
            state_pub.publish(state_msg);
            break;
        case 1: // Home
            while (x != 0 || y != 0) {
                std_msgs::Int32MultiArray move_msg;
                move_msg.data = {next_x, next_y};
                move_pub.publish(move_msg);
            }
            state_msg.data = {0, 0, 0, 0};
            state_pub.publish(state_msg);
            break;
        case 2: // Explore
            // Implement exploration logic
            break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_interface");
    ros::NodeHandle nh;
    
    ros::Subscriber action_sub = nh.subscribe("/gazebo_actions", 10, actionCallback);
    state_pub = nh.advertise<std_msgs::Int32MultiArray>("/state_updates", 10);
    move_pub = nh.advertise<std_msgs::Int32MultiArray>("/position_updates", 10);
    
    ros::spin();
    return 0;
}