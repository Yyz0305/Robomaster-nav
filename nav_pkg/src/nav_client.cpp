#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc,char** argv){
    ros::init(argc,argv,"nav_client");
    MoveBaseClient ac("move_base",true);

    while(!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("等待Move_base启动");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 4;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();
    if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_WARN("succeed");
    }
        
    else
        ROS_WARN("Mission failed");

    return 0;
}