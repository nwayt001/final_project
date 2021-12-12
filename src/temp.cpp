
/*
 * Publish randomly-generated velocity Messages to /cmd_vel Topic.
 */
#include <geometry_msgs/PoseStamped.h>//for geometry_msgs::PoseStamped
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdlib.h>             //for rand() and RAND_MAX
nav_msgs::Odometry explorer_odom;
void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    // ROS_INFO("I received odom: [%f,%f,%f]",
    //          odom->pose.pose.position.x,
    //          odom->pose.pose.position.y,
    //          odom->pose.pose.position.z); //store x,y,z position values
    
    ROS_INFO_STREAM("Pose of the robot: " 
    << "[" << odom->pose.pose.position.x 
    << "," 
    << odom->pose.pose.position.y 
    << "," 
    << odom->pose.pose.position.z
    << "]");
    explorer_odom.pose.pose.position.x = odom->pose.pose.position.x;
    explorer_odom.pose.pose.position.y = odom->pose.pose.position.y;
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "my_pub");

  ros::NodeHandle nh;

  ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/explorer/move_base_simple/goal", 10);
  ros::Subscriber sub_odom = nh.subscribe("/explorer/odom", 1000, &OdomCallback);
  //seed the random number generator.
  srand(time(0));

  XmlRpc::XmlRpcValue my_target1;
  XmlRpc::XmlRpcValue my_target2;
  XmlRpc::XmlRpcValue my_target3;
  XmlRpc::XmlRpcValue my_target4;
  nh.getParam("/aruco_lookup_locations/target_1",my_target1);
  nh.getParam("/aruco_lookup_locations/target_2",my_target2);
  nh.getParam("/aruco_lookup_locations/target_3",my_target3);
  nh.getParam("/aruco_lookup_locations/target_4",my_target4);
  geometry_msgs::PoseStamped msg;
    msg.header.seq = 1;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = my_target2[0];
    msg.pose.position.y = my_target2[1];
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    ROS_INFO_STREAM(static_cast<double>(my_target2[0]));
    ROS_INFO_STREAM(my_target2[1]);
  ROS_INFO("Before Ros OK Down");
  // pub_goal.publish(msg);
  while (ros::ok())
  {
    pub_goal.publish(msg);
    ros::spinOnce();
  }
  return 0;
}
