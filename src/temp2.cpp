#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <map>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool fid_flag = false;
bool next_target_flag = true;
bool start_follower_flag = false;
bool follower_next_target_flag = true;
std::vector<std::array<double,4>> follower_target_vector;
int32_t fid;
std::map<int32_t,std::array<double,4>> my_follower_targets;

void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_transform) {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";

  transformStamped.transform.translation.x = fid_transform->transforms[0].transform.translation.x;
  transformStamped.transform.translation.y = fid_transform->transforms[0].transform.translation.y;
  transformStamped.transform.translation.z = fid_transform->transforms[0].transform.translation.z;
  transformStamped.transform.rotation.x = fid_transform->transforms[0].transform.rotation.x;
  transformStamped.transform.rotation.y = fid_transform->transforms[0].transform.rotation.y;
  transformStamped.transform.rotation.z = fid_transform->transforms[0].transform.rotation.z;
  transformStamped.transform.rotation.w = fid_transform->transforms[0].transform.rotation.w;
  ROS_INFO_STREAM("Broadcasting");
  fid = fid_transform->transforms[0].fiducial_id;
  br.sendTransform(transformStamped);
}

void listen(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0));
    std::array<double,4> trans;
    trans[0] = transformStamped.transform.translation.x;
    trans[1] = transformStamped.transform.translation.y;
    trans[2] = transformStamped.transform.translation.z;
    trans[3] = transformStamped.transform.rotation.w;
    follower_target_vector.push_back(trans);
    ROS_INFO_STREAM("Position in map frame: ["
      << trans[0] << ","
      << trans[1] << ","
      << trans[2] << "]"
    );
    my_follower_targets.insert(make_pair(fid,trans));
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void rotate_robot(ros::Publisher& pub_cmd_vel){
  geometry_msgs::Twist msg;
  msg.angular.z = 0.1;
  msg.linear.x = 0.0;
  while(true){
    ros::spinOnce();
    if(fid_flag){
      fid_flag = false;
      ROS_INFO_STREAM("Breaking Rotate Bot");
      break;
    }
    pub_cmd_vel.publish(msg);
    ros::Duration(0.1).sleep();
  }
}

void fid_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_transform){
  if(!fid_transform->transforms.empty()){
      ROS_INFO_STREAM("FID: "<<fid_transform->transforms[0].fiducial_id);
      fid_flag = true;
      ROS_INFO_STREAM("GOT FID AND EXITING THE ROTATE BOT FUNCTION");
      broadcast(fid_transform);
  }
}
void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& explorer_amcl){
  
}

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 10);
  ros::Subscriber sub_fid = nh.subscribe("/fiducial_transforms", 1, &fid_callback);
  ros::Subscriber sub_amcl = nh.subscribe("/explorer/amcl_pose", 1, &amcl_callback);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;
  std::array<XmlRpc::XmlRpcValue,4> targets;
  XmlRpc::XmlRpcValue my_target1;
  XmlRpc::XmlRpcValue my_target2;
  XmlRpc::XmlRpcValue my_target3;
  XmlRpc::XmlRpcValue my_target4;
  nh.getParam("/aruco_lookup_locations/target_1",targets[0]);
  nh.getParam("/aruco_lookup_locations/target_2",targets[1]);
  nh.getParam("/aruco_lookup_locations/target_3",targets[2]);
  nh.getParam("/aruco_lookup_locations/target_4",targets[3]);

  // explorer_client.waitForResult();

  // ROS_INFO("Sending goal");
  // follower_client.sendGoal(follower_goal);
  // explorer_client.waitForResult();

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);
  unsigned int i=0;
  unsigned int j=0;
  while (ros::ok()) {
    if (next_target_flag){
      explorer_goal.target_pose.header.frame_id = "map";
      explorer_goal.target_pose.header.stamp = ros::Time::now();
      explorer_goal.target_pose.pose.position.x = targets[i][0];//
      explorer_goal.target_pose.pose.position.y = targets[i][1];//
      explorer_goal.target_pose.pose.orientation.w = 1.0;
      explorer_client.sendGoal(explorer_goal);
      ROS_INFO("Sending goal for explorer");
      explorer_client.waitForResult();
      ROS_INFO("Hooray, follower robot reached goal");
      rotate_robot(pub_cmd_vel);
      ros::Duration(1.0).sleep();
      listen(tfBuffer);
      if(i<3){
        i+=1;
      }
      else{
        explorer_goal.target_pose.header.frame_id = "map";
        explorer_goal.target_pose.header.stamp = ros::Time::now();
        explorer_goal.target_pose.pose.position.x = -4;//
        explorer_goal.target_pose.pose.position.y = 2.5;//
        explorer_goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(explorer_goal);//this should be sent only once
        explorer_goal_sent = true;
        explorer_client.waitForResult();
        next_target_flag = false;
        start_follower_flag = true;
        ROS_INFO_STREAM(follower_target_vector.size()<<" is the size of the vector");
        next_target_flag = false;
      }
    }
    if(start_follower_flag){
      // auto goal = follower_target_vector.at(0);
      // ROS_INFO_STREAM("MAP CHECKING.....: "<<my_follower_targets[j].at(0)<<", "<<my_follower_targets[j].at(1)<<", "<<my_follower_targets[j].at(2)<<", "<<my_follower_targets[j].at(3));
      ROS_INFO_STREAM("MAP FINDING...: " << my_follower_targets.find(j)->second.at(0) << ", " <<my_follower_targets.find(j)->second.at(1) << ", " <<my_follower_targets.find(j)->second.at(2) << ", " <<my_follower_targets.find(j)->second.at(3));
      follower_goal.target_pose.header.frame_id = "map";
      follower_goal.target_pose.header.stamp = ros::Time::now();
      follower_goal.target_pose.pose.position.x = follower_target_vector.at(j)[0]-0.314;//
      follower_goal.target_pose.pose.position.y = follower_target_vector.at(j)[1]-0.314;//
      follower_goal.target_pose.pose.position.z = follower_target_vector.at(j)[2];
      follower_goal.target_pose.pose.orientation.w = follower_target_vector.at(j)[3];
      ROS_INFO_STREAM("SENDING GOAL TO FOLLOWER");
      follower_client.sendGoal(follower_goal);
      follower_client.waitForResult();
      ROS_INFO("Hooray, follower robot reached goal");
      if(j<3){
        j+=1;
      }
      else{
        start_follower_flag = false;
        follower_goal.target_pose.header.frame_id = "map";
        follower_goal.target_pose.header.stamp = ros::Time::now();
        follower_goal.target_pose.pose.position.x = -4;//
        follower_goal.target_pose.pose.position.y = 3.5;//
        follower_goal.target_pose.pose.position.z = 1.0;
        follower_goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO_STREAM("SENDING GOAL TO FOLLOWER");
        follower_client.sendGoal(follower_goal);
        follower_client.waitForResult();
        ROS_INFO("Hooray, follower robot reached goal!!! Exiting ROS");
        ros::shutdown();
      }
    }
    loop_rate.sleep();
  }


}