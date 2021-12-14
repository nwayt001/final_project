#ifndef USAR_H
#define USAR_H
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
#include <tf/transform_datatypes.h>
#include <cmath>

class Urban_Search_And_Rescue{
    public:
    Urban_Search_And_Rescue(ros::NodeHandle *nh);
    void get_targets();
    void search();
    void rescue();

    private:
    bool m_fid_flag;
    bool m_next_target_flag;
    bool m_start_follower_flag;
    bool m_follower_next_target_flag;
    bool m_search_flag;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    std::vector<std::array<double,7>> m_follower_target_vector;
    int32_t m_fid;
    std::map<int32_t,std::array<double,7>> m_my_follower_targets;
    std::array<double,2> m_explorer_curr_pose;
    ros::NodeHandle m_nh;
    ros::Publisher m_pub_cmd_vel;
    ros::Subscriber m_sub_fid;
    MoveBaseClient m_explorer_client;
    MoveBaseClient m_follower_client;
    ros::Rate m_loop_rate;


    move_base_msgs::MoveBaseGoal m_explorer_goal;
    move_base_msgs::MoveBaseGoal m_follower_goal;
    std::array<XmlRpc::XmlRpcValue,4> m_targets;

    bool m_explorer_goal_sent;
    bool m_follower_goal_sent;
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;

    void m_init_pubs();
    void m_init_subs();

    void m_fid_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_transform);
    void m_rotate_robot(ros::Publisher& pub_cmd_vel);
    void m_listen(tf2_ros::Buffer& tfBuffer);
    void m_broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_transform);
    };
#endif