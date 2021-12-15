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
/**
 * @brief Urban Search and Rescue class to drive two turtlebots. One turtlebot to explore the ArUco markers and another turtlebot to
 * follow the markers in ascending order of their IDs.
 * 
 */

class Urban_Search_And_Rescue{
    
    public:

    /**
     * @brief Construct a new Urban_Search_And_Rescue object
     * 
     * @param nh node handle
     */
    Urban_Search_And_Rescue(ros::NodeHandle *nh);

    /**
     * @brief Get the four targets from rosparam server
     * 
     */
    void get_targets();

    /**
     * @brief This function starts the search operation. The explorer robot goes to the target locations and checks for the aruco markers and updates 
     * its position to the follower robot by broadcasting the aruco frame relative to its camera to the world map frame.
     * 
     */
    void search();

    /**
     * @brief This function starts the rescue operation. The follower robot goes to the detected aruco marker frames one by one in the ascending 
     * order of its fiducial id. The follower will go near to the target location with 0.4 meters tolerance.
     * 
     * 
     */
    void rescue();

    private:

    bool m_fid_flag; //fiducial id detection flag

    bool m_next_target_flag; //flag to check whether we can go for next target for explorer or not

    bool m_start_follower_flag; //flag to start the follower

    bool m_follower_next_target_flag; //flag to check whether we can go for the next target of follower

    bool m_search_flag; //flag is true when we are in search operation

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    int32_t m_fid; //stores the fiducial id of the detected aruco marker

    std::map<int32_t,std::array<double,7>> m_my_follower_targets; //map that stores the fiducial id and the marker_frame pose as the key value pair

    ros::NodeHandle m_nh; //node handle

    ros::Publisher m_pub_cmd_vel; //command velocity publisher for rotating the explorer

    ros::Subscriber m_sub_fid; //subscriber for fiducial_transforms topic

    MoveBaseClient m_explorer_client; //explorer action client

    MoveBaseClient m_follower_client; //follower action client

    ros::Rate m_loop_rate; //loop rate in hz.

    move_base_msgs::MoveBaseGoal m_explorer_goal; //explorer goal pose - stores the next goal of explorer.

    move_base_msgs::MoveBaseGoal m_follower_goal; //follower goal pose - stores the next goal of the follower.

    std::array<XmlRpc::XmlRpcValue,4> m_targets; //array that stores the target locations from the rosparam server

    tf2_ros::Buffer m_tfBuffer; //rostf2 buffer object

    tf2_ros::TransformListener m_tfListener; //transform listener object

    /**
     * @brief This function initializes the publisher that is required for the application.
     * 
     */
    void m_init_pubs();

    /**
     * @brief This function initializes the subscriber that is required for the application.
     * 
     */
    void m_init_subs();

    /**
     * @brief This is a callback function for fiducial_transforms topic subscriber
     * 
     * @param fid_transform constant reference to fiducial_msgs::FiducialTransformArray::ConstPtr&
     */
    void m_fid_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_transform);

    /**
     * @brief This function is called when there is a need to rotate the robot with angular velocity 0.1
     * 
     * @param pub_cmd_vel reference to ros::Publisher
     */
    void m_rotate_robot(ros::Publisher& pub_cmd_vel);

    /**
     * @brief This function listens to the frame broadcasted by the broadcaster and gets its transformation with map frame. 
     * 
     * @param tfBuffer reference to tf2_ros::Buffer
     */
    void m_listen(tf2_ros::Buffer& tfBuffer);

    /**
     * @brief THis function broadcasts the fiducial_transform frame to the tf tree map frame.
     * 
     * @param fid_transform constant reference to fiducial_msgs::FiducialTransformArray::ConstPtr
     */
    void m_broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_transform);

    /**
     * @brief This function calculates the yaw angle of the aruco marker (its orientation with respect to the map frame)
     * 
     * @param follower_quat reference to geometry_msgs::Quaternion
     * @return double yaw angle
     */
    double m_calculate_yaw(geometry_msgs::Quaternion& follower_quat);
    };
#endif