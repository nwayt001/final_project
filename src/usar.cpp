#include "../include/usar.h"

Urban_Search_And_Rescue::Urban_Search_And_Rescue(ros::NodeHandle *nh) : 
    m_nh{*nh}, //initializing the node handle
    m_fid_flag{false}, //initializing the fiducial flag
    m_next_target_flag{true}, //initializing the next target flag for explorer
    m_start_follower_flag{false}, //initializing the start follower flag
    m_follower_next_target_flag{true}, //initializing the next target flag for follower 
    m_explorer_client{"/explorer/move_base", true}, //initializing the explorer movebase client
    m_follower_client{"/follower/move_base", true}, //initializing the follower movebase client
    m_tfBuffer{}, //initializing the tfBuffer
    m_tfListener{m_tfBuffer}, //initializing the transform listener
    m_loop_rate{10}, //loop rate of 10 Hz
    m_search_flag{true} //initializing the search flag
{
    m_init_pubs(); //calling the function to initialize the publisher
    m_init_subs(); //calling the function to initialize the subscriber
}

void Urban_Search_And_Rescue::m_init_pubs(){
    m_pub_cmd_vel = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 10); //publisher to the /explorer/cmd_vel topic
}

void Urban_Search_And_Rescue::m_init_subs(){
    m_sub_fid = m_nh.subscribe("/fiducial_transforms", 1, &Urban_Search_And_Rescue::m_fid_callback, this); //subscribing to the /fiducial_transforms topic
}

void Urban_Search_And_Rescue::get_targets(){
    m_nh.getParam("/aruco_lookup_locations/target_1", m_targets[0]);//gets target_1 location from rosparam server and stores in m_target
    m_nh.getParam("/aruco_lookup_locations/target_2", m_targets[1]);//gets target_2 location from rosparam server and stores in m_target
    m_nh.getParam("/aruco_lookup_locations/target_3", m_targets[2]);//gets target_3 location from rosparam server and stores in m_target
    m_nh.getParam("/aruco_lookup_locations/target_4", m_targets[3]);//gets target_4 location from rosparam server and stores in m_target
}

void Urban_Search_And_Rescue::m_listen(tf2_ros::Buffer &tfBuffer){
    geometry_msgs::TransformStamped transformStamped;
    try{ //for exception handling
        tfBuffer.canTransform("map", "marker_frame", ros::Time(0), ros::Duration(3.0)); //checks if we can transform from marker_frame to map
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0)); //gets the transform of marker_frame with respect to map frame
        std::array<double, 7> trans;
        //Storing the pose of marker frame
        trans[0] = transformStamped.transform.translation.x;
        trans[1] = transformStamped.transform.translation.y;
        trans[2] = transformStamped.transform.translation.z;
        trans[3] = transformStamped.transform.rotation.x;
        trans[4] = transformStamped.transform.rotation.y;
        trans[5] = transformStamped.transform.rotation.z;
        trans[6] = transformStamped.transform.rotation.w;
        ROS_INFO_STREAM("Position in map frame: ["
                        << trans[0] << ","
                        << trans[1] << ","
                        << trans[2] << "]");
        m_my_follower_targets.insert(make_pair(m_fid, trans)); //inserting the values with fiducial id as key and the pose of the marker as value
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Urban_Search_And_Rescue::m_broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr &fid_transform){
    //for broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    std::array<double, 2> fid_pose;

    //broadcast the marker_frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";
    transformStamped.transform.translation.x = fid_transform->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = fid_transform->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = fid_transform->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = fid_transform->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = fid_transform->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = fid_transform->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = fid_transform->transforms[0].transform.rotation.w;
    //calculate the distance between the camera and aruco frame. If it is greater than 3, then look for other marker nearby
    double fid_exp_dist = std::sqrt(std::pow((fid_transform->transforms[0].transform.translation.x), 2) + std::pow((fid_transform->transforms[0].transform.translation.y), 2));
    //displaying the position of aruco
    ROS_INFO_STREAM("x position of aruco: " << fid_transform->transforms[0].transform.translation.x << " y postion of aruco: " << fid_transform->transforms[0].transform.translation.y);
    if (fid_exp_dist >= 3.0){
        ROS_INFO_STREAM("Distance is: " << fid_exp_dist<<". Hence Ignoring this marker");
        m_fid_flag = false;
    }
    else{
        m_fid_flag = true;
        ROS_INFO_STREAM("Broadcasting");
        m_fid = fid_transform->transforms[0].fiducial_id;
        br.sendTransform(transformStamped); //broadcast the frame
        ros::Duration(1.0).sleep(); //sleep for one second
    }
}

void Urban_Search_And_Rescue::m_rotate_robot(ros::Publisher &pub_cmd_vel){
    geometry_msgs::Twist msg;
    msg.angular.z = 0.1;
    msg.linear.x = 0.0;
    while (true){
        ros::spinOnce(); //calls the call back functions
        if (m_fid_flag){
            m_fid_flag = false;
            // ROS_INFO_STREAM("Breaking Rotate Bot"); //Uncomment for debugging
            break;
        }
        pub_cmd_vel.publish(msg); //publish the angular velocity 
        ros::Duration(0.1).sleep(); //sleep for 0.1 second
    }
}

void Urban_Search_And_Rescue::m_fid_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &fid_transform){
    if (!fid_transform->transforms.empty()){
        ROS_INFO_STREAM("FID: " << fid_transform->transforms[0].fiducial_id); //Display the aruco marker ID
        // ROS_INFO_STREAM("GOT FID"); //Uncomment for debugging
        m_broadcast(fid_transform); //call the broadcaster
    }
}

double Urban_Search_And_Rescue::m_calculate_yaw(geometry_msgs::Quaternion &follower_quat){
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(follower_quat, tf_quat);
    double roll{};
    double pitch{};
    double yaw{};
    tf_quat.normalize(); //normalize the quaternion
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw); //calculates the roll, pitch and yaw from the quaternion
    yaw = fmod(yaw, 2.0 * M_PI);
    if (yaw <= 0.0){
        yaw = yaw + 2.0 * M_PI; //limits the angle [0, 2*Pi]
    }
    return yaw;
}

void Urban_Search_And_Rescue::search(){
    while (!m_explorer_client.waitForServer(ros::Duration(5.0))){ //wait for the explorer action server to start
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }
    unsigned int i = 0;
    while (ros::ok() && m_search_flag){
        if (m_next_target_flag){
            m_explorer_goal.target_pose.header.frame_id = "map";
            m_explorer_goal.target_pose.header.stamp = ros::Time::now();
            //get the target location
            m_explorer_goal.target_pose.pose.position.x = m_targets[i][0];
            m_explorer_goal.target_pose.pose.position.y = m_targets[i][1];
            m_explorer_goal.target_pose.pose.orientation.w = 1.0;
            //send the goal
            m_explorer_client.sendGoal(m_explorer_goal);
            ROS_INFO("Sending goal for explorer");
            //wait till the explorer reaches the goal
            m_explorer_client.waitForResult();
            ROS_INFO("Hooray, explorer robot reached goal");
            //call the rotate bot function
            Urban_Search_And_Rescue::m_rotate_robot(m_pub_cmd_vel);
            ros::Duration(1.0).sleep(); //sleep for 1 second
            m_listen(m_tfBuffer); //call the listener function 
            if (i < 3){
                i += 1; //count to update the next target
            }
            else{
                //send the explorer to its initial position
                m_explorer_goal.target_pose.header.frame_id = "map";
                m_explorer_goal.target_pose.header.stamp = ros::Time::now();
                m_explorer_goal.target_pose.pose.position.x = -4;
                m_explorer_goal.target_pose.pose.position.y = 2.5;
                m_explorer_goal.target_pose.pose.orientation.w = 1.0;
                ROS_INFO("Sending goal to explorer");
                //send the initial position values as target location for explorer
                m_explorer_client.sendGoal(m_explorer_goal);
                //wait till the explorer reaches the target location
                m_explorer_client.waitForResult();
                //set the flags so that the follower robot starts its rescue operation
                m_next_target_flag = false;
                m_start_follower_flag = true;
                m_next_target_flag = false;
                m_search_flag = false;
            }
        }
        m_loop_rate.sleep();
    }
}

void Urban_Search_And_Rescue::rescue(){
    while (!m_follower_client.waitForServer(ros::Duration(5.0))){ //wait for the follower action server to start
        ROS_INFO("Waiting for the move_base action server to come up for follower");
    }
    unsigned int j = 0;
    while (ros::ok()){
        if (m_start_follower_flag){
            //Uncomment the below line to debug the <map>
            // ROS_INFO_STREAM("MAP FINDING...: " << m_my_follower_targets.find(j)->second.at(0) << ", " << m_my_follower_targets.find(j)->second.at(1) << ", " << m_my_follower_targets.find(j)->second.at(2) << ", " << m_my_follower_targets.find(j)->second.at(3));
            m_follower_goal.target_pose.header.frame_id = "map";
            m_follower_goal.target_pose.header.stamp = ros::Time::now();
            //get the quaternion
            geometry_msgs::Quaternion follower_quat;
            follower_quat.x = m_my_follower_targets.find(j)->second.at(3);
            follower_quat.y = m_my_follower_targets.find(j)->second.at(4);
            follower_quat.z = m_my_follower_targets.find(j)->second.at(5);
            follower_quat.w = m_my_follower_targets.find(j)->second.at(6);
            //calculate the yaw angle
            double yaw = m_calculate_yaw(follower_quat);
            //get the target location with yaw angle (0.4 m away from the aruco location)
            m_follower_goal.target_pose.pose.position.x = m_my_follower_targets.find(j)->second.at(0) + 0.4 * (sin(yaw));
            m_follower_goal.target_pose.pose.position.y = m_my_follower_targets.find(j)->second.at(1) - 0.4 * (cos(yaw));
            m_follower_goal.target_pose.pose.position.z = m_my_follower_targets.find(j)->second.at(2);
            m_follower_goal.target_pose.pose.orientation.w = m_my_follower_targets.find(j)->second.at(6);
            //display yaw angle
            ROS_INFO_STREAM("Going to the target location: "<<j);
            ROS_INFO_STREAM("Yaw Angle: " << yaw * 180.0 / M_PI);
            ROS_INFO_STREAM("Sending Goal to Follower");
            //send the goal to follower
            m_follower_client.sendGoal(m_follower_goal);
            //wait till the follower reaches goal
            m_follower_client.waitForResult();
            ROS_INFO("Hooray, follower robot reached goal");
            if (j < 3){
                j += 1; //count to update the next target
            }
            else{
                //send the follower to its initial position
                m_start_follower_flag = false;
                m_follower_goal.target_pose.header.frame_id = "map";
                m_follower_goal.target_pose.header.stamp = ros::Time::now();
                m_follower_goal.target_pose.pose.position.x = -4;
                m_follower_goal.target_pose.pose.position.y = 3.5;
                m_follower_goal.target_pose.pose.position.z = 1.0;
                m_follower_goal.target_pose.pose.orientation.w = 1.0;
                ROS_INFO_STREAM("Sending Goal to Follower");
                //send the initial position values as target location for follower
                m_follower_client.sendGoal(m_follower_goal);
                //wait for the follower to reach the goal
                m_follower_client.waitForResult();
                ROS_INFO("Hooray, follower robot reached goal!!! Exiting ROS");
                ros::shutdown(); //shutdown after finishing the rescue task
            }
        }
        m_loop_rate.sleep();
    }
}