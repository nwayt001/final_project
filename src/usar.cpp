#include "../include/usar.h"

Urban_Search_And_Rescue::Urban_Search_And_Rescue(ros::NodeHandle *nh) :
    m_nh{*nh},
    m_fid_flag{false},
    m_next_target_flag{true},
    m_start_follower_flag{false},
    m_follower_next_target_flag{true},
    m_explorer_client{"/explorer/move_base", true},
    m_follower_client{"/follower/move_base", true},
    m_tfBuffer{},
    m_tfListener{m_tfBuffer},
    m_loop_rate{10},
    m_search_flag{true}
{
    m_init_pubs();
    m_init_subs();
}

void Urban_Search_And_Rescue::m_init_pubs(){
    m_pub_cmd_vel = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 10);
}

void Urban_Search_And_Rescue::m_init_subs(){
    m_sub_fid = m_nh.subscribe("/fiducial_transforms", 1, &Urban_Search_And_Rescue::m_fid_callback,this);
}

void Urban_Search_And_Rescue::get_targets(){
    m_nh.getParam("/aruco_lookup_locations/target_1",m_targets[0]);
    m_nh.getParam("/aruco_lookup_locations/target_2",m_targets[1]);
    m_nh.getParam("/aruco_lookup_locations/target_3",m_targets[2]);
    m_nh.getParam("/aruco_lookup_locations/target_4",m_targets[3]);
}

void Urban_Search_And_Rescue::m_listen(tf2_ros::Buffer& tfBuffer){
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
    std::array<double,7> trans;
    trans[0] = transformStamped.transform.translation.x;
    trans[1] = transformStamped.transform.translation.y;
    trans[2] = transformStamped.transform.translation.z;
    trans[3] = transformStamped.transform.rotation.x;
    trans[4] = transformStamped.transform.rotation.y;
    trans[5] = transformStamped.transform.rotation.z;
    trans[6] = transformStamped.transform.rotation.w;
    m_follower_target_vector.push_back(trans);
    ROS_INFO_STREAM("Position in map frame: ["
      << trans[0] << ","
      << trans[1] << ","
      << trans[2] << "]"
    );
    m_my_follower_targets.insert(make_pair(m_fid,trans));
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void Urban_Search_And_Rescue::m_broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_transform) {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  std::array<double,2> fid_pose;

  //broadcast the new frame to /tf Topic
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

  double fid_exp_dist = std::sqrt(std::pow((fid_transform->transforms[0].transform.translation.x),2) + std::pow((fid_transform->transforms[0].transform.translation.y),2));
  ROS_INFO_STREAM("x poisition of exp: "<<m_explorer_curr_pose[0]<<" y position of exp: "<<m_explorer_curr_pose[1]);
  ROS_INFO_STREAM("x poisition of aruco: "<<fid_transform->transforms[0].transform.translation.x<<" y position of aruco: "<<fid_transform->transforms[0].transform.translation.y);
  if (fid_exp_dist >= 3.0){
    ROS_INFO_STREAM("DISTANCE: "<<fid_exp_dist);
    m_fid_flag = false;
  }
  else {
    m_fid_flag=true;
    ROS_INFO_STREAM("Broadcasting");
    m_fid = fid_transform->transforms[0].fiducial_id;
    br.sendTransform(transformStamped);
    ros::Duration(1.0).sleep();
  }
}

void Urban_Search_And_Rescue::m_rotate_robot(ros::Publisher& pub_cmd_vel){
    geometry_msgs::Twist msg;
    msg.angular.z = 0.1;
    msg.linear.x = 0.0;
    while(true){
        ros::spinOnce();
        if(m_fid_flag){
            m_fid_flag = false;
            ROS_INFO_STREAM("Breaking Rotate Bot");
            break;
        }
        pub_cmd_vel.publish(msg);
        ros::Duration(0.1).sleep();
    }
}

void Urban_Search_And_Rescue::m_fid_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_transform){
  if(!fid_transform->transforms.empty()){
      ROS_INFO_STREAM("FID: "<<fid_transform->transforms[0].fiducial_id);
      // fid_flag = true;
      ROS_INFO_STREAM("GOT FID");
      m_broadcast(fid_transform);
  }
}

void Urban_Search_And_Rescue::search(){
    while (!m_explorer_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }
    unsigned int i=0;
    while (ros::ok() && m_search_flag) {
        if (m_next_target_flag){
            m_explorer_goal.target_pose.header.frame_id = "map";
            m_explorer_goal.target_pose.header.stamp = ros::Time::now();
            m_explorer_goal.target_pose.pose.position.x = m_targets[i][0];//
            m_explorer_goal.target_pose.pose.position.y = m_targets[i][1];//
            m_explorer_goal.target_pose.pose.orientation.w = 1.0;
            m_explorer_client.sendGoal(m_explorer_goal);
            ROS_INFO("Sending goal for explorer");
            m_explorer_client.waitForResult();
            ROS_INFO("Hooray, follower robot reached goal");
            Urban_Search_And_Rescue::m_rotate_robot(m_pub_cmd_vel);
            ros::Duration(1.0).sleep();
            m_listen(m_tfBuffer);
            if(i<3){
                i+=1;
            }
            else{
                m_explorer_goal.target_pose.header.frame_id = "map";
                m_explorer_goal.target_pose.header.stamp = ros::Time::now();
                m_explorer_goal.target_pose.pose.position.x = -4;//
                m_explorer_goal.target_pose.pose.position.y = 2.5;//
                m_explorer_goal.target_pose.pose.orientation.w = 1.0;
                ROS_INFO("Sending goal for explorer");
                m_explorer_client.sendGoal(m_explorer_goal);//this should be sent only once
                m_explorer_goal_sent = true;
                m_explorer_client.waitForResult();
                m_next_target_flag = false;
                m_start_follower_flag = true;
                ROS_INFO_STREAM(m_follower_target_vector.size()<<" is the size of the vector");
                m_next_target_flag = false;
                m_search_flag = false;
            }
        }
        m_loop_rate.sleep();
    }
}

void Urban_Search_And_Rescue::rescue(){
    while (!m_follower_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }
    unsigned int j=0;
    while (ros::ok()){
        if(m_start_follower_flag){
        // auto goal = follower_target_vector.at(0);
        // ROS_INFO_STREAM("MAP CHECKING.....: "<<my_follower_targets[j].at(0)<<", "<<my_follower_targets[j].at(1)<<", "<<my_follower_targets[j].at(2)<<", "<<my_follower_targets[j].at(3));
            ROS_INFO_STREAM("MAP FINDING...: " << m_my_follower_targets.find(j)->second.at(0) << ", " <<m_my_follower_targets.find(j)->second.at(1) << ", " <<m_my_follower_targets.find(j)->second.at(2) << ", " <<m_my_follower_targets.find(j)->second.at(3));
            m_follower_goal.target_pose.header.frame_id = "map";
            m_follower_goal.target_pose.header.stamp = ros::Time::now();
            
            geometry_msgs::Quaternion follower_quat;
            follower_quat.x = m_my_follower_targets.find(j)->second.at(3);
            follower_quat.y = m_my_follower_targets.find(j)->second.at(4);
            follower_quat.z = m_my_follower_targets.find(j)->second.at(5);
            follower_quat.w = m_my_follower_targets.find(j)->second.at(6);
            // follower_quat = tf::Quaternion::normalize();
            tf::Quaternion tf_quat;
            tf::quaternionMsgToTF(follower_quat, tf_quat);
            double roll{};
            double pitch{};
            double yaw{};
            tf_quat.normalize();
            tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
            yaw = fmod(yaw, 2.0 * M_PI);
            if(yaw<=0.0){
                yaw = yaw+2.0*M_PI;
            }
            // follower_goal.target_pose.pose.position.x = (my_follower_targets.find(j)->second.at(0)<=0)?(my_follower_targets.find(j)->second.at(0)+0.4):my_follower_targets.find(j)->second.at(0)-0.4;//
            // follower_goal.target_pose.pose.position.y = (my_follower_targets.find(j)->second.at(1)<=0)?(my_follower_targets.find(j)->second.at(1)+0.4):my_follower_targets.find(j)->second.at(1)-0.4;//
            m_follower_goal.target_pose.pose.position.x = m_my_follower_targets.find(j)->second.at(0)+0.4*(sin(yaw));
            m_follower_goal.target_pose.pose.position.y = m_my_follower_targets.find(j)->second.at(1)-0.4*(cos(yaw));
            m_follower_goal.target_pose.pose.position.z = m_my_follower_targets.find(j)->second.at(2);
            // follower_goal.target_pose.pose.orientation.x = my_follower_targets.find(j)->second.at(3);
            // follower_goal.target_pose.pose.orientation.y = my_follower_targets.find(j)->second.at(4);
            // follower_goal.target_pose.pose.orientation.z = my_follower_targets.find(j)->second.at(5);
            m_follower_goal.target_pose.pose.orientation.w = m_my_follower_targets.find(j)->second.at(6);
            
            ROS_INFO_STREAM("YAW ANGLE!!.. : "<<yaw * 180.0 / M_PI); 

            ROS_INFO_STREAM("SENDING GOAL TO FOLLOWER");
            m_follower_client.sendGoal(m_follower_goal);
            m_follower_client.waitForResult();
            ROS_INFO("Hooray, follower robot reached goal");
            if(j<3){
                j+=1;
            }
            else{
                m_start_follower_flag = false;
                m_follower_goal.target_pose.header.frame_id = "map";
                m_follower_goal.target_pose.header.stamp = ros::Time::now();
                m_follower_goal.target_pose.pose.position.x = -4;//
                m_follower_goal.target_pose.pose.position.y = 3.5;//
                m_follower_goal.target_pose.pose.position.z = 1.0;
                m_follower_goal.target_pose.pose.orientation.w = 1.0;
                ROS_INFO_STREAM("SENDING GOAL TO FOLLOWER");
                m_follower_client.sendGoal(m_follower_goal);
                m_follower_client.waitForResult();
                ROS_INFO("Hooray, follower robot reached goal!!! Exiting ROS");
                ros::shutdown();
            }
        }
        m_loop_rate.sleep();
    }
}