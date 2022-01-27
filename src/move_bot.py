#!/usr/bin/env python3

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def movebase_client(goal_x, goal_y, move_base_server = 'move_base'):
    client = actionlib.SimpleActionClient(move_base_server, MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    rospy.loginfo(wait)
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        res = client.get_result()
        rospy.loginfo(res)
        return res


if __name__ == '__main__':
    rospy.myargv(argv=sys.argv)
    if len(sys.argv) == 3:
        movebase_client(float(sys.argv[1]), float(sys.argv[2]))
