#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal

client = None

def goal_cb(msg):
    if client is None:
        rospy.logwarn("MBF client not ready yet, goal dropped")
        return
    goal = MoveBaseGoal()
    goal.target_pose = msg
    rospy.loginfo("RViz goal → MBF: x=%.2f, y=%.2f, frame=%s", 
                  msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)
    client.send_goal(goal)

if __name__ == '__main__':
    rospy.init_node('mbf_rviz_bridge')
    
    # 1. 先创建 Subscriber（确保 ROS Master 立即注册订阅）
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_cb, queue_size=10)
    rospy.loginfo("已订阅 /move_base_simple/goal")
    
    # 2. 再连接 MBF Action Server（带超时，不阻塞）
    client = actionlib.SimpleActionClient('/move_base_flex/move_base', MoveBaseAction)
    rospy.loginfo("等待 MBF Action Server...")
    
    if not client.wait_for_server(rospy.Duration(100.0)):
        rospy.logerr("MBF 未在 10 秒内启动！Subscriber 已注册，但 goal 会被丢弃。")
    else:
        rospy.loginfo("已连接 MBF，桥接就绪。")
    
    rospy.spin()