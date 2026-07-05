#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

def cb(msg):
    data = list(msg.data)
    n = len(data)
    free = sum(1 for v in data if v == 0)
    occ  = sum(1 for v in data if v == 100)
    unk  = sum(1 for v in data if v == -1)
    print(f"total: {n}, free: {free} ({100*free/n:.1f}%), occupied: {occ} ({100*occ/n:.1f}%), unknown: {unk} ({100*unk/n:.1f}%)")

rospy.init_node("check_costmap")
rospy.Subscriber("/move_base_flex/global_costmap/costmap", OccupancyGrid, cb)
rospy.spin()