"""
2022/06/27 by Evin Hsu
---------------------------------------
Function Test / Application
"""

#! /usr/bin/env python3
import os
import time
import math
import subprocess
import argparse
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from rrt_planner_client import GlobalClient

path_map = '/home/user/Documents/rrt_rl_ws/src/utils/maps'
file_map = 'stage_1'

check = 0
parser = argparse.ArgumentParser()
parser.add_argument("--map", action="store_true")
parser.add_argument("--planner", action="store_true")
parser.add_argument("--random_plan", action="store_true")
a = parser.parse_args()
if a.map:               ## Check occupied value
    check = 1
elif a.planner:         ## planning with given start & goal
    check = 0
elif a.random_plan:     ## planning with random start & goal
    check = 2
else:
    check = 0

def open_map():
    path_yaml = os.path.join(path_map, file_map, "map.yaml")
    subprocess.Popen(["rosrun", "map_server", "map_server", path_yaml])
    rospy.sleep(0.5)

## PointStamp(/clicked_point) --> PoseStamp
def point_to_pose(point, seq=0, timestamp=None):
    pose = PoseStamped()
    pose.header.frame_id = point.header.frame_id
    pose.pose.position.x = point.point.x
    pose.pose.position.y = point.point.y
    pose.pose.position.z = point.point.z
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    return pose


if __name__ == "__main__":

    rospy.init_node('rrt_rl_dynamic_node')
    
    print(f'\nOpen Map [{file_map}]', end='')
    open_map()
    print(f' Done.')
    print(f'Preparing Global Service Client ...', end='')
    
    global_cl = GlobalClient()
    _, m_res, m_wid, m_hei, m_orx, m_ory = global_cl.get_map_info()
    print(f' Done.')

    seq = 0
    

    if check == 0:
        
        while not rospy.is_shutdown():
            print('\n====================================\nGiven-Start-Goal Path\n====================================\n')
            
            print('\n-----------------------------------------------')
            print('Click start point: ')
            start = None
            while not rospy.is_shutdown() and start is None:
                try:
                    start = rospy.wait_for_message("/clicked_point", PointStamped, timeout=0.5)
                except:
                    pass
            print('  (%.2lf, %.2lf)' % (start.point.x, start.point.y))
            print('Click goal point: ')
            goal = None
            while not rospy.is_shutdown() and goal is None:
                try:
                    goal = rospy.wait_for_message("/clicked_point", PointStamped, timeout=0.5)
                except:
                    pass
            print('  (%.2lf, %.2lf)' % (goal.point.x, goal.point.y))
            
            start_pose = point_to_pose(start, seq, None)
            goal_pose = point_to_pose(goal, seq+1, None)
            
            print("\nStart Global Planning >>")
            
            start_t = time.time()
            ret, _, inters = global_cl.get_global_path(start_pose, goal_pose)
            
            if not ret:
                print("Planning Failed.")
            else:
                using_time = time.time() - start_t                
                print("Planning Success.")
                print("Use Time : %.6lf." % using_time)
                print("In Map %s with size(%.4lf, %.4lf)" % (file_map, m_wid*m_res, m_hei*m_res))
                print("Distance between start and goal is %.4lf." % (math.hypot( (start.point.x - goal.point.x), (start.point.y - goal.point.y) )))
                seq += 2
    

    elif check == 1:
        print('\n====================================\nCheck Map Occupancy\n====================================\n')
        while not rospy.is_shutdown():
            print('\n-----------------------------------------------')
            print('Click point: ')
            start = None
            while not rospy.is_shutdown() and start is None:
                try:
                    start = rospy.wait_for_message("/clicked_point", PointStamped, timeout=0.5)
                except:
                    pass
            print('> get point (%.2lf, %.2lf)' % (start.point.x, start.point.y))
            _, occ = global_cl.get_occ_value(start.point.x, start.point.y)
            if occ == 0:
                print(f'> {occ}. is Free.')
            elif occ == 100:
                print(f'> {occ}. Occupied.')
            else:
                print(f'> {occ}. Unknown.')
    
    elif check == 2:
        print('\n====================================\nRandom-Start-Goal Path\n====================================\n')
        while not rospy.is_shutdown():
            print('\n-----------------------------------------------')
            print('Click any point to start: ')
            pt = None
            while not rospy.is_shutdown() and pt is None:
                try:
                    pt = rospy.wait_for_message("/clicked_point", PointStamped, timeout=0.5)
                except:
                    pass
                
            print("\nStart Global Planning >>")
            start_t = time.time()
            ret, start, goal, inters = global_cl.get_global_random()
            
            if not ret:
                print("Planning Failed.")
            else:
                using_time = time.time() - start_t                
                print("Planning Success.")
                print("Start (%.2lf, %.2lf) --> Goal (%.2lf, %.2lf)" % (start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y))
                print("Use Time : %.6lf." % using_time)
                print("In Map %s with size(%.4lf, %.4lf)" % (file_map, m_wid*m_res, m_hei*m_res))
                print("Distance between start and goal is %.4lf." % (math.hypot( (start.pose.position.x - goal.pose.position.x), (start.pose.position.x - goal.pose.position.x) )))
                seq += 2
        
    else: 
        print("Wrong Mode. Terminate.")

