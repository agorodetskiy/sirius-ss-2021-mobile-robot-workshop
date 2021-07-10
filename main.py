#!/usr/bin/env python3
from utils import timeit
import rospy
import tf
import numpy as np
import math
from sys import float_info
EPS = float_info.epsilon

from pnc_task_msgs.msg import PlanningTask
from rtp_msgs.msg import PathStamped, PathPointWithMetadata
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from world_map import Occup_map
from structures import Node, OpenList, YourClosed

#TODO croped -> cropped

def ComputeCost(i1, j1, i2, j2):
    '''
    Computes cost of simple moves between cells
    '''
    if abs(i1 - i2) + abs(j1 - j2) == 1: #cardinal move
        return 1
    else: #diagonal move
        return math.sqrt(2) 


def MakePath(goal):
    '''
    Creates a path by tracing parent pointers from the goal node to the start node
    It also returns path's length.
    '''

    length = goal.g
    current = goal
    path = []
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1], length


@timeit
def Dijkstra(gridMap, iStart, jStart, iGoal, jGoal, heuristicFunction = None, diagonalMoves = False):
    OPEN = OpenList()
    CLOSED = YourClosed()
    OPEN.AddNode(Node(iStart, jStart, g=0, parent=None))

    while not OPEN.isEmpty():
      s = OPEN.GetBestNode() #method also delete from OPEN
      
      CLOSED.AddNode(s)
      if s.i == iGoal and s.j == jGoal:
        return (True, s, CLOSED.to_list(), OPEN.to_list())
      for i, j in gridMap.GetNeighbors(s.i, s.j, diagonalMoves):
        s_ = Node(i, j, g=s.g+ComputeCost(s.i, s.j, i, j), parent=s)
        if CLOSED.WasExpanded(s_):
          continue
        OPEN.AddNode(s_)

    return (False, None, CLOSED.to_list(), OPEN.to_list())

def DrawMap(map, path):
    for node in path:
        map.cells_croped[node.i][node.j] = -1


class Planning():
    def __init__(self):
        
        rospy.init_node("planner_sb_node_custom", disable_signals=True)

        self.pub = rospy.Publisher("/planner_sb_node/trajectory", PathStamped, queue_size=15)
        self.sub = rospy.Subscriber("/planning_node/task", PlanningTask, self.callback)
        self.tf_listener = tf.TransformListener()
        
    def run(self, once=False):
        self.once = once
        rospy.spin()
    
    def callback(self, planning_task):
        print("-----------MSG------")
        print('******Start******')
        print(planning_task.init_point.pose.position)
        print('******Stop******')
        print(planning_task.goal_point.pose.position)
        print('********Occup*******')


        occup_map = Occup_map()
        occup_map.ReadInfoFromMsg(planning_task)

        init_pose_st = PoseStamped(pose=planning_task.init_point.pose, header=Header(frame_id="local_map"))
        occup_map.Set_point("init_point", 
                            self.tf_listener.transformPose("grid_origin", init_pose_st))

        goal_pose_st = PoseStamped(pose=planning_task.goal_point.pose, header=Header(frame_id="local_map"))
        occup_map.Set_point("goal_point", 
                            self.tf_listener.transformPose("grid_origin", goal_pose_st))

        print("INIT----------------", occup_map.init_point.i, occup_map.init_point.j)
        print("GOAL----------------", occup_map.goal_point.i, occup_map.goal_point.j)

        occup_map.Crop_submap(planning_task)
        
        #print("--------Map---------")
        #print(*occup_map.cells_croped, sep='\n')
        #print("--------------------")

        #n = occup_map.GetNeighbors(0, 0)
        #print("--------Neighbors---------")
        #print(*n, sep='\n')
        #print("--------------------")

        print("--------Dijkstra---------")
        result = Dijkstra(occup_map, 
                          0, 0, occup_map.height_croped-1, occup_map.width_croped-1, diagonalMoves=True)
        path = MakePath(result[1])[0]
        DrawMap(occup_map, path)
        print(*occup_map.cells_croped, sep='\n')
        print("--------------------")



        if self.once:
            rospy.signal_shutdown("spin_once")


def main():
    node = Planning()
    node.run(once=True)


if __name__ == '__main__':
    main()
