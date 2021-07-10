#!/usr/bin/env python

import rospy
import numpy as np

import tf

from pnc_task_msgs.msg import PlanningTask
from rtp_msgs.msg import PathStamped, PathPointWithMetadata, PathPointMetadata
from geometry_msgs.msg import Pose, Point

from planning_utils import OccupancyGrid, plan_path


class ROSNode:
    def __init__(self):
        rospy.init_node('trajectory_planner', anonymous=True)
        PathPointWithMetadata
        self.sub = rospy.Subscriber("/planning_node/task", PlanningTask, self.callback)
        self.pub = rospy.Publisher("/planner_sb_node/trajectory", PathStamped, queue_size=15)
        self.tf_listener = tf.TransformListener()
        
        rospy.spin()

    def callback(self, planning_task):  
        # rtp_msgs/PathPointWithMetadata init_point
        # rtp_msgs/PathPointWithMetadata goal_point
        # .position.x\y\z
        # nav_msgs/OccupancyGrid occ_grid

        ros_occ_grid = planning_task.occ_grid
        occ_grid = OccupancyGrid(ros_occ_grid)
        
        start_point = (
            planning_task.init_point.pose.position.x,
            planning_task.init_point.pose.position.y
        )
        end_point = (
            planning_task.goal_point.pose.position.x,
            planning_task.goal_point.pose.position.y
        )
        
        path = plan_path(start_point, end_point, occ_grid, self.tf_listener, show_animation=False)
        
        # print(origin)
        
        msg = PathStamped()
        msg.path_with_metadata = [PathPointWithMetadata(pose=pose, metadata=planning_task.init_point.metadata)
                                  for pose in path]
        msg.path_type = planning_task.path_type
        msg.route_index_to = planning_task.route_index_to
        msg.route_index_from = planning_task.route_index_from
        
        self.pub.publish(msg)
        
    

if __name__ == '__main__':
    try:
        node = ROSNode()
    except rospy.ROSInterruptException:
        pass
