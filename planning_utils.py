import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, Pose, Point
import tf
from std_msgs.msg import Header
from pythonrobotics.rrt_star_dubins import RRTStarDubins


class OccupancyGrid:
    # rosrun tf tf_echo 1 2
    def __init__(self, ros_occ_grid):
        # http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        metadata = ros_occ_grid.info
    
        self.width = metadata.width
        self.height = metadata.height
        self.resolution = metadata.resolution  #  [m/cell]
        self.grid_origin = metadata.origin
        self.data_buff = ros_occ_grid.data  # tuple, len 360 000

        self.grid = np.array(self.data_buff).reshape((self.height, self.width)) > 50
        
    def get_cell_coords(self, i, j):
        x = self.grid_origin.position.x + j * self.resolution
        y = self.grid_origin.position.x + i * self.resolution
        return (x, y)
        

def plan_path(start_point, end_point, occ_grid: OccupancyGrid, tf_listener, show_animation=True):
    print("Starting RRT* in Dubins space...")
    print(f'start point: {start_point}')
    print(f'end point: {end_point}')

    obstacle_list = [occ_grid.get_cell_coords(i, j) + (occ_grid.resolution,)
                     for i in range(occ_grid.height) for j in range(occ_grid.width)
                     if occ_grid.grid[i][j]
    ]
    
    print(*obstacle_list, sep='\n')
    
    # print(f'Found {len(obstacle_list)} obstacles')
     
    # plt.matshow(occ_grid.grid)
    # plt.show()
    # import time
    # time.sleep(3)

    # two_points = [PoseStamped(
    #     pose=Pose(position=Point(x // 0.2, y // 0.2, 0)),
    #     header=Header(frame_id="local_map")
    #     ) for (x, y) in [start_point, end_point]]
    # poses = [tf_listener.transformPose("grid_origin", p).pose for p in two_points]
    # start_point, end_point = [(pose.position.x, pose.position.y) for pose in poses]
    
    start = [start_point[0], start_point[1], np.deg2rad(0.0)]
    goal = [end_point[0], end_point[1], np.deg2rad(0.0)]

    rrtstar_dubins = RRTStarDubins(start, goal, rand_area=[-60, 60], obstacle_list=obstacle_list)
    path = rrtstar_dubins.planning(animation=show_animation)
    
    # path = [PoseStamped(
    #     pose=Pose(position=Point(round(x) * 0.2, round(y) * 0.2, 0)),
    #     header=Header(frame_id="grid_origin")
    #     ) for (x, y) in path]
    # path = [tf_listener.transformPose("local_map", p).pose for p in path] # list of poses
    
    path = [Pose(position=Point(x, y, 0))
            for (x, y) in path]
    

    # Draw final path
    if show_animation:  # pragma: no cover
        rrtstar_dubins.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)

        plt.show()
        
    return path