"""
King's College London 
6CCE3ROS & 7CCEMROB Robotic Systems 
Week 28 Tutorial Question 2 - To complete
Path planning Sample Code with Probabilistic Roadmap (PRM)
"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree
import time

show_animation = True


class PRM:
    """
    Class for PRM planning
    """

    class Node:
        """
        PRM Node
        """

        def __init__(self, x, y, cost=None, parIndx=None):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.cost = cost
            self.parent = parIndx

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 # rand_area,
                 MAX_EDGE_LEN=3.0,
                 path_resolution=0.5,
                 # goal_sample_rate=5,
                 # max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 N_SAMPLE=500,
                 N_NearestNeighb=5,
                 N_KNN=None
                 ):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius
        N_SAMPLE: The number of samples
        """
        self.start = self.Node(start[0], start[1], 0.0, -1)
        self.end = self.Node(goal[0], goal[1], 0.0, -1)
        # self.min_rand = rand_area[0]
        # self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.path_resolution = path_resolution
        # self.goal_sample_rate = goal_sample_rate
        # self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius
        self.N_SAMPLE = N_SAMPLE
        self.N_NearestNeighb = N_NearestNeighb
        self.obstacle_kd_tree = self.kd_tree_gen()
        self.sample_x, self.sample_y = self.rand_sampling()
        self.N_KNN = N_KNN

    def kd_tree_gen(self):
        obstacle_x_list, obstacle_y_list = [], []
        for (ix, iy, _) in self.obstacle_list:
            obstacle_x_list.append(ix)
            obstacle_y_list.append(iy)

        obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)
        return obstacle_kd_tree

    def planning(self, roadmap, animation=True):
        """
        Dijkstra path planning
        animation: flag for animation on or off
        """

        # todo: complete the code for PRM path planning
        self.node_list = [self.start]

        open_set, closed_set = dict(), dict()

        # Exclude the last two nodes: the start and end node
        open_set[len(roadmap) - 2] = self.start
        path_found = True

        while True:
            if not open_set:
                print("Cannot find path")
                path_found = False
                break

            # Priority queue: 以cost为比较标准，返回index索引
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # Show graph
            # if show_animation and len(closed_set.keys()) % 2 == 0:
            #     # for stopping simulation with the esc key
            #     plt.gcf().canvas.mpl_connect(
            #         'key_release_event',
            #         lambda event: [exit(0) if event.key == 'escape' else None]
            #     )
            #     plt.plot(current.x, current.y, "xg")
            #     plt.pause(0.0001)

            # Reached to the destination
            if c_id == (len(roadmap) - 1):
                print("Goal is found!")
                self.end.parent = current.parent
                self.end.cost = current.cost
                break

            # De-queue
            del open_set[c_id]
            # Visited set
            closed_set[c_id] = current

            # Expand search grid based on motion model
            for i in range(len(roadmap[c_id])):
                n_id = roadmap[c_id][i] # Neighbour index
                dx = self.sample_x[n_id] - current.x
                dy = self.sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = self.Node(self.sample_x[n_id], self.sample_y[n_id], current.cost + d, c_id)

                if n_id in closed_set:
                    continue

                # Otherwise if it is already in the open set (Someone else's neighbour), update the path length
                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent = c_id
                else:
                    open_set[n_id] = node

        if path_found is False:
            return [], []

        # Generate final course: trace back from end note through parent node indexes
        rx, ry = [self.end.x], [self.end.y]
        parent_index = self.end.parent
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent

        return rx, ry

    def generate_map(self):
        """
        Road map generation

        sample_kd_tree: 用来找最近的k个邻居

        """
        # Use KD-tree to find the indexes of k nearest neighbours
        sample_kd_tree = KDTree(np.vstack((self.sample_x, self.sample_y)).T)
        road_map = []

        for (i, ix, iy) in zip(range(len(self.sample_x)), self.sample_x, self.sample_y):

            # Select the k-nearest neighbours with regard to the sample [ix, iy]
            ''' Do we need exact k nearest neighbours or just legit neighbours among k=6 who can form collision-free 
            edge with? '''
            dists, indexes = sample_kd_tree.query([ix, iy], k=self.N_SAMPLE)  # k = 6
            edge_id = []

            '''
            Roadmap: [[],[], ..., []], where roadmap[1] contains all the nearest neighbours' positions of 
            1st random sample
            
            ii: the index of ii-th nearest neighbour in the sample list
            '''
            for ii in range(1, len(indexes)):  # range(1, len(indexes): Excludes itself from the nearest neighbour list

                nx = self.sample_x[indexes[ii]]
                ny = self.sample_y[indexes[ii]]

                '''
                Pre-process: construct node structure
                '''
                r_node = self.Node(ix, iy)
                obs_node = self.Node(nx, ny)
                new_node = self.steer(r_node, obs_node)

                if self.check_collision(new_node):
                    edge_id.append(indexes[ii])
                # To check if we have enough legit neighbours already
                if len(edge_id) >= self.N_KNN:
                    break
            road_map.append(edge_id) # road_map[i -- edge_id[j]]: the i-th sample's j-th nearest neighbour's index

        return road_map

    def plot_road_map(self, roadmap):

        for i, _ in enumerate(roadmap):
            for ii in range(len(roadmap[i])):
                ind = roadmap[i][ii]

                plt.plot([self.sample_x[i], self.sample_x[ind]], [self.sample_y[i], self.sample_y[ind]], "-k")

    def rand_sampling(self):
        """
        --- Essential parameters ---
        self.end: The goal
        self.start: The start positions
        self.robot_radius: The robot radius
        self.play_area: The workspace boundary positions, self.play_area.xmin/xmax/ymin/ymax
        self.obstacle_list: List of obstacles [(x, y, radius), ...]
        def obstacle_sample_overlapping_checker(): --> def check_collision(node, obstacleList, robot_radius):
        randNode = self.get_random_node(): Acquire random nodes
        """

        '''
        --- Initialisation ---
        sample_x, sample_y: Samples' x & y positions
        rng: Random number generator setup
        '''
        sample_x, sample_y = [], []
        rng = np.random.default_rng()

        '''
        --- Randomly sampling ---
        Pick N_SAMPLE samples in the free space
        '''

        while len(sample_y) <= self.N_SAMPLE:
            # Generate random x, y positions within the workspace
            rx = (rng.random() * (self.play_area.xmax - self.play_area.xmin))
            ry = (rng.random() * (self.play_area.ymax - self.play_area.ymin))
            rnode = (rx, ry)

            # Check if the sample falls into free space
            if self.check_collision(rnode):
                sample_x.append(rx)
                sample_y.append(ry)

        # Append start and end node
        sample_x.append(self.start.x)
        sample_y.append(self.start.y)
        sample_x.append(self.end.x)
        sample_y.append(self.end.y)

        return sample_x, sample_y

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    # def get_random_node(self):
    #     if random.randint(0, 100) > self.goal_sample_rate:
    #         rnd = self.Node(
    #             random.uniform(self.min_rand, self.max_rand),
    #             random.uniform(self.min_rand, self.max_rand))
    #     else:  # goal point sampling
    #         rnd = self.Node(self.end.x, self.end.y)
    #     return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
                node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    # @staticmethod
    def check_collision(self, node):

        if node is None:
            return False

        '''
        To check if samples overlap with obstacles
        '''
        if type(node) is tuple:
            n = [node[0], node[1]]
            for (obx, oby, size) in self.obstacle_list:
                ob = [obx, oby]
                dist = math.dist(ob, n)
                # To see if the distance checks out
                if dist <= (size + self.robot_radius):
                    return False  # collision
            return True  # Out of collision area, in the free space

        '''
        To check if edges are collision-free
        '''
        for (ox, oy, size) in self.obstacle_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size + self.robot_radius) ** 2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    '''
    --- Search Path with PRM ---
    '''
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]

    '''
    Set Initial parameters
    '''

    '''
    Start counting time
    '''
    tic = time.time()
    prm = PRM(
        start=[0, 0],
        goal=[gx, gy],
        # rand_area=[-2, 15],
        obstacle_list=obstacleList,
        play_area=[-2, 15, -2, 15],
        robot_radius=0.8,
        N_KNN=10)

    '''
    Plot random samples
    '''
    if show_animation:
        plt.plot(prm.sample_x, prm.sample_y, '.b')

    '''
    Generate road map
    '''
    roadmap = prm.generate_map()

    # Plot the map network
    prm.plot_road_map(roadmap)

    '''
    Motion planning starts here
    '''
    rx, ry = prm.planning(roadmap, animation=show_animation)
    path = np.vstack((rx, ry)).T

    if path is None:
        print("Cannot find path")
    else:
        print("Found path!!")

    '''
    Stop counting time
    '''
    toc = time.time()
    print("Time taken by PRM planner: " + str(toc - tic) + " s.")

    '''
    Draw final path
    '''
    if show_animation:
        prm.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    main()

"""
增加robustness：
设置固定的随机取样范围 T
只在free space取样  T
增加neighbour数量，取到k个有效neighbour为止

绘图：
结果展示不全
没有过程动画展示
"""