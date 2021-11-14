import math
import numpy as np

import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state(node_new, self.s_goal)
                    # path = self.extract_path(node_new)
                    return self.extract_path(node_new)
        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):            #nearest with rand node
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)   #return index of nearest node
                                        for nd in node_list]))]             

    def new_state(self, node_start, node_end):         #create new node
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        node = (self.s_goal.x, self.s_goal.y)
        path = [Node(node)]
        node_now = node_end
        while node_now.parent is not None:
            node_now = node_now.parent
            path.append(node_now)
        return path
    
    def coordinate_path(self,path):
        coordinate_path = []
        for node in path:
            coordinate_path.append((node.x,node.y))
        # print(len(coordinate_path))
        # print(coordinate_path)
        return coordinate_path

    def convert_path(self,path):
        path_input = []
        path_input.append(path[0])
        node_status = path[0]
        a = len(path)
        i = 1
        for i in range(len(path)):
            if self.utils.is_collision(node_status, path[i]): #and self.utils.is_collision(node_status,path[i+1]):
                if i > 1:
                    path_input.append(path[i-1])
                    node_status = path[i-1]
        path_input.append(path[len(path)-1])
        return path_input

    def coordinate_convert_path(self,path):
        path_convert = []
        path = self.convert_path(path)
        for node in path:
            path_convert.append((node.x,node.y))
        return path_convert
    def get_circle(self,path):
        circles = []
        path = self.convert_path(path)
        for i in range(len(path)):
            if i >0 and i < len(path)-1:
                node = path[i]
                r = self.utils.get_radius(node)
                circles.append([node.x,node.y,r])
        print(circles)
        return circles

    def get_point_circle(self,path):
        path= self.convert_path(path)
        output = []
        for i in range(len(path)):
            if i >0 and i < len(path)-1:
                node = path[i]
                r = self.utils.get_radius(node)
                #print (r)
                point1 = self.utils.get_intersection(path[i-1],path[i], r)
                output.append(point1)
                #print(point1)
                point2 = self.utils.get_intersection(path[i+1],path[i], r)
                output.append(point2)
                #print(point2)
        #print(output)
        return output

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    start = (600, 100) # Starting node
    goal = (30, 200) # Goal node

    rrt = Rrt(start, goal, 50, 0.01, 10000)

    path = rrt.planning()
    path_input = rrt.convert_path(path)
    path_coordinate = rrt.coordinate_path(path)
    path_convert = rrt.coordinate_convert_path(path_input)
    circles = rrt.get_circle(path_input)
    output = rrt.get_point_circle(path)

    if path:
        rrt.plotting.animation(rrt.vertex, path_coordinate, path_convert,circles,output, "RRT", True)
    else:
        print("No Path Found!")
   

if __name__ == '__main__':
    main()
