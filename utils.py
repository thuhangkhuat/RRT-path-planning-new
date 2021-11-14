import math
import numpy as np
import os
import sys

import env
from RRTalgorithm import Node
from config import *

class Utils:
    def __init__(self):
        self.env = env.Env()

        self.delta = rr
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def get_obs_vertex(self):   
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obs_rectangle:         #collision between robot and obstacles
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)     #v2[0]*v3[0]+v2[1]*v3[1]

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div     #crossproduct v2[0]*v1[1]-v2[1]*v1[0]
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)      # distance two node
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True

        return False

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        if self.obs_circle is not None:
            for (x, y, r) in self.obs_circle:
                if self.is_intersect_circle(o, d, [x, y], r):
                    return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta

        if self.obs_circle is not None:
            for (x, y, r) in self.obs_circle:
                if math.hypot(node.x - x, node.y - y) <= r + delta:       #collision between circles and node
                    return True

        if self.obs_rectangle is not None:
            for (x, y, w, h) in self.obs_rectangle:
                if 0 <= node.x - (x - delta) <= w + 2 * delta \
                        and 0 <= node.y - (y - delta) <= h + 2 * delta:    #collision between rectangles and node
                    return True

        if self.obs_boundary is not None:
            for (x, y, w, h) in self.obs_boundary:
                if 0 <= node.x - (x - delta) <= w + 2 * delta \
                        and 0 <= node.y - (y - delta) <= h + 2 * delta:
                    return True

        return False
    def belong_to_obs_bound(self,node):
        radius = []
        r = 0
        delta = self.delta
        for (x, y, w, h) in self.obs_boundary:
            if node.y > (y+h+delta):
                r = abs(node.y-y-h-delta)
            else:
                r = abs(node.y-y+delta)
            radius.append(r)
            if node.x > (x+w+delta):
                r = abs(node.x-x-w-delta)
            else:
                r = abs(node.x-x+delta)
            radius.append(r)
        print(min(radius))
        return min(radius)

    def belong_to_obs_width(self,node):
        radius = []
        r = 0
        delta = self.delta
        radius.append(r)
        if self.obs_rectangle is not None:
            for (x, y, w, h) in self.obs_rectangle:
                if 0 <= node.x - (x - delta) <= w + 2 * delta:
                    r = abs(node.y-y-h-delta)
                    radius.append(r)  
                    r = abs(node.y-y+delta)
                    radius.append(r)
        print(min(radius))
        return min(radius)
    def belong_to_obs_high(self,node):
        radius = []
        r = 0
        delta = self.delta
        radius.append(r)
        if self.obs_rectangle is not None:
            for (x, y, w, h) in self.obs_rectangle:
                if 0 <= node.y - (y - delta) <= h + 2 * delta:
                    r = abs(node.x-x-w-delta)
                    radius.append(r)
                    r = abs(node.x-x+delta)
                    radius.append(r)
        return min(radius)
    def get_radius(self,node):
        delta = self.delta
        radius = []
        r1 = self.belong_to_obs_high(node)
        r2 = self.belong_to_obs_width(node)
        r3 = self.belong_to_obs_bound(node)
        if r1 != 0:
            radius.append(r1)
        if r2 != 0:
            radius.append(r2)
        if r3 != 0:
            radius.append(r3)
        obs_vertex = self.get_obs_vertex()
        for (v1, v2, v3, v4) in obs_vertex:
            radius.append(math.hypot(node.x-v1[0],node.y-v1[1]))
            radius.append(math.hypot(node.x-v2[0],node.y-v2[1]))
            radius.append(math.hypot(node.x-v3[0],node.y-v3[1]))
            radius.append(math.hypot(node.x-v4[0],node.y-v4[1]))
        if self.obs_circle is not None:
            for (x, y, r) in self.obs_circle:
                rc =  math.hypot(node.x - x, node.y - y) - r - delta      #collision between circles and node
                radius.append(rc)
        return min(radius)

    def get_intersection(self,start,end,r):  #with end_node is center of circle
        px = end.x - start.x
        py = end.y - start.y
        if r > math.hypot(py,px):
            return (start.x,start.y)
        else:
            t1 = math.sqrt((r*r/(px*px+py*py)))
            t2 = -t1
            if end.x <= t1*px + end.x <= start.x and end.y <= t1*py + end.y <= start.y:
                return (t1*px + end.x,t1*py + end.y)
            else:
                return (t2*px + end.x,t2*py + end.y)
        
    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
