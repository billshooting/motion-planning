import os
import math
from collections import deque
import random

import numpy as np

class MapLoader:
    def __init__(self, maps_folder = "maps"):
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.maps_dir = os.path.join(self.base_dir, maps_folder)
        self.connections = []
        self.conn_size = {}

        if not os.path.exists(self.maps_dir):
            raise FileNotFoundError(f"Maps folder not found: {self.maps_dir}")

    def load(self, name):
        """
        Load a MovingAI map file (.map) and return:
        - grid: 2D list of chars
        - height, width
        """
        path = os.path.join(self.maps_dir, name)
        if not os.path.exists(path):
            raise FileNotFoundError(f"Map file not found: {path}")
        
        with open(path, "r") as f:
            lines = [line.strip() for line in f.readlines()]

        height = int(lines[1].split()[1])
        width  = int(lines[2].split()[1])

        map_start = 4
        grid = [list(lines[map_start + i]) for i in range(height)]

        return grid, height, width


    def to_occupancy(self, grid):
        occ = []
        for row in grid:
            occ.append([1 if c in ['@', 'T', 'W'] else 0 for c in row])

        self.connections, self.conn_size = self.compute_connected_components(occ)
        return occ
    
    def list_maps(self):
        return [f for f in os.listdir(self.maps_dir) if f.endswith(".map")]
    
    def load_all(self):
        maps = {}
        for name in self.list_maps():
            maps[name] = self.load(name)
        return maps
    
    def compute_connected_components(self, occ):
        h, w = len(occ), len(occ[0])
        comp = [[-1]*w for _ in range(h)]
        comp_sizes = {}
        comp_id = 0

        for y in range(h):
            for x in range(w):
                if occ[y][x] == 0 and comp[y][x] == -1:
                    q = deque([(x, y)])
                    comp[y][x] = comp_id
                    comp_sizes[comp_id] = 1

                    while q:
                        cx, cy = q.popleft()
                        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                            nx, ny = cx + dx, cy + dy
                            if 0 <= nx < w and 0 <= ny < h:
                                if occ[ny][nx] == 0 and comp[ny][nx] == -1:
                                    comp[ny][nx] = comp_id
                                    comp_sizes[comp_id] += 1
                                    q.append((nx, ny))
                    comp_id += 1

        return comp, comp_sizes
    
    def reachable(self, a, b):
        self.connections[a[1]][a[0]] == self.connections[b[1]][b[0]]
    
    def get_start_goal(self, occ, quantile = 0.95):
        h = len(occ)
        w = len(occ[0])

        connections = self.connections
        conn_size = self.conn_size

        # Identify largest CC
        largest_cc = max(conn_size.keys(), key=lambda cid: conn_size[cid])
        print(f"Largest CC = {largest_cc}, size = {conn_size[largest_cc]}")

        # Collect all points in largest CC
        main_free = [(x, y) for y in range(h) for x in range(w) if connections[y][x] == largest_cc]


        p = random.choice(main_free)
        reachable_points = main_free
        print(f"The reachable area has {len(reachable_points)} points")

        # Compute distances from anchor
        dists = [math.sqrt((x - p[0])**2 + (y - p[1])**2) for (x, y) in reachable_points]
        threshold = np.quantile(dists, quantile)

        candidates = [c for c, d in zip(reachable_points, dists) if d >= threshold]

        if len(candidates) == 0:
            raise RuntimeError("No candidates found; try smaller quantile.")

        # Pick a goal far away
        goal = random.choice(candidates)

        # Pick a start close to p
        start = p

        return start, goal
    