import heapq
import time
import math

class AStar:
    def __init__(self, occ_grid):
        self.grid = occ_grid
        self.h = len(occ_grid)
        self.w = len(occ_grid[0])

    def heuristic(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    
    def get_neighbors(self, node):
        x, y = node
        moves = [(-1,0),(1,0),(0,-1),(0,1),
                (-1,-1),(-1,1),(1,-1),(1,1)]

        for dx, dy in moves:
            nx, ny = x + dx, y + dy

            # bounds check
            if not (0 <= nx < self.w and 0 <= ny < self.h):
                continue

            # obstacle check
            if self.grid[ny][nx] == 1:
                continue

            # diagonal corner-cut check
            if dx != 0 and dy != 0:
                if self.grid[y][nx] == 1 or self.grid[ny][x] == 1:
                    continue

            # cost
            cost = math.hypot(dx, dy)

            yield (nx, ny), cost

    def reconstruct(self, came_from, start, goal):
        path = []
        node = goal
        visited = set()
        while node is not None and node not in visited:
            visited.add(node)
            path.append(node)
            node = came_from.get(node, None)
        path.reverse()
        return path
    
    def compute_path_length(self, path):
        if len(path) < 2:
            return math.inf
        length = 0
        for i in range(len(path)-1):
            length += math.hypot(
                path[i+1][0] - path[i][0],
                path[i+1][1] - path[i][1]
            )
        return length
    
    def compute_smoothness(self, path):
        if len(path) < 3:
            return 0.0

        angle_sum = 0.0
        for i in range(1, len(path)-1):
            x1, y1 = path[i-1]
            x2, y2 = path[i]
            x3, y3 = path[i+1]

            v1 = (x2-x1, y2-y1)
            v2 = (x3-x2, y3-y2)

            # compute angle between v1 and v2
            dot = v1[0]*v2[0] + v1[1]*v2[1]
            mag1 = math.hypot(*v1)
            mag2 = math.hypot(*v2)
            if mag1 == 0 or mag2 == 0:
                continue
            cos_theta = max(-1, min(1, dot/(mag1*mag2)))
            angle = math.acos(cos_theta)
            angle_sum += angle

        return angle_sum
    
    def plan(self, start, goal):
        t0 = time.time()

        open_set = []
        heapq.heappush(open_set, (0, start))

        g = {start: 0}
        came_from = {start: None}

        nodes_expanded = 0

        while open_set:
            _, current = heapq.heappop(open_set)
            nodes_expanded += 1

            if current == goal:
                path = self.reconstruct(came_from, start, goal)
                path_len = self.compute_path_length(path)
                smoothness = self.compute_smoothness(path)
                planning_time = time.time() - t0

                return {
                    "success": True,
                    "path": path,
                    "path_length": path_len,
                    "time": planning_time * 1000,
                    "expanded": nodes_expanded,
                    "smoothness": smoothness
                }

            for nbr, cost in self.get_neighbors(current):
                tentative = g[current] + cost
                if nbr not in g or tentative < g[nbr]:
                    g[nbr] = tentative
                    priority = tentative + self.heuristic(nbr, goal)
                    heapq.heappush(open_set, (priority, nbr))
                    came_from[nbr] = current

        # no path found
        return {
            "success": False,
            "path": None,
            "path_length": math.inf,
            "time": (time.time() - t0) * 1000,
            "expanded": nodes_expanded,
            "smoothness": math.inf
        }