import os

class MapLoader:
    def __init__(self, maps_folder = "maps"):
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.maps_dir = os.path.join(self.base_dir, maps_folder)

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
        """
        Convert character grid to 0 = free, 1 = obstacle.
        """
        occ = []
        for row in grid:
            occ.append([1 if c in ['@', 'T'] else 0 for c in row])
        return occ
    
    def list_maps(self):
        return [f for f in os.listdir(self.maps_dir) if f.endswith(".map")]
    
    def load_all(self):
        maps = {}
        for name in self.list_maps():
            maps[name] = self.load(name)
        return maps
    
    def get_start_goal(self, occ):
        h = len(occ)
        w = len(occ[0])

        # Gather all free cells
        free = [(x, y) for y in range(h) for x in range(w) if occ[y][x] == 0]

        if len(free) < 2:
            raise ValueError("Map does not contain enough free cells!")

        p = free[0]

        # Squared distance
        def d2(a, b):
            return (a[0]-b[0])**2 + (a[1]-b[1])**2

        q = max(free, key=lambda c: d2(c, p))

        r = max(free, key=lambda c: d2(c, q))

        return q, r
    