from load_map import MapLoader
from algorithm.a_star import AStar
from plot import plot_path

loader = MapLoader()

# load map
maps = loader.load_all()
for key, value in maps.items():
    name = key
    grid, h, w = value
    occ = loader.to_occupancy(grid)
    # get consistent start/goal
    start, goal = loader.get_start_goal(occ)

    print(f"Map: {name}, start / goal : {start, goal}")
    planner = AStar(occ)
    result = planner.plan(start, goal)

    path = result.pop('path')
    plot_path(occ, path, start, goal)
    print(f"Map: {name}, result: {result}")