import matplotlib.pyplot as plt
import numpy as np

def plot_path(grid, path, start, goal, title="Path"):
    """
    grid  : 2D numpy array (0 = free, 1 = obstacle)
    path  : list of (x,y) coordinates
    """
    grid = np.array(grid)
    h, w = grid.shape

    plt.figure(figsize=(8,8))
    plt.title(title)

    # Show obstacles (black = obstacle)
    plt.imshow(grid, cmap='gray_r', origin='upper')

    # Extract X and Y from path
    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]

        # Draw path in red
        plt.plot(xs, ys, color='red', linewidth=2, label="Path")

        # Draw open circles on path
        plt.scatter(xs, ys, s=5, color='red')

    # Start & goal markers
    plt.scatter(start[0], start[1], color='green', s=80, marker='o', label="Start")
    plt.scatter(goal[0], goal[1], color='gold', s=80, marker='*', label="Goal")

    plt.xlim(0, w)
    plt.ylim(h, 0)  # invert y-axis for correct orientation

    plt.legend()
    plt.grid(False)
    plt.show()