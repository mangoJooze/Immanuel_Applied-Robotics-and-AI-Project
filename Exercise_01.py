import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# Global Variables
boundary = []
obstacles = []
robot_path = []
pos_orig = ()
pos_dest = ()

def setBoundary():
    return [0, 0, 9, 9]  # x_min, y_min, x_max, y_max

def setObstacles():
    return [(0,4), (0,5), (4,0), (4,4), (4,5), (4,9), (5,0), (5,4), (5,5), (5,9), (9,4), (9,5)]

def setStartStop():
    return (0, 0), (9, 9)

def is_valid(pos):
    x, y = pos
    return (boundary[0] <= x <= boundary[2] and
            boundary[1] <= y <= boundary[3] and
            pos not in obstacles)

# Movement functions
def goNorth(current_position):
    next_pos = (current_position[0], current_position[1] + 1)
    return next_pos if is_valid(next_pos) else current_position

def goSouth(current_position):
    next_pos = (current_position[0], current_position[1] - 1)
    return next_pos if is_valid(next_pos) else current_position

def goEast(current_position):
    next_pos = (current_position[0] + 1, current_position[1])
    return next_pos if is_valid(next_pos) else current_position

def goWest(current_position):
    next_pos = (current_position[0] - 1, current_position[1])
    return next_pos if is_valid(next_pos) else current_position

def find_path(start, goal):
    queue = deque()
    queue.append((start, [start]))
    visited = set()

    while queue:
        current, path = queue.popleft()
        if current == goal:
            return path
        if current in visited:
            continue
        visited.add(current)

        for move_func in [goNorth, goSouth, goEast, goWest]:
            next_pos = move_func(current)
            if next_pos != current and next_pos not in visited:
                queue.append((next_pos, path + [next_pos]))
    return []

def plot_path():
    msize = 20
    x0 = [x for x, y in obstacles]
    y0 = [y for x, y in obstacles]
    x1 = [x for x, y in robot_path]
    y1 = [y for x, y in robot_path]

    fig, ax = plt.subplots(figsize=(6,6))
    plt.axis([-0.5, 9.5, -0.5, 9.5])
    ax.set_xticks(np.arange(0, 10, 1))
    ax.set_yticks(np.arange(0, 10, 1))
    ax.grid(True)

    # Plot elements
    ax.plot(pos_orig[0], pos_orig[1], 's', color='limegreen', markersize=msize*1.5, label='Start')
    ax.plot(pos_dest[0], pos_dest[1], 's', color='red', markersize=msize*1.5, label='Destination')
    ax.plot(x0, y0, 's', color='grey', markersize=msize*1.5, label='Obstacle')
    ax.plot(x1, y1, 's', color='blue', markersize=msize, label='Path')
    ax.plot(x1, y1, color='blue', linewidth=3)

    for i, (x, y) in enumerate(robot_path):
        ax.text(x, y, str(i), ha='center', va='center', color='white', fontweight='bold')

    ax.legend()
    plt.title("Robot Path Planning")
    plt.show()

def main():
    global boundary, obstacles, robot_path, pos_orig, pos_dest
    boundary = setBoundary()
    obstacles = setObstacles()
    pos_orig, pos_dest = setStartStop()
    robot_path = find_path(pos_orig, pos_dest)
    print("Robot Path:", robot_path)
    plot_path()

main()
