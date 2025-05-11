import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import random
import time

# Parameters
grid_size = (20, 20)
obstacle_ratio = 0.2

# Generate grid
grid = np.zeros(grid_size, dtype=int)

# Random obstacles
num_obstacles = int(grid_size[0] * grid_size[1] * obstacle_ratio)
obstacle_indices = random.sample(range(grid_size[0] * grid_size[1]), num_obstacles)
for idx in obstacle_indices:
    x, y = divmod(idx, grid_size[1])
    grid[x, y] = 1

def random_free_cell():
    while True:
        x = random.randint(0, grid_size[0] - 1)
        y = random.randint(0, grid_size[1] - 1)
        if grid[x, y] == 0:
            return (x, y)

start = random_free_cell()
goal = random_free_cell()
while goal == start:
    goal = random_free_cell()

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def astar(grid, start, goal):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] == 0:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []

def dijkstra(grid, start, goal):
    rows, cols = grid.shape
    pq = []
    heapq.heappush(pq, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while pq:
        current_cost, current = heapq.heappop(pq)
        if current == goal:
            return reconstruct_path(came_from, current)

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] == 0:
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(pq, (new_cost, neighbor))
    return []

# Run A*
start_time = time.time()
astar_path = astar(grid, start, goal)
astar_time = time.time() - start_time

# Run Dijkstra
start_time = time.time()
dijkstra_path = dijkstra(grid, start, goal)
dijkstra_time = time.time() - start_time

# Print results
print("=== Kết quả ===")
print(f"Start: {start}")
print(f"Goal:  {goal}")
print(f"A*     → Thời gian: {astar_time:.6f}s | Độ dài đường đi: {len(astar_path)}")
print(f"Dijkstra → Thời gian: {dijkstra_time:.6f}s | Độ dài đường đi: {len(dijkstra_path)}")

# Plot
fig, ax = plt.subplots(figsize=(10, 10))
for x in range(grid_size[0]):
    for y in range(grid_size[1]):
        if grid[x, y] == 1:
            rect = patches.Rectangle((y, grid_size[0] - x - 1), 1, 1, facecolor='gray')
            ax.add_patch(rect)

for (x, y) in astar_path:
    rect = patches.Rectangle((y, grid_size[0] - x - 1), 1, 1, facecolor='blue', alpha=0.5)
    ax.add_patch(rect)

for (x, y) in dijkstra_path:
    rect = patches.Rectangle((y, grid_size[0] - x - 1), 1, 1, facecolor='red', alpha=0.5)
    ax.add_patch(rect)

ax.scatter(start[1] + 0.5, grid_size[0] - start[0] - 0.5, color='green', s=200, label='Start')
ax.scatter(goal[1] + 0.5, grid_size[0] - goal[0] - 0.5, color='purple', s=200, label='Goal')

for x in range(grid_size[0] + 1):
    ax.plot([0, grid_size[1]], [x, x], 'k', lw=0.5)
for y in range(grid_size[1] + 1):
    ax.plot([y, y], [0, grid_size[0]], 'k', lw=0.5)

legend_patches = [
    patches.Patch(color='blue', alpha=0.5, label="A* Path"),
    patches.Patch(color='red', alpha=0.5, label="Dijkstra Path"),
    patches.Patch(color='gray', label="Obstacle"),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label="Start"),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='purple', markersize=10, label="Goal")
]

ax.legend(handles=legend_patches, loc='upper right')
ax.set_xticks([])
ax.set_yticks([])
ax.set_xlim(0, grid_size[1])
ax.set_ylim(0, grid_size[0])
ax.set_aspect('equal')
ax.set_title("A* vs Dijkstra Pathfinding (Random Map)")
plt.show()
