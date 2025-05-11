import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import random
import time
import csv

# Parameters
grid_size = (20, 20)
obstacle_ratio = 0.2
num_runs = 200  # Chạy thử nghiệm 100 lần

# Heuristic function for A* (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Reconstruct path from came_from dictionary
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# A* Algorithm
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

# Dijkstra Algorithm
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

# Random start and goal
def random_free_cell():
    while True:
        x = random.randint(0, grid_size[0] - 1)
        y = random.randint(0, grid_size[1] - 1)
        if grid[x, y] == 0:
            return (x, y)

# Run multiple trials
astar_times = []
dijkstra_times = []
astar_lengths = []
dijkstra_lengths = []

for run in range(num_runs):
    # Reset grid and obstacles for each run
    grid = np.zeros(grid_size, dtype=int)
    num_obstacles = int(grid_size[0] * grid_size[1] * obstacle_ratio)
    obstacle_indices = random.sample(range(grid_size[0] * grid_size[1]), num_obstacles)
    for idx in obstacle_indices:
        x, y = divmod(idx, grid_size[1])
        grid[x, y] = 1

    start = random_free_cell()
    goal = random_free_cell()
    while goal == start:
        goal = random_free_cell()

    # Run A* and measure time
    start_time = time.time()
    astar_path = astar(grid, start, goal)
    astar_time = time.time() - start_time
    astar_times.append(astar_time)
    astar_lengths.append(len(astar_path))

    # Run Dijkstra and measure time
    start_time = time.time()
    dijkstra_path = dijkstra(grid, start, goal)
    dijkstra_time = time.time() - start_time
    dijkstra_times.append(dijkstra_time)
    dijkstra_lengths.append(len(dijkstra_path))

# Save results to CSV (updated precision)
with open('pathfinding_results.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Run', 'A* Time (s)', 'Dijkstra Time (s)', 'A* Path Length', 'Dijkstra Path Length'])
    for i in range(num_runs):
        writer.writerow([
            i + 1,
            f"{astar_times[i]:.10f}",
            f"{dijkstra_times[i]:.10f}",
            astar_lengths[i],
            dijkstra_lengths[i]
        ])

# Calculate averages
avg_astar_time = np.mean(astar_times)
avg_dijkstra_time = np.mean(dijkstra_times)
avg_astar_length = np.mean(astar_lengths)
avg_dijkstra_length = np.mean(dijkstra_lengths)

print(f"A* average time: {avg_astar_time:.6f}s, average path length: {avg_astar_length}")
print(f"Dijkstra average time: {avg_dijkstra_time:.6f}s, average path length: {avg_dijkstra_length}")

# Plot comparison with better styling
plt.figure(figsize=(14, 8))

# Time comparison plot with transparency
plt.subplot(1, 2, 1)
plt.plot(range(1, num_runs + 1), astar_times, label="A* Time", color='royalblue', alpha=0.7, linewidth=2)
plt.plot(range(1, num_runs + 1), dijkstra_times, label="Dijkstra Time", color='tomato', alpha=0.7, linewidth=2)
plt.fill_between(range(1, num_runs + 1), astar_times, dijkstra_times, where=(np.array(astar_times) > np.array(dijkstra_times)), facecolor='lightblue', alpha=0.3)
plt.fill_between(range(1, num_runs + 1), astar_times, dijkstra_times, where=(np.array(astar_times) < np.array(dijkstra_times)), facecolor='lightcoral', alpha=0.3)
plt.xlabel('Run')
plt.ylabel('Time (seconds)')
plt.title('Time Comparison (A* vs Dijkstra)')
plt.legend()

# Path length comparison plot with transparency
plt.subplot(1, 2, 2)
plt.plot(range(1, num_runs + 1), astar_lengths, label="A* Path Length", color='royalblue', alpha=0.7, linewidth=2)
plt.plot(range(1, num_runs + 1), dijkstra_lengths, label="Dijkstra Path Length", color='tomato', alpha=0.7, linewidth=2)
plt.fill_between(range(1, num_runs + 1), astar_lengths, dijkstra_lengths, where=(np.array(astar_lengths) > np.array(dijkstra_lengths)), facecolor='lightblue', alpha=0.3)
plt.fill_between(range(1, num_runs + 1), astar_lengths, dijkstra_lengths, where=(np.array(astar_lengths) < np.array(dijkstra_lengths)), facecolor='lightcoral', alpha=0.3)
plt.xlabel('Run')
plt.ylabel('Path Length')
plt.title('Path Length Comparison (A* vs Dijkstra)')
plt.legend()

plt.tight_layout()
plt.show()
