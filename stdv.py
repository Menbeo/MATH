import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the data from CSV
df = pd.read_csv("pathfinding_results.csv")  # Replace with your actual file name
df.columns = [col.strip() for col in df.columns]  # Clean column names

# Extract relevant columns
astar_times = df['A* Time (s)'].values
astar_lengths = df['A* Path Length'].values
dijkstra_times = df['Dijkstra Time (s)'].values
dijkstra_lengths = df['Dijkstra Path Length'].values

# Calculate statistics
avg_astar_time = np.mean(astar_times)
std_astar_time = np.std(astar_times)
avg_dijkstra_time = np.mean(dijkstra_times)
std_dijkstra_time = np.std(dijkstra_times)

avg_astar_length = np.mean(astar_lengths)
std_astar_length = np.std(astar_lengths)
avg_dijkstra_length = np.mean(dijkstra_lengths)
std_dijkstra_length = np.std(dijkstra_lengths)

# Calculate improvement percentages
time_improvement = (avg_dijkstra_time - avg_astar_time) / avg_dijkstra_time * 100
length_improvement = (avg_dijkstra_length - avg_astar_length) / avg_dijkstra_length * 100

# Print results
print(f"A* average time: {avg_astar_time:.6f}s ± {std_astar_time:.6f}, path length: {avg_astar_length:.2f} ± {std_astar_length:.2f}")
print(f"Dijkstra average time: {avg_dijkstra_time:.6f}s ± {std_dijkstra_time:.6f}, path length: {avg_dijkstra_length:.2f} ± {std_dijkstra_length:.2f}")
print(f"A* is faster by {time_improvement:.2f}% in execution time")
print(f"A* is more optimal by {length_improvement:.2f}% in path length")

# Bar chart setup
labels = ['A*', 'Dijkstra']

# Times and standard deviations
times = [avg_astar_time, avg_dijkstra_time]
time_errors = [std_astar_time, std_dijkstra_time]

# Path lengths and standard deviations
lengths = [avg_astar_length, avg_dijkstra_length]
length_errors = [std_astar_length, std_dijkstra_length]

x = np.arange(len(labels))
width = 0.35

fig, axs = plt.subplots(1, 2, figsize=(12, 6))

# Execution time bar chart
axs[0].bar(x, times, yerr=time_errors, capsize=10, color=['royalblue', 'tomato'])
axs[0].set_title("Average Execution Time")
axs[0].set_xticks(x)
axs[0].set_xticklabels(labels)
axs[0].set_ylabel("Time (seconds)")

# Path length bar chart
axs[1].bar(x, lengths, yerr=length_errors, capsize=10, color=['royalblue', 'tomato'])
axs[1].set_title("Average Path Length")
axs[1].set_xticks(x)
axs[1].set_xticklabels(labels)
axs[1].set_ylabel("Steps (cells)")

plt.tight_layout()
plt.show()
