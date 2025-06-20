#     __________  _____ ____     __  __      _ __        
#    / ____/ __ \/ ___// __ \   / / / /___  (_) /_  ____ 
#   / /   / /_/ /\__ \/ /_/ /  / / / / __ \/ / __ \/ __ \
#  / /___/ ____/___/ / ____/  / /_/ / / / / / /_/ / /_/ /
#  \____/_/    /____/_/       \____/_/ /_/_/_.___/\____/ 
                                                       
                                                               
#  Authors: Filippo Samorì, Filippo Ugolini and Daniele Crivellari
#  20/06/2025
#  University of Bologna, Italy
#  License: BSD-3-Clause

#  This script processes a log file from a Crazyflie drone, extracting position data and delays,
#  and visualizes the trajectory in 3D along with delays and frames per second (FPS) data.
#  It also computes a weighted mean of FPS values over a specified window size.


import re
import numpy as np
import matplotlib.pyplot as plt

logfile = "console_crazyflie.txt"
fps_file = "fps.txt"

positions = []
delays = []

with open(logfile, "r") as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    line = lines[i]
    if "Position" in line:
        # Merge the current line and the next one (data is split across two lines)
        data_line = line
        # Look for following lines starting with ":" (split position data)
        while i + 1 < len(lines) and lines[i + 1].strip().startswith(":"):
            data_line += lines[i + 1]
            data_line += lines[i + 2]
            i += 1
            data_line = data_line.replace('\n', '')
        # Extract numbers and delay
        match = re.search(r":\s*([-\d\.\s]+)delay:\s*(-?\d+)", data_line)
        if match:
            pos_str = match.group(1)
            pos = [float(x) for x in pos_str.split()]
            delay = int(match.group(2))
            positions.append(pos)
            delays.append(delay)
    i += 1

positions = np.array(positions)
delays = np.array(delays)

print("positions shape:", positions.shape)
print("delays shape:", delays.shape)

# Plotting the positions in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('3D Trajectory of Crazyflie')
ax.grid(True)
plt.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'o-')
plt.show()

# Plotting the delays
plt.figure()
plt.plot(delays[1:], label='Delay')
plt.xlabel('Sample Index')
plt.ylabel('Delay (ms)')
plt.title('Delays in Position Data')
plt.legend()
plt.grid(True)
plt.show()

# Read FPS values from file
with open(fps_file, "r") as f:
    lines = f.readlines()

fps = [int(line.strip()) for line in lines if line.strip().isdigit()]

# Plot FPS values and mean
plt.figure()
plt.plot(fps, label='FPS')
fps_mean = np.mean(fps)
fps_mean = int(fps_mean)
plt.axhline(y=fps_mean, color='orange', linestyle='--', label='Mean FPS')

plt.xlabel('Sample Index')
plt.ylabel('FPS')
plt.title('Frames Per Second (FPS)')
plt.legend()
plt.grid(True)
plt.show()

window_size = 50  # Number of data points in the window
half_window = window_size // 2

fps_weighted_mean = []
for i in range(len(fps)):
    start = i
    end = window_size + i
    if end > len(fps):
        end = len(fps)
    window = fps[start:end]
    weighted_mean = np.mean(window)
    fps_weighted_mean.append(weighted_mean)

# Plot FPS and weighted mean FPS
plt.figure()
plt.plot(fps, label='FPS')
plt.plot(fps_weighted_mean, label=f'Weighted Mean FPS (window={window_size})', color='orange')
plt.xlabel('Sample Index')
plt.ylabel('FPS')
plt.title('Frames Per Second (FPS)')
plt.legend()
plt.grid(True)
plt.show()
