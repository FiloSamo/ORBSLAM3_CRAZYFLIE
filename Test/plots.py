#     __________  _____ ____     __  __      _ __        
#    / ____/ __ \/ ___// __ \   / / / /___  (_) /_  ____ 
#   / /   / /_/ /\__ \/ /_/ /  / / / / __ \/ / __ \/ __ \
#  / /___/ ____/___/ / ____/  / /_/ / / / / / /_/ / /_/ /
#  \____/_/    /____/_/       \____/_/ /_/_/_.___/\____/ 
                                                       
                                                               
#  Authors: Filippo Samorì, Filippo Ugolini and Daniele Crivellari
#  20/06/2025
#  University of Bologna, Italy
#  License: GPL-3.0

#  This script processes a log file from a Crazyflie drone, extracting position data and latency times,
#  and visualizes the trajectory in 3D along with latency and frames per second (FPS) data.
#  It also computes a weighted mean of FPS values over a specified window size.


import re
import numpy as np
import matplotlib.pyplot as plt

logfile = "console_crazyflie_demo.txt"
fps_file = "fps.txt"

positions = []
latency = []

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
            pos = [np.float32(x) for x in pos_str.split()]
            delay = int(match.group(2))
            positions.append(pos)
            latency.append(delay)
    i += 1

positions = np.array(positions)
latency = np.array(latency)

# print("positions shape:", positions.shape)
# print("latency shape:", latency.shape)

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


# Plotting the latency times with mean 

plt.figure()
plt.plot(latency[4:], label='Latency', color='blue')
latency_mean = np.mean(latency)
plt.axhline(y=latency_mean, color='orangered', linestyle='--', label='Mean Latency')

plt.xlabel('Sample Index')
plt.ylabel('Latency time (ms)')
plt.title('Latency in Position Feedback')
plt.legend(loc='upper right')
plt.grid(True)
plt.show()

# Read FPS values from file
with open(fps_file, "r") as f:
    lines = f.readlines()

fps = [int(line.strip()) for line in lines if line.strip().isdigit()]

# Plot FPS values with mean
plt.figure()
plt.plot(fps[1:270], label='FPS', color='blue')
fps_mean = np.mean(fps)
plt.axhline(y=fps_mean, color='orangered', linestyle='--', label='Mean FPS')

plt.xlabel('Seconds')
plt.ylabel('FPS')
plt.title('Frames Per Second (FPS)')
plt.legend(loc='upper right')
plt.grid(True)
plt.show()
