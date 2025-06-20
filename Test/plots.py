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
        # Unisci la riga corrente e la successiva (i dati sono spezzati su due righe)
        data_line = line
        # Cerca le righe successive che iniziano con ":" (posizione spezzata)
        while i + 1 < len(lines) and lines[i + 1].strip().startswith(":"):
            data_line += lines[i + 1]
            data_line += lines[i + 2]
            i += 1
            data_line = data_line.replace('\n', '')
        # Estrai i numeri e il delay
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

# Plotting the positions
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('3D Trajectory of Crazyflie')
ax.grid(True)
plt.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'o-')
plt.show()

plt.figure()
plt.plot(delays, label='Delay')
plt.xlabel('Sample Index')
plt.ylabel('Delay (ms)')
plt.title('Delays in Position Data')
plt.legend()
plt.grid(True)
plt.show()

with open(fps_file, "r") as f:
    lines = f.readlines()

fps = [int(line.strip()) for line in lines if line.strip().isdigit()]
print("FPS values:", fps)

plt.figure()
plt.plot(fps, label='FPS')
fps_mean = np.mean(fps)
#fps_mean = int(fps_mean)
plt.axhline(y=fps_mean, color='r', linestyle='--', label='Mean FPS')
plt.xlabel('Sample Index')
plt.ylabel('FPS')
plt.title('Frames Per Second (FPS)')
plt.legend()
plt.grid(True)
plt.show()

# Salva su file se vuoi
# np.save("positions.npy", positions)
# np.save("delays.npy", delays)