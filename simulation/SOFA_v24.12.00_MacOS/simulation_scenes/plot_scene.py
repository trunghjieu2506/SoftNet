import numpy as np
import matplotlib.pyplot as plt

# Function to load data from a .txt file
def load_data(filename):
    data = np.genfromtxt(filename, delimiter=None, skip_header=2)  # Auto-detect delimiter
    return data

# Load data from generated .txt files
positions = load_data(r"/Users/trunghjieu/Desktop/SSLSoftneet/simulation/SOFA_v24.12.00_MacOS/fingerMonitorA_x.txt")
positionsB = load_data(r"/Users/trunghjieu/Desktop/SSLSoftneet/simulation/SOFA_v24.12.00_MacOS/fingerMonitorB_x.txt")
#velocities = load_data(r"C:\Users\justi\SOFA\v24.06.00\fingerMonitor_v.txt")
forces = load_data(r"/Users/trunghjieu/Desktop/SSLSoftneet/simulation/SOFA_v24.12.00_MacOS/fingerMonitorA_f.txt")

# Ensure both have the same length
min_len = min(len(positions), len(positionsB))
positions = positions[:min_len]
positionsB = positionsB[:min_len]

# Extract time and values
time = positions[:, 0]  # First column is the timestamp
x_pos, y_pos, z_pos = positions[:, 1], positions[:, 2], positions[:, 3]
x_posB, y_posB, z_posB = positionsB[:, 1], positionsB[:, 2], positionsB[:, 3]
x_force, y_force, z_force = forces[:, 1], forces[:, 2], forces[:, 3]

# Compute Resultant Displacement (distance from origin)
resultant_disp = np.sqrt(x_pos**2 + y_pos**2 + z_pos**2)

# Compute Resultant Force
resultant_force = np.sqrt(x_force**2 + y_force**2 + z_force**2)

# take the perpendicular vector of the top surface, (x_pos - 10, 0, 0)
top_surface_vector = -1 * np.column_stack((x_posB - x_pos, y_posB - y_pos, z_posB - z_pos))
axis_vector = np.array([0, 0, -120])

# Trim arrays to match the length of time
resultant_disp = resultant_disp[:len(time)]
resultant_force = resultant_force[:len(time)]
# send back up to the training routine 
print("max result force", np.max(resultant_force))

# Compute Bending Angle
top_surface_unit = top_surface_vector / np.linalg.norm(top_surface_vector, axis=1, keepdims=True)

# reference_vector = np.array([-1, 0, 0])  # Along the x-axis

# # Compute the normal by taking the cross product
# bending_vector = np.cross(top_surface_vector, reference_vector)

# bending_vector = np.column_stack((
#     -top_surface_unit[:, 2],  # -z
#     np.zeros_like(y_pos),     # no y component
#     top_surface_unit[:, 0]    # x
# ))
bending_vector = 2 * top_surface_vector
axis_matrix = np.tile(axis_vector, (len(bending_vector), 1))

# Dot product and norms
magnitudes = np.linalg.norm(bending_vector, axis=1) * 120

# Avoid numerical errors with clip
dot_products = np.einsum('ij,j->i', bending_vector, axis_vector)
theta_xz = np.arccos(dot_products / magnitudes) * (180 / np.pi) - 15

# Compute torque
torque = np.abs(resultant_force * z_pos * np.sin(np.deg2rad(theta_xz)))

# # Create subplots
## **Figure 1: Displacement & Force**
fig1, axs1 = plt.subplots(2, 1, figsize=(10, 12))  # 2 rows, 1 column

# Plot **Resultant Displacement**
axs1[0].plot(time, resultant_disp, label='Resultant Displacement', color='b')
axs1[0].set_xlabel("Time (s)")
axs1[0].set_ylabel("Displacement (m)")
axs1[0].set_title("Resultant Displacement Over Time")
axs1[0].legend()
axs1[0].grid()

# Plot **Resultant Force**
axs1[1].plot(time, resultant_force, label='Resultant Force', color='r')
axs1[1].set_xlabel("Time (s)")
axs1[1].set_ylabel("Force (N)")
axs1[1].set_title("Resultant Force Over Time")
axs1[1].legend()
axs1[1].grid()

plt.tight_layout(pad=3.0)
# plt.show()  # Show first figure

### **Figure 2: Bending Angle & Torque**
fig2, axs2 = plt.subplots(2, 1, figsize=(10, 12))  # 2 rows, 1 column

# Plot **Bending Angle**
axs2[0].plot(time, theta_xz, label='Bending Angle', color='orange')
axs2[0].set_xlabel("Time (s)")
axs2[0].set_ylabel("Bending Angle (degrees)")
axs2[0].set_title("Bending Angle Over Time")
axs2[0].legend()
axs2[0].grid()

# Plot **Torque**
axs2[1].plot(time, torque, label='Torque', color='g')
axs2[1].set_xlabel("Time (s)")
axs2[1].set_ylabel("Torque (Nm)")
axs2[1].set_title("Torque Over Time")
axs2[1].legend()
axs2[1].grid()

plt.tight_layout(pad=3.0)
plt.show()  # Show second figure
percent_disp = (resultant_disp / np.max(resultant_disp)) * 100
percent_disp = ((resultant_disp - resultant_disp[0]) / (np.max(resultant_disp) - resultant_disp[0])) * 100

# plt.figure(figsize=(10, 6))
# plt.plot(percent_disp, resultant_force, color='purple')
# plt.xlabel("Displacement (%)")
# plt.ylabel("Resultant Force (N)")
# plt.title("Resultant Force vs Displacement (%)")
# plt.grid(True)
# plt.show()
