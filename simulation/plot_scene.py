import numpy as np
import matplotlib.pyplot as plt

# Function to load data from a .txt file
def load_data(filename):
    data = np.genfromtxt(filename, delimiter=None, skip_header=2)  # Auto-detect delimiter
    return data

def cal_resultant_force(position_file, force_file):
    positions = load_data(position_file)
    forces = load_data(force_file)
        # Ensure both have the same length

    # Extract time and values
    time = positions[:, 0]  # First column is the timestamp
    x_force, y_force, z_force = forces[:, 1], forces[:, 2], forces[:, 3]

    # Compute Resultant Force
    resultant_force = np.sqrt(x_force**2 + y_force**2 + z_force**2)
    resultant_force = resultant_force[:len(time)]
    return max(resultant_force)
