import numpy as np
import h5py

# Input and output paths
txt_file = "hand_g_recording.txt"
h5_file = "trajectory.h5"

# Column names: position (x,y,z), orientation (roll,pitch,yaw), force (Fx,Fy,Fz)
columns = ["x", "y", "z", "roll", "pitch", "yaw", "Fx", "Fy", "Fz"]

# Load text data
data = np.loadtxt(txt_file)

# Save to HDF5
with h5py.File(h5_file, "w") as f:
    for i, col in enumerate(columns):
        f.create_dataset(col, data=data[:, i])

print(f"✅ Converted {txt_file} to {h5_file}")

