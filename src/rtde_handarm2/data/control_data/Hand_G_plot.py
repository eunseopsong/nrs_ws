import matplotlib.pyplot as plt
import numpy as np

data2 = np.loadtxt('hand_g_recording.txt')

PosiX = data2[:, 0]  # X position
PosiY = data2[:, 1]  # Y position
PosiZ = data2[:, 2]  # Z position
OriR = data2[:, 3]   # Roll orientation
OriP = data2[:, 4]   # Pitch orientation
OriY = data2[:, 5]   # Yaw orientation
Fz = data2[:, 8]     # Force Z

# Position (X, Y, Z) in subplots
plt.figure(num=1, dpi=100, facecolor='white')

plt.subplot(3, 1, 1)
plt.plot(PosiX, 'k-', label="Position X")
plt.title('Position X')
plt.xlabel('time')
plt.ylabel('dist.')
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(PosiY, 'k-', label="Position Y")
plt.title('Position Y')
plt.xlabel('time')
plt.ylabel('dist.')
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(PosiZ, 'k-', label="Position Z")
plt.title('Position Z')
plt.xlabel('time')
plt.ylabel('dist.')
plt.grid()

plt.tight_layout()  # Adjust spacing between subplots

# Orientation (Roll, Pitch, Yaw) in subplots
plt.figure(num=2, dpi=100, facecolor='white')

plt.subplot(3, 1, 1)
plt.plot(OriR, 'k-', label="Orientation Roll")
plt.title('Orientation Roll')
plt.xlabel('time')
plt.ylabel('dist.')
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(OriP, 'k-', label="Orientation Pitch")
plt.title('Orientation Pitch')
plt.xlabel('time')
plt.ylabel('dist.')
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(OriY, 'k-', label="Orientation Yaw")
plt.title('Orientation Yaw')
plt.xlabel('time')
plt.ylabel('dist.')
plt.grid()

plt.tight_layout()  # Adjust spacing between subplots

# Force Z plot
plt.figure(num=3, dpi=100, facecolor='white')
plt.plot(Fz, 'k-', label="Force Z")
plt.title('Force Z')
plt.xlabel('time')
plt.ylabel('dist.')
plt.grid()

plt.show()
