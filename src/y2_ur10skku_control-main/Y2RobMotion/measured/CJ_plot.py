import numpy as np
import matplotlib.pyplot as plt



data = np.loadtxt("currentJ.txt")


num_rows, num_cols = data.shape

print(f"Loaded data was {num_rows} rows, {num_cols} cols.")


plt.figure(figsize=(10,6))
for i in range(num_cols):
    plt.plot(data[:,i], label=f"column {i+1}")


plt.title("Columns from currentJ.txt")
plt.xlabel("Row Index")
plt.ylabel("Value")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()