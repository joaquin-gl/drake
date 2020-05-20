import matplotlib
import matplotlib.pyplot as plt

import numpy as np

data = np.genfromtxt('output.csv', delimiter=',', skip_header=1)

fig, ax = plt.subplots()
print(data.shape[1])
data = data.T
print(data.shape[1])
print(data[-1][0])
for i in range(data.shape[0]-1):
    # ax.plot(data[-1], data[i], label=i-1)
    ax.plot(data[0], data[i+1]*100)

# ax.legend()
ax.legend(["x", "y", "z"])
ax.set_xlabel("time (s)")
ax.set_ylabel("translation (cm)")
plt.show()
