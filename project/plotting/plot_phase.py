import matplotlib
import matplotlib.pyplot as plt

import numpy as np

# data = np.genfromtxt('roll_theta_thetadot.csv', delimiter=',', skip_header=1)
# data = np.genfromtxt('output.csv', delimiter=',', skip_header=1)
data = np.genfromtxt('backandforth_velpos.csv', delimiter=',', skip_header=1)

fig, ax = plt.subplots()
data = data.T
print(data.shape[1])

# for i in range(int((data.shape[0]-1)/2)):
#     ax.plot(data[i+5], data[i], label=i)

# t_mark = [1]
# t_mark = [1,4]
t_mark = [1, 1.39, 1.44, 1.58, 1.7, 4]
# t_mark = [1,4,7]
sample_rate = 0.005

i_mark = [int(t / sample_rate) for t in t_mark]
print(i_mark)

t_range =[0, data.shape[1]/2]
i_choose = 3
ax.plot(data[i_choose+5+1], data[i_choose+1])
# ax.scatter(data[i_choose+5], data[i_choose], label=i_choose)
ax.scatter(data[i_choose+5+1][i_mark], data[i_choose+1][i_mark], color="tab:orange")
# ax.set_xlim([-np.pi/6, np.pi/6])
ax.set_xlabel(r'$\theta$')
ax.set_ylabel(r'$\frac{d\theta}{dt}$')

# ax.legend()
plt.show()
