import numpy as np
import matplotlib.pyplot as plt

lo = np.loadtxt('../results/seq1/lo.txt')
vo = np.loadtxt('../results/seq1/vo.txt')
mo = np.loadtxt('../results/seq1/mo.txt')

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.scatter(lo[:,0], lo[:,1], lo[:,2], c='r')
ax.scatter(vo[:,0], vo[:,1], vo[:,2], c='g')
ax.scatter(mo[:,0], mo[:,1], mo[:,2], c='b')
plt.show()
