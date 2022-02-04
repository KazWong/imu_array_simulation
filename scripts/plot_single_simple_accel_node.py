#!/usr/bin/env python3

import math
import rospy
import time
import  threading
from std_msgs.msg import Float64MultiArray
from matplotlib.pylab import *
import matplotlib.animation as animation

grid_num = 3
plot_per_grid = 2
xlabel = 'time(sec)'
ylabel = 'accel'

fig = figure(num = 0, figsize=(10,10))

lock = []
ax = []
for i in range(grid_num):
    lock.append(threading.Lock())
    ax.append(subplot2grid((grid_num, 1), (i, 0)))

ylim = [5.0, 5.0, 12.0]# [1.3]*grid_num
for i in range(grid_num):
    ax[i].set_ylim(-ylim[i], ylim[i])
    ax[i].set_xlim(0.0, 20.0)
    ax[i].grid(True)
    ax[i].set_ylabel(ylabel)
ax[grid_num - 1].set_xlabel(xlabel)
tight_layout()

t = [ [[] for _ in range(plot_per_grid)] for _ in range(grid_num) ] # [[] for _ in range(grid_num)]
ydata = [ [[] for _ in range(plot_per_grid)] for _ in range(grid_num) ]
py = [[] for _ in range(grid_num)]

for i in range(grid_num):
    tmp, = ax[i].plot(t[i][0],ydata[i][0],'b-', label="measured")
    py[i].append(tmp)
    tmp, = ax[i].plot(t[i][1],ydata[i][1],'g-', label="estimated")
    py[i].append(tmp)

for i in range(grid_num):
    legend_x = []
    legend_y = []
    for j in range(plot_per_grid):
        legend_x.append(py[i][j])
        legend_y.append(py[i][j].get_label())
    ax[i].legend(legend_x, legend_y)


t_base = time.time()
t_step = [0.0]*plot_per_grid
def callback(pos, msg):
	global t, ydata, t_step, t_base

	t_step[pos] = time.time() - t_base

	lock[pos].acquire()
	for i in range(grid_num):
		t[i][pos] = append(t[i][pos], t_step[pos])
		ydata[i][pos] = append(ydata[i][pos], msg.data[i])

	if (len(ydata[0][pos]) > 1800):
		for i in range(grid_num):
			t[i][pos] = np.delete(t[i][pos], 0)
			ydata[i][pos] = np.delete(ydata[i][pos], 0)
	lock[pos].release()

def callback1(msg):
	callback(0, msg)
def callback2(msg):
	callback(1, msg)

def update(frame):
	global py, ydata, t

	for i in range(plot_per_grid):
		for j in range(grid_num):
			lock[i].acquire()
			py[j][i].set_data(t[j][i], ydata[j][i])
			lock[i].release()

	for i in range(grid_num):
		max_ydata = np.amax(ydata[i][0])
		min_ydata = np.amin(ydata[i][0])

		for j in range(plot_per_grid):
			lock[i].acquire()
			plot_min = np.amin(ydata[i][j])
			plot_max = np.amax(ydata[i][j])
			if plot_min < min_ydata:
				min_ydata = plot_min
			if plot_max > max_ydata:
				max_ydata = plot_max
			lock[i].release()

		if len(ydata[i]) > 0:
			py[i][0].axes.set_ylim(min_ydata - 0.1, max_ydata + 0.1)

		if len(t[i][0]) > 0 and t[i][0][-1] >= 17.00:
			py[i][0].axes.set_xlim(t[i][0][-1] - 17.0,t[i][0][-1] + 3.0)

	return py[0][0], py[1][0], py[2][0], py[0][1], py[1][1], py[2][1]

if __name__ == '__main__':
	rospy.init_node('plot_single_accel_node_data', anonymous=True)
	rospy.Subscriber("measured_data", Float64MultiArray, callback1)
	rospy.Subscriber("estimated_data", Float64MultiArray, callback2)

	ani = animation.FuncAnimation(fig, update, frames=100, interval=20, blit=False)
	plt.show()
	rospy.spin()

	#try:
	#  Puber()
	#except rospy.ROSInterruptException:
	#  pass
