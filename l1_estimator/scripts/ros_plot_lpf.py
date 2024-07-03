#! /usr/bin/env python3.8
import numpy as np
import rospy
import message_filters
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation
from l1_estimator.msg import Lpf_test


class PlotNode():
    def __init__(self):
        # lpf_input
        self.lpf_input = np.zeros((3,))

        self.x_data = []
        self.y_data = []

        self.ux_data = []
        self.uy_data = []

        self.fig, self.ax = plt.subplots()

        self.line1 = self.ax.plot([],[],label='data')[0]
        self.ax.set(xlim=[0, 5], ylim=[-7, 7])

        self.t_curr = 0
        self.t_offset = 0
        self.first_callback = False

        self.ros_setup()



    def ros_setup(self):
        self.lpf_input_subscriber = rospy.Subscriber('/input_signal', Lpf_test, self.callback_data, queue_size=1)


    def callback_data(self, msg1):

        if self.first_callback == False:
            self.first_callback = True
            self.t_offset = rospy.Time.now().to_sec() + rospy.Time.now().to_nsec()*1e-9

        self.temp_time = 0
        self.temp_time = rospy.Time.now().to_sec() + rospy.Time.now().to_nsec()*1e-9
        self.t_curr = self.temp_time - self.t_offset

        self.lpf_input[:] = msg1.v[:]
        self.x_data.append(self.t_curr)
        self.y_data.append(self.lpf_input[0])
        print(self.t_curr, self.lpf_input[0])

        if len(self.x_data) > 500:
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.ax.set(xlim=[self.t_curr-2.5, self.t_curr+2.5], ylim=[-7, 7])

    def update(self, frame):
        self.line1.set_xdata(self.x_data[:frame])
        self.line1.set_ydata(self.y_data[:frame])
        return self.line1

if __name__ == '__main__':
    rospy.init_node('plot_node')
    plot_node = PlotNode()
    ani = animation.FuncAnimation(plot_node.fig, plot_node.update, interval=1)
    plt.show()
    rospy.spin()