'''
    Visualization code for sanity checking simulation.
'''

import os

from simulation import Plane, unsafe_state

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import patches, animation
from matplotlib.image import BboxImage
from matplotlib.transforms import Bbox, TransformedBbox
from matplotlib.collections import LineCollection
from matplotlib.path import Path
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation

ownship_color = (0, 0, 1.0)
intruder_color = (1.0, 0, 0)

min_alpha = 0.2

ownship_idx = 0

colors = [ownship_color, intruder_color]

def generate_animation(trajectories, tau):
    trajectories = np.array(trajectories)
    axis_max = np.max(trajectories.flatten())
    axis_min = np.min(trajectories.flatten())

    fig, ax = plt.subplots()

    ax.set_ylim(axis_min - 1000, axis_max + 1000)
    ax.set_xlim(axis_min - 1000, axis_max + 1000)

    ownship_x = [x for x, y in trajectories[0]]
    ownship_y = [y for x, y in trajectories[0]]
    intruder_x = [x for x, y in trajectories[1]]
    intruder_y = [y for x, y in trajectories[1]]


    plt.grid()
    ax.set_axisbelow(True)

    plt.xlabel("X Position")
    plt.ylabel("Y Position")

    ax.plot(ownship_x, ownship_y)
    ax.plot(ownship_x[0], ownship_y[0], 'b.')
    plt.arrow(ownship_x[-1], ownship_y[-1], 1, 10, shape='full', lw=0, length_includes_head=False, head_width=500)
    ax.plot(intruder_x, intruder_y)
    ax.plot(intruder_x[0], intruder_y[0], 'r.')
    plt.arrow(intruder_x[-1], intruder_y[-1], 70, 1, color='r', shape='full', lw=0, length_includes_head=False, head_width=500)


    # def animate(t):
    #     for v in range(len(trajectories)):
    #         traj = trajectories[v]
    #         if v == ownship_idx:
    #             alpha = 1.0
    #         else:
    #             alpha = 0
    #         # else:
    #         #     alpha = (tau / t) if tau <= t else np.max(t / tau, min_alpha)
    #         ax.plot([i for i, j in traj][:t], [j for i, j in traj][:t], color = colors[v], alpha = 0.1)

    # ani = animation.FuncAnimation(fig, animate, interval=20, save_count = 50)

    plt.show()