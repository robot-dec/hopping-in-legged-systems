import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from math import sin, cos


def get_body_coords(center, width, height, theta):
    coords = np.transpose(np.array([
        [-1 * width / 2.0, -1 * height / 2.0],
        [width / 2.0, -1 * height / 2.0],
        [width / 2.0, height / 2.0],
        [-1 * width / 2.0, height / 2.0],
        [-1 * width / 2.0, -1 * height / 2.0]]))
    rot = np.array([[cos(theta), sin(theta)], [-1 * sin(theta), cos(theta)]])
    rot_coords = rot @ coords
    return rot_coords[0, :] + center[0], rot_coords[1, :] + center[1]


def get_leg_coords(foot, width, height, theta):
    coords = np.transpose(np.array([
        [-1 * width / 2.0, 0.0],
        [width / 2.0, 0.0],
        [width / 2.0, height],
        [-1 * width / 2.0, height],
        [-1 * width / 2.0, 0.0]]))
    rot = np.array([[cos(theta), sin(theta)], [-1 * sin(theta), cos(theta)]])
    rot_coords = rot @ coords
    return rot_coords[0, :] + foot[0], rot_coords[1, :] + foot[1]


def get_stem_coords(foot, body, leg_length, theta):
    coords = np.transpose(np.array([
        [foot[0] + leg_length * sin(theta), foot[1] + leg_length * cos(theta)],
        [body[0], body[1]]]))
    return coords[0, :], coords[1, :]


def animate_solution(t, x, y):
    fig = plt.figure(figsize=(8, 4), dpi=100)

    frames = len(t)

    theta0, theta1, x0, y0, w = x[0], x[1], x[2], x[3], x[4]
    x1, y1, x2, y2 = y[0], y[1], y[2], y[3]

    ax = plt.subplot(1, 1, 1, aspect=1, frameon=False)
    (l1,) = ax.plot(
        [],
        [],
        color="C0",
        clip_on=False,
        markevery=[-1],
        markeredgewidth=2,
        markerfacecolor="C0",
        markeredgecolor="white",
    )
    (l2,) = ax.plot(
        [],
        [],
        color="C0",
        clip_on=False,
        markevery=[-1],
        markeredgewidth=2,
        markerfacecolor="C0",
        markeredgecolor="white",
    )
    (l3,) = ax.plot(
        [],
        [],
        color="C0",
        clip_on=False,
        markevery=[-1],
        markeredgewidth=2,
        markerfacecolor="C0",
        markeredgecolor="white",
    )

    ax.set_xlim([min(min(x0), min(x1), min(x2)) - 0.5,
                 max(max(x0), max(x1), max(x2)) + 0.5])
    ax.set_xticks([])
    ax.set_ylim([min(min(y0), min(y1), min(y2)) - 0.5,
                 max(max(y0), max(y1), max(y2)) + 0.5])
    ax.set_yticks([])

    square_1 = []
    square_2 = []
    square_3 = []
    for i in range(frames):
        square_1.append(
            get_leg_coords((x0[i], y0[i]), 0.1, w[i], theta0[i]))
        square_2.append(
            get_body_coords((x2[i], y2[i]), 0.5, 0.25, theta1[i]))
        square_3.append(
            get_stem_coords((x0[i], y0[i]), (x2[i], y2[i]), w[i], theta0[i]))

    def animate(frame):
        l1.set_data(square_1[frame][0], square_1[frame][1])
        l2.set_data(square_2[frame][0], square_2[frame][1])
        l3.set_data(square_3[frame][0], square_3[frame][1])
        return l1, l2, l3, 

    delta = (t[-1] - t[0]) / len(t) * 1000
    ani = animation.FuncAnimation(fig, animate, interval=delta, frames=frames)
    plt.show()
