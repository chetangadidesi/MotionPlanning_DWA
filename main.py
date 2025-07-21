# main.py
import math
import numpy as np
import matplotlib.pyplot as plt

from config import Config, RobotType
from motion import motion
from planner import dwa_control
from utils import generate_random_goal, generate_random_obstacles, plot_robot, plot_arrow

show_animation = True

def main():
    print("🚀 DWA Random Planner Start")
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    gx, gy = generate_random_goal()
    goal = np.array([gx, gy])
    config = Config()
    config.robot_type = RobotType.rectangle
    config.ob = np.array(generate_random_obstacles())

    trajectory = np.array(x)

    while True:
        u, predicted_trajectory = dwa_control(x, config, goal, config.ob)
        x = motion(x, u, config.dt)
        trajectory = np.vstack((trajectory, x))

        if show_animation:
            plt.cla()
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(config.ob[:, 0], config.ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

        if math.hypot(x[0] - goal[0], x[1] - goal[1]) <= config.robot_radius:
            print(f"🎯 Goal Reached at: {goal}")
            break

    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.show()

if __name__ == "__main__":
    main()
