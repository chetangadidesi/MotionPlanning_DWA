# utils.py
import math
import random
import numpy as np
import matplotlib.pyplot as plt

def generate_random_obstacles(n=15, map_size=(20, 20), min_dist_from_origin=2.0):
    obs = []
    while len(obs) < n:
        x = random.uniform(0, map_size[0])
        y = random.uniform(0, map_size[1])
        if math.hypot(x, y) > min_dist_from_origin:
            obs.append([x, y])
    return obs

def generate_random_goal(map_size=(20, 20), min_dist=5.0):
    while True:
        gx = random.uniform(0, map_size[0])
        gy = random.uniform(0, map_size[1])
        if math.hypot(gx, gy) > min_dist:
            return gx, gy

def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def plot_robot(x, y, yaw, config):
    if config.robot_type.name == "rectangle":
        outline = [[-config.robot_length / 2, config.robot_length / 2,
                    config.robot_length / 2, -config.robot_length / 2,
                    -config.robot_length / 2],
                   [config.robot_width / 2, config.robot_width / 2,
                    -config.robot_width / 2, -config.robot_width / 2,
                    config.robot_width / 2]]
        Rot1 = [[math.cos(yaw), math.sin(yaw)],
                [-math.sin(yaw), math.cos(yaw)]]
        outline = (np.array(outline).T @ np.array(Rot1)).T
        outline[0] += x
        outline[1] += y
        plt.plot(outline[0], outline[1], "-k")
    else:
        circle = plt.Circle((x, y), config.robot_radius, color="b", fill=False)
        plt.gcf().gca().add_artist(circle)
        out_x = x + math.cos(yaw) * config.robot_radius
        out_y = y + math.sin(yaw) * config.robot_radius
        plt.plot([x, out_x], [y, out_y], "-k")
