#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import json
import time
import numpy as np
import logging
from utils.mathutils import quaternion_to_euler_angle
from point_location import calc_box_id

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class robot:
    def __init__(self, v_const, d_stop, robot_id, mqtt_client):
        self.robot_id = robot_id
        self.mqtt_client = mqtt_client
        self.initialized = True
        self.running = False
        self.finished = False

        self.goal = np.ndarray(shape=(2,))
        self.speed = np.ndarray(shape=(2,))

        # robot calibration angle difference
        diff_file = "save_variable/diff_{}.npz".format(robot_id)
        npzfile = np.load(diff_file)
        self.diff = npzfile["diff"]

        # robot max straight speed
        self.v_const = v_const
        # robot initial speed is 0
        self.v = 0

        # robot fixed value
        self.R = 21 / 1000  # m
        self.L = 104.1 / 1000  # m

        # threshold to stop the robot
        self.d_stop = d_stop

        self.x = 0
        self.y = 0
        self.z = 0
        self.qw = 0
        self.qx = 0
        self.qy = 0
        self.qz = 0
        self.box_id = -1
        self.theta = 0
        self.k_P = 0
        self.k_I = 0
        self.k_D = 0
        self.last_time = 0

        # loop variable
        self.value_p = 0
        self.value_i = 0
        self.value_d = 0
        self.ek_last = 0

    # data is an object containing x,y,z, qw,qx,qy,qz
    # this function is called in PID mqtt callback
    def set_position_angle(self, data):
        self.x = data["x"]
        self.y = data["y"]
        self.z = data["z"]
        self.qw = data["qw"]
        self.qx = data["qx"]
        self.qy = data["qy"]
        self.qz = data["qz"]

        # euler angle
        eulers = quaternion_to_euler_angle(self.qw, self.qx, self.qy, self.qz)
        self.theta = eulers[2] + self.diff  # orientation angle
        self.theta = - np.pi + (self.theta + np.pi) % (2 * np.pi)

        # update box_id
        self.box_id = calc_box_id(self.x, self.y)

    def set_PID(self, k_P, k_I, k_D):
        logger.info("PID: [%f, %f, %f]", k_P, k_D, k_I)
        # PID parameter
        self.k_P = k_P
        self.k_I = k_I
        self.k_D = k_D

    def update_goal(self, x, y):
        logger.info("Goal: [%f, %f]", x, y)
        self.goal = np.array([x, y])
        self.finished = False

    def start(self):
        self.running = True
        self.finished = False
        self.last_time = time.time()

    def stop(self):
        self.running = False
        self.set_speed(0, 0)

    def set_speed(self, l, r):
        self.speed = np.array([l, r])
        data = {"l": int(l), "r": int(r)}
        try:
            self.mqtt_client.publish(topic="robot_{}".format(self.robot_id), payload=json.dumps(data))
        except TypeError as TE:
            logger.error("TypeError, {}".format(TE))
            logger.error("Data is: {}".format(data))
            self.kill()

    # stop and set speed to 0. Kill the whole program.
    def kill(self):
        self.running = False
        self.set_speed(0, 0)
        self.mqtt_client.disconnect()
        exit(10)

    def rotate(self, period=3):
        logger.info("Robot {} rotate for {} seconds".format(self.robot_id, period))
        self.set_speed(-200, 200)
        time.sleep(period)
        self.set_speed(0, 0)

    def one_step(self):
        if not self.running:
            return
        # calculate angle to the goal
        u = self.goal - [self.x, self.y]
        theta_g = np.arctan2(u[1], u[0])

        # angle diff
        d_theta = theta_g - self.theta

        if np.linalg.norm(u) < self.d_stop:
            logger.info("Robot {}: distance close enough...".format(self.robot_id))
            self.set_speed(0,0)
            self.running = False
            self.finished = True
            return

        cur_time = time.time()
        time_diff = cur_time - self.last_time
        self.last_time = cur_time

        # still need to move
        # Adjust angle first
        ek = - np.pi + (d_theta + np.pi) % (2 * np.pi)
        if np.abs(ek) <= np.pi / 12:
            self.v = self.v_const
        else:
            self.v = self.v * 0.5

        self.value_p = ek
        self.value_i = self.value_i + ek * time_diff
        self.value_d = (ek - self.ek_last) / time_diff

        w = self.k_P * self.value_p + self.k_I * self.value_i + self.k_D * self.value_d

        v_r = (2 * self.v + w * self.L) / (2 * self.R)
        v_l = (2 * self.v - w * self.L) / (2 * self.R)
        logger.debug("Set robot {} speed(m/s): [{}, {}]".format(self.robot_id, v_l, v_r))
        self.set_speed(v_l * 1000, v_r * 1000)
        self.ek_last = ek