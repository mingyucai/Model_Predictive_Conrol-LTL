#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import signal
import argparse
import paho.mqtt.client as mqtt
import json
import time
import logging

from robot import robot
from obstacle import obstacle

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


class MultiRobot:
    def __init__(self, robot_confs=[], obstacle_confs=[]):
        self.running = False

        # mqtt client
        self.client = mqtt.Client(client_id="MultiRobot")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_location_update
        self.client.enable_logger()
        self.client.connect("localhost", 1883, 60)

        self.robots = {}
        for robot_conf in robot_confs:
            self.robots[robot_conf["id"]] = robot(
                robot_conf["v_const"], robot_conf["d_stop"], robot_conf["id"], self.client)
            self.robots[robot_conf["id"]].update_goal(robot_conf["goal_x"], robot_conf["goal_y"])
            self.robots[robot_conf["id"]].set_PID(robot_conf["k_P"], robot_conf["k_I"], robot_conf["k_D"])

        self.obstacles = {}
        for obstacle_conf in obstacle_confs:
            self.obstacles[obstacle_conf["id"]] = obstacle(**obstacle_conf)

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print('Exiting...')
        self.kill()
        sys.exit(0)

    # start all robot. This is [NONE] blocking call.
    def start(self):
        logger.info("Starting multi robot")
        self.running = True
        self.start_time = time.time()
        self.last_step = self.start_time
        for id, robot in self.robots.items():
            robot.start()
    @property
    def all_finished(self):
        finished = True
        for id, robot in self.robots.items():
            finished = finished and robot.finished
        return finished

    @property
    def obstacles_stable(self):
        stable = True
        for id, obstacle in self.obstacles.items():
            stable = stable and obstacle.is_stable()
        return stable

    # only stop
    def stop(self):
        self.running = False
        for id, robot in self.robots.items():
            robot.stop()

    # stop and set speed to 0
    def kill(self):
        self.stop()
        self.client.loop(timeout=1.0)
        self.client.disconnect()
        exit(11)  # PID kill

    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self, client, userdata, flags, rc):
        logger.debug("Connected with result code " + str(rc))
        self.client.subscribe("socket_mqtt")

    # on location update
    def on_location_update(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload)
        except ValueError as e:
            logger.error("MQTT parse error. Message is: {}".format(msg.payload))
            return

        if int(data["id"]) < 10:
            self.robots[data["id"]].set_position_angle(data)
        else:
            self.obstacles[data["id"]].set_position(data)

    def wait_finish(self):
        while not self.all_finished:
            self.one_step()

    def halt(self, period=0):
        self.client.loop(period)

    def wait_obstacle_stable(self):
        while not self.obstacles_stable:
            self.client.loop()
        logger.debug("Obstacle stable.")

    # one step, must be ran continuously
    def one_step(self):
        self.client.loop(timeout=0.5)
        # time interval 0.1s
        if self.running and time.time() - self.last_step >= 0.1:
            self.last_step = time.time()
            for id, robot in self.robots.items():
                robot.one_step()


    def debug(self):
        last_debug = time.time()
        debug_interval = 0.1

        while True:
            self.client.loop()
            print("%s: %s" % (time.time(), self.obstacles[11].box_history))

    def initialize(self, period=3):
        start_time = time.time()
        while time.time() - start_time < period:
            self.client.loop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot control algorithm. ")
    parser.add_argument("-v", "--verbose", default=False, help="Verbose output. ", action="store_true")
    parser.add_argument("x", help="x_goal, in m", type=float)
    parser.add_argument("y", help="y_goal, in m", type=float)

    args = parser.parse_args()

    verbose = args.verbose

    # MQTT
    controller = MultiRobot(
        robot_confs=[
            {"id": 1, "v_const": 0.003, "d_stop": 0.1, "goal_x": 0, "goal_y": 0,
             "k_P": 0.1, "k_I": 0.01, "k_D": 0.1}
        ],
        obstacle_confs=[
            {"id": 11, "period": 0.1},
            {"id": 12, "period": 0.1},
            {"id": 13, "period": 0.1}
        ]
    )
    logger.info("Sleep 5 seconds to stabilize the system. ")
    controller.initialize(period=5)
    controller.debug