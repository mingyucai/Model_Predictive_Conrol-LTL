#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import numpy as np
import logging
import time
from point_location import calc_box_id

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)


class obstacle:
    def __init__(self, id, period):
        if int(id) < 10:
            logger.error("Obstacle id cannot be less than 10. {} is given.".format(id))

        self.obstacle_id = id
        self.period = period
        self.history_num = 30
        self.i = 0
        self.box_id = -1

        self.location = np.ndarray(shape=(3, self.history_num), dtype=float, )
        self.box_history = - np.ones(self.history_num)

        self.last_timestamp = 0

    def set_position(self, data):
        if time.time() - self.last_timestamp <= self.period:
            return
        self.last_timestamp = time.time()

        if self.i < self.history_num:  # array not filled
            self.location[:, self.i] = np.array([ data["x"], data["y"], data["z"]])
            self.box_id = calc_box_id(self.location[0, self.i], self.location[1, self.i])
            self.box_history[self.i] = self.box_id
            self.i += 1

        else:
            self.location = np.roll(self.location, -1)
            self.box_history = np.roll(self.box_history, -1)

            self.location[:, -1] = np.array([ data["x"], data["y"], data["z"]])

            self.box_id = calc_box_id(self.location[0, -1], self.location[1, -1])
            self.box_history[-1] = self.box_id
        logger.debug("%s: %s", (time.time(), self.box_history))

    def check_movement(self):
        logger.info("Box: {}".format(self.box_id))

    @property
    def stable(self):
        return self.is_stable()

    def is_stable(self):
        ## return np.all(self.box_history == self.box_history[0])
        return np.linalg.norm(self.location[0:2, self.i-1] - self.location[0:2, 0]) <= 0.05 and self.box_id == self.box_history[0]


