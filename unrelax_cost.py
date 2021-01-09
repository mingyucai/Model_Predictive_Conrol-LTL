from PATH_PLAN.P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from PATH_PLAN.P_MAS_TG.planner import ltl_planner
from point_location import pl, edges, ap, regions, calc_box_id, Obstacle_Moved
from path_plan import calc_path, sanitize_path
from Controller_main import MultiRobot
import paho.mqtt.client as mqtt
import logging
import copy
import time
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# main massion
mission = [3, 13, 31, 3, 13, 31]

# mqtt_client
client = mqtt.Client()

controller = MultiRobot(
    robot_confs=[
        {"id": 1, "v_const": 0.0025, "d_stop": 0.13, "goal_x": 0, "goal_y": 0,
         "k_P": 0.1, "k_I": 0.01, "k_D": 0.1}
    ],
    obstacle_confs=[
        {"id": 11, "period": 0.1},
        {"id": 12, "period": 0.1},
        {"id": 13, "period": 0.1}
    ]
)
controller.initialize(period=3)
print("Robot is in {}. Obstacle is in {}".format(
    controller.robots[1].box_id,
    controller.obstacles[11].box_id
))

regions_raw = copy.deepcopy(regions)
# initial obstacle recognition
for id, obstacle in controller.obstacles.items():
    o_x = int(obstacle.box_id / 4)
    o_y = obstacle.box_id % 4
    regions[(o_x, o_y, 1)].discard("r")
    regions[(o_x, o_y, 1)].add("b")

rids = list(controller.robots.keys())
rid = rids[0]
try:
    for minor_mission_target in mission:
        # get path to this minor mission (only for one robot)
        logger.info("\tMajor mission target: %d", minor_mission_target)
        aborted_by_obstacle_movement = True
        while aborted_by_obstacle_movement:
            try:
                # wait till obstacle stable
                logger.info("Waiting obstacle to be stable.")
                controller.wait_obstacle_stable()
                logger.info("Obstacles are stable now.")

                regions = copy.deepcopy(regions_raw)
                for id, obstacle in controller.obstacles.items():
                    o_x = int(obstacle.box_id / 4)
                    o_y = obstacle.box_id % 4
                    regions[(o_x, o_y, 1)].discard("r")
                    regions[(o_x, o_y, 1)].add("b")

                minor_mission_path = calc_path(
                    robot_box_id=controller.robots[rid].box_id,
                    target_box_id=minor_mission_target,
                    regions=regions,
                    ap=ap,
                    edges=edges)
                minor_mission_path = sanitize_path(minor_mission_path)  # element is sub mission, move one box per step
                logger.info("Minor mission path: %s", minor_mission_path)
                if len(minor_mission_path) == 0:
                    logger.info("Minor mission already finished after obstacle stabilized. Move to next mission.")
                    break

                for sub_mission in minor_mission_path:
                    logger.info("\tSub mission: %d", sub_mission)
                    controller.robots[rid].update_goal(x=pl[sub_mission, 0], y=pl[sub_mission, 1])
                    controller.start()
                    while not controller.all_finished:
                        if not controller.obstacles_stable:
                            raise Obstacle_Moved
                        controller.one_step()
                    controller.halt(1)
                aborted_by_obstacle_movement = False
            except Obstacle_Moved:
                logger.info("Obstacle movement detected.")
                # finish current sub_mission
                while not controller.all_finished:
                    controller.one_step()
                logger.info("Robot finished current job.")
                aborted_by_obstacle_movement = True
        controller.robots[rid].rotate(period=3)

except KeyboardInterrupt:
    controller.kill()
