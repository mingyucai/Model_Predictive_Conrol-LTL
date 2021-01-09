from PATH_PLAN.P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from PATH_PLAN.P_MAS_TG.planner import ltl_planner
from point_location import pl, edges, ap, regions, calc_box_id, Obstacle_Moved
from path_plan import calc_path, sanitize_path
from Controller_main import MultiRobot
import paho.mqtt.client as mqtt
import logging
import copy


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# main massion
path = [0, 4, 8, 12, 13, 14, 15, 11, 7, 3, 7, 11, 15, 14, 13, 14, 18, 22, 26, 25, 29, 28, 24, 20, 16, 12, 8, 9, 10, 11, 7, 3]
rotate_point = [3, 13, 31]
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

for i, robot in controller.robots.items():
    print("Robot {} is in {}".format(robot.robot_id, robot.box_id))

for i, obstacle in controller.obstacles.items():
    print("Obstacle {} is in {}".format(obstacle.obstacle_id, obstacle.box_id))

rids = list(controller.robots.keys())
rid = rids[0]

try:
    for sub_mission in path:
        logger.info("\tSub mission: %d", sub_mission)
        controller.robots[rid].update_goal(x=pl[sub_mission, 0], y=pl[sub_mission, 1])
        controller.start()
        while not controller.all_finished:
            if not controller.obstacles_stable:
                raise Obstacle_Moved
            controller.one_step()
        if sub_mission in rotate_point:
            controller.robots[rid].rotate(period=3)
        controller.halt(1)

except KeyboardInterrupt:
    controller.kill()
