#!/usr/bin/env python3

from PATH_PLAN.P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from PATH_PLAN.P_MAS_TG.planner import ltl_planner


# Operate sub mission.
def calc_path(robot_box_id, target_box_id, regions, ap, edges):  # 0-31
    robot_x = int(robot_box_id / 4)
    robot_y = robot_box_id % 4

    robot_motion = MotionFts(regions, ap, 'office')
    robot_motion.set_initial((robot_x, robot_y, 1))
    robot_motion.add_un_edges(edges, unit_cost=0.1)

    action = {'pick': (100, 'r', set(['pick'])),
              'drop': (50, 'b', set(['drop']))}
    robot_action = ActionModel(action)
    robot_model = MotActModel(robot_motion, robot_action)
    hard_task = '(([]<> r{}) && ([]! b)) && ([]! r26)) && ([]! r17))'.format(target_box_id)
    soft_task = None
    robot_planner = ltl_planner(robot_model, hard_task, soft_task)
    robot_planner.optimal(10, 'static')
    return [robot_planner.product.graph['ts'].node[n]['label'] for n in robot_planner.run.line]


def sanitize_path(list_of_set):
    for s in list_of_set:
        s.discard("r")
    path = [int(min(s)[1:]) for s in list_of_set]
    del path[-1], path[0]
    return path