from PATH_PLAN.P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from PATH_PLAN.P_MAS_TG.planner import ltl_planner
from point_location import pl
from Controller_main import PID
import paho.mqtt.client as mqtt
import logging

import time
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(sys.argv[0])

##############################
# motion FTS
ap = {'r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r', 'b'}
# +-----+-----+-----+
# | r4,r| r5,b| r6,b|
# +-----+-----+-----+
# | r1,r| r2,b| r3,r|
# +-----+-----+-----+
regions = {   (0, 0, 1): set(['r1', 'r']),
              (1, 0, 1): set(['r2', 'b']),
              (2, 0, 1): set(['r3', 'r']),
              (0, 1, 1): set(['r4', 'r']),
              (1, 1, 1): set(['r5', 'b']),
              (2, 1, 1): set(['r6', 'b']),
}

edges = [((0, 0, 1), (1, 0, 1)),
         ((1, 0, 1), (2, 0, 1)),
         ((0, 1, 1), (1, 1, 1)),
         ((1, 1, 1), (2, 1, 1)),
         ((0, 0, 1), (0, 1, 1)),
         ((1, 0, 1), (1, 1, 1)),
         ((2, 0, 1), (2, 1, 1)),
]

robot_motion = MotionFts(regions, ap, 'office' )
robot_motion.set_initial((0, 0, 1))
robot_motion.add_un_edges(edges, unit_cost = 0.1)


##############################
# action FTS
############# no action model
#action = dict()
############# with action
action = { 'pick': (100, 'r', set(['pick'])),
           'drop': (50, 'b', set(['drop']))
}


robot_action = ActionModel(action)


##############################
# complete robot model
robot_model = MotActModel(robot_motion, robot_action)



##############################
# specify tasks
########## only hard
#hard_task = '<>(r1 && <> (r2 && <> r6)) && (<>[] r6)'
#hard_task = '<>(r1 && <> (r2 && <> r6)) && ([]<> r6) && ([]<> r1) && [] <>(r1 -> X r5)'
soft_task = None


########## soft and hard
hard_task = '(([]<> r3) && ([]<> r4)) '
# soft_task = '([]! b)'



##############################
# set planner
robot_planner = ltl_planner(robot_model, hard_task, soft_task)

# synthesis
start = time.time()
robot_planner.optimal(10,'static')

# the prefix of plan **aps**:
# [{'r', 'r1'}, {'r2', 'b'}, {'r', 'r3'}, {'r2', 'b'}, {'r', 'r1'}, {'r', 'r4'}, {'r', 'r4'}]
# the suffix of plan **aps**:
# [{'r', 'r4'}, {'r5', 'b'}, {'r6', 'b'}, {'r', 'r3'}, {'r2', 'b'}, {'r', 'r1'}, {'r', 'r4'}]

prefix = [robot_planner.product.graph['ts'].node[n]['label'] for n in robot_planner.run.line]
suffix = [robot_planner.product.graph['ts'].node[n]['label'] for n in robot_planner.run.loop]

def parse_nodes(path):
    nodes = []
    for point in path:
        point.discard("r")
        point.discard("b")
        for i in point:
            nodes.append(i)
    return nodes

prefix_path = parse_nodes(prefix)
# mqtt_client
client = mqtt.Client()
for node in prefix_path:
    logger.info("Now moving to {}, {}".format(pl[node][0], pl[node][1]))
    controller = PID(v_const=0.003, d_stop=0.1, logger=logger)
    controller.update_goal(x_goal=pl[node][0], y_goal=pl[node][1])
    controller.start()
    if node == "r3" or node == "r4":
        controller.rotate(3)

suffix_path = parse_nodes(suffix)
# mqtt_client
client = mqtt.Client()
while True:
    for node in suffix_path:
        logger.info("Now moving to {}, {}".format(pl[node][0], pl[node][1]))
        controller = PID(v_const=0.004, d_stop=0.1, logger=logger)
        controller.update_goal(x_goal=pl[node][0], y_goal=pl[node][1])
        controller.start()
        if node == "r3" or node == "r4":
            controller.rotate(3)

logger.info('full construction and synthesis done within %.2fs \n' %(time.time()-start))
