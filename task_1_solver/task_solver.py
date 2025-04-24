from .Controller import Controller 
from ocs2_msgs.msg import mode_schedule
from std_srvs.srv import TriggerRequest
import rospy
from sensor_msgs.msg import Joy

class TaskSolver:
    def __init__(self, task_params, agent_params) -> None:
        # implement your own TaskSolver here
        self.task_params = task_params
        self.agent_params = agent_params
        self.controller = Controller()
        self.controller.start_launch()
    
    def next_action(self, obs: dict) -> dict:
        self.controller.last_obs = obs
        # implement your own TaskSolver here
        self.controller.PID_run()
        return self.controller.get_action(obs)
