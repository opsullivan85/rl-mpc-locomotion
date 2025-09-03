from src.control.mpc.common.Quadruped import Quadruped
from src.control.mpc.common.LegController import LegController
# from src.control.MPC_Controller.Parameters import Parameters
from src.control.mpc.common.StateEstimator import StateEstimator
from src.control.mpc.common.DesiredStateCommand import DesiredStateCommand

class ControlFSMData:
    def __init__(self):
        self._quadruped:Quadruped = None
        self._stateEstimator:StateEstimator = None
        self._legController:LegController = None
        self._desiredStateCommand:DesiredStateCommand = None
        # self._gaitScheduler:GaitScheduler = None
        # self.userParameters:Parameters = None

