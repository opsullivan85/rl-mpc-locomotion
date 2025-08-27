from MPC_Controller.common.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.Parameters import Parameters
from MPC_Controller.common.Quadruped import Quadruped, RobotType
from MPC_Controller.common.LegController import LegController
from MPC_Controller.common.StateEstimator import StateEstimator
from MPC_Controller.convex_MPC.ConvexMPCLocomotion import ConvexMPCLocomotion
from nptyping import NDArray, Float32, Shape

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np


class RobotRunnerMin:
    def __init__(self):
        pass

    def init(self, robotType: RobotType, dt: float = 0.01):
        """
        Initializes the robot model, state estimator, leg controller,
        robot data, and any control logic specific data.
        """
        self.robotType = robotType

        # TODO tune these parameters
        self.cMPC = ConvexMPCLocomotion(dt, int(27 / (1000.0 * dt)))

        # init quadruped
        print(f"{self.robotType = }")
        print(f"{RobotType = }")
        print(f"{self.robotType in RobotType = }")
        print(f"{self.robotType == RobotType.GO1 = }")
        print(f"{RobotType.GO1 in RobotType = }")
        if self.robotType in RobotType:
            self._quadruped = Quadruped(self.robotType)
        else:
            raise Exception("Invalid RobotType")

        # init leg controller
        self._legController = LegController(self._quadruped)

        # init state estimator
        self._stateEstimator = StateEstimator(self._quadruped)

        # init desired state command
        self._desiredStateCommand = DesiredStateCommand()

        # init controller data
        self.data = ControlFSMData()
        self.data._quadruped = self._quadruped
        self.data._stateEstimator = self._stateEstimator
        self.data._legController = self._legController
        self.data._desiredStateCommand = self._desiredStateCommand

        # init convex MPC controller
        self.cMPC.initialize(self.data)

    def reset(self):
        self.cMPC.initialize(self.data)
        self._desiredStateCommand.reset()
        self._stateEstimator.reset()

    def run(
        self,
        dof_states: NDArray[Shape["12, 2"], Float32],
        body_states: NDArray[Shape["13"], Float32],
        commands: NDArray[Shape["3"], Float32],
    ) -> NDArray[Shape["12"], Float32]:
        """Runs the overall robot control system by calling each of the major components
        to run each of their respective steps.

        Args:
            dof_states (np.ndarray): (12,2) float32 array of dof states:
                pos = [0:12,0]
                vel = [0:12,1]
                TODO: verify this
                where each leg's 3 dofs are in row order: hip, upper leg, lower leg
            body_states (np.ndarray): (13,) float32 array of body states:
                position = [0:3]
                orientation xyzw = [3:7]
                velocity xyz = [7:10]
                omega xyz = [10:13]
            commands (np.ndarray): (3,) float32 array of user commanded
                x velocity = [0]
                y velocity = [1]
                omega velocity = [2]

        Returns:
            np.ndarray: (12,) float32 array of leg torques
                TODO: verify this
                where each leg's 3 dofs are in row order: hip, upper leg, lower leg
        """
        # Update desired commands
        self._desiredStateCommand.updateCommand(commands)

        # Update the joint states
        self._legController.updateData(dof_states)
        self._legController.zeroCommand()
        # self._legController.setEnable(True)

        # Update robot states
        self._stateEstimator.update(body_states)

        # Run the Control FSM code
        self.cMPC.run(self.data)

        # Sets the leg controller commands for the robot
        legTorques = self._legController.updateCommand()

        return legTorques  # numpy (12,) float32
