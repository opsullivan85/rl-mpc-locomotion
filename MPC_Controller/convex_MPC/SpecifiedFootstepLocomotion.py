from src.control.MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from src.control.MPC_Controller.Parameters import Parameters
from src.control.MPC_Controller.math_utils.orientation_tools import CoordinateAxis, coordinateRotation
import numpy as np
from src.control.MPC_Controller.convex_MPC.ConvexMPCLocomotion import ConvexMPCLocomotion
from src.control.MPC_Controller.utils import DTYPE, getSideSign


class SpecifiedFootstepLocomotion(ConvexMPCLocomotion):
    def __init__(self, dt: float, iterations_between_mpc: int):
        self.footstep_locations_global = np.zeros((4, 3), dtype=DTYPE)
        """Four feet, desired x, y, z positions in world frame
        """
        super().__init__(dt, iterations_between_mpc)

    def _update_footstep_placement(
        self,
        i: int,
        gait,
        data: ControlFSMData,
        state_estimator_result,
        desired_velocity_robot_frame: np.ndarray,
    ):
        """Calculate the footstep placement for swing trajectory.

        Args:
            i: Index of the leg (0-3)
            gait: Current gait object
            data: Control FSM data
            state_estimator_result: State estimator result
            desired_velocity_robot_frame: Desired velocity in robot frame (3x1 array)
        """
        if self.first_swing_flags[i]:
            self.swing_time_remaining[i] = self.swing_times[i].item()
        else:
            self.swing_time_remaining[i] -= self.dt

        # Set swing height
        self.foot_swing_trajectories[i].setHeight(self.body_height / 3)

        # Calculate hip offset and foot position in robot frame
        hip_offset = np.array(
            [0, getSideSign(i) * data._quadruped._abadLinkLength, 0], dtype=DTYPE
        ).reshape((3, 1))
        foot_position_robot_frame = data._quadruped.getHipLocation(i) + hip_offset

        # Apply yaw correction for stance time
        stance_time = gait.getCurrentStanceTime(self.mpc_dt, i)
        foot_position_yaw_corrected = (
            coordinateRotation(
                CoordinateAxis.Z, -self.desired_yaw_rate * stance_time / 2
            )
            @ foot_position_robot_frame
        )

        # Calculate basic foot position in global frame
        foot_position_global = state_estimator_result.position + (
            foot_position_yaw_corrected
            + desired_velocity_robot_frame * self.swing_time_remaining[i]
        )

        # Calculate relative position offsets for better tracking
        max_relative_position = 0.3
        foot_x_offset_relative = (
            state_estimator_result.vBody[0]
            * (0.5 + Parameters.cmpc_bonus_swing)
            * stance_time
            + 0.03 * (state_estimator_result.vBody[0] - desired_velocity_robot_frame[0])
            + (0.5 * state_estimator_result.position[2] / 9.81)
            * (state_estimator_result.vBody[1] * self.desired_yaw_rate)
        )

        foot_y_offset_relative = (
            state_estimator_result.vBody[1] * 0.5 * stance_time * self.mpc_dt
            + 0.03 * (state_estimator_result.vBody[1] - desired_velocity_robot_frame[1])
            + (0.5 * state_estimator_result.position[2] / 9.81)
            * (-state_estimator_result.vBody[0] * self.desired_yaw_rate)
        )

        # Clamp offsets to prevent extreme foot placement
        foot_x_offset_relative = min(
            max(foot_x_offset_relative, -max_relative_position), max_relative_position
        )
        foot_y_offset_relative = min(
            max(foot_y_offset_relative, -max_relative_position), max_relative_position
        )

        # Apply offsets and set final position
        foot_position_global[0] += foot_x_offset_relative
        foot_position_global[1] += foot_y_offset_relative
        foot_position_global[2] = -0.003

        self.foot_swing_trajectories[i].setFinalPosition(foot_position_global)
    
    def set_footstep_locations(self, footstep_locations: np.ndarray):
        """Set the desired footstep locations in the global frame.

        Args:
            footstep_locations (np.ndarray): Desired footstep locations in the global frame.
        """
        self.footstep_locations_global = footstep_locations
