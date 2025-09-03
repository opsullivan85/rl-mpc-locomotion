from mpc.common.StateEstimator import StateEstimate
from mpc.convex_MPC.Gait import CalculatedGait
from src.control.mpc.FSM_states.ControlFSMData import ControlFSMData
from src.control.mpc.convex_MPC.Gait import GaitABC
from src.control.mpc.math_utils.orientation_tools import rpy_to_rot
import numpy as np
from src.control.mpc.convex_MPC.ConvexMPCLocomotion import ConvexMPCLocomotion
from src.control.mpc.utils import DTYPE
from nptyping import NDArray, Float32, Shape


class SpecifiedFootstepLocomotion(ConvexMPCLocomotion):
    def __init__(self, dt: float, iterations_between_mpc: int):
        super().__init__(dt, iterations_between_mpc)
        self.footstep_locations_hip = np.zeros((4, 2), dtype=DTYPE)
        """Four feet, desired x, y positions in respective hip frames
        """
        
        self.gait = CalculatedGait(
            dt, iterations_between_mpc, self.horizon_length
        )

    def _get_gait(self, gait_number: int) -> GaitABC:
        return self.gait

    def _update_footstep_placement(
        self,
        i: int,
        gait: GaitABC,
        data: ControlFSMData,
        state_estimator_result: StateEstimate,
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

        # Get the specified footstep location in the respective hip frame
        footstep_hip_frame = np.array([
            self.footstep_locations_hip[i, 0],
            self.footstep_locations_hip[i, 1], 
            # this should (roughly) put the foot in contact with the ground
            # assuming the body frame has this height in the world frame
            # there will be some sin error if the body is not horizontal
            # but that should be minimal
            -state_estimator_result.position[2]
        ], dtype=DTYPE).reshape((3, 1))

        # Transform from hip frame to global frame
        # 1. Get hip position in robot body frame
        hip_position_body_frame = data._quadruped.getHipLocation(i)
        
        # 2. Transform footstep from hip frame to body frame
        # For most quadruped robots, the hip frame is aligned with the body frame 
        # (same orientation, just translated), so we can directly add the position.
        # If there were any hip joint rotations to consider, they would be applied here.
        foot_position_body_frame = hip_position_body_frame + footstep_hip_frame
        
        # 3. Apply robot's rotation to transform from body frame to world frame
        rotation_matrix = rpy_to_rot(state_estimator_result.rpy.flatten())
        foot_position_world_frame = rotation_matrix @ foot_position_body_frame
        
        # 4. Add robot's global position to get final global position
        foot_position_global = state_estimator_result.position + foot_position_world_frame
        foot_position_global[2] = 0.0  # Project z down to zero

        self.foot_swing_trajectories[i].setFinalPosition(foot_position_global)

    def initiate_footstep(
        self,
        leg: int,
        location_hip: NDArray[Shape["2"], Float32],
        duration: float,
    ):
        """initiates a footstep for the specified leg at the specified location

        Args:
            leg (int): Index of the leg (0-3)
            location_hip (NDArray[Shape["2"], Float32]): Desired foot position in the respective hip frame (x, y)
                This position is relative to the hip of the specified leg.
                z will be projected down to zero.
            duration (float): Duration of the footstep
        """
        # Store the position in the respective hip frame - x, y from input, z projected to zero
        self.footstep_locations_hip[leg] = location_hip 
        self.swing_times[leg] = duration
        self.gait.initiate_footstep(
            leg,
            duration
        )