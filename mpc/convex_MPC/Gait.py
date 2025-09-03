import numpy as np
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nptyping import NDArray, Float32, Shape


class GaitABC(ABC):
    """
    A gait pattern generator for quadruped robots using offset-duration parameterization.

    Leg indexing convention: [FR, FL, RR, RL] (Front Right, Front Left, Rear Right, Rear Left)

    The class provides real-time gait state information including:
    - Contact states (0-1 values indicating stance phase progress)
    - Swing states (0-1 values indicating swing phase progress)
    - MPC table (binary contact schedule for the prediction horizon)
    - Timing information for footstep planning and trajectory generation
    """

    @abstractmethod
    def setIterations(self, iterationsPerMPC: int, currentIteration: int):
        """Update the current gait timing based on the global iteration counter.

        Args:
            iterationsPerMPC (int): Number of control iterations between MPC updates.
            currentIteration (int): Global iteration counter since system start.
        """
        ...

    @abstractmethod
    def getContactState(self) -> NDArray[Shape["4, 1"], Float32]:
        """Calculate the current stance phase progress for each leg.

        Returns:
            np.ndarray: Stance progress for each leg, shape (4, 1).
                       Values range [0, 1] where:
                       - 0.0: Beginning of stance phase (foot just made ground contact)
                       - 0.5: Middle of stance phase (maximum ground reaction force)
                       - 1.0: End of stance phase (about to lift off)
                       - 0.0: Leg is in swing phase (no ground contact)
        """
        ...

    @abstractmethod
    def getSwingState(self) -> NDArray[Shape["4, 1"], Float32]:
        """Calculate the current swing phase progress for each leg.

        Returns:
            np.ndarray: Swing progress for each leg, shape (4, 1).
                       Values range [0, 1] where:
                       - 0.0: Beginning of swing phase (foot just lifted off)
                       - 0.5: Middle of swing phase (maximum foot height/velocity)
                       - 1.0: End of swing phase (about to touch down)
                       - 0.0: Leg is in stance phase (foot on ground)
        """
        ...

    @abstractmethod
    def getMpcTable(self) -> list[int]:
        """Generate binary contact schedule for the MPC prediction horizon.

        Returns:
            list: Binary contact states for the prediction horizon.
                 Length = nSegment * 4, arranged as [leg0_t0, leg1_t0, leg2_t0, leg3_t0,
                                                      leg0_t1, leg1_t1, leg2_t1, leg3_t1, ...]
        """
        ...

    @abstractmethod
    def getCurrentGaitPhase(self) -> float:
        """Get the current discrete gait phase as a segment index.

        Returns:
            int: Current gait segment [0, nSegment-1].
        """
        ...

    @abstractmethod
    def getCurrentSwingTime(self, dtMPC: float, leg: int) -> float:
        """Calculate the total swing duration for any leg in the gait.

        Args:
            dtMPC (float): MPC time step duration in seconds.
            leg (int): Leg index [0-3] for [FR, FL, RR, RL].

        Returns:
            float: Total swing phase duration in seconds.
        """
        ...

    @abstractmethod
    def getCurrentStanceTime(self, dtMPC: float, leg: int) -> float:
        """Calculate the total stance duration for any leg in the gait.

        Args:
            dtMPC (float): MPC time step duration in seconds.
            leg (int): Leg index [0-3] for [FR, FL, RR, RL].

        Returns:
            float: Total stance phase duration in seconds.
        """
        ...


class OffsetDurationGait(GaitABC):
    def __init__(
        self, nSegment: int, offset: np.ndarray, durations: np.ndarray, name: str
    ):
        """Initialize a gait pattern with offset-duration parameterization.

        Args:
            nSegment (int): Total number of MPC segments in one gait cycle. Typically 30 (10*3)
                           where 10 is the MPC horizon and 3 provides finer temporal resolution.
            offset (np.ndarray): Start time of stance phase for each leg in MPC segments.
                                Shape (4,) for [FR, FL, RR, RL] legs. Values should be in range [0, nSegment-1].
            durations (np.ndarray): Duration of stance phase for each leg in MPC segments.
                                   Shape (4,) for [FR, FL, RR, RL] legs. Values should be in range [1, nSegment].
                                   Higher values = longer ground contact = more stable but slower gaits.
            name (str): Human-readable name for the gait pattern (e.g., "Trotting", "Galloping").
        """
        # Implementation Details:
        #     - Swing duration = nSegment - stance duration for each leg
        #     - Phase values are normalized to [0, 1] for internal calculations
        #     - MPC table is pre-allocated as a flat array of size nSegment*4
        #     - Assumes first leg (FR) determines overall stance/swing timing for the gait

        # offset in mpc segments
        self.__offsets = offset.flatten()
        # duration of step in mpc segments
        self.__durations = durations.flatten()
        # offsets in phase (0 to 1)
        self.__offsetsFloat = offset / nSegment
        # durations in phase (0 to 1)
        self.__durationsFloat = durations / nSegment
        self.__nIterations = nSegment
        self.__name = name
        self.__stance = durations[0]
        self.__swing = nSegment - durations[0]
        self.__mpc_table = [0 for _ in range(nSegment * 4)]

    def setIterations(self, iterationsPerMPC: int, currentIteration: int) -> None:
        # Implementation Details:
        #     - __iteration: Current discrete gait segment [0, nSegment-1]
        #     - __phase: Continuous gait phase [0, 1] where 0=start of cycle, 1=end of cycle
        #     - Phase calculation ensures smooth transitions and proper gait timing
        #     - Used by getContactState(), getSwingState(), and getMpcTable()

        self.__iteration = (currentIteration / iterationsPerMPC) % self.__nIterations
        self.__phase = float(
            currentIteration % (iterationsPerMPC * self.__nIterations)
        ) / float(iterationsPerMPC * self.__nIterations)

    def getContactState(self) -> NDArray[Shape["4, 1"], Float32]:
        # Implementation Details:
        #     - Progress calculated as (current_phase - offset_phase) / duration_phase
        #     - Handles phase wrapping when offset occurs late in gait cycle
        #     - Returns 0.0 when leg is outside its designated stance window
        #     - Used by ConvexMPCLocomotion.run() for state_estimator.setContactPhase()

        progress = self.__phase - self.__offsetsFloat

        for i in range(4):
            if progress[i] < 0:
                progress[i] += 1.0

            if progress[i] > self.__durationsFloat[i]:
                progress[i] = 0.0
            else:
                progress[i] = progress[i] / self.__durationsFloat[i]

        return progress[:, None]  # convert to matrix

    def getSwingState(self) -> NDArray[Shape["4, 1"], Float32]:
        # Implementation Details:
        #     - Swing starts when stance phase ends: swing_offset = offset + duration
        #     - Swing duration = 1.0 - stance_duration (in normalized phase)
        #     - Handles phase wrapping and zero-duration swings (pure stance gaits)
        #     - Used by ConvexMPCLocomotion.run() to drive FootSwingTrajectory.computeSwingTrajectoryBezier()

        swing_offset = self.__offsetsFloat + self.__durationsFloat
        for i in range(4):
            if swing_offset[i] > 1:
                swing_offset[i] -= 1.0
        swing_duration = np.ones_like(self.__durationsFloat) - self.__durationsFloat

        progress = self.__phase - swing_offset

        for i in range(4):
            if progress[i] < 0:
                progress[i] += 1.0

            if progress[i] > swing_duration[i]:
                progress[i] = 0.0
            else:
                if swing_duration[i] == 0.0:
                    progress[i] = 0.0
                else:
                    progress[i] = progress[i] / swing_duration[i]

        return progress[:, None]

    def getMpcTable(self) -> list[int]:
        # Implementation Details:
        #     - Predicts contact states from current time to horizon end
        #     - Uses discrete segment-based timing (not continuous phase)
        #     - Accounts for current gait phase when generating future states
        #     - Fed directly to ConvexMpc.compute_contact_forces() for optimization
        #     - Critical for MPC to plan appropriate ground reaction forces

        # Usage in MPC:
        #     - MPC uses this table to determine which legs can generate forces
        #     - Enables predictive control of contact transitions
        #     - Allows optimization of forces only when feet are expected to be in contact

        for i in range(self.__nIterations):

            iter = (i + self.__iteration + 1) % self.__nIterations
            progress = iter - self.__offsets
            for j in range(4):
                if progress[j] < 0:
                    progress[j] += self.__nIterations
                if progress[j] < self.__durations[j]:
                    self.__mpc_table[i * 4 + j] = 1
                else:
                    self.__mpc_table[i * 4 + j] = 0

        return self.__mpc_table

    def getCurrentGaitPhase(self) -> float:
        return self.__iteration

    def getCurrentSwingTime(self, dtMPC: float, leg: int) -> float:
        # Note:
        #     Current implementation assumes all legs have the same swing duration,
        #     using only the first leg's stance duration. For asymmetric gaits,
        #     this could be modified to use leg-specific durations.
        
        return dtMPC * self.__swing

    def getCurrentStanceTime(self, dtMPC: float, leg: int) -> float:
        return dtMPC * self.__stance
