import numpy as np

class GaitScheduler:
    def __init__(self,
                 n_phases: int,
                 dt: float,
                 overlap_time: float,
                 swing_time: float,
                 contact_phases: np.array):
        self.contact_phases = contact_phases
        self.n_phases = n_phases

        overlap_ticks = overlap_time / dt
        swing_ticks = swing_time / dt
        self.phase_ticks = np.array([overlap_ticks, swing_ticks, overlap_ticks, swing_ticks])
        self.phase_length = 2 * (overlap_ticks + swing_ticks)


    def phase_index(self, ticks):
        """Calculates which part of the gait cycle the robot should be in given the time in ticks.
        
        Parametersq
        ----------
        ticks : int
            Number of timesteps since the program started
        
        Returns
        -------
        Int
            The index of the gait phase that the robot should be in.
        """
        phase_time = ticks % self.phase_length
        phase_sum = 0
        for i in range(self.n_phases):
            phase_sum += self.phase_ticks[i]
            if phase_time < phase_sum:
                return i
        assert False


    def subphase_ticks(self, ticks):
        """Calculates the number of ticks (timesteps) since the start of the current phase.

        Parameters
        ----------
        ticks : Int
            Number of timesteps since the program started
        
        Returns
        -------
        Int
            Number of ticks since the start of the current phase.
        """
        phase_time = ticks % self.phase_length
        phase_sum = 0
        subphase_ticks = 0
        for i in range(self.n_phases):
            phase_sum += self.phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + self.phase_ticks[i]
                return subphase_ticks
        assert False


    def contacts(self, ticks):
        """Calculates which feet should be in contact at the given number of ticks
        
        Parameters
        ----------
        ticks : Int
            Number of timesteps since the program started.
        
        Returns
        -------
        numpy array (4,)
            Numpy vector with 0 indicating flight and 1 indicating stance.
        """
        return self.contact_phases[:, self.phase_index(ticks)]