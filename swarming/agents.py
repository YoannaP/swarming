import numpy as np
import random

from utils import rescale


class Agent:
    MASS = 1

    def __init__(self, coords, v_mag, delta_t):
        # cosntants from the environment
        self.V_MAG = v_mag
        self.DELTA_T = delta_t

        self.position = {"t": coords, "t+1": coords}
        self.velocity = rescale(
            self.V_MAG, (random.uniform(-1, 1), random.uniform(-1, 1))
        )
        self.acceleration = np.array([0, 0])

    def update(self, force):
        # calc acceleration force/mass
        self.acceleration = force / self.MASS

        # v_t+1 = v_t + acc * delta_t
        self.velocity = rescale(
            self.V_MAG, self.velocity + self.acceleration * self.DELTA_T
        )

        #  x_t+1 = x_t + v_t+1 * delta_t
        self.position["t+1"] = tuple(
            map(
                lambda x, y, z: (x + y) % z,
                self.position["t"],
                self.velocity * self.DELTA_T,
                [1, 1],
            )
        )
