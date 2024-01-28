"""
Rotor code that contain the objects/rotors/polygons for the simualtion.
"""

from utils import centroid, clockwiseangle_and_distance
from forces import torque_force
from constants import (
    L,
    outer_radius,
    M,
    delta_t,
    mass_object,
    mom_inertia,
    dimensions,
    beta,
)

import numpy as np
import math


class Rotor:
    def __init__(self, origin, spikes, inner_radius, outer_radius, angle_diff):
        self.position = origin
        self.angular_velocity = 0
        self.angular_acceleration = 0
        self.outer_rad = outer_radius
        self.verticies = self._get_verticies(
            origin, spikes, inner_radius, outer_radius, angle_diff
        )
        self.mass = 500
        pass

    def _get_verticies(self, origin, spikes, inner_radius, outer_radius, angle_diff):
        # lists to store the pts
        pts_in = []
        pts_out = []
        pts_tot = []

        x_0, y_0 = origin  # centre of circle

        for i in range(spikes):
            value_out = [
                x_0 + outer_radius * np.cos(2 * np.pi * i / spikes),
                y_0 + outer_radius * np.sin(2 * np.pi * i / spikes),
            ]
            value_in = [
                x_0 + inner_radius * np.cos(angle_diff + 2 * np.pi * i / spikes),
                y_0 + inner_radius * np.sin(angle_diff + 2 * np.pi * i / spikes),
            ]
            pts_out.append(value_out)
            pts_in.append(value_in)

        sorted_out = sorted(pts_out, key=clockwiseangle_and_distance)
        sorted_in = sorted(pts_in, key=clockwiseangle_and_distance)

        for i in range(len(sorted_in)):
            pts_tot.append(sorted_out[i])
            pts_tot.append(sorted_in[i])

        return np.array(pts_tot)

    def update(self, force, delta_t):
        moment_of_inertia = (1 / 3) * self.mass
        self.angular_acceleration = force / moment_of_inertia

        self.angular_velocity = (
            self.angular_velocity + self.angular_acceleration * delta_t
        )

        self.update_pos()

    def update_pos(self):
        angular_v = self.angular_velocity + self.angular_acceleration * delta_t

        # get the change in angle from the angular velocuty
        angle = angular_v * delta_t

        # get the relative positions of the polygon, i.e with respective to the centre of the polygon
        polygon_respective = [
            (self.verticies[i] - self.position).tolist()
            for i in range(self.verticies.shape[0])
        ]

        # build the rotation matrix
        rot_mat = np.array(
            [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        )

        # multiply by old points in polygon
        new_pos = []
        for i in range(self.verticies.shape[0]):
            new_pos_i = np.dot(rot_mat, polygon_respective[i]) + self.position
            new_pos.append(new_pos_i.tolist())

        self.verticies = np.array(new_pos)
