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
        print(self.angular_velocity)
        moment_of_inertia = (1 / 3) * self.mass
        self.angular_acceleration = force / moment_of_inertia

        self.angular_velocity = (
            self.angular_velocity + self.angular_acceleration * delta_t
        )
        print(self.angular_velocity, self.angular_acceleration)

        self.update_pos()

    def update_pos(self):
        print("updating position")
        angular_v = self.angular_velocity + self.angular_acceleration * delta_t

        print(angular_v)
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

        self.position = new_pos


def polygon(origin, a, angle_diff, spikes):
    """
    Define the polygon from the points on the verticies.
    """
    refvec = (0, 1)

    pts_in = []
    pts_out = []
    pts_tot = []

    x_0, y_0 = origin  # centre of circle

    for i in range(spikes):
        value_out = [
            x_0 + outer_radius * np.cos(2 * np.pi * i / spikes),
            y_0 + b * np.sin(2 * np.pi * i / spikes),
        ]
        value_in = [
            x_0 + a * np.cos(angle_diff + 2 * np.pi * i / spikes),
            y_0 + a * np.sin(angle_diff + 2 * np.pi * i / spikes),
        ]
        pts_out.append(value_out)
        pts_in.append(value_in)

    sorted_out = sorted(pts_out, key=clockwiseangle_and_distance)
    sorted_in = sorted(pts_in, key=clockwiseangle_and_distance)

    for i in range(len(sorted_in)):
        pts_tot.append(sorted_out[i])
        pts_tot.append(sorted_in[i])

    return pts_tot


def objects(polygons):
    """
    Create a set of M objects, defining them just by there centre of mass.
    As of now they are basically particles of different species.
    """

    # will hold each posi/vel/acc for each particle in the system
    positions = []
    ang_velocities = []
    accelerations = []

    for i in range(M):
        # lsit containing positions and velocities at random
        init_position = centroid(polygons[i])
        init_velocity = 0
        init_acceleration = [0 for i in range(dimensions)]

        # append the positions to the bigger lists
        positions.append(init_position)
        ang_velocities.append(init_velocity)
        accelerations.append(init_acceleration)

    return positions, ang_velocities, accelerations


# ----------------------- Update Functions for objects ------------------------------


def update_system_object(
    polygons, positions_obj, ang_velocities_obj, position_particles, velocity_particles
):
    """
    Updates the positons and velocities of ALL the objects in a system.
    """
    # lists which will contain the updated values
    new_ang_vels = []
    new_polygons = []

    # loop through each index in the positions, vel, acc
    for i in range(len(positions_obj)):
        # # get the new torque on the force
        # torque = update_torque_object(polygons[i], positions_obj[i], velocity_particles, position_particles)
        # new_pol = update_velocity_object()
        # update the anngular acceleration on the object
        ang_acceleration = update_ang_acceleration_object(
            polygons[i],
            positions_obj[i],
            ang_velocities_obj[i],
            positions_obj,
            position_particles,
            velocity_particles,
        )
        # update the angular velocity of the object
        new_ang_vel = update_ang_velocity_object(
            ang_velocities_obj[i], ang_acceleration
        )
        # update the position of the vertex
        new_vers = update_position_object_vertex(
            polygons[i], positions_obj[i], new_ang_vel
        )

        # append them to the list of new position
        new_ang_vels.append(new_ang_vel)
        new_polygons.append(new_vers)

    return new_polygons, new_ang_vels


def update_ang_velocity_object(velocities_obj, accelerations_obj):
    """
    Update the velocity of a particle and returns the new velocity.
    """

    # create a new lsit which will contain the new position
    new_ang_vel = velocities_obj + accelerations_obj * delta_t

    return new_ang_vel


def update_ang_acceleration_object(
    polygon,
    position_obj,
    ang_vel_object,
    positions_obj,
    position_particles,
    velocity_particles,
):
    """
    Algorithm which updates the acceleration of the com of the object
    """
    # define two inital forces dependent on the particles and on hte object
    torque_particles = 0

    # loop through each particle and calculate the repulsive force from the particle
    for particle in range(len(position_particles)):
        torque_particles += torque_force(
            polygon,
            position_obj,
            ang_vel_object,
            velocity_particles[particle],
            position_particles[particle],
        )

    new_acceleration = beta * torque_particles / mom_inertia

    return new_acceleration


def update_position_object_vertex(polygon, position_obj, ang_vel_object):
    """
    Update the location of a particle and returns the new location.
    """
    polygon = np.array(polygon)

    new_pos = []
    # get the change in angle from the angular velocuty
    angle = ang_vel_object * delta_t

    # get the relative positions of the polygon, i.e with respective to the centre of the polygon
    polygon_respective = [
        (polygon[i] - position_obj).tolist() for i in range(polygon.shape[0])
    ]
    # print(polygon_respective)

    # build the rotation matrix
    rot_mat = np.array(
        [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
    )

    # print(rot_mat)
    # print(polygon_respective[0])

    # multiply by old points in polygon
    for i in range(polygon.shape[0]):
        new_pos_i = np.dot(rot_mat, polygon_respective[i]) + position_obj
        new_pos.append(new_pos_i.tolist())

    return new_pos
