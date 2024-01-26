"""
Force functions used to simulate the physics in the code.
"""

import math
import random
import numpy as np

from shapely.geometry import LineString, Point, LinearRing, Polygon
from utils import rescale, per_boun_distance, distance_fun, angle_to_xy
from forces_utils import particles_in_radius, k_particles

# from constants import bound_cond, v_mag, delta_t, r_c, r_e, r_a, r_o, fric_force, noise, model


def allignment_force(current_agent, agents, R, model):
    """
    Add a force which changes the velocity in the direction of the desired one.
    """
    position_particle = current_agent.position["t"]
    velocity_particle = current_agent.velocity

    # If using the Vicsek Model get velocity of particles in radius
    if model == "SVM":
        agents_in_r = particles_in_radius(position_particle, agents, R)
    # If using kNN neighbours get the velocity of k nearest neighbours
    if model == "kNN":
        vel_in_r = np.array(
            k_particles(position_particle, position_particles, velocities_particles)[0]
        )

    vel_in_r = np.array([agent.velocity for agent in agents_in_r])
    vel_wanted = np.mean(vel_in_r, axis=0)

    # get the force by subtracting the current vel from the desired one
    force = vel_wanted - velocity_particle

    return force


def part_repulsive_force(i, j, r_o):
    """
    calculates the force used in the repulsive_force function. As per chate 2008
    """

    # calculate the distance between the points
    distance_x, distance_y = per_boun_distance(i, j)
    # calcualte the magnitude of the distance between the points
    distance = (distance_x**2 + distance_y**2) ** (1 / 2)

    try:
        # magnitude of force
        magnitude = -1 / (1 + math.exp(distance / r_o))

    except OverflowError as err:
        magnitude = 0

    # get the x direction of the force
    F_x = (magnitude * distance_x) / distance
    # get the y direction of the force
    F_y = (magnitude * distance_y) / distance

    return np.array([F_x, F_y])


def torque_force(
    polygon,
    position_obj,
    ang_vel_object,
    position_particle,
    v_mag,
    fric_force,
):
    """
    Calcualte the torque on an object due to a particle hitting it.
    """
    force = obj_repulsive_force(position_particle, polygon, v_mag)

    # make the lists np arrays
    position_obj = np.array(position_obj)
    position_particle = np.array(position_particle)

    # get r from centroid and force
    r = position_particle - position_obj
    print(r)
    # get the angle between r and force
    v1_u = rescale(1, r)
    v2_u = rescale(1, force)
    angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    # insert 0s in the third dimension of the torque
    r = np.insert(r, 2, 0)
    force = np.insert(force, 2, 0)

    # get the torque from t = r X F
    torque = np.cross(r, force)

    # get only th emagnitude of the force
    torque = torque[2] - fric_force * ang_vel_object

    return torque


def obj_repulsive_force(particle_position, polygon, v_mag, bound_cond=True):
    """
    calculates the force used in the repulsive_force function. As per chate 2008
    """
    # make the polygon a linear ring
    poly = LinearRing(polygon)
    # create a particle moving straight down

    point = Point(particle_position)

    # get the closest point on polygon to particle
    d = poly.project(point)
    p = poly.interpolate(d)
    closest_point = list(p.coords)[0]

    # call that j and call particle_position i
    i = particle_position
    j = closest_point

    if bound_cond == True:
        # calculate the distance between the points
        distance_x, distance_y = per_boun_distance(i, j)
        # calcualte the magnitude of the distance between the points
        distance = (distance_x**2 + distance_y**2) ** (1 / 2)

    else:
        distance_x, distance_y = j[0] - i[0], j[1] - i[1]
        distance = distance_fun(i, j)

    try:
        # magnitude of force
        magnitude = 1 / (1 + math.exp(distance / v_mag))

    except OverflowError as err:
        magnitude = 0

    # get the x direction of the force
    F_x = (magnitude * distance_x) / distance

    # get the y direction of the force
    F_y = (magnitude * distance_y) / distance

    return np.array([F_x, F_y])


def inverse_force(i, j):
    """
    (1/r)^2 repulsive force
    """
    if bound_cond == True:
        # calculate the distance between the points
        distance_x, distance_y = per_boun_distance(i, j)
        # calcualte the magnitude of the distance between the points
        distance = (distance_x**2 + distance_y**2) ** (1 / 2)

    else:
        distance_x, distance_y = j[0] - i[0], j[1] - i[1]
        distance = distance_fun(i, j)

    # magnitude of force
    magnitude = -((1 / distance) ** 2)

    # get the x direction of the force
    F_x = (magnitude * distance_x) / distance

    # get the y direction of the force
    F_y = (magnitude * distance_y) / distance

    return np.array([F_x, F_y])


def chate_rep_att_force(i, j):
    """
    Attractive and repulsive force between the particles as described in the
    chate paper 2003.
    """
    # check for bounfy conditions
    if bound_cond == True:
        # calculate the distance between the points
        distance_x, distance_y = per_boun_distance(i, j)
        # calcualte the magnitude of the distance between the points
        distance = (distance_x**2 + distance_y**2) ** (1 / 2)

    else:
        distance_x, distance_y = j[0] - i[0], j[1] - i[1]
        distance = distance_fun(i, j)

    # if distance smaller than r_c
    if distance < r_c:
        # basically inifinite force
        magnitude = 1e6

    # if distnace between r_c and r_a (the radius of attraction)
    if r_c < distance < r_a:
        # force towards r_e (the equilibrium distance)
        magnitude = (1 / 4) * (distance - r_e) / (r_a - r_e)

    # if beyond ra but smaller than r_0
    if r_a < distance < r:
        # magnitude attraction
        magnitude = 1

    # else no force
    else:
        magnitude = 0

    # get the x direction of the force
    F_x = (magnitude * distance_x) / distance

    # get the y direction of the force
    F_y = (magnitude * distance_y) / distance

    return np.array([F_x, F_y])


def error_force(incoming_velocity):
    """
    Adds a random perturbation to the angle of the incoming velocity and
    returns the new randomly affected acceleration.
    """
    # get the magnitude of the velocity
    incoming_velocity = np.array(incoming_velocity)
    mag = np.sqrt(incoming_velocity.dot(incoming_velocity))

    # change the velocity term to an angle
    acc_angle = np.arctan2(incoming_velocity[1], incoming_velocity[0])

    # add a random perturbation based on 'noise'
    acc_angle += random.uniform(-noise / 2, noise / 2)

    # change back to vector form
    new_vel = angle_to_xy(mag, acc_angle)

    return new_vel


def contact_force(
    polygon, position_obj, position_particle, velocity_particle, v_mag, delta_t
):
    """
    Contact force between object and particle.
    """

    # make the polygon a linear ring and a polygon
    poly = LinearRing(polygon)

    # create a particle moving straight down
    point = Point(position_particle)

    # get the distance between the object and the particle
    dist = point.distance(poly)

    # check if the particle is not touching
    if dist > 5 * v_mag * delta_t:
        return np.array([0, 0])

    # get the closest point
    d = poly.project(point)
    p = poly.interpolate(d)
    closest_point = list(p.coords)[0]

    # now you have the points,get the vecetor normal to the plane
    n = rescale(
        1,
        [
            position_particle[0] - closest_point[0],
            position_particle[1] - closest_point[1],
        ],
    )
    # get the value of n, the normalised normal vector to the surface of reflection
    n = np.array(n)

    if dist < 3 * v_mag * delta_t:
        return obj_repulsive_force(position_particle, polygon, v_mag)

    # define the magntiude of the vector force
    magnitude = np.dot(velocity_particle, n)

    # get the force in the direction of the surface normal
    Force = magnitude * n

    return Force
