"""
Main code that will run the simulation.
"""


# from environment import pop_box
# from results import allignment, show_path_2D
# from agents import update_system
# from rotors import polygon, objects, update_system_object
# from constants import bound_cond, N, L, k, M, delta_t, spikes, U, dimensions, time_pause
#
from environment import Environment

import time
import math
import random


def simulation(L, N, n_steps, disp=True):
    # initialise environment
    env = Environment(L, N)

    # for each step - U is no of steps
    for i in range(n_steps):
        env.step()

        if disp:
            env.display()

    return


def simulation_old(plot=True):
    """
    One simulation of a total run by the system.
    """

    # produce the polygons (verticeies of the polygon)
    positions_polygons = [
        polygon([L / 2, L / 2], L * 0.1, math.pi / 2, spikes) for i in range(M)
    ]

    # fill up a box with particles and objects
    positions, velocities, accelerations = pop_box(positions_polygons)

    # MAYBE MAKE THIS POP_OBJECTS
    # returns positions, velocities, accelerations of com of objects
    positions_obj, ang_velocities_obj, accelerations_obj = objects(positions_polygons)

    # append the positions to the positions over time
    angle_over_t = [0]
    pos_part_over_t = [positions]
    vel_part_over_t = [velocities]
    pos_poly_over_t = [positions_polygons]
    ang_vel_obj_over_t = [ang_velocities_obj]

    # get the allignment
    align_start = allignment(velocities)

    # update the position for 10 times
    for i in range(U):
        # call update to get the new positions of the particles
        positions, velocities = update_system(
            positions, velocities, positions_obj, positions_polygons
        )

        # update the positions of the objects
        positions_polygons, ang_velocities_obj = update_system_object(
            positions_polygons, positions_obj, ang_velocities_obj, positions, velocities
        )

        # get the angle variaition due to the ang velocity
        new_angle = angle_over_t[-1] + ang_velocities_obj[0] * delta_t

        # append in positions over time
        pos_part_over_t.append(positions)
        vel_part_over_t.append(velocities)
        pos_poly_over_t.append(positions_polygons)
        ang_vel_obj_over_t.append(ang_velocities_obj)
        angle_over_t.append(new_angle)

    ang_velocities_obj_end = ang_velocities_obj
    align_end = allignment(velocities)

    # plot the movment of the particles if plot is set to true
    if plot == True:
        show_path_2D(0, U, pos_part_over_t, pos_poly_over_t, clear=True)

    return angle_over_t[-1]


if __name__ == "__main__":
    L = 1
    N = 100
    n_steps = 1000

    start = time.time()
    simulation(L, N, n_steps, disp=True)
    print(
        "------------------------- Time Taken: {} -------------------".format(
            time.time() - start
        )
    )
