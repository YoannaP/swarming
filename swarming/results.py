"""
Functions used to get results for the code.
"""

import math
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
import matplotlib.cm as cm


from utils import centroid, distance_fun
from constants import N, v_mag, M, L, delta_t, bound_cond, time_pause


def allignment(velocities):
    """
    Calculates the net allignment of the velocities of all the particles and
    normmailses this value so that if they are all alligned the allignment is 1.
    """
    # initialise values for sum of all vx and vy
    vx = 0
    vy = 0

    # sum all velocities in velocities array
    for particle in velocities:
        #add vx particle to sum of all vx
        vx += particle[0]

        #add vy particle to sum of all vy
        vy += particle[1]

    # Total magnitude of velocity of particles
    v_mag_tot = math.sqrt(vx**2 + vy**2)

    # Check alignment of particles
    v_a = (1/(N * v_mag)) * (v_mag_tot)

    return v_a

def SD_COM(position_particles):
    """
    Calcualte the sum of scalar distance of all the particles from the centre of mass of
    the particles.
    """

    # calculate the centre of mass of the object
    com = np.array(centroid(position_particles))

    sum = 0
    # loop over each particle in the positions
    for particle in position_particles:
        sum += distance_fun(particle, com)

    return sum

# ----------------------- Visualise Functions ------------------------------

def show_path_2D(start, end, coordinates, polygons, clear = True):
    """
    Function which takes in the coordinates as described in straight_particle and
    plots the result on a scatter graph.
    """
    global L, N, delta_t

    # start interactive mode
    plt.ion()

    # crete eempty figure on which data will go and first subplot
    fig = plt.figure()

    # get into the correct time step
    for time_step in range(start, end):
        # list of colours used for animation
        colours = cm.rainbow(np.linspace(0, 1, N))

        # loop over each particle and colour
        for i in range(N):
            # plot x, y poistion of particle in a given colour and set axis to size of box
            plt.scatter(coordinates[time_step][i][0], coordinates[time_step][i][1], s = 1, color = 'r')

            # plot the object
            if i < M:
                polygon = np.array(polygons[time_step][i])
                # get the points of the polygon to plot it
                x, y = polygon.T

                # print(x, y)

                x = np.append(x, x[0])
                y = np.append(y, y[0])

                # print(x, y)

                # plot the polygon
                plt.plot(x , y)
                # plt.scatter(polygons_com[time_step][i][0], polygons_com[time_step][i][1], s = 5, color = 'g')

            if bound_cond == True:
                plt.axis([0, L, 0, L])
            plt.axis([0, L, 0, L])
            # plt.axis([-L*2, L*2, -L*2, L*2])

        # show graph
        plt.show()
        plt.pause(time_pause)

        # decide if you want to clear
        if clear == True:
            plt.clf()

    return None

def show_allignment_plot(time, allignment):

    # plot time vs allignment for x vs y
    plt.clf()
    plt.plot(time, allignment, linewidth=1, marker=".", markersize=3)
    plt.xlabel("Time")
    plt.ylabel("Allignment value")
    plt.show()

    return None

def phase_transition(order_parameter_values, control_parameter_values):
    """
    Plots a potential phase diagram between an order parameter, such as alignment
    against a control parameter such as nosie.
    """
    # plot the order parameter on the y axis and the control on the x
    plt.scatter(control_parameter_values, order_parameter_values,
                s = 2, label = "N = {}, L = {}".format(N, L))
    plt.xlabel("nosie") # these should be changed for other parameters
    plt.ylabel("allignment") # these should be changed for other parameters
    plt.legend()
    plt.show()

    return None

def SD_graph(SD_list):
    """
    Graph the results of the sum of distances.
    """
    # get the x values, the timesteps
    x = [i for i in range(U)]

    # plot the results
    plt.scatter(x, SD_list, s = 3)
    plt.xlabel("Time Step")
    plt.ylabel("Sum of Distance from Centre of Mass")
    plt.show()

    return None
