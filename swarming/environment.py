"""
Envvironement that will contain the simulation.
"""

# from utils import centroid, rescale
# from constants import N, v_mag, L, delta_t, dimensions

from rotors import Rotor
from agents import Agent
from forces import part_repulsive_force, allignment_force, contact_force

import numpy as np
import random
import cv2

import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
import matplotlib.cm as cm


class Environment:
    # some constants
    MODEL = "SVM"  # choose between "SVM" (standard Viscek Model) or "kNN"

    M = 1  # number of objects
    V_MAG = 0.01  # total magnitude of each particle velocity
    DELTA_T = 1  # time increment

    # constants for stregnth of each force
    ALPHA = 1  # allignment force
    BETA = 0  # repulsive stregnth
    GAMMA = 0.5  # rotor stregnth

    # # distance metrics in the code
    R = 0.1  # radius of allignment
    R_C = 0.05  # radius within repulsion
    R_E = 0.5  # radius of equilibrium between the particles
    R_A = 0.8  # radius when attraction starts
    R_O = 0.05  # radius of attraction between the particels and the objects
    DIMENSIONS = 2  # dimensions
    TIME_PAUSE = 1e-2  # time pause for interactive graph
    PERIODIC_BOUNDARIES = True

    def __init__(self, L, N, mag=800):
        self.mag = mag
        self.L = L  # height of box
        self.N = N  # number of particles

        # image that contains simulation
        self.image = np.zeros(
            [int(self.mag * self.L), int(self.mag * self.L), 3], dtype=np.uint8
        )

        self._pop_box()
        # forces parameters
        # alpha = 0 # stregnth of repulsive force between to the particles
        # beta = 1 # stregnth of the force due to the objects on the particles
        # gamma = 1 # stregnth of allignment
        # fric_force = 0.2  # frictional force of the object when rotating
        # noise = 1  # noise added to the velocity

        # model type (SVM, kNN)

        pass

    def _pop_box(self):
        self.agents = []

        # create rotor at centre of box
        self.rotor = Rotor(
            (0.5 * self.L, 0.5 * self.L), 15, self.L * 0.1, self.L * 0.2, np.pi / 2
        )

        agent_no = 0
        while agent_no < self.N:
            random_coords = (random.uniform(0, self.L), random.uniform(0, self.L))
            if self._coord_outside_rotor(random_coords):
                self.agents.append(Agent(random_coords, self.V_MAG, self.DELTA_T))
                agent_no += 1

    def _coord_outside_rotor(self, coordinates):
        """
        If coordinates are outside of rotor, return True
        """
        rtr_center = self.rotor.position
        pos_vector = (rtr_center[0] - coordinates[0]) ** 2 + (
            rtr_center[1] - coordinates[1]
        ) ** 2
        buffer = (self.rotor.outer_rad + 5 * self.V_MAG * self.DELTA_T) ** 2

        # If the random position is inside rotor, return False
        if pos_vector < buffer:
            return False
        return True

    def step(self):
        # for number of particles
        for agent in self.agents:
            force = self._force(agent, self.rotor)
            agent.update(force)

            # print(not self._coord_outside_rotor(agent.position["t"]))
            # agent.update(force)

        for agent in self.agents:
            agent.position["t"] = agent.position["t+1"]

    def _force(self, current_agent, rotor):
        force_rep = np.array([0.0, 0.0])
        for agent in self.agents:
            if current_agent.position != agent.position:
                force_rep += part_repulsive_force(
                    current_agent.position["t"], agent.position["t"], self.R_O
                )

        force_contact = contact_force(
            self.rotor.verticies,
            self.rotor.position,
            current_agent.position["t"],
            current_agent.velocity,
            self.V_MAG,
            self.DELTA_T,
        )

        force = (
            self.ALPHA
            * allignment_force(current_agent, self.agents, self.R, self.MODEL)
            + self.BETA * force_rep
            - self.GAMMA * force_contact
        )

        return force

    def display(self):
        self.image.fill(0)

        # rotor as a polygon
        rotor_pts = np.array(list(map(lambda x: x * self.mag, self.rotor.verticies)))
        rotor_pts = rotor_pts.reshape((-1, 1, 2))
        cv2.fillPoly(self.image, np.int32([rotor_pts]), (0, 255, 255))

        # display agents as circles
        for agent in self.agents:
            agent_centre = (
                int(agent.position["t"][0] * self.mag),
                int(agent.position["t"][1] * self.mag),
            )
            agent_radius = int(self.mag * 0.005)

            cv2.circle(self.image, agent_centre, agent_radius, (0, 0, 255), -1)

        cv2.imshow("Simulation", self.image)
        cv2.waitKey(1)


# ---------------------------- OLD ---------------------------------------
def pop_box(polygons):
    """
    Function which creates one particle with a ramdom position and velocity
    in a square of dimensions L by L.
    """

    # will hold each posi/vel/acc for each particle in the system
    positions = []
    velocities = []
    accelerations = []

    count = 0
    while count < N:
        # lsit containing positions and velocities at random
        # position can't be in the box
        init_position = [random.uniform(0, L) for i in range(dimensions)]  # IMPROVE

        #  check if point is within the polygon any of the polygons
        inside = False
        for poly in polygons:
            cnt = centroid(poly)
            cond = (cnt[0] - init_position[0]) ** 2 + (cnt[1] - init_position[1]) ** 2
            if cond < (L * 0.2 + 5 * v_mag * delta_t) ** 2:
                inside = True
                break
        if inside == True:
            continue

        init_velocity = [random.uniform(-1, 1) for i in range(dimensions)]
        init_acceleration = [0 for i in range(dimensions)]

        # append the positions to the bigger lists
        positions.append(init_position)
        velocities.append(rescale(v_mag, init_velocity))
        accelerations.append(init_acceleration)

        # add 1 to the count
        count += 1

    return positions, velocities, accelerations


def periodic_boundaries(position):
    """
    If particle is over the limit of the box run this function and it will return
    the correct position of the particle.
    """
    global L

    # check if its under 0
    if position < 0:
        return position + L

    # if its over L
    elif position > L:
        return position - L

    # otherwise just return the position
    else:
        return position


def show_path_2D(start, end, coordinates, polygons, clear=True):
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
            plt.scatter(
                coordinates[time_step][i][0],
                coordinates[time_step][i][1],
                s=1,
                color="r",
            )

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
                plt.plot(x, y)
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


# ---------------------------- OLD ---------------------------------------
