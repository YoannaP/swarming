from rotors import Rotor
from agents import Agent
from forces import part_repulsive_force, allignment_force, contact_force, torque_force

import numpy as np
import random
import cv2


class Environment:
    # some constants
    MODEL = "SVM"  # choose between "SVM" (standard Viscek Model) or "kNN"

    M = 1  # number of objects
    V_MAG = 0.01  # total magnitude of each particle velocity
    DELTA_T = 1  # time increment

    # constants for stregnth of each force
    ALPHA = 1  # allignment force strenght
    BETA = 1  # repulsive strenght between particles
    GAMMA = 1  # rotor strenght (0 means no rotor)
    FRIC = 0  # rotor friction when rotating

    # distance metrics in the code
    R = 0.07  # radius of allignment
    R_O = 0.007  # repulsive force between particles (higher means more spread out)

    R_C = 0.05  # radius within repulsion
    R_E = 0.5  # radius of equilibrium between the particles
    R_A = 0.8  # radius when attraction starts
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
            force_agents = self._force_agents(agent)
            agent.update(force_agents)

        force_rotor = self._force_rotor()
        self.rotor.update(force_rotor, self.DELTA_T)

        for agent in self.agents:
            agent.position["t"] = agent.position["t+1"]

    def _force_rotor(self):
        torque_particles = 0
        for agent in self.agents:
            torque_particles += torque_force(
                self.rotor.verticies,
                self.rotor.position,
                self.rotor.angular_velocity,
                agent.position["t"],
                self.V_MAG,
                self.FRIC,
            )
        return self.BETA * torque_particles

    def _force_agents(self, current_agent):
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
