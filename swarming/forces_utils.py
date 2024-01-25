"""
Utils functions used in Forces.
"""

import numpy as np
from utils import per_boun_distance
from constants import bound_cond, L, N, r, dimensions, k
#
# # constants
# bound_cond = True   # set the boundry conditions on or off
# L = 5 # size of the box
# N = 2  # number of particles
# r = 1.0   # radius of allignment
# dimensions = 2   # dimensions
# k = 2 # nearest neighbours



def particles_in_radius(position_particle, agents, R):
    """
    Checks and records the particles which are within radius r.
    Returns the veloicities and positions of those particles that
    are within radius r.
    """

    # array with all indecies of all particles within range for velocities
    agents_within_R = []

    # check over all particles in positions
    for agent in agents:
        # check if it is smaller than the radius in all
        distance = per_boun_distance(position_particle, agent.position["t"])
        if np.linalg.norm(distance) < R:
            agents_within_R.append(agent)

    return agents_within_R


def k_particles(chosen_particle, positions, velocities):
    """
    Checks and records the k closest particles of chosen_particle.
    Returns the velocities and positions of those k particles.
    """


    # array with all indecies of all k particles for positions
    positions_k = []
    velocities_k = []

    # array of new distances considering boundary conditions
    new_distances = []

    # check over all particles in positions
    for index in range(N):

        distance_x, distance_y = per_boun_distance(chosen_particle, positions[index])

        # distance from selected particle to particle with index
        d = np.sqrt(distance_x**2 + distance_y**2)

        # append this distance to array of distances
        new_distances.append(d)

    # Now we need a sorting algorithm (merge)
    for j in range(k+1):
        low = min(new_distances)

        index_k = new_distances.index(low)

        # get the index of the particle for velocity
        velocities_k.append(velocities[index_k])

        # get the index of the particle for position
        # and add position to all positions within r
        positions_k.append(positions[index_k])

        new_distances.pop(index_k)

    return velocities_k, positions_k
