# constants used in the program
bound_cond = True   # set the boundry conditions on or off
L = 5 # size of the box
N = 10  # number of particles
k = 2 # nearest neighbours

M = 1   # number of objects
v_mag = 0.05      # total magnitude of each particle velocity
delta_t = 1     # time increment
mass_par = 1 # masss of the particles
outer_radius = L*0.2 # outer size of radius of object

mass_object = 500 # masss of the object
mom_inertia = (1/3) * mass_object # PERHAPS CHANGE A BIT BUT ITS JUST A DAMPING TERM SO DON'T WORRY TOO MUCH

# distance metrics in the code
r = 1.0   # radius of allignment
r_c = 0.05 # radius within repulsion
r_e = 0.5 # radius of equilibrium between the particles
r_a = 0.8 # radius when attraction starts
r_o = v_mag # radius of attraction between the particels and the objects

# force parrameters
alpha = 0 # stregnth of repulsive force between to the particles
beta = 1 # stregnth of the force due to the objects on the particles
gamma = 1 # stregnth of allignment
fric_force = 0.2  # frictional force of the object when rotating
noise = 1  # noise added to the velocity


# picking a model
model = "SVM" # select SVM for standard Vicsek Model and kNN for nearest neighbours

spikes = 15
U = 1000   # number of updates
dimensions = 2   # dimensions
time_pause = 1e-2 # time pause for interactive graph
