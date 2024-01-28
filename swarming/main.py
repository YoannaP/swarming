import time
from environment import Environment


def simulation(L, N, n_steps, disp=True):
    # initialise environment
    env = Environment(L, N)

    # for each step - U is no of steps
    for i in range(n_steps):
        env.step()

        if disp:
            env.display()

    return


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
