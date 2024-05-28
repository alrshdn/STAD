import time

import numpy as np

from environment import Environment


def get_discrete_state(state, low, win_size):
    discrete_state = (state - low) / win_size
    return tuple(discrete_state.astype(np.int32))


def main():
    start_q_table = "q_table-1716545420.npy"

    env = Environment()

    LEARNING_RATE = 0.1
    DISCOUNT = 0.95

    EPISODES = 250
    SAVE_EVERY = 10

    epsilon = 0.8
    EPS_DECAY = 0.9998
    EPS_START_DECAY = 1
    EPS_END_DECAY = EPISODES // 2
    epsilon_decay_value = epsilon / (EPS_END_DECAY - EPS_START_DECAY)

    DISCRETE_OS_SIZE = [25, 25, 20]
    discrete_os_win_size = \
            (env.observation_space_high - env.observation_space_low) / DISCRETE_OS_SIZE

    if start_q_table is None:
        q_table = np.random.uniform(low=-5, high=0, size=(DISCRETE_OS_SIZE + [env.action_space_n]))
    else:
        with open(start_q_table, "rb") as f:
            q_table = np.load(f)

    for episode in range(EPISODES):
        discrete_state = get_discrete_state(
                state=env.reset(),
                low=env.observation_space_low,
                win_size=discrete_os_win_size
                )

        done = False
        while not done:
            if np.random.random() > epsilon:
                action = np.argmax(q_table[discrete_state])
            else:
                action = np.random.randint(0, env.action_space_n)

            new_state, reward, done = env.step(action)

            new_discrete_state = get_discrete_state(
                    state=new_state,
                    low=env.observation_space_low,
                    win_size=discrete_os_win_size
                    )

            if not done:
                max_future_q = np.max(q_table[new_discrete_state])
                current_q = q_table[discrete_state + (action, )]

                new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)
                q_table[discrete_state + (action, )] = new_q
            
            else:
                print(f"Done: {discrete_state + (action, )}")
                q_table[discrete_state + (action, )] = reward

            discrete_state = new_discrete_state

        if EPS_END_DECAY >= episode >= EPS_START_DECAY:
            epsilon -= epsilon_decay_value

        if episode % SAVE_EVERY == 0:
            with open(f"q_table-{int(time.time())}.npy", "wb") as f:
                np.save(f, q_table)

    env.close()

    with open(f"q_table-{int(time.time())}.npy", "wb") as f:
        np.save(f, q_table)


if __name__ == "__main__":
    main()
