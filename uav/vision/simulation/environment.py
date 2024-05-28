import time
import pickle
import ctypes
from mmap import mmap

import numpy as np

from posix_ipc import SharedMemory

from video_stream import VideoStream
from aruco_single_tracker import ArucoSingleTracker


SHM_OBJECT_NAME = "/shm-v2c-1"

ERR_THRESHOLD = 20  # target detection failure

PENALTY_OUT_OF_BOUND = 300
REWARD = 25


class V2C_Memory(ctypes.Structure):
    _fields_ = [
            ("timestep", ctypes.c_int),
            ("flag", ctypes.c_int),
            ("forward_m_s", ctypes.c_float),
            ("right_m_s", ctypes.c_float),
            ("down_m_s", ctypes.c_float),
            ]


    def write_to_memory(self,
                        timestep,
                        flag,
                        forward_m_s=None,
                        right_m_s=None,
                        down_m_s=None):

        self.timestep = timestep
        self.flag = flag

        if forward_m_s is not None:
            self.forward_m_s = forward_m_s
        if right_m_s is not None:
            self.right_m_s = right_m_s
        if down_m_s is not None:
            self.down_m_s = down_m_s


    def wait_for_update(self, timestep):
        while not self.timestep > timestep:
            time.sleep(0.0167)  # 60 iterations per second

        return self.timestep


class Environment(object):
    def __init__(self):
        self.FLAG_RESET = -1     # resetting simulation flag
        self.FLAG_INIT = 0       # arming and taking off flag
        self.FLAG_ACTION = 1     # velocity actions flag
        self.FLAG_FINALIZE = 2   # landing and disarming flag

        self.timestep = 0
        self.finalized = False

        self.setup_control_module()
        self.setup_vision_module()
        self.drone_initialize()

        self.goal_state = np.array([0, 0, 810])

        x_action_space = [2*i/-1000 for i in range(3, 7)]
        y_action_space = [2*i/1000 for i in range(3, 7)]
        z_action_space = [2*i/10000 for i in range(1, 5)]
        print(x_action_space)
        print(y_action_space)
        print(z_action_space)

        self.action_space = []
        for x in range(len(x_action_space)):
            for y in range(len(y_action_space)):
                for z in range(len(z_action_space)):
                    self.action_space.append([
                        x_action_space[x],
                        y_action_space[y],
                        z_action_space[z]
                        ])

        # environment variables
        self.action_space_n = len(self.action_space)
        print("Action space size:", self.action_space_n)

        # cm
        self.y_min = -100
        self.y_max = 100
        
        # cm
        self.x_min = -100
        self.x_max = 100
        
        # mm
        self.z_min = 450
        self.z_max = 4000

        self.observation_space_high = np.array([self.x_max, self.y_max, self.z_max])
        self.observation_space_low = np.array([self.x_min, self.y_min, self.z_min])


    def close(self):
        if not self.finalized:
            self.drone_finalize()


    def setup_control_module(self):
        # shared memory for writing and reading to the `Control` module
        shm = SharedMemory(
                name=SHM_OBJECT_NAME,
                mode=0o644,
                size=ctypes.sizeof(V2C_Memory),
                read_only=False
                )

        mm = mmap(shm.fd, shm.size)

        self.v2c = V2C_Memory.from_buffer(mm)

    
    def setup_vision_module(self):
        source = VideoStream()

        self.aruco_tracker = ArucoSingleTracker(
            id_to_find=0,
            marker_size=50,
            source=source,
            )


    def drone_initialize(self):
        self.timestep += 1

        print(f"Writing FLAG_INIT to memory... (Flag {self.FLAG_INIT})")
        self.v2c.write_to_memory(
            self.timestep,
            self.FLAG_INIT
            )

        self.timestep = self.v2c.wait_for_update(self.timestep)

        self.finalized = False

    
    def drone_finalize(self):
        self.timestep += 1

        print(f"Writing FLAG_FINALIZE to memory... (Flag {self.FLAG_FINALIZE})")
        self.v2c.write_to_memory(
            self.timestep,
            self.FLAG_FINALIZE
            )

        self.timestep = self.v2c.wait_for_update(self.timestep)
        
        self.finalized = True
        
    
    def get_target(self):
        found, target, _, _ = self.aruco_tracker.track_once()

        error = 0
        while not found:
            found, target, _, _ = self.aruco_tracker.track_once()

            error += 1
            time.sleep(0.05)
            if error > ERR_THRESHOLD:
                return found, target

        return found, np.array(target)


    def check_target_out_of_bound(self, target):
        x_in_bound = self.x_min <= target[0] <= self.x_max
        y_in_bound = self.y_min <= target[1] <= self.y_max
        z_in_bound = self.z_min <= target[2] <= self.z_max

        if (not x_in_bound) or (not y_in_bound) or (not z_in_bound):
            if not x_in_bound:
                print(f"X was out of bound... X = {self.x_min} <= {target[0]} <= {self.x_max}")

            if not y_in_bound:
                print(f"Y was out of bound... Y = {self.y_min} <= {target[1]} <= {self.y_max}")
                
            if not z_in_bound:
                print(f"Z was out of bound... Z = {self.z_min} <= {target[2]} <= {self.z_max}")

            return True

        return False


    def step(self, action):
        self.timestep += 1

        found, current_target = self.get_target()
        if found:
            right_m_s = current_target[0] * self.action_space[action][0]
            forward_m_s = current_target[1] * self.action_space[action][1]
            down_m_s = current_target[2] * self.action_space[action][2]

            print(f"Writing FLAG_ACTION to memory... (Flag {self.FLAG_ACTION})")
            self.v2c.write_to_memory(
                self.timestep,
                self.FLAG_ACTION,
                forward_m_s,
                right_m_s,
                down_m_s
                )
            
            print(f"X distrance: {current_target[0]}, X propotion: {self.action_space[action][0]}")
            print(f"Y distrance: {current_target[1]}, Y propotion: {self.action_space[action][1]}")
            print(f"Z distrance: {current_target[2]}, Z propotion: {self.action_space[action][2]}")
            print("-----------------------------------------------------")
            self.timestep = self.v2c.wait_for_update(self.timestep)


        found, new_target = self.get_target()
        if not found:
            print("Target not found... Resetting...")
            new_state = self.reset()
            reward = -2000
            done = True

        elif self.check_target_out_of_bound(new_target):
            print("UAV out of bound... Resetting...")
            new_state = self.reset()
            reward = -2000
            done = True

        else:
            new_state = new_target[:-1]
            distance = np.linalg.norm(new_state - self.goal_state)
            reward = -distance
            done = False

        return new_state, reward, done


    def reset(self):
        self.timestep += 1

        print(f"Writing FLAG_RESET to memory... (Flag {self.FLAG_RESET})")
        self.v2c.write_to_memory(
            self.timestep,
            self.FLAG_RESET
            )

        self.timestep = self.v2c.wait_for_update(self.timestep)

        found, current_target = self.get_target()
        if found:
            return current_target[:-1]

        return self.reset()


def main():
    env = Environment()
    
    for _ in range(60):
        action = 25
        env.step(action)

    time.sleep(6)

    env.close()


if __name__ == "__main__":
    main()


"""
class Environment(object):
    FLAG_INIT = 0
    FLAG_ACTION = 1
    FLAG_RESET = -1

    def __init__(
            self,
            target_x=0,
            target_y=0,
            target_z=2000,
            max_timesteps=100,
            ):

        self.state = None
        self.done = False

        self.target_position = np.array([target_x, target_y, target_z])
        self.timestep = 0
        self.max_timesteps = max_timesteps

        # shared memory for writing and reading to the `Control` module
        shm = SharedMemory(
                name=SHM_OBJECT_NAME,
                flags=O_CREAT,
                mode=0o644,
                size=ctypes.sizeof(Data),
                read_only=False
                )
        
        mm = mmap(shm.fd, shm.size)

        self.v2c_velocity = V2C_Velocity.from_buffer(mm)

        # arm and takeoff
        self.v2c_velocity.write_to_memory(0, self.FLAG_INIT, 0, 0, 0)
        time.sleep(8)

        self.source = VideoStream()
        self.aruco_tracker = ArucoSingleTracker(
            id_to_find=0,
            marker_size=50,
            source=self.source,
            )

    
    def _calculate_reward(self, position):
        # difference between target position and passed position
        distance = np.linalg.norm(self.target_position - position)
        reward = -distance
        return reward
    

    def reset(self):
        # update shared memory with reset message
        self.v2c_velocity.write_to_memory(, self.FLAG_RESET)
        time.sleep(8)

        self.state = None
        self.done = False

        # return state as array


    def step(self, action):
        # update shared memory with action and wait for state update

        self.timestep += 1

        self.v2c_velocity.write_to_memory(
                self.timestep,
                self.FLAG_ACTION,
                action[0],
                action[1],
                action[2],
                )
        time.sleep(1)

        found, target, _, _ = self.aruco_tracker.track_once()
        error = 0
        while not found:
            found, target, _, _ = self.aruco_tracker.track_once()
            error += 1
            time.sleep(0.05)
            if error > 20:
                reward = -20_000
                return

        current_position = np.array(target)
        state = current_position
        reward = self._calculate_reward(current_position)
        
        self.done = (self.timestep >= self.max_timestep) \
                or (reward > -1_000) \
                or (reward <= -20_000)


        return state, reward, self.done


    def close(self):
        # update shared memory with close message
        pass
"""

