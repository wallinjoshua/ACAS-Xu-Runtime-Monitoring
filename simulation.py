import numpy as np

tolerance = 1e-6

class Plane:

    # Constant velocity plane -- part of previous paper assumption
    def __init__(self, x, y, theta, v, store_positions = False):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        if store_positions:
            self.positions = [(self.x, self.y)]

    def __str__(self):
        return f"({self.x}, {self.y}) moving at {self.v} ft/s along heading {self.theta} -- ({self.v * np.cos(self.theta * (np.pi / 180.0))}, {self.v * np.sin(self.theta * (np.pi / 180.0))})"

    # Assumes Dubin's dynamics model for aircraft
    def step(self, cmd, dt = 1, dz = 0):
        c = cmd * (np.pi / 180.0)

        #theta_rad = self.theta * (np.pi / 180.0)
        v_x = self.v * np.cos(self.theta)
        v_y = self.v * np.sin(self.theta)

        transition_matrix = [[0, 0, 1, 0],
                             [0, 0, 0, 1],
                             [0, 0, 0, -c],
                             [0, 0, c, 0]]
        transition_matrix = np.array(transition_matrix, dtype = np.float32)
        
        current_state = [self.x, self.y, v_x, v_y]        
        current_state = np.array(current_state, dtype=np.float32)

        d_theta = c * dt
       # while dt > 0:
        next_state_prime = transition_matrix @ current_state
        current_state = current_state + next_state_prime
        dt -= 1
        if hasattr(self, 'positions'): 
            self.positions.append((current_state[0], current_state[1]))
        
        self.x = current_state[0]
        self.y = current_state[1]
        #theta_rad += d_theta
        #self.theta = theta_rad * (180.0 / np.pi)
        self.theta += d_theta

def horiz_distance_between_planes(plane1, plane2):
    return np.sqrt((plane1.x - plane2.x) ** 2 + (plane1.y - plane2.y) ** 2)

def unsafe_state(plane1, plane2, tau):
    return horiz_distance_between_planes(plane1, plane2) <= 500.0 and \
        tau < 1