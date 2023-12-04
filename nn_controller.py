'''
    Loading and running the NN controller

    Input - Current state (x_own, y_own, v_xown, v_yown, x_int, y_int, v_xint, v_yint)
    Output - Action {SL, WL, COC, WR, SR}

    Initialize as process (loads in models) then wait for calls
'''

import onnxruntime
import numpy as np
import os

#cmds = [0.0, 1.5, -1.5, 3.0, -3.0]
cmds = [0.0, 1.5, -1.5, 3.0, -3.0]
taus = [0, 1, 5, 10, 20, 50, 60, 80, 100]

command_names = ['Clear of Conflict', 'Weak Left', 'Weak Right', 'Strong Left', 'Strong Right']

input_means = [30130.0, 0.0, 0.0, 650.0, 600.0]
input_maxes = [60260, 2 * np.pi, 2 * np.pi, 1200.0, 1200.0]

rho_mean = 19791.091 # Mean taken from Bak (2021), assuming from sim data
rho_range = 60261.0

theta_mean = 0.0
theta_range = 6.28318530718

psi_mean = 0.0
psi_range = 6.28318530718

v_own_mean = 650.0
v_own_range = 1100.0

v_int_mean = 600.0
v_int_range = 1200.0

nets_loc_template = "resources/ACASXU_run2a_{}_{}_batch_2000.onnx"

def get_closest_tau_index(tau):

    if tau < taus[0]:
        idx = 0
    elif tau >= taus[-1]:
        idx = len(taus) - 1
    else:
        for i, t in enumerate(taus[:-1]):
            tau_max = taus[i+1]

            if t <= tau < tau_max:
                idx = i

    return idx

class NN_Controller:

    def __init__(self):
        self.load_networks()

    def load_networks(self):
        self.nets = []

        for cmd in range(len(cmds)):
            n = []
            for tau in range(len(taus)):
                #TODO: fix path here
                path = os.path.join('C:/Users/walli/Desktop/Assured Autonomy Project/acas_runtime_monitoring',nets_loc_template.format(cmd+1, tau+1))
                n.append(onnxruntime.InferenceSession(path))
            self.nets.append(n)

    def run_network(self, alpha, tau, rho, theta, psi, v_own, v_int):

        if rho >= 60260:
            return 0

        tau_index = get_closest_tau_index(tau)
        #print(f"Using network: [alpha: {alpha} - {cmds.index(alpha)}, tau: {tau} - {tau_index} / {taus[tau_index]}]")
        net = self.nets[cmds.index(alpha)][tau_index]
        input = np.array((rho, theta, psi, v_own, v_int), dtype=np.float32)
        input.shape = (1, 1, 1, 5)
        output = net.run(None, {'input' : input})
        #print(np.argmin(output))
        return cmds[int(np.argmin(output))]

    @staticmethod
    def convert_inputs(x_own, y_own, v_xown, v_yown, x_int, y_int, v_xint, v_yint):
        theta_own = np.arctan(v_yown / v_xown)
        theta_int = np.arctan(v_yint / v_xint)
        theta = np.arctan((y_int - y_own) / (x_int - x_own)) - theta_own
        psi = theta_int - theta_own
        rho = np.sqrt((x_own - x_int) ** 2 + (y_own - y_int) ** 2)

        if theta_own == 0.0:
            v_own = v_xown
        else:
            v_own = v_xown / np.cos(theta_own)

        if theta_int == 0.0:
            v_int = v_xint
        else:
            v_int = v_xint / np.cos(theta_int)

        return rho, theta, psi, v_own, v_int
    
    @staticmethod
    def scale_inputs(rho, theta, psi, v_own, v_int):
        rho = (rho - rho_mean) / rho_range
        theta = (theta - theta_mean) / theta_range
        psi = (psi - psi_mean) / psi_range
        v_own = (v_own - v_own_mean) / v_own_range
        v_int = (v_int - v_int_mean) / v_int_range

        #print("Scaled Inputs:", rho, theta, psi, v_own, v_int)

        return rho, theta, psi, v_own, v_int
    
    @staticmethod
    def dist(l, r):
        return np.sqrt((l.x - r.x) ** 2 + (l.y - r.y) ** 2)

    def get_command(self, alpha, tau, x_own, y_own, theta_own, v_own, x_int, y_int, theta_int, v_int, use_radians = True):
        
        # if not use_radians:
        #     theta_own *= (np.pi / 180.0)
        #     theta_int *= (np.pi / 180.0)

        rho = np.sqrt((x_own - x_int)**2 + (y_own - y_int)**2)
        
        dy = y_int - y_own
        dx = x_int - x_own

        theta = np.arctan2(dy, dx)
        psi = theta_int - theta_own

        theta -= theta_own

        while theta < -np.pi:
            theta += 2 * np.pi

        while theta > np.pi:
            theta -= 2 * np.pi

        if psi < -np.pi:
            psi += 2 * np.pi

        while psi > np.pi:
            psi -= 2 * np.pi

        #print(f"running network: {cmds.index(alpha)} // {tau}")
        #print(f"Input(before scaling): {[rho, theta, psi, v_own, v_int]}")
        rho, theta, psi, v_own, v_int = self.scale_inputs(rho, theta, psi, v_own, v_int)
        #print(f"input (after scaling): {[rho, theta, psi, v_own, v_int]}")

        cmd = self.run_network(alpha, tau, rho, theta, psi, v_own, v_int)

        #print(f"ACAS Active (Horizontal Distance = {np.sqrt((x_own - x_int) ** 2 + (y_own - y_int) ** 2)})")
        #print(f"\tProposed Command: {command_names[cmds.index(cmd)]}")

        return cmd