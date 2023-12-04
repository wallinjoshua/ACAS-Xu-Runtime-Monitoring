from monitor import Monitor
from simulation import Plane, unsafe_state, horiz_distance_between_planes
from visualization import generate_animation
import numpy as np
import os

simulation_length = 100
use_radians = True

stdout = True

initial_tau = 0
delta_tau = 0

cur_tau = initial_tau

intruders = []
ownships = []


f = open(os.path.join('C:/Users/walli/Desktop/Assured Autonomy Project/acas_runtime_monitoring', 'cleaned_dubins_output.txt'))


for i in range(761):
    if stdout:
        print(f"Reading case {i}")
    # Read in ownship
    line = f.readline()
    vals = line.split(' ')
    ownships.append(Plane(float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])))

    # Read in intruder
    line = f.readline()
    vals = line.split(' ')
    intruders.append(Plane(float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])))

num_collisions = 0
num_simulations = 0

runtime_monitor = Monitor()

for true_intruder, true_ownship in zip(intruders, ownships):
    runtime_monitor.prev_cmd = 0.0
    runtime_monitor.cmd_queue = []

    num_simulations += 1

    init_state_ownship = f"true_ownship = Plane(x={true_ownship.x}, y={true_ownship.y}, theta={true_ownship.theta}, v={true_ownship.v}, store_positions=True)"
    init_state_intruder = f"true_intruder = Plane(x={true_intruder.x}, y={true_intruder.y}, theta={true_intruder.theta}, v={true_intruder.v}, store_positions=True)"

    for i in range(simulation_length):
        visible_intruder = true_intruder if (horiz_distance_between_planes(true_intruder, true_ownship) <= 60760.0) else None

        generated_command = runtime_monitor.generate_command(true_ownship, visible_intruder, cur_tau, lookahead_scheme=0, lookahead_distance=25, stdout=stdout)
        true_intruder.step(0.0)
        true_ownship.step(generated_command)

        # If a collision is detected, no point in generating further
        if unsafe_state(true_ownship, true_intruder, cur_tau):
            if stdout:
                print("SAFETY VIOLATION. HORIZONTAL DISTANCE BROKEN.")
                print("Initial State:")
                print(init_state_ownship)
                print(init_state_intruder)
            num_collisions += 1
            break

        cur_tau += delta_tau

print(f"Confirmed Collisions: {num_collisions} / {num_simulations}")

# WITHOUT ANY CORRECTION: 598 / 761

# SINGLE-STEP CORRECTION WITH FIVE STEP WINDOW: 539 / 761
# SINGLE-STEP CORRECTION WITH TEN STEP WINDOW: 299 / 761
# SINGLE-STEP CORRECTION WITH FIFTEEN STEP WINDOW: 57 / 761
# SINGLE-STEP CORRECTION WITH TWENTY STEP WINDOW: 50 / 761
# SINGLE-STEP CORRECTION WITH TWENTY-FIVE STEP WINDOW: 40 / 761

# MULTI-REPAIR WITH FIVE STEP WINDOW: 530 / 761
# MULTI-REPAIR WITH TEN STEP WINDOW: 274 / 761
# MULTI-REPAIR WITH FIFTEEN STEP WINDOW: 6 / 761
# MULTI-REPAIR WITH TWENTY STEP WINDOW: 2 / 761
# MULTI-REPAIR WITH TWENTY-FIVE STEP WINDOW: 0  761
