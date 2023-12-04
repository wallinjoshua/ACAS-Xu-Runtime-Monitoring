from monitor import Monitor
from simulation import Plane, unsafe_state, horiz_distance_between_planes
from visualization import generate_animation
import numpy as np
import math

def from_bak_counterexample(rho, theta, psi, v_own, v_int):
    # Always center intruder at (0,0) going along 0 degrees
    intruder = Plane(x = 0, y = 0, theta = 0, v = v_int, store_positions = True)
    
    phi = np.pi - theta + psi
    x_own = np.cos(phi) * rho
    y_own = -np.sin(phi) * rho
    
    ownship = Plane(x = x_own, y = y_own, theta = -psi, v = v_own, store_positions = True)
    return ownship, intruder

simulation_length = 75
use_radians = True

initial_tau = 0
delta_tau = 0

#true_ownship, true_intruder = from_bak_counterexample(61019.45806978694, 0.8007909138337812, -1.5953555128455696, 964.0586611224201, 1198.4375)
#true_ownship, true_intruder = from_bak_counterexample(55391.83570123739, 1.5606312585873852, -0.8172791641202594, 881.5515816989571, 1199.91375356)
#CONFIRMED WORKING COUNTEREXAMPLE -- 59s
#true_ownship, true_intruder = from_bak_counterexample(62001.19897399513, 1.105638365566048, -1.9313853026445638, 140.4154485909307, 1113.19526)
#CONFIRMED WORKING COUNTEREXAMPLE -- 62s
# true_ownship, true_intruder = from_bak_counterexample(61462.16874158125, 2.8797448888478536, -0.2973898012094359, 114.27575493691512, 1100.31313)
# true_ownship, true_intruder = from_bak_counterexample(60959.597800102, -0.7461997148243538, 2.1997877266124295, 110.84814862335269, 390.10329256)
#CONFIRMED WORKING COUNTEREXAMPLE
# true_ownship = Plane(x=0.0, y=0.0, theta=1.5707963267948966, v=113.84518308744457, store_positions=True)
# true_intruder = Plane(x=-61985.77127264738, y=5030.598066631885, theta=0.0, v=938.6070780751859, store_positions=True)
#CONFIRMED WORKING COUNTEREXAMPLE
# true_ownship = Plane(x=0.0, y=0.0, theta=1.5707963267948966, v=146.68365228187136, store_positions=True)
# true_intruder = Plane(x=-62633.06529162899, y=6428.842417877267, theta=0.0, v=1059.6258301434582, store_positions=True)

#CONFIRMED WORKING COUNTEREXAMPLE
# true_ownship = Plane(x=0.0, y=0.0, theta=1.5707963267948966, v=111.12581999958, store_positions=True)
# true_intruder = Plane(x=-62434.045570574526, y=4891.088725504937, theta=0.0, v=980.0389294107125, store_positions=True)

# true_ownship = Plane(x=0.0, y=0.0, theta=1.5707963267948966, v=123.77661157570824, store_positions=True)
# true_intruder = Plane(x=-62161.35250269141, y=4458.012050421263, theta=0.0, v=1199.6522569453407, store_positions=True)

# true_ownship = Plane(x = 10000.0, y = -10000.0, theta = np.pi/2.0, v = 1000.0, store_positions=True)
# true_intruder = Plane(x = 0.0, y = 0.0, theta = 0.0, v = 500.0, store_positions=True)

# 0.0 0.0 1.5707963267948966 125.95671633579087
# -61248.1367478616 4662.215846121733 0.0 1193.7158926932645

true_ownship = Plane(x=0.0, y=0.0, theta=1.5707963267948966, v=127.85404449630423, store_positions=True)
true_intruder = Plane(x=-60638.62205804177, y=5408.482161585932, theta=0.0, v=1008.4024558675674, store_positions=True)

runtime_monitor = Monitor()

# true_ownship = Plane(0, 100000, 0, 1000, True)
# true_intruder = Plane(100000, 0, 90, 1000, True)

# sample_unsafe_tau = 0
# sample_unsafe_delta_tau = 0

# sample_unsafe_ownship = Plane(x=-51184.2361552, y=-38308.2060179, theta=0.8172791641202594, v=881.5515816989571, store_positions=True)
# sample_unsafe_intruder = Plane(x=-91193.44527064, y=0, theta=0.0, v=1199.91375356, store_positions=True)

#sample_unsafe_ownship = Plane(x=-51184.2361552, y=-38308.2060179, theta=46.8266467880706023, v=881.5515817, store_positions=True)
#sample_unsafe_intruder = Plane(x=-91193.44527064, y=0, theta=0, v=1199.91375356, store_positions=True)

# 
# true_ownship = sample_unsafe_ownship
# true_intruder = sample_unsafe_intruder
# initial_tau = sample_unsafe_tau
# delta_tau = sample_unsafe_delta_tau
cur_tau = initial_tau

# sample_flightplan = [0.0 for i in range(100)] + \
#                     [1.5 for i in range(100)] + \
#                     [0.0 for i in range(100)] + \
#                     [-1.5 for i in range(100)] + \
#                     [0.0 for i in range(100)]

for i in range(simulation_length):

    print(f'main() at {i}s', horiz_distance_between_planes(true_intruder, true_ownship))

    visible_intruder = true_intruder if (horiz_distance_between_planes(true_intruder, true_ownship) <= 60760.0) else None

    generated_command = runtime_monitor.generate_command(true_ownship, visible_intruder, cur_tau, lookahead_scheme=0, lookahead_distance=15)

    true_intruder.step(0.0)
    true_ownship.step(generated_command)


    # If a collision is detected, no point in generating further
    # if unsafe_state(true_ownship, true_intruder, cur_tau):
    #     print("SAFETY VIOLATION. HORIZONTAL DISTANCE BROKEN.")
    #     print(true_ownship)
    #     print(true_intruder)
    #     break

    cur_tau += delta_tau

generate_animation([true_ownship.positions, true_intruder.positions], initial_tau)