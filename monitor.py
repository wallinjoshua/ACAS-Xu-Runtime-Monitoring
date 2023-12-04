'''
    Runtime monitor module
    Sits between neural network controller and sim
    For a given set of commands

    Note: the simulation code is both used by the simulator
    (tracking the results of actions) and the runtime monitor
    (evaluating potential actions for safety). In reality, the
    monitor would have its own capacity for sim.
'''

# Controller maintains an internal state, representing the monitored system

from nn_controller import NN_Controller
from simulation import Plane, unsafe_state, horiz_distance_between_planes
from functools import lru_cache
import copy

cmds = [0.0, 1.5, -1.5, 3.0, -3.0]

class Monitor:

    MAX_LOOKAHEAD = 25

    def __init__(self, init_cmd = 0):
        self.controller = NN_Controller()
        self.cmd_queue = []
        self.prev_cmd = init_cmd

    # Determine if there will be a collision in the next t seconds
    # Assumes intruder maintains path
    # Executes cmds first, then uses networks for rest
    # Returns False if a cmd leads to a failure
    def lookahead(self, o, i, tau, dt, cmds, d_tau=0, stdout=False):
        t = 0

        ownship = Plane(o.x, o.y, o.theta, o.v)
        intruder = Plane(i.x, i.y, i.theta, i.v)
        sep_distance = horiz_distance_between_planes(ownship, intruder)
        next_cmd = cmds[0]
        while t < dt:
            ownship.step(next_cmd)
            intruder.step(0.0)
            sep_distance = horiz_distance_between_planes(ownship, intruder)
            if(unsafe_state(ownship, intruder, tau)):
                if stdout:
                    print(f"COLLISION DETECTED IN {t} SECONDS.")
                return sep_distance
            
            next_cmd = cmds[t] if t < len(cmds) else self.controller.get_command(next_cmd, tau, ownship.x, ownship.y, ownship.theta, ownship.v, \
                                                                                  intruder.x, intruder.y, intruder.theta, intruder.v)
            tau += d_tau
            t += 1
        return sep_distance
    
    # def multistep_lookahead(self, o, i, tau, dt, cmds, d_tau=0, stdout=False):
    #     t = 0

    #     ownship = Plane(o.x, o.y, o.theta, o.v)
    #     intruder = Plane(i.x, i.y, i.theta, i.v)

    #     next_cmd = cmds[0]

    # #TODO: Add backed in caching strategy (with approximation within tolerance)
    # #@lru_cache
    # def generate_command(self, o, i, tau):


    #     # Start a timer for 1 second:
    #         # Simulates the full period that the monitor has
    #         # to generate and return a command to be executed

    #     #If intruder is too far away, ACAS doesn't engage
    #     if i == None:
    #        return 0.0
        
    #     # Get a command proposed by the NN controller
    #     proposed_command = self.controller.get_command(self.prev_cmd, tau, o.x, o.y, \
    #                                             o.theta, o.v, i.x, i.y, \
    #                                             i.theta, i.v)

    #     # remaining_commands = cmds.copy()
    #     # remaining_commands.remove(proposed_command)

    #     # ownship = Plane(o.x, o.y, o.theta, o.v)
    #     # intruder = Plane(i.x, i.y, i.theta, i.v)

    #     # Evaluate what the next 10 time steps will look
    #     # like, if you follow that command, the intruder
    #     # continues, and you keep calling to the NN_Controller

    #     # fwd_timestep = 0

    #     # next_cmd = proposed_command
    #     # while(fwd_timestep < self.MAX_LOOKAHEAD):
    #     #     ownship.step(next_cmd)
    #     #     intruder.step(0.0)

    #     #     if(unsafe_state(ownship, intruder, tau - fwd_timestep)):
    #     #         # Have a problem, executing this action leads to a collision
    #     #         print('COLLISION UNDER PROPOSED COMMAND.')
    #     #         print(f'\tCOLLISION IN {fwd_timestep} STEPS.')

    #     #         # for alt_cmd in remaining_commands:
    #     #         #     ow = Plane(o.x, o.y, o.theta, o.v)
    #     #         #     intr = Plane(i.x, i.y, i.theta, i.v)
    #     #         #     fwd = 1
    #     #         #     ow.step(alt_cmd)
    #     #         #     intr.step(0.0)
    #     #         #     good = True
    #     #         #     while(fwd < self.MAX_LOOKAHEAD):
    #     #         #         if(unsafe_state(ow, intr, tau - fwd_timestep)):
    #     #         #             print(f"COLLISION UNDER ALTERNATE COMMAND {cmds[alt_cmd]}")
    #     #         #             good = False
    #     #         #             pass
    #     #         #         nextc = self.controller.get_command(next_cmd, tau, ownship.x, ownship.y, \
    #     #         #                                 ownship.theta, ownship.v, intruder.x, intruder.y, \
    #     #         #                                 intruder.theta, intruder.v)
    #     #         #         ow.step(nextc)
    #     #         #         intr.step(0.0)
    #     #         #         fwd += 1
    #     #         #     if good:
    #     #         #         print(f"Found better alternate: {alt_cmd}.")
    #     #         #         return alt_cmd
    #     #         #         break

    #     #         break

    #     #     next_cmd = self.controller.get_command(next_cmd, tau, ownship.x, ownship.y, \
    #     #                                         ownship.theta, ownship.v, intruder.x, intruder.y, \
    #     #                                         intruder.theta, intruder.v)
            
    #     #     fwd_timestep += 1

    #     self.prev_cmd = proposed_command

    #     return proposed_command

    def generate_command(self, o, i, tau, lookahead_scheme = None, lookahead_distance = MAX_LOOKAHEAD, stdout=False):

        #If no intruder is detected (i == None), then just continue current heading
        # Set the previous command equal to this
        if i == None:
            self.prev_cmd = 0.0
            return 0.0
        
        #If a command has already been queued up in the monitor, use that as the 
        # proposed command instead of one given by the neural networks
        # if self.cmd_queue:
        #     proposed_command = self.cmd_queue.pop()
        # else:
        proposed_command = self.controller.get_command(self.prev_cmd, tau,
                                                        o.x, o.y, o.theta, o.v,
                                                        i.x, i.y, i.theta, i.v)
        self.prev_cmd = proposed_command

        #Switch based on the lookahead scheme
        # None: no lookahead, just pass through the proposed command
        # Single-step repair: lookahead for the current control period, pick the best command
        # Multi-step repaid: lookahead, building a chain of commands as necessary to maximize sep distance
        if not lookahead_scheme:
            self.prev_command = proposed_command
            return proposed_command
        elif lookahead_scheme == 1: # Single step repair
            # Check to see if collision projected; if so, repair single command
            if self.lookahead(o, i, tau, lookahead_distance, [proposed_command], stdout = False) < 500:
                alternative = None 
                remaining_commands = copy.deepcopy(cmds)
                remaining_commands.remove(proposed_command)
                best_separation_distance = 0
                best_cmd = -1
                for cmd in remaining_commands:
                    if self.lookahead(o, i, tau, lookahead_distance, [cmd], stdout=stdout) > 500:
                        alternative = cmd
                        break
                if alternative:
                    self.prev_cmd = alternative
                else:
                    self.prev_cmd = proposed_command
            return self.prev_cmd
        elif lookahead_scheme == 2: #Multi-step repair
            cmd_prefix = [proposed_command]

            if self.lookahead(o, i, tau, lookahead_distance, cmd_prefix, stdout=False) < 500:
                # Generate as much of a prefix as needed to avoid a collision, or to maximize separation distance
                cmd_prefix = []
                for _ in range(self.MAX_LOOKAHEAD):
                    remaining_commands = copy.deepcopy(cmds)
                    best_cmd = -1
                    best_sep_distance = -1
                    for cmd in remaining_commands:
                        cur_sep_distance = self.lookahead(o, i, tau, lookahead_distance, cmd_prefix + [cmd], stdout=False) 
                        if cur_sep_distance > best_sep_distance:
                            best_sep_distance = cur_sep_distance
                            best_cmd = cmd
                    cmd_prefix += [best_cmd]
                    if cur_sep_distance > 500:
                        break
                if stdout:
                    print(f"Repair Length: {len(cmd_prefix)}")
            self.prev_cmd = cmd_prefix.pop()
            self.cmd_queue = cmd_prefix
            return self.prev_cmd