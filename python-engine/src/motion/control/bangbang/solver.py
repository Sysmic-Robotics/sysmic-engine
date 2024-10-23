import math
from motion.control.bangbang.utils import State, Constraints, sign
from motion.control.bangbang.cases import Cases

# Solver for two DOF's
class DOFSolver:
    def __init__(self, state_x : State, state_y : State, constraints : Constraints, wfx, wfy):
        self.state_x = state_x
        self.state_y = state_y
        self.constraints = constraints
        self.wfx = wfx
        self.wfy = wfy
    
    def bisection_method(self, a, b, max_iter=1000):
        iter_count = 0
        t_diff = 1000
        while abs(t_diff) > 0.2 and iter_count < max_iter:
            mid = (a + b)/2
            f_mid = self.get_diff_time(mid)
            #print(f"Iteration {iter_count}: f(mid)={f_mid}")
            # Check if the middle value is closer to the minimum
            if f_mid > 0:
                b = mid
            else:
                a = mid
            t_diff = f_mid
            iter_count += 1
        #print(f"angle mid: {mid}")
        return mid

    # Solve for one DOF
    def get_traj(self, wf, initial_state : State, constraints : Constraints):
        tf = 0
        wf = sign(wf)*wf
        initial_state.v = sign(initial_state.v)*initial_state.v
        solution : list[type[State]] = []
        solution.append(initial_state)
        epsilon = 0.05
        last_state = initial_state
        while wf > epsilon:
            # Compute next state
            next_states = Cases.compute_next_state(last_state, constraints, wf)
            if(len(next_states) == 0):
                print(f"no next state: last_state: {last_state} wf: {wf} constraints : {constraints}")
                break
            # Update global state
            for state in next_states:
                tf += state.t
                state.t = tf
                wf -= math.ceil(state.d*100.0)/100.0 # always round up to solve float precision problem
                solution.append(state)
                last_state = state
        return solution

    # Flips velocities
    def flip_sol(self, sol):
        new_sol = []
        for state in sol:
            state.v = -state.v
            new_sol.append(state)
        return new_sol
    
    def get_diff_time(self, angle):
        # Find the best segment
        if self.wfx == 0 or self.wfy == 0:
            return 0
        v_max = self.constraints.v_max
        a_max = self.constraints.a_max
        x_constraints = Constraints(a_max*math.cos(angle), v_max*math.cos(angle))
        y_constraints = Constraints(a_max*math.sin(angle), v_max*math.sin(angle))
        x_sol = self.get_traj(self.wfx, self.state_x, x_constraints)
        y_sol = self.get_traj(self.wfy, self.state_y, y_constraints)
        tfx = x_sol[-1].t
        tfy = y_sol[-1].t
        return tfx - tfy

    def solve(self):
        best_angle = self.bisection_method(0, math.pi/2)
        v_max = self.constraints.v_max
        a_max = self.constraints.a_max
        x_constraints = Constraints(a_max*math.cos(best_angle), v_max*math.cos(best_angle))
        y_constraints = Constraints(a_max*math.sin(best_angle), v_max*math.sin(best_angle))
        x_sol = self.get_traj(self.wfx, self.state_x, x_constraints)
        if self.wfx < 0:
            x_sol = self.flip_sol(x_sol)
        y_sol = self.get_traj(self.wfy, self.state_y, y_constraints)
        if self.wfy < 0:
            y_sol = self.flip_sol(y_sol)
        return x_sol, y_sol