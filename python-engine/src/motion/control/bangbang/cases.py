import math
from motion.control.bangbang.utils import State, Constraints

class Cases:
    # The initial velocity is negative; the vehicle has to accelerate 
    # with maximal control effort until it reaches v = 0
    def case_1(last_state : State, constraints : Constraints):
        t = -(last_state.v/constraints.a_max)
        d = (last_state.v**2)/constraints.a_max
        new_state = State(0, t, constraints.a_max, d)
        return new_state


    # The vehicle has to accelerate, either because the destination is far away or the initial velocity is small.
    def case_2_1(last_state : State, constraints : Constraints, v):
        #print("Case 2.1")
        t = (v - last_state.v)/constraints.a_max
        d = last_state.v*t + (constraints.a_max*(t**2))/2
        new_state = State(v, t, constraints.a_max , d)
        return new_state

    # The vehicle is cruising at maximum velocity until it has to decelerate.
    def case_2_2(last_state : State, constraints : Constraints, wf): # Same velocity
        v0 = last_state.v
        a_max = constraints.a_max
        t = wf/v0 - (v0/(2*a_max))
        #t = (wf/last_state.v) - ( last_state.v/(2*constraints.a_max) )
        #if last_state.v != 0:
        #    t += wf/last_state.v
        d = wf - ( ( last_state.v**2 )/( 2*constraints.a_max ) )
        new_state = State(constraints.v_max, t, 0, d)
        return new_state

    # The vehicle has to decelerate until it reaches zero final velocity.
    def case_2_3(last_state : State, constraints : Constraints):
        #print("Case 2.3")
        t = last_state.v/constraints.a_max
        d = (last_state.v**2)/(2*constraints.a_max)
        new_state = State(0, t, -constraints.a_max, d)
        return new_state

    # The vehicle moves faster than the allowed maximum velocity.
    def case_3(last_state : State, constraints : Constraints):
        #print("Case 3")
        t = (last_state.v - constraints.v_max)/constraints.a_max
        d = ( 1/(2*constraints.a_max) )*(last_state.v**2 - constraints.a_max)
        new_state = State(constraints.v_max, t, -constraints.a_max, d)
        return new_state

    def compute_next_state(last_state : State, constraints : Constraints, wf) -> list[type[State]] :
        a_max = constraints.a_max
        v_max = constraints.v_max
        v0 = last_state.v
        states : list[type[State]] = []
        # Case 1
        if v0 < 0:
            next_state = Cases.case_1(last_state, constraints)
            states.append(next_state)
        # Case 2.1
        elif wf > ( v0**2 )/(2*a_max) and v_max > v0 >= 0:
            # Time to reach v_max
            tI = (v_max - v0)/a_max
            # Time to decelerate
            v1 = math.sqrt( wf*a_max + (v0**2)/2 )
            tII = (v1 - v0)/a_max
            # Subcase I: the vehicle reaches vw,max and is cruising
            if tI < tII:
                # Subcase I
                state_0 = Cases.case_2_1(last_state, constraints, v_max)
                wf -= state_0.d
                state_1 = Cases.case_2_2(state_0, constraints, wf)
                states += [state_0, state_1]
            # Subcase II: the vehicle decelerates until it reaches the final destination
            else:
                state_0 = Cases.case_2_1(last_state, constraints, v1)
                wf -= state_0.d
                state_1 = Cases.case_2_3(state_0, constraints)
                states += [state_0, state_1]
        # Case 2.2
        elif wf > (v0**2)/(2*a_max) and round(v0,2) == round(v_max,2):
            next_state = Cases.case_2_2(last_state, constraints, wf)
            states.append(next_state)
        # Case 2.3
        elif wf <= (v0**2)/(2*a_max) and 0 < v0 <= v_max:
            next_state = Cases.case_2_3(last_state, constraints)
            states.append(next_state)
        # Case 3
        elif v0 > v_max:
            next_state = Cases.case_3(last_state, constraints)
            states.append(next_state)
        return states