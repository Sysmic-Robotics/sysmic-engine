class State:
    def __init__(self, v, t, a, d):
        self.v = v # final velocity
        self.t = t # final time
        self.a = a # acceleration applied from last state to this state
        self.d = d # distance traveled from last state and this state
    def __repr__(self):
        return "(vf: {0}, t: {1}, a: {2}, d: {3})".format(self.v, self.t, self.a, self.d)

class Constraints:
    def __init__(self, a_max, v_max):
        self.a_max = a_max # <- constant
        self.v_max = v_max
    def __repr__(self):
        return f" a_max:{self.a_max} v_max:{self.v_max}"

def sign(n):
    if n < 0:
        return -1
    return 1