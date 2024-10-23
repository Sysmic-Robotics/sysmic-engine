
class Bezier:
    def __init__(self):
        self.path = []
        self.t_max = 10

    def calculate_points(self, raw_path):
        points = []
        for point in raw_path:
            points.append(point)
            if len(points) == 3:
                bezier = lambda t: self.bezier(points[0], points[1],points[2], t)
                for t in range(0, self.t_max):
                    t = t/self.t_max
                    self.path.append(bezier(t))
                points = [points[2]]
        for point in points:
            self.path.append(point)
        return self.path
            

    def bezier(self, p_o, p_1, p_2, t):
        first = p_o[0]* (1-t)**2, p_o[1] * (1-t)**2
        second = p_2[0]*t**2, p_2[1]* t**2
        third = p_1[0]*2*(1-t)*t,p_1[1]*2*(1-t)*t
        return first[0] + second[0] + third[0], first[1] + second[1] + third[1]
    