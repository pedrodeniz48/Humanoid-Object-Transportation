class Controller:
    
    def __init__(self, variable=None, ref=None, dst = None):
        self.variable = variable
        self.ref = ref
        self.u1 = 0
        self.ang_error = None

        self.dst = dst
    
    def PID(self, Kp, Ki, Kd, Tm, sat_min=-20, sat_max=20, sat_min_value=-20, sat_max_value=20):
        error0 = self.ref - self.variable

        error1 = 0
        error2 = 0

        u = self.u1 + ( Kp + Kd/Tm)*error0 + (-Kp + Ki*Tm - 2*Kd/Tm)*error1 + (Kd/Tm)*error2
        #ux = ux1 + (Kpx + Kdx/Tm)*errx0 + (-2*Kdx/Tm)*errx1 + (-Kpx + Kdx/Tm)*errx2

        if u >= sat_max: u = sat_max_value
        elif u <= sat_min: u = sat_min_value

        self.u1 = u
        error1 = error0
        error2 = error1

        #if ux > 3 or ux < -3:

        if error0 < -5 or error0 > 5:
            return u
    
    def trajectory_control(self):

        M = self._orientation_control()
        
        return M
    
    def _orientation_control(self):
        self.ang_error = self.ref - self.variable
    
        if self.ang_error < -15 or self.ang_error > 15:
            if self.variable > 0:
                print(self.ref, self.variable, self.ang_error)
                if self.ang_error > 0:
                    M = [1, 0, 0, 0, 0] #turn_right, turn_left, right_walk, left_walk, forward
                else:
                    M = [0, 1, 0, 0, 0]
            else:
                if self.ang_error < 0:
                    M = [1, 0, 0, 0, 0] #turn_right, turn_left, right_walk, left_walk, forward
                else:
                    M = [0, 1, 0, 0, 0]
        else:
            M = self._lateral_displacement_control()

        return M
    
    def _lateral_displacement_control(self):
    
        if self.dst < -20 or self.dst > 20:
            if self.dst > 0:
                M = [0, 0, 1, 0, 0] #turn_right, turn_left, right_walk, left_walk, forward
            else:
                M = [0, 0, 0, 1, 0]
        else:
            M = [0, 0 , 0, 0, 1]
        
        return M