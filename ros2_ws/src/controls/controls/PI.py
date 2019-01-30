import time


class PI:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, F=(0.0, 0.0), minimum=-1.0, maximum=1.0):
        self.Kp = P
        self.Ki = I
        self.Kf = F
        self.P = 0
        self.ff = 0
        self.I = 0
        self.min = minimum
        self.max = maximum
        self.sample_time = 0.00
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.i = 0.0
        self.last_error = 0.0
        self.current_time = time.time()
        self.last_time = self.current_time

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 10.0
        self.output = 0.0

    def update(self, target, feedback_value, feed_forward):
        error = target - feedback_value
        if feed_forward >= 0:
            kff = self.Kf[0]
        else:
            kff = self.Kf[1]

        ff = kff * feed_forward

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        if delta_time >= self.sample_time:
            p = self.Kp * error
            self.i += error * delta_time

            if self.i < -self.windup_guard:
                self.i = -self.windup_guard
            elif self.i > self.windup_guard:
                self.i = self.windup_guard

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.P = p
            self.FF = ff
            self.I = self.Ki * self.i
            self.output = max(self.min, min(self.max, p + self.I + ff))
            return self.output


    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time
