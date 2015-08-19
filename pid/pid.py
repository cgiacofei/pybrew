#
#p=PID(3.0,0.4,1.2)
#p.setPoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#

import datetime

from heat import Element

now = datetime.datetime.now

class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, **kwargs):

        self.gains = {
            'Kp': P,
            'Ki': I,
            'Kd': D
        }

        self.element = kwargs['element']
        self.probe = kwargs['probe']

        try:
            self.limits = kwargs['limits']
        except:
            self.limits = (0, 100)

        try:
            self.on_error = kwargs['on_error']
        except:
            self.on_error = False

        self.target = 0.0

    @staticmethod
    def get_P(Kp, error):
        return Kp * error

    @staticmethod
    def get_I(Ki, Iterm, error, time_delta, limits):
        Iterm += Ki * error * time_delta

        if Iterm > max(limits):
            Iterm = max(limits)

        elif Iterm < min(limits):
            Iterm = min(limits)

        return Iterm

    @staticmethod
    def get_D(Kd, error, Dterm, time_delta, on_error):
        # calculate derivative on error
        if on_error:
            D_value = Kd * (error - Dterm) / time_delta
            Dterm = error

        # calculate derivative on the input value
        else:
            self.D_value = Kd * (current_value - Dterm) / time_delta
            Dterm = current_value

        return D_value, Dterm

    def read_temp(self):
        # Get the current temperature reading
        pass

    def update(self):
        """
        Calculate PID output value for given reference input and feedback
        """

        last_input = current_value
        self.last_sample = now()

        if not self.manual:
            time_delta = now() - self.last_sample
            self.error = self.target - current_value

            p_value = get_P(self.gains['Kp'], self.error)

            self.Integrator = get_I(
                self.gains['Ki'],
                self.error,
                time_delta,
                self.limits,
            )

            d_value, self.Derivator = get_D(
                self.gains['Kd'],
                self.error,
                self.Derivator,
                time_delta,
                self.on_error
            )

            self.PID = p_value + self.Integrator + d_value

            # Check output against hardware min/max capabilities
            if self.PID > max(limits):
                self.PID = max(limits)

            elif self.PID < min(limits):
                self.PID = min(limits)

            return self.PID

    def setpoint(self, **kwargs=None):
        """
        Initilize the setpoint of PID

        Keyword Arguments:
            temperature: Desired temperature (auto)
            duty: Desired duty ration (manual)
            Iterm: Integral value to start with
            Dterm: Derivative value to start with
        """

        if kwargs:
            # Initialize I and D
            # setting to previous values will avoid spikes if switching from
            # manual-mode to auto
            try:
                self.Integrator = kwargs['Iterm']
            except:
                self.Integrator = 0

            try:
                self.Derivator = kwargs['Dterm']
            except:
                self.Derivator = 0

            # Set manual/auto
            try:
                self.target = kwargs['temperaure']
                self.manual = False
            except:
                self.element.PWM(duty=kwargs['duty'])
                self.manual = True

            self.last_sample=now()

        else:
            if self.manual:
                return self.element.PWM()
            else
                return self.target

def avg_dist(*args):
    sum_diff = 0
    intervals = len(args) - 1

    for i in range(intervals):
        try:
            sum_diff += abs(args[i] - args[i + 1])
        except:
            sum_diff += abs(args[i] - args[i + 1]).total_seconds()

    return sum_diff / intervals

def calculate_gain(A, Pu, D):
    Ku = 4 * D / (A * PI)

    Kp = 0.6 * Ku
    Ki = 1.2 * Ku / Pu
    Kd = 0.075 * Ku * Pu

    return Kp, Ki, Kd

def min_max(reference, value_list):
    is_max = True
    is_min = True

    for val in value_list:
        if is_max:
            is_max = reference > val
        if is_min:
            is_min = reference < val

    return is_max, is_min

def set_output(output):
    """Set vessel output to specified level
    """
    raise NotImplementedError

def high_band(temperature, limit):
    if temperature > limit:
        return True

    return False

def low_band(temperature, limit):
    if temperature < limit:
        return True

    return False

def auto_tune():

