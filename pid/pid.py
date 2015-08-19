#
#p=PID(3.0,0.4,1.2)
#p.setPoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#

import datetime
now = datetime.datetime.now

class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, **kwargs):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        try:
            self.output_max = kwargs['output_max']
        except:
            self.output_max = 100

        try:
            self.output_min = kwargs['output_min']
        except:
            self.output_min = 0

        try:
            self.on_error = kwargs['on_error']
        except:
            self.on_error = True

        self.last_input = 0.0
        self.set_point = 0.0
        self.error = 0.0
        self.PID = 0.0


    # Proportional Calculation
    def get_P(Kp, error):
        return Kp * error


    # Integral Calculation
    def get_I(Ki, Iterm, error, time_delta, minimum, maximum):
        Iterm += Ki * error * time_delta

        if Iterm > maximum:
            Iterm = maximum

        elif Iterm < minimum:
            Iterm = minimum

        return Iterm


    # Derivative Calculation
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


    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.last_input = current_value
        self.last_sample = now()

        if not self.manual:
            time_delta = now() - self.last_sample
            self.error = self.set_point - current_value

            p_value = get_P(self.Kp, self.error)

            self.Integrator = get_I(
                self.Ki,
                self.error,
                time_delta,
                self.output_min,
                self.output_max
            )

            d_value, self.Derivator = get_D(
                self.Kd,
                self.error,
                self.Derivator,
                time_delta,
                self.on_error
            )

            self.PID = p_value + self.Integrator + d_value

            # Check output against hardware min/max capabilities
            if self.PID > output_max:
                self.PID = output_max

            elif self.PID < output_min:
                self.PID = output_min

            return self.PID


    def setPoint(self,set_point, manual=False):
        """
        Initilize the setpoint of PID
        """
        if manual:
            self.manual = True
            self.PID = set_point
        else:
            self.manual = False
            self.set_point = set_point

        # Initialize I and D
        # setting to previous values will avoid spikes if switching from
        # manual-mode to auto
        self.Integrator = self.PID
        self.Derivator=self.last_input

        self.last_sample=now()
