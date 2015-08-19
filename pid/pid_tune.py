#!/usr/bin/env python3
"""

"""

import datetime
from time import sleep
import pickle
import os
import sys

from pid import PID
from brew_control import Vessel

now = datetime.datetime.now

PI = 3.14159
MAX = 1
MIN = -1

CONFIG_PATH = ''

MIN_CYCLE = 5


class AutoTune:
    """

    """

    def __init__(self, **kwargs):
        """Class constructor

        Keyword Arguments:
          temp:    Initial steady-state input of system
          duty:   Initial duty cycle of the output
          vessel:   Object of vessel class to use for I/O
        """

        self.counter = 0

        self.vessel = kwargs['vessel']

        self.noise_band = 1
        self.duty_step = 50
        self.lookback = 30

        self.peak_type = 0
        self.count_peaks = 0
        self.justchanged = False
        self.setpoint = self.vessel.read_temp()
        self.duty_center = self.vessel.duty
        self.output = self.vessel.duty

        self.last_time = now()

        self.peaks = []
        self.valleys = []
        self.input_list = []

        self.latest_max = {}
        self.latest_min = {}

        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

    @staticmethod
    def avg_dist(*args):
        sum_diff = 0
        intervals = len(args) - 1

        for i in range(intervals):
            try:
                sum_diff += abs(args[i] - args[i + 1])
            except:
                sum_diff += abs(args[i] - args[i + 1]).total_seconds()

        return sum_diff / intervals

    @staticmethod
    def calculate_gain(A, Pu, D):
        Ku = 4 * D / (A * PI)

        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / Pu
        Kd = 0.075 * Ku * Pu

        return Kp, Ki, Kd

    @staticmethod
    def min_max(reference, value_list):
        is_max = True
        is_min = True

        for val in value_list:
            if is_max:
                is_max = reference > val
            if is_min:
                is_min = reference < val

        return is_max, is_min

    @staticmethod
    def set_output(output):
        """Set vessel output to specified level
        """
        raise NotImplementedError

    @staticmethod
    def high_band(temperature, limit):
        if temperature > limit:
            return True

        return False

    @staticmethod
    def low_band(temperature, limit):
        if temperature < limit:
            return True

        return False

    def oscillate(self, input_value, duty):
        """Oscillate the output base on the input_value's
        relation to the setpoint
        """
        output = None

        if self.high_band(input_value, self.setpoint + self.noise_band):
            output = self.duty_center - self.duty_step

        if self.low_band(input_value, self.setpoint - self.noise_band):
            output = self.duty_center + self.duty_step

        return output

    def step_tune(self):
        self.counter += 1
        current_time = now()

        self.last_time = current_time

        refVal = self.vessel.read_temp()

        new_output = self.oscillate(refVal, self.vessel.duty)
        if new_output:
            self.vessel.set_duty(new_output)

        # find peaks and valleys
        lookback_values = self.input_list[-self.lookback:]
        is_max, is_min = self.min_max(refVal, lookback_values)
        self.input_list.append(refVal)

        if len(self.input_list) < self.lookback:
            return None

        if is_max:
            if self.peak_type == MIN:
                print('{},{},MIN'.format(self.latest_min['time'], self.latest_min['value']))
                self.peak_type = MAX
                justchanged = True
                self.valleys.append(self.latest_min)
                self.count_peaks += 1

            self.peak_type = MAX
            self.latest_max = {'value': refVal, 'time': current_time}

        elif is_min:
            if self.peak_type == MAX:
                print('{},{},MAX'.format(self.latest_max['time'], self.latest_max['value']))
                self.peaks.append(self.latest_max)
                self.peak_type = MIN
                self.justchanged = True

            self.peak_type = MIN
            self.latest_min = {'value': refVal, 'time': current_time}

        num_periods = min(len(self.peaks), len(self.valleys))

        if num_periods >= MIN_CYCLE and self.justchanged:
            # Get last N peaks/valleys
            final_peaks = self.peaks[-MIN_CYCLE:]
            final_valleys = self.valleys[-MIN_CYCLE:]

            # Check for consistent amplitude
            peak_sep = self.avg_dist(*[peak['value'] for peak in final_peaks])
            valley_sep = self.avg_dist(*[valley['value'] for valley in final_valleys])

            # Calculate values from data
            high = sum(peak['value'] for peak in final_peaks) / len(final_peaks)
            low = sum(valley['value'] for valley in final_valleys) / len(final_valleys)

            amplitude = high - low
            amp_variation = max(peak_sep, valley_sep)
            tolerance = 0.05 * amplitude

            if amp_variation < tolerance:
                self.Kp, self.Ki, self.Kd = self.calculate_gain(
                    amplitude,
                    self.avg_dist(*[peak['time'] for peak in final_peaks]),
                    2 * self.duty_step
                )

                out_dict = {'Kp': self.Kp, 'Ki': self.Ki, 'Kd': self.Kd}

                print(
                    'Kp:{Kp}\nKi:{Ki}\nKd:{Kd}'.format(**out_dict),
                    file=sys.stderr
                )

                self.output = self.duty_center
                return out_dict

        self.justchanged = False
        return None

    def tune(self):

        gains = None
        timer = 1400

        while not gains and timer > 0:
            gains = self.step_tune()
            timer -= 1
            # sleep(1)

        if gains:
            return gains
        else:
            return None


if __name__ == '__main__':

    test_vessel = Vessel(dat_file='dat', duty=50)
    tuner = AutoTune(vessel=test_vessel)
    pid_gain = tuner.tune()

    if pid_gain:
        pid_obj = PID(
            P=pid_gain['Kp'],
            I=pid_gain['Ki'],
            D=pid_gain['Kd'],
            on_error=False,
            output_min=0,
            output_max=100,
        )

        pickle.dump(pid_obj, open(os.path.join(CONFIG_PATH, 'PID.p'), 'wb'))

    else:
        print('PID gains not found', file=sys.stderr)
