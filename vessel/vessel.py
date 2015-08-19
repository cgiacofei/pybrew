
class Vessel(object):
    """Dummy class to return pre-generated data
    """
    def __init__(self, dat_file='dat', duty=0):
        self.dfile = dat_file
        self.line_num = 0
        self.duty = duty


    def read_temp(self):
        temp = open(self.dfile)
        temp_lines = temp.readlines()
        temp_output = float(temp_lines[self.line_num])

        self.line_num +=1 

        return temp_output


    def get_duty(self):
        """Dummy method, returns fake duty cycle
        """
        return 50


    def set_duty(self, duty):
        """Dummy method, will be used to set the output duty cycle
        """
        pass


if __name__ == '__main__':
    test = Vessel('dat')

    for i in range(10):
        print(float(test.read_data()))
