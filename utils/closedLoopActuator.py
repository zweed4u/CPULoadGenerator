# Authors: Gaetano Carlucci
#         Giuseppe Cofano

import time


# TODO sleep time governs the cpu usage - the smaller the value the higher the load

class closedLoopActuator():
    """
        Generates CPU load by tuning the sleep time
    """

    def __init__(self, controller, monitor, target):
        """
        The constructor for out 'actuator'
        :param controller: thread that controls the CPU status
        :param monitor: thread that monitors the CPU status
        :param target: float - .50 = 50% the cpu load desired
        """
        self.controller = controller
        self.monitor = monitor
        self.target = target
        self.controller.setCpu(self.monitor.getCpuLoad())
        self.period = 0.05  # actuation period  in seconds
        self.last_plot_time = time.time()
        self.start_time = time.time()

    def generate_load(self, sleep_time):
        interval = time.time() + self.period - sleep_time
        # generates some getCpuLoad for interval seconds
        while (time.time() < interval):
            pr = 213123 # generates some load
            pr * pr
            pr = pr + 1

        time.sleep(sleep_time)

    def run(self):
        while 1:
            self.controller.setCpu(self.monitor.getCpuLoad())
            sleep_time = self.controller.getSleepTime()
            self.generate_load(sleep_time)
