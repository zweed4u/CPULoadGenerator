# Authors: Gaetano Carlucci
#         Giuseppe Cofano

import os, psutil
import threading


class MonitorThread(threading.Thread):
    """
       Monitors the CPU status
    """

    def __init__(self, cpu_cores, interval):
        self.sampling_interval = interval  # sample time interval
        self.sample = 0.5  # cpu load measurement sample
        self.cpu = 0.5  # cpu load filtered
        self.running = 1  # thread status
        self.alpha = 1  # filter coefficient
        self.sleepTimeTarget = 0.03
        self.sleepTime = 0.03
        self.cpuTarget = 0.5
        self.cpu_cores = cpu_cores  # array to support on process on multiple cores
        super(MonitorThread, self).__init__()

    def getCpuLoad(self):
        return self.cpu

    def setSleepTime(self, sleepTime):
        self.sleepTime = sleepTime

    def setCPUTarget(self, cpuTarget):
        self.cpuTarget = cpuTarget

    def run(self):
        p = psutil.Process(os.getpid())
        p.cpu_affinity(self.cpu_cores)

        while self.running:
            self.sample = p.cpu_percent(self.sampling_interval)
            # first order filter on the measurement samples
            self.cpu = self.alpha * self.sample + (1 - self.alpha) * self.cpu
