#!/usr/bin/python
"""
Authors: Gaetano Carlucci, Giuseppe Cofano
Modified: Zachary Weeden
"""
import os
import sys
import subprocess
import multiprocessing

class CPULoad:
    """
    Constructs necessary python files on instantiation and runs the package with parameters specified. For each core a new
    process is opened to mitigate issues and locks similar to piped invocation in the original script.
    """
    def __init__(self):

        self.file_path = os.path.expanduser('~')
        with open(f'{self.file_path}/cpuStress.py', 'w') as f:
            f.write(cpu_load_main)

        with open(f'{self.file_path}/Monitor.py', 'w') as f:
            f.write(monitor)

        with open(f'{self.file_path}/Controller.py', 'w') as f:
            f.write(controller)

        with open(f'{self.file_path}/closedLoopActuator.py', 'w') as f:
            f.write(actuator)

    def run(self, core_num, load_percent):
        subprocess.Popen(['python', f'{self.file_path}/cpuStress.py', core_num, load_percent])


cpu_load_main = """
import sys
from Monitor import MonitorThread
from Controller import ControllerThread
from closedLoopActuator import closedLoopActuator

cpu_load_float = float(sys.argv[-1])
core_num = int(sys.argv[-2])
monitor = MonitorThread([core_num], 0.1)
monitor.start()

control = ControllerThread(0.1)
control.start()
control.setCpuTarget(cpu_load_float) # float eg. .1 =10% 1.0=100%

# control thread, monitor thread, desired cpu usage
actuator = closedLoopActuator(control, monitor, cpu_load_float)
actuator.run()

monitor.running = 0
control.running = 0
monitor.join()
control.join()
"""

monitor = """
import os, psutil
import threading
import multiprocessing

class MonitorThread(threading.Thread):

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

    def run(self):
        p = psutil.Process(os.getpid())
        if len(self.cpu_cores) > 0:
            p.cpu_affinity(self.cpu_cores)
        else:
            p.cpu_affinity([coreNum for coreNum in range(multiprocessing.cpu_count())])
        while self.running:
            self.sample = p.cpu_percent(self.sampling_interval)
            # first order filter on the measurement samples
            self.cpu = self.alpha * self.sample + (1 - self.alpha) * self.cpu
"""

controller = """
import threading
import time

class ControllerThread(threading.Thread):
    def __init__(self, interval, ki=None, kp=None):
        self.running = 1  # thread status
        self.sampling_interval = interval
        self.period = 0.1  # actuation period  in seconds
        self.sleepTime = 0.02  # this is controller output: determines the sleep time to achieve the requested CPU load
        self.alpha = 0.2  # filter coefficient
        self.CT = 0.20  # target CPU load should be provided as input 
        self.cpu = 0  # current CPU load returned from the Monitor thread
        self.cpuPeriod = 0.03
        if ki is None:
            self.ki = 0.2  # integral constant of th PI regulator
        if kp is None:
            self.kp = 0.02  # proportional constant of th PI regulator
        self.int_err = 0  # integral error
        self.last_ts = time.time()  # last sampled time
        super(ControllerThread, self).__init__()

    def getSleepTime(self):
        return self.sleepTime

    def setCpu(self, cpu):
        # first order filter on the measurement samples
        self.cpu = self.alpha * cpu + (1 - self.alpha) * self.cpu

    def setCpuTarget(self, CT):
        self.CT = CT

    def run(self):
        while self.running:
            # ControllerThread has to have the same sampling interval as MonitorThread
            time.sleep(self.sampling_interval)
            self.err = self.CT - self.cpu * 0.01  # computes the proportional error
            ts = time.time()

            samp_int = ts - self.last_ts  # sample interval
            self.int_err = self.int_err + self.err * samp_int  # computes the integral error
            self.last_ts = ts
            self.cpuPeriod = self.kp * self.err + self.ki * self.int_err

            # anti wind up control
            if self.cpuPeriod < 0:
                self.cpuPeriod = 0
                self.int_err = self.int_err - self.err * samp_int
            if self.cpuPeriod > self.period:
                self.cpuPeriod = self.period
                self.int_err = self.int_err - self.err * samp_int
            self.sleepTime = self.period - self.cpuPeriod
"""

actuator = """
import time

class closedLoopActuator():

    def __init__(self, controller, monitor, target):
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
"""

cpu_load_percent = sys.argv[-1] # ./deploy .25
stressor = CPULoad()
for core in range(multiprocessing.cpu_count()):
    stressor.run(str(core), cpu_load_percent)

