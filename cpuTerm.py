#!/usr/bin/python3.6
"""
Authors: Gaetano Carlucci, Giuseppe Cofano
Modified: Zachary Weeden
"""
import os
import time
import psutil
import logging
import threading
import multiprocessing


class ClosedLoopActuator(threading.Thread):
    def __init__(self, controller, monitor, target: float):
        super().__init__()
        self.running = 1
        self.controller = controller
        self.monitor = monitor
        self.target = target
        self.controller.setCpu(self.monitor.getCpuLoad())
        self.period = 0.05  # actuation period  in seconds

    def generate_load(self, sleep_time: float):
        interval = time.time() + self.period - sleep_time
        # generates some getCpuLoad for interval seconds
        while (time.time() < interval):
            pr = 213123  # generates some load
            pr * pr
            pr = pr + 1

        time.sleep(sleep_time)

    def run(self):
        while self.running:
            self.controller.setCpu(self.monitor.getCpuLoad())
            sleep_time = self.controller.getSleepTime()
            self.generate_load(sleep_time)


class ControllerThread(threading.Thread):
    def __init__(self, interval: float, ki: float=None, kp: float=None):
        super().__init__()
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

    def getSleepTime(self):
        return self.sleepTime

    def setCpu(self, cpu: float):
        # first order filter on the measurement samples
        self.cpu = self.alpha * cpu + (1 - self.alpha) * self.cpu

    def setCpuTarget(self, CT: float):
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


class MonitorThread(threading.Thread):
    def __init__(self, cpu_core: list, interval: float):
        super().__init__()
        self.sampling_interval = interval  # sample time interval
        self.sample = 0.5  # cpu load measurement sample
        self.cpu = 0.5  # cpu load filtered
        self.running = 1  # thread status
        self.alpha = 1  # filter coefficient
        self.sleepTimeTarget = 0.03
        self.sleepTime = 0.03
        self.cpuTarget = 0.5
        self.cpu_core = cpu_core

    def getCpuLoad(self):
        return self.cpu

    def run(self):
        p = psutil.Process(os.getpid())
        if len(self.cpu_core) > 0:
            p.cpu_affinity(self.cpu_core)
        else:
            p.cpu_affinity([coreNum for coreNum in range(multiprocessing.cpu_count())])
        while self.running:
            self.sample = p.cpu_percent(self.sampling_interval)
            # first order filter on the measurement samples
            self.cpu = self.alpha * self.sample + (1 - self.alpha) * self.cpu


def cpu_stress(cpu_core: int, cpu_load: float, keep_running: multiprocessing.Value):
    monitor = MonitorThread([cpu_core], 0.1)
    monitor.start()

    control = ControllerThread(0.1)
    control.start()
    control.setCpuTarget(cpu_load)

    # control thread, monitor thread, desired cpu usage
    actuator = ClosedLoopActuator(control, monitor, cpu_load)
    actuator.start()
    actuator.join()

class CPULoad:
    """CPU load process generator for each core"""

    def __init__(self, load: int, keep_running: multiprocessing.Value=1):
        """
        :param load: the input from apogee config supports floats - average CPU usage (0.0-100.0)
        """
        try:
            self.load_percent = float(load)
        except:
            raise TypeError('CPU load average must be a number')
        if self.load_percent < 0.0 or self.load_percent > 100.0:
            raise ValueError('CPU load average must be within 0.0 and 100.0')
        self.keep_running = keep_running
        self.core_procs = [] # list of processes for each core load

    def load(self):
        load_percent = float(self.load_percent / 100)
        for core in range(multiprocessing.cpu_count()):
            core_process = multiprocessing.Process(target=cpu_stress, args=(core, load_percent, self.keep_running))
            print(f'Generating {self.load_percent}% CPU load for Core-{core}')
            core_process.start()
            self.core_procs.append(core_process)

a=CPULoad(29.5)
a.load()
for core_proc in a.core_procs:
    print(core_proc.is_alive())

input('')
for core_proc in a.core_procs:
    core_proc.terminate()
input('')
for core_proc in a.core_procs:
    print(core_proc.is_alive())
