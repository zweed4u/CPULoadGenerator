#!/usr/bin/python

# Authors: Gaetano Carlucci
#         Giuseppe Cofano


import sys

sys.path.insert(0, 'utils')

from utils.Monitor import MonitorThread
from utils.Controller import ControllerThread
from utils.closedLoopActuator import closedLoopActuator




if __name__ == "__main__":

    # Invoke ./CPULoadGenerator.py <percentage> eg. .01 - 1.0
    cpuLoad = float(sys.argv[-1])

    # 0 = core0
    cores = [0]
    #monitor = MonitorThread([coreNum for coreNum in range(multiprocessing.cpu_count())], 0.1)
    monitor = MonitorThread(cores, 0.1)
    monitor.start()

    control = ControllerThread(0.1)
    control.start()
    control.setCpuTarget(cpuLoad) # float eg. .1 =10% 1.0=100%

    # control thread, monitor thread, desired cpu usage
    actuator = closedLoopActuator(control, monitor, cpuLoad)
    actuator.run()

    monitor.running = 0
    control.running = 0
    monitor.join()
    control.join()
