#!/usr/bin/python

#Authors: Gaetano Carlucci
#         Giuseppe Cofano

import multiprocessing
from twisted.python import usage

import sys
sys.path.insert(0, 'utils')

from utils.Monitor import MonitorThread
from utils.Controller import ControllerThread
from utils.closedLoopActuator import closedLoopActuator

class Options(usage.Options):
    """
       Defines the default input parameters
    """
    optParameters = [
            ["cpuLoad", "l", 0.2, "Cpu Target Load", float],
            ["cpu_core", "c" , 0, "Select the CPU on which generate the load", int]
        ]
                 
if __name__ == "__main__":

    import sys
    options = Options()
    try:
        options.parseOptions()
    except Exception, e:
        print '%s: %s' % (sys.argv[0], e)
        print '%s: Try --help for usage details.' % (sys.argv[0])
        sys.exit(1)
    else:
        if options['cpuLoad'] < 0 or options['cpuLoad'] > 1: 
            print "CPU target load out of the range [0,1]"
            sys.exit(1)
        if options['cpu_core'] >= multiprocessing.cpu_count(): 
            print "You have only %d cores on your machine" % (multiprocessing.cpu_count())
            sys.exit(1)
    
    monitor = MonitorThread(options['cpu_core'], 0.1)
    monitor.start()

    control = ControllerThread(0.1)
    control.start()
    control.setCpuTarget(options['cpuLoad'])

    actuator = closedLoopActuator(control, monitor, options['cpu_core'], options['cpuLoad'])
    actuator.run()

    monitor.running = 0;
    control.running = 0;
    monitor.join()
    control.join()
