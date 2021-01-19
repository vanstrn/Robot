#!/usr/bin/python
"""
0.4 Amps/phase
0.11 Nm holding torque
0.9 deg step angle - 400 steps
"""
import time
import numpy as np
from multiprocessing import Process, Queue

class Binary:
    def __init__(self, queue, pin, axis="A", verbose="False"):
        self.q = queue
        self.pin = pin
        self.state = 0
        self.debounce_time = 0.2
        self.last_button_ts = 0
        self.axis=axis
        self.verbose = verbose

    def start(self):
        self.p = Process(target=self.run, args=((self.q),))
        self.p.start()

    def run(self, queue):
        inp = (0,0,0)

        # GPIO.setwarnings(False)
        # GPIO.setmode(GPIO.BOARD)
        # GPIO.setup(self.pin, GPIO.OUT) # step

        while True:
            try:
                inp = queue.get_nowait()
                print(inp)
                if inp[0] == self.axis and time.time() - self.last_button_ts > self.debounce_time:
                    self.state = (self.state + 1) % 2
                    if self.verbose: print( "[Binary] Changed state to", self.state)
                    self.last_button_ts = time.time()
            except:
                time.sleep(0.01)
                pass
