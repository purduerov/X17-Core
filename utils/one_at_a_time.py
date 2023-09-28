#! /usr/bin/python3

import signal
import sys
import time

from spidev import SpiDev

import numpy as np
spi = SpiDev()
spi.open(0,1)
spi.max_speed_hz = 500000
OFF_THRUST = 127


def signal_handler(sig, frame) -> None:
    print("CTRL+C detected")
    zeroOutThrusters()
    print("Thrusters zero-ed out")
    sys.exit(0)

def zeroOutThrusters() -> None:
    thrusts = [OFF_THRUST] * 8
    thrusts = bytearray(thrusts)
    writeSPI(thrusts, printOut=True)

def writeSPI(packet, printOut=True) -> None:
    # publish = bytearray(packet)
    publish = bytearray(packet)
    if printOut:
        print(publish)
    spi.writebytes(publish)

def mainLoop(timesleep=1, bound=5):
    
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        thrusts = [OFF_THRUST] * 8
        num = -1
        while(int(num) < 1 or int(num) > 8):
            print("Enter a thruster number(1-8): ")
            num = input()
            if(int(num) < 1 or int(num) > 8):
                print("Nope try again\n")
            else:
                break
        thrusts[int(num) - 1] = 150
        print("Thrusters numsetting to {}".format(num))

        
        writeSPI(thrusts)
        print(thrusts)



        time.sleep(timesleep)
        thrusts[int(num) - 1] = 127
        writeSPI(thrusts)


    zeroOutThrusters()

if __name__ == "__main__":
    bound = 127
    inc = 15
    print(mainLoop(bound=bound, timesleep=10.0))