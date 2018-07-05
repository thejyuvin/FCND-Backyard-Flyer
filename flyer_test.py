import argparse
import time
from enum import Enum

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)
drone = Drone(conn)

time.sleep(2)
print ('starting drone...')
drone.start()

print ('taking control...')
drone.take_control()

print('arming...')
drone.arm()

drone.set_home_position(drone.global_position[0], 
                        drone.global_position[1], 
                        drone.global_position[2])

drone.takeoff(1)
time.sleep(5)
drone.cmd_position(0,0,3, 0)
time.sleep(5)
drone.cmd_position(0,10,3, 0)
time.sleep(5)
drone.cmd_position(10,10,3, 0)
time.sleep(5)
drone.cmd_position(10,0,3, 0)
time.sleep(5)
drone.cmd_position(0,0,3, 0)
time.sleep(5)
drone.cmd_position(0,0,3, 0)
time.sleep(5)

drone.disarm()
drone.stop()
