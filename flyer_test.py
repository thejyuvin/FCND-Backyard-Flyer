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

drone.takeoff(3)

drone.takeoff(0)
drone.disarm()
drone.stop()
