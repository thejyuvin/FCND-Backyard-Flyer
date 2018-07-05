import argparse
import time
import datetime
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.height_reached = False

        # initial state
        self.flight_state = States.MANUAL

        # Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def log_print(self, *arg):
        print(datetime.datetime.now(), arg)

    def get_abs_distances(self, local_pos, target_pos):
        distance = ( 
            abs( local_pos[0] - target_pos[0]), 
            abs( local_pos[1] - target_pos[1]), 
            abs( local_pos[2] - (-1*target_pos[2]) ) 
        )

        self.log_print('get_distance: distance: ',distance)
        return distance

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        self.log_print('lpc: -----Enter----------')
        self.log_print('lpc: local_position: ',self.local_position)
        self.log_print('lpc: target_position: ',self.target_position)
        self.log_print('lpc: flight_state', self.flight_state)
        self.log_print('lpc: Mission Status:', self.in_mission)

        if self.flight_state == States.TAKEOFF:
            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]

            #assume height-reached if within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.log_print('lpc: Height reached')
                print('******Height reached*****')
                self.waypoint_transition()
        
        if self.flight_state == States.WAYPOINT:
            self.log_print('lpc: current waypoints:', self.all_waypoints)

            distance = self.get_abs_distances(self.local_position, self.target_position)
            # check if distances on all 3 axis are within X cm of target point
            min_dist = 0.40
            if distance[0] < min_dist and distance[1] < min_dist and distance[2] < min_dist:
                self.log_print('lpc: Waypoint reached')
                print('******Waypoint reached*****')
                # remove the reached waypoint
                if len(self.all_waypoints) > 0:
                    del self.all_waypoints[0]
                    self.log_print('lpc: deleting first waypoint')
                    self.log_print('lpc: remaining waypoints:', self.all_waypoints)

                # move to next waypoint in list
                if len(self.all_waypoints) > 0:
                    self.target_position = self.all_waypoints[0]
                else:
                    self.landing_transition()
            else:
                self.log_print('lpc: Sending to Waypoint Transition')
                self.waypoint_transition()

        if self.flight_state == States.ARMING:
            self.log_print('lpc: ARMING / pass')
        
        if self.flight_state == States.DISARMING:
            print('******Disarming*****')
            self.log_print('lpc: DISARMING / pass')
        
        if self.flight_state == States.LANDING:
            self.log_print('lpc: LANDING')
            print('******Landing*****')
            #self.in_mission = False
            if abs(self.local_position[2]) < 0.01:
                self.disarming_transition()

        if self.flight_state == States.MANUAL:
            if self.in_mission:
                print('******Arming*****')
                self.arming_transition()

        self.log_print('lpc: -----Exit----------')

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        self.log_print('velocity_callback: Pass')

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        self.log_print('sc: enter')
        self.log_print('sc: flight_state: ', self.flight_state)

        if not self.in_mission:
            self.log_print('sc: Not In Mission')
            return
            
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        if self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        if self.flight_state == States.TAKEOFF and self.height_reached:
            self.log_print('sc: go to waypoint transition')  
            self.waypoint_transition()
        if self.flight_state == States.WAYPOINT:
            if len(self.all_waypoints) == 0:
                self.landing_transition()
            else:
                self.log_print('sc: Keeping States.WAYPOINT')
        if self.flight_state == States.DISARMING:
            self.in_mission = False
            print('******Shutting Down*****')
            self.manual_transition()
        
        self.log_print('sc: exit')

    def calculate_box(self):
        """
        Return waypoints to fly a box
        """
        self.log_print('calculate_box: ')
        return [
                [00, 10, 3],
                [10, 10, 3],
                [10, 00, 3],
                [00, 00, 3]
        ]

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        self.log_print("arming_transition: enter")
        drone.take_control()
        drone.arm()
        drone.set_home_position(drone.global_position[0], 
                                drone.global_position[1], 
                                drone.global_position[2])
        self.flight_state = States.ARMING
        self.log_print("arming_transition: state changed to :", self.flight_state)
        self.log_print("arming_transition: exit")

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        self.log_print("takeoff_transition: enter")
        print('******Taking Off*****')
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        drone.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        self.log_print("takeoff_transition: exit")

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.log_print("waypoint_transition: enter")
        
        if self.flight_state == States.TAKEOFF:
            self.log_print("waypoint_transition: updating target position to first waypoint")
            if len(self.all_waypoints) > 0:
                self.target_position = self.all_waypoints[0]
            else:
                self.log_print('waypoint_transition: No waypoints available', self.all_waypoints)

        if len(self.all_waypoints) == 0:
            self.landing_transition()
        else:
            self.log_print('waypoint_transition: target_position: ',self.target_position)
            drone.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0)
            self.flight_state = States.WAYPOINT
            #time.sleep(5)
        self.log_print("waypoint_transition: exit")

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        self.log_print("landing_transition: enter")
        drone.land()
        self.flight_state = States.LANDING
        self.log_print("landing_transition: exit")

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        self.log_print("disarming_transition: enter")
        drone.disarm()
        self.flight_state = States.DISARMING
        self.log_print("disarming_transition: exit")

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        self.log_print("manual_transition: enter")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
        self.log_print("manual_transition: exit")

    def start(self):
        """This method is provided        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        self.log_print("start: Creating log file")
        self.start_log("Logs", "NavLog.txt")

        self.all_waypoints = self.calculate_box()
        self.log_print('start: all_waypoints - ',self.all_waypoints)
        
        self.log_print("start: starting connection")
        self.connection.start()

        while self.in_mission:
            pass

        self.log_print("start: Closing log file")
        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=True, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
