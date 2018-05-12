import argparse
import time
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

    def __init__(self, connection, side_length, altitude):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.side_length = side_length
        self.altitude = altitude

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def next_waypoint(self):
        try:
            return self.all_waypoints.pop()
        except IndexError:
            return None


    def acceptable_waypoint_range(self, target_y, target_x):
        '''
            for a given waypoint, this returns waypoint with allowed error (10 % or the square's length)
            e.g
            ---
            input: target_y = 0, target_x = 0    output: target_y = [-0.3, 0.3], target_x = [-0.3, 0.3]
            input: target_y = 3, target_x = 0    output: target_y = [2.7, 3.3], target_x = [-0.3, 0.3]
            input: target_y = -3, target_x = 0    output: target_y = [-3.3, -2.7], target_x = [-0.3, 0.3]
        '''

        acceptable_error = 0.1 * self.side_length,
        y_range = [target_y + acceptable_error, target_y - acceptable_error]
        y_range.sort()
        x_range = [target_x + acceptable_error, target_x - acceptable_error]
        x_range.sort()

        return y_range, x_range


    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        This handles transition from TAKEOFF -> WAYPOINTS
        """
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]

            if altitude > (0.95 * self.target_position[2]):
                self.target_position = self.next_waypoint()
                self.waypoint_transition()

        if self.flight_state == States.WAYPOINT:
            # these values can be stored in object to not calculate them everytime during callback
            y_range, x_range = self.acceptable_waypoint_range(self.target_position[0], self.target_position[1])
            
            print('current position = ', self.local_position, 'target_y_range = ', y_range, 'target_x_range', x_range)
            if y_range[0] <= self.local_position[0] <= y_range[1] and x_range[0] <= self.local_position[1] <= x_range[1]:
                self.target_position = self.next_waypoint()
                self.waypoint_transition()


    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data.
        This callback triggers disarm transition at appropriate z axis position
        """
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        Triggers initial state transition. MANUAL -> ARMING -> TAKEOFF and DISARMING -> MANUAL
        """
        if self.flight_state == States.MANUAL and self.in_mission:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

        # TAKEOFF -> WAYPOINTS -> LANDING transition will be taken care by local position update callback
        # LANDING -> DISARM Will be taken by velocity callback as vehicle should disarm only when the vehicle is on ground


    def calculate_box(self):
        """
        1. Return waypoints to fly a box
        """
        self.all_waypoints.append(np.array([self.side_length, 0.0, self.altitude, 0.0]))
        self.all_waypoints.append(np.array([self.side_length, self.side_length, self.altitude, 0.0]))
        self.all_waypoints.append(np.array([0.0, self.side_length, self.altitude, 0.0]))
        self.all_waypoints.append(np.array([0.0, 0.0, self.altitude, 0.0]))

        self.all_waypoints.reverse()
        

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")

        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """   
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position = np.array([0.0, 0.0, self.altitude])
        self.takeoff(self.altitude)

        self.calculate_box()
        print('all waypoints = ', self.all_waypoints)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition. next = ", self.target_position)

        if self.target_position is None:
            print("no next waypoint. landing")
            self.landing_transition()
        else:
            print("travelling to: ", self.target_position)
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
            self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING
        
    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--len', type=int, default=3.0, help="Length of square's side")
    parser.add_argument('--altitude', type=int, default=3.0, help="Mission altitude")

    args = parser.parse_args()

    print(args)
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = BackyardFlyer(conn, args.len, args.altitude)
    time.sleep(2)
    drone.start()
