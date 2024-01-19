#!/usr/bin/env python3
from __future__ import print_function
import getopt, sys
import time
import math

if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    import collections.abc
    setattr(collections, "MutableMapping", collections.abc.MutableMapping)

from dronekit import connect, VehicleMode, LocationGlobal, \
    LocationGlobalRelative, Command
import logging
import json
from datetime import datetime
from argparse import ArgumentParser

connection_string = '127.0.0.1:14551'
autopilot_logger = logging.getLogger('autopilot')
autopilot_logger.setLevel(logging.DEBUG)

dronekit_logger = logging.getLogger('dronekit')
dronekit_logger.setLevel(logging.DEBUG)

fly_logger = logging.getLogger('fly')
fly_logger.setLevel(logging.DEBUG)


def distance(a: LocationGlobalRelative, b: LocationGlobalRelative):
    d_lat = a.lat - b.lat
    d_lon = a.lon - b.lon
    return math.sqrt(d_lat * d_lat + d_lon * d_lon) * 1.113195e5


class Controller:
    def __init__(self, conn=connection_string):
        self.batter_level = 100
        self.base_location = None
        self.traveled_distance = 0
        fly_logger.info(f'Connecting to {conn}...')
        self.vehicle = connect(conn, wait_ready=False)
        self.vehicle.wait_ready(still_waiting_interval=1,
                                still_waiting_callback=self.still_waiting_callback,
                                timeout=60)
        fly_logger.info('connected!')
        self.vehicle.parameters['FENCE_ENABLE'] = 0

    def still_waiting_callback(self, atts):
        fly_logger.debug(
            f'Wait for {atts}: loading vehicle parameters '
            f'{len(self.vehicle._params_map)}/{self.vehicle._params_count}')

    def disconnect(self):
        self.vehicle.close()

    def prepare(self):
        if self.vehicle.mode.name != 'STABILIZE':
            fly_logger.critical(
                'Copter is in already in a mission. Please restart the ArduPilot!')
            exit(0)

    def arm_and_takeoff(self, alt):
        fly_logger.info('Pre-arm checks')
        while not self.vehicle.is_armable:
            fly_logger.info('Waiting for vehicle to initialise...')
            time.sleep(1)
        self.base_location = self.vehicle.location.global_relative_frame

        fly_logger.info('Arming motors')
        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True

        while not self.vehicle.armed:
            fly_logger.info('Waiting for arming...')
            time.sleep(1)

        fly_logger.info('Taking off!')
        self.vehicle.simple_takeoff(alt)

        while True:
            fly_logger.info(
                f'Altitude: {self.vehicle.location.global_relative_frame.alt}m')
            if self.vehicle.location.global_relative_frame.alt >= alt * 0.95:
                fly_logger.info('Reached target altitude')
                break
            time.sleep(1)

    def move_to(self, north, east):
        fly_logger.info('Find current location...')
        pos = self.vehicle.location.global_relative_frame
        fly_logger.info(f'Current location: {pos.lat:.5f}, {pos.lon:.5f}')

        r_earth = 6378137.0
        d_lat = north / r_earth
        d_lon = east / (r_earth * math.cos(math.pi * pos.lat / 180))
        lat = pos.lat + d_lat * 180 / math.pi
        lon = pos.lon + d_lon * 180 / math.pi
        print(f'Target location: {lat:.4f}, {lon:.4f}')

        self.vehicle.simple_goto(LocationGlobalRelative(lat, lon, pos.alt))

    def callback_battery_status(self, name, param, msg):
        self.batter_level = msg.battery_remaining

    def crash_check(self) -> bool:
        dist_base = distance(self.vehicle.location.global_relative_frame,
                             self.base_location)
        if self.vehicle.battery.level == 0 and dist_base > 1:
            return True
        return False

    def return_to_base_check(self):
        dist_base = distance(self.vehicle.location.global_relative_frame,
                             self.base_location)
        if self.vehicle.mode.name == 'RTL' and dist_base < 1 and \
           self.vehicle.location.global_relative_frame.alt < 1:
            return True
        return False

    def monitor(self):
        self.vehicle.add_message_listener('BATTERY_STATUS',
                                          self.callback_battery_status)
        prev_pos = self.vehicle.location.global_relative_frame
        while True:
            pos = self.vehicle.location.global_relative_frame
            d_delta = distance(pos, prev_pos)
            prev_pos = pos
            self.traveled_distance += d_delta
            d_base = distance(pos, self.base_location)
            fly_logger.info(
                f'distance to base {d_base:.2f}m, '
                f'traveled {self.traveled_distance:.2f}m, '
                f'battery {self.batter_level}%')
            if self.crash_check():
                fly_logger.critical(
                    f'Copter CRASHED due to the battery depletion!')
                break
            if self.return_to_base_check():
                fly_logger.info(f'Copter returned to the base!')
                break
            time.sleep(1)

    def save(self, case_name):
        date = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        with open(f'fly-{case_name}-{date}.json', 'w') as f:
            json.dump({
                'Test for': case_name,
                'Remaining battery': self.batter_level,
                'Traveled distance': self.traveled_distance,
                'Crashed': self.crash_check(),
                'Remaining battery unit': '%',
                'Traveled distance unit': 'm'
            }, f, indent=2)


def main():
    parser = ArgumentParser()
    parser.add_argument('--connection', default=connection_string, help='connection string')
    parser.add_argument('--alt', default=15, help='target altitude')
    parser.add_argument('--north', default=10000, help='target north distance')
    parser.add_argument('--east', default=0, help='target east distance')
    parser.add_argument('--case', default='', help='the current test case name')
    args = parser.parse_args()

    controller = Controller(args.connection)
    controller.prepare()
    controller.arm_and_takeoff(args.alt)
    controller.move_to(args.north, args.east)
    controller.monitor()
    controller.save(args.case)
    controller.disconnect()


if __name__ == '__main__':
    main()
