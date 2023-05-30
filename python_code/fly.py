#!/usr/bin/env python3
from __future__ import print_function
import getopt, sys
import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, \
    LocationGlobalRelative, Command
import logging

connection_string = '127.0.0.1:14551'
autopilot_logger = logging.getLogger('autopilot')
autopilot_logger.setLevel(logging.DEBUG)

dronekit_logger = logging.getLogger('dronekit')
dronekit_logger.setLevel(logging.DEBUG)


def distance(a: LocationGlobalRelative, b: LocationGlobalRelative):
    d_lat = a.lat - b.lat
    d_lon = a.lon - b.lon
    return math.sqrt(d_lat * d_lat + d_lon * d_lon) * 1.113195e5


class Controller:
    def __init__(self):
        self.batter_level = 100
        self.base_location = None
        self.traveled_distance = 0
        print(f'Connecting to {connection_string}...')
        self.vehicle = connect(connection_string, wait_ready=False)
        self.vehicle.wait_ready(still_waiting_interval=1,
                                still_waiting_callback=self.still_waiting_callback,
                                timeout=60)
        print('connected!')
        self.vehicle.parameters['FENCE_ENABLE'] = 0

    def still_waiting_callback(self, atts):
        print(
            f'Wait for {atts}: loading vehicle parameters {len(self.vehicle._params_map)}/{self.vehicle._params_count}')

    def disconnect(self):
        self.vehicle.close()

    def prepare(self):
        if not self.vehicle.mode == VehicleMode('STABILIZE'):
            print(
                'Vehicle is already in a mission. Please restart the ArduPilot!')
            exit(0)

    def arm_and_takeoff(self, alt):
        print('Pre-arm checks')
        while not self.vehicle.is_armable:
            print('Waiting for vehicle to initialise...')
            time.sleep(1)
        self.base_location = self.vehicle.location.global_relative_frame

        print('Arming motors')
        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print('Waiting for arming...')
            time.sleep(1)

        print('Taking off!')
        self.vehicle.simple_takeoff(alt)

        while True:
            print(
                f'Altitude: {self.vehicle.location.global_relative_frame.alt}m')
            if self.vehicle.location.global_relative_frame.alt >= alt * 0.95:
                print('Reached target altitude')
                break
            time.sleep(1)

    def move_to(self, north, east):
        print('Find current location...')
        pos = self.vehicle.location.global_relative_frame
        print(f'Current location: {pos.lat:.4f}, {pos.lon:.4f}')

        r_earth = 6378137.0
        d_lat = north / r_earth
        d_lon = east / (r_earth * math.cos(math.pi * pos.lat / 180))
        lat = pos.lat + d_lat * 180 / math.pi
        lon = pos.lon + d_lon * 180 / math.pi
        print(f'Target location: {lat:.4f}, {lon:.4f}')

        self.vehicle.simple_goto(LocationGlobalRelative(lat, lon, pos.alt))

    def callback_battery_status(self, name, param, msg):
        self.batter_level = msg.battery_remaining

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
            print(
                f'distance base {d_base:.2f}m, traveled {self.traveled_distance:.2f}m, battery {self.batter_level}%')

            time.sleep(1)


def main():
    controller = Controller()
    controller.prepare()
    controller.arm_and_takeoff(15)
    controller.move_to(10000, 0)
    # controller.monitor()
    controller.disconnect()


if __name__ == '__main__':
    main()
