#!/usr/bin/env python3

import logging
import struct
import threading
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPort

logging.basicConfig(level=logging.ERROR)


class AppchannelTest:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.appchannel.packet_received.add_callback(
            self._app_packet_received)

        self._cf.add_port_callback(CRTPPort.CONSOLE, self._console)

        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._test_appchannel).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _app_packet_received(self, data):
        (code, ) = struct.unpack_from("<B", data, 0)

        if code == 0:
            (cd, battery) = struct.unpack("<Bf", data)
            # print(f"Received : {cd}, Battery : {battery}")
        elif code == 1:
            (cd, timestamp) = struct.unpack("<Bq", data)
            # print(f"Received : {cd}, {timestamp}")
        elif code == 2:
            (cd, speed) = struct.unpack("<Bf", data)
            # print(f"Received : {cd}, {speed}")
        elif code == 3:
            (cd, positionX, positionY, positionZ) = struct.unpack("<Bfff", data)
            # print(f"Received : {cd}, {positionX}, {positionY}, {positionZ}")
        elif code == 4:
            (cd, front, left, back, right, up) = struct.unpack("<BHHHHH", data)
            # print(f"Received : {cd}, {front}, {left}, {back}, {right}, {up}")
        elif code == 5:
            (cd, state, ledOn) = struct.unpack("<BB?", data)
            # print(f"Received : {cd}, {state}, {ledOn}")
            print(f"state {state}")
        else:
            print("Received Unknown code")

    def _test_appchannel(self):
        print('Successfully connected to Crazyflie')
        threading.Thread(target=self._input_thread, args=(self._cf, )).start()

    def _input_thread(self, cf: Crazyflie):
        while True:
            codeToSend = int(input(">"))
            dataSend = struct.pack("<B", codeToSend)
            cf.appchannel.send_packet(dataSend)
            print(f"Sent code: {codeToSend}")

    def _console(self, packet):
        """
        Callback for data received from the copter.
        """
        # This might be done prettier ;-)
        console_text = packet.data.decode('UTF-8')

        print(f"{console_text}", end="")
        pass


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = [
        *cflib.crtp.scan_interfaces(0xE7E7E7E701),
        *cflib.crtp.scan_interfaces(0xE7E7E7E702)
    ]

    if len(available) > 0:
        le = AppchannelTest(available[0][0])
        print('Crazyflies found:')
        for i in available:
            print(i[0])
    else:
        print('No Crazyflies found, cannot run example')
