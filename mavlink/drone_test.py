#!/usr/bin/env python3

"""
This example shows how to use the manual controls plugin.

Note: Manual inputs are taken from a test set in this example to decrease
complexity. Manual inputs can be received from devices such as a joystick
using third-party python extensions.

Note: Taking off the drone is not necessary before enabling manual inputs.
It is acceptable to send positive throttle input to leave the ground.
Takeoff is used in this example to decrease complexity
"""

import asyncio
import random
from mavsdk import System

counter = 0

#TIhi fnis

# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
manual_inputs = [
    [0, 0, 0.5, 0],  # no movement
    [-1, 0, 0.5, 0],  # minimum roll
    [1, 0, 0.5, 0],  # maximum roll
    [0, -1, 0.5, 0],  # minimum pitch
    [0, 1, 0.5, 0],  # maximum pitch
    [0, 0, 0.5, -1],  # minimum yaw
    [0, 0, 0.5, 1],  # maximum yaw
    [0, 0, 1, 0],  # max throttle
    [0, 0, 0, 0],  # minimum throttle
]


async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    counter = 0

    # This waits till a mavlink based drone is connected
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

 

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()


    await asyncio.sleep(5)

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_altitude_control()

    while counter < 100:
        # grabs a random input from the test list
        # WARNING - your simulation vehicle may crash if its unlucky enough
        input_index = random.randint(0, len(manual_inputs) - 1)
        input_list = manual_inputs[input_index]

        # get current state of roll axis (between -1 and 1)
        roll = float(input_list[0])
        # get current state of pitch axis (between -1 and 1)
        pitch = float(input_list[1])
        # get current state of throttle axis
        # (between -1 and 1, but between 0 and 1 is expected)
        throttle = float(input_list[2])
        # get current state of yaw axis (between -1 and 1)
        yaw = float(input_list[3])

        await drone.manual_control.set_manual_control_input(
            pitch, roll, throttle, yaw)
            
        counter = counter + 1

        await asyncio.sleep(0.1)
    asyncio.cancel()

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(manual_controls())
