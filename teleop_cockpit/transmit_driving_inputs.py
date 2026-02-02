import zenoh
import time
from dataclasses import dataclass
from pycdr2 import IdlStruct, float32, int32, uint32, sequence
import pygame

# --- 1. Define the ROS 2 Message Structures ---
# These classes tell Python how to pack data into the binary format (CDR)
# that the Jetson's ROS node expects.

@dataclass
class Time(IdlStruct):
    sec: int32
    nanosec: uint32

@dataclass
class Header(IdlStruct):
    stamp: Time
    frame_id: str

@dataclass
class Joy(IdlStruct):
    header: Header
    axes: sequence[float32]
    buttons: sequence[int32]

# --- 2. Setup Zenoh ---
session = zenoh.open()
# The topic key. 'rt' stands for 'ros topic'.
# Ensure this matches the topic your F1Tenth stack listens to.
key_expr = 'rt/joy' 

print(f"Publishing to: {key_expr}")

# --- 3. Main Loop ---
def get_time_now():
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)

try:
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    joystick_name = joystick.get_name()
    joystick_num_axes = joystick.get_numaxes()
    print(f"Detected joystick: {joystick_name} with {joystick_num_axes} axes")
    
    while True:
        # Replace this with your actual wheel reading function
        # TODO hannah work from here to figure out which wheel inputs are which (steering vs throttle vs brake etc)
        pygame.event.pump()
        jsInputs = [float(joystick.get_axis(i)) for i in range(joystick_num_axes)] 
        # Hannah also don't forget to scale/normalize these inputs as needed since joystick axes typically range from -1 to 1
        steering_normalized = jsInputs[2]  # TODO Example axis index for steering in reality you need to normalize it
        throttle_normalized = jsInputs[1]  # TODO Example axis index for throttle in reality you need to normalize it

        # Create the Stamped message with Header
        joy_msg = Joy(
            header=Header(frame_id="joy"),
            axes=[0.0, throttle_normalized, steering_normalized, 0.0],  # axes[1]=throttle, axes[2]=steering
            #buttons=[0, 0, 0, 0, 1, 0, 0]  # button[4]=deadman (if needed)
        )

        # Serialize to binary (CDR) and send
        # This creates the byte array that ROS 2 understands
        payload = joy_msg.serialize()
        
        session.put(key_expr, payload)
        
        time.sleep(0.05) # 20Hz publish rate

except KeyboardInterrupt:
    session.close()