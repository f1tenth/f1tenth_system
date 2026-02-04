import zenoh
import time
import pygame
import sys
from dataclasses import dataclass
from pycdr2 import IdlStruct #u would import int32 and stuff here

@dataclass
class Time(IdlStruct):
    sec: int
    nanosec: int

@dataclass
class Header(IdlStruct):
    stamp: Time
    frame_id: str

@dataclass
class Joy(IdlStruct):
    header: Header
    axes: list[float]     #originally the list was sequences, but that also
    buttons: list[int]    #gave us an error so I changed it

# --- 2. Setup Zenoh ---
conf = zenoh.Config()
conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')
session = zenoh.open(conf)
key_expr = 'rt/joy' 

def get_time_now():
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)

# --- 3. Main Loop ---
try:
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No wheel detected")
        sys.exit()
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected: {joystick.get_name()}")
    
    while True:
        pygame.event.pump()
        
        raw_steer = joystick.get_axis(0)
        raw_gas = joystick.get_axis(2)
        raw_brake = joystick.get_axis(3)

        # Normalize math
        steer = float(raw_steer)
        gas = (1.0 - raw_gas) / 2.0
        brake = (1.0 - raw_brake) / 2.0

        # Build message using the expected types
        joy_msg = Joy(
            header=Header(stamp=get_time_now(), frame_id="joy"),
            axes=[float(steer), float(gas), float(brake), float(0.0)],
            buttons=[int(0) for _ in range(10)]
        )

        # Serialize and Publish
        payload = joy_msg.serialize()
        session.put(key_expr, payload)

        print(f"Steer: {steer:.2f} | Gas: {gas:.2f} | Brake: {brake:.2f}    ", end="\r")
        time.sleep(0.05) 

except KeyboardInterrupt:
    print("\nStopping...")
    session.close()
    pygame.quit()