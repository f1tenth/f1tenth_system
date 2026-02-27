import zenoh
import time
import pygame
import sys
import struct 

# --- 1. Helper Function for Manual Serialization ---
# Replaces pycdr2 classes to guarantee perfect 4-byte alignment for ROS 2.
def create_manual_joy_message(steer, gas):
    # 1. CDR Header (Little Endian)
    msg = b'\x00\x01\x00\x00'
    
    # 2. Timestamp (8 bytes total)
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1_000_000_000)
    msg += struct.pack('<I', sec)
    msg += struct.pack('<I', nanosec)

    # 3. Frame ID: "dev" (To ensure 4-byte alignment)
    msg += struct.pack('<I', 4)  # Length: 4
    msg += b'dev\x00'            # Data: d, e, v, null (4 bytes)
    
    # 4. Axes (Sequence of float32)
    # originally the list was sequences, but that also gave us an error so I changed it
    msg += struct.pack('<I', 4) 
    # The Floats (0.0, gas, steer, 0.0)
    msg += struct.pack('<f', 0.0)
    msg += struct.pack('<f', float(gas))
    msg += struct.pack('<f', float(steer))
    msg += struct.pack('<f', 0.0)

    # 5. Buttons (Sequence of int32)
    msg += struct.pack('<I', 10)
    for _ in range(10):
        msg += struct.pack('<i', 0)

    return msg

# --- 2. Setup Zenoh ---
conf = zenoh.Config()
conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')
session = zenoh.open(conf)
print("Zenoh session established")

# We broadcast to ALL common topic patterns to ensure the Bridge catches one
keys_to_try = [
    'rt/joy',       
    'joy',          
    'rt/vesc/joy'   
] 

def get_time_now():
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1_000_000_000)
    return sec, nanosec

# --- 3. Main Loop ----
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
        
        # # For steering wheel
        raw_steer = joystick.get_axis(0)
        raw_gas = joystick.get_axis(2)
        raw_reverse = joystick.get_axis(3)

        # # For XBOX controller (uncomment if needed)
        # raw_steer = joystick.get_axis(0)
        # raw_gas = joystick.get_axis(1)
        # raw_steer = -raw_steer if abs(raw_steer) > 0.03 else 0.0  # Deadzone for steering
        # raw_gas = -raw_gas/4 if abs(raw_gas) > 0.03 else 0.0  # Invert because up is -1, and apply deadzone


        # Normalize math for steering wheel
        steer = float(-raw_steer)
        gas = (1.0 - raw_gas) / 8.0
        reverse = (1.0 - raw_reverse) / 8.0
        if gas < 0.02 and reverse > 0.02:
            gas = -reverse  # Use negative gas to indicate reverse


        # Normalize math for controller
        steer = float(raw_steer)
        gas = float(raw_gas)

        # Build message using the manual byte packer
        payload = create_manual_joy_message(steer, gas)

        # Serialize and Publish
        for key in keys_to_try:
            # We explicitly mark this as 'application/cdr' so the bridge accepts it
            session.put(key, payload, encoding=zenoh.Encoding("application/cdr"))

        # print(f"Steer: {steer:.2f} | Gas: {gas:.2f} | Brake: {brake:.2f}    ", end="\r")
        print(f"Steer: {steer:.2f} | Gas: {gas:.2f} | Payload: {len(payload)} bytes", end="\r", flush=True)

        time.sleep(0.05) 

except KeyboardInterrupt:
    print("\nStopping...")
    session.close()
    pygame.quit()
