
# import zenoh
# import time
# import pygame
# from dataclasses import dataclass
# # 1. FIXED: We need these specific types for ROS 2 serialization
# from pycdr2 import IdlStruct, Sequence

# # --- 2. Define the ROS 2 Message Structures ---
# @dataclass
# class Time(IdlStruct):
#     sec: int
#     nanosec: int

# @dataclass
# class Header(IdlStruct):
#     stamp: Time
#     frame_id: str

# @dataclass
# class Joy(IdlStruct):
#     header: Header
#     axes: Sequence[float]    # FIXED: Must be sequence[float32]
#     buttons: Sequence[int]   # FIXED: Must be sequence[int32] (Integers!)

# # --- 3. Setup Zenoh (FIXED CONNECTION) ---
# conf = zenoh.Config()

# # FIXED: This line opens the "door" so the Jetson can connect
# conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')

# print("Opening Zenoh Session (Listening for connections)...")
# session = zenoh.open(conf)

# key_expr = 'rt/joy' 

# # --- 4. Main Loop ---
# def get_time_now():
#     now = time.time()
#     sec = int(now)
#     nanosec = int((now - sec) * 1_000_000_000)
#     return Time(sec=sec, nanosec=nanosec)

# try:
#     pygame.init()
#     pygame.joystick.init()
    
#     # Check if joystick is plugged in before crashing
#     if pygame.joystick.get_count() == 0:
#         print("No joystick detected! Plug it in.")
#         exit(1)
        
#     joystick = pygame.joystick.Joystick(0)
#     joystick.init()
#     print(f"Detected joystick: {joystick.get_name()}")
    
#     while True:
#         pygame.event.pump()
        
#         # Read Axes
#         steer = float(joystick.get_axis(0))
#         # Normalize Gas/Brake from [-1, 1] to [0, 1] if needed, or keep raw
#         # Note: Your math: (1.0 - axis) / 2.0 converts 1.0(pressed) -> 0.0, -1.0(released) -> 1.0?
#         # Verify this math matches what your robot expects.
#         gas_raw = joystick.get_axis(2)
#         brake_raw = joystick.get_axis(3)
        
#         gas = (1.0 - gas_raw) / 2.0
#         brake = (1.0 - brake_raw) / 2.0

#         # Construct the Axes list (Order matters!)
#         # Ensure 'steer', 'gas', etc are wrapped in float32() implies precision
#         current_axes = [
#             float(steer), 
#             float(gas), 
#             float(brake), 
#             float(0.0)
#         ]

#         # Construct Buttons (Must be integers)
#         # We use a list comprehension to force them to int32
#         # Example: Just sending 10 zeros for now
#         current_buttons = [int(0) for _ in range(10)]

#         joy_msg = Joy(
#             header=Header(stamp=get_time_now(), frame_id="joy"),
#             axes=current_axes,
#             buttons=current_buttons
#         )

#         # Serialize
#         payload = joy_msg.serialize()
#         session.put(key_expr, payload)

#         print(f"Steer: {steer:.2f} | Gas: {gas:.2f} | Brake: {brake:.2f}", end="\r")
#         time.sleep(0.05) # 20Hz

# except KeyboardInterrupt:
#     print("\nClosing session...")
#     session.close()

