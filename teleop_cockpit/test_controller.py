import pygame

try:
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    joystick_name = joystick.get_name()
    joystick_num_axes = joystick.get_numaxes()
    print(f"Detected joystick: {joystick_name} with {joystick_num_axes} axes")
    
    while True:
        pygame.event.pump()
        jsInputs = [float(joystick.get_axis(i)) for i in range(joystick_num_axes)] 
        # For XBOX controller:
            # Left Stick X-axis: Axis 0 (steering)
            # Left Stick Y-axis: Axis 1 (throttle/brake) (Note: Up is -1, Down is 1)
            # Right Stick X-axis: Axis 2
            # Right Stick Y-axis: Axis 3 (Note: Up is -1, Down is 1)
            # Left Trigger: Axis 4 (Begins at -1, ends at 1 when fully pressed)
            # Right Trigger: Axis 5 (Begins at -1, ends at 1 when fully pressed)
        # For steering wheel (Logitech G923):
            # Steering: Axis 0
            # Clutch: Axis 1
            # Gas: Axis 2
            # Brake: Axis 3
        
        output = " | ".join([f"Axis {i}: {axis:.2f}" for i, axis in enumerate(jsInputs)])
        
        print(output, end="\r")

except KeyboardInterrupt:
    pygame.quit()