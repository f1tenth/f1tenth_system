import pygame
import sys

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No wheel found.")
    sys.exit()

joy = pygame.joystick.Joystick(0)
joy.init()

print(f"Testing {joy.get_name()}... Press Ctrl+C to stop.")

try:
    while True:
        for event in pygame.event.get(): # Properly clear the event queue
            if event.type == pygame.QUIT:
                sys.exit()

        steer = round(joy.get_axis(0), 2)
        #gas, break outputs normalized (originally [1, -1])
        gas = round((1.0 - joy.get_axis(2)) / 2.0, 2)
        brake = round((1.0 - joy.get_axis(3)) / 2.0, 2)
        
        print(f"Steer: {steer} | Gas: {gas} | Brake: {brake}    ", end="\r")
        pygame.time.wait(50) 
        
except Exception as e:
    print(f"\nCaught an error: {e}")
finally:
    pygame.quit()