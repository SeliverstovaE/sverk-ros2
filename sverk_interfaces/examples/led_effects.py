"""
Example of working with an LED strip via drone.led: effects and color.

Requires the led_control node to be running (ros2 launch led_control led.launch.py)
and the led_interfaces package in the workspace.

Effects: fill, blink, blink_fast, fade, wipe, flash, rainbow, rainbow_fill.
"""

import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="example_led_effects")
try:
    if not drone.led:
        print("LED API unavailable: install led_interfaces and start led_control.")
        exit(1)

    print("Fill red (fill)...")
    drone.led.set_effect("fill", r=255, g=0, b=0)
    time.sleep(2.0)

    print("Fade to green (fade)...")
    drone.led.set_effect("fade", r=0, g=255, b=0)
    time.sleep(2.0)

    print("Blink blue (blink)...")
    drone.led.set_effect("blink", r=0, g=0, b=255)
    time.sleep(3.0)

    print("Rainbow (rainbow)...")
    drone.led.set_effect("rainbow", r=0, g=0, b=0)
    time.sleep(4.0)

    print("Fast blink orange (blink_fast)...")
    drone.led.set_effect("blink_fast", r=255, g=165, b=0)
    time.sleep(3.0)

    print("Return to fill white (fill)...")
    drone.led.set_effect("fill", r=255, g=255, b=255)
    time.sleep(1.0)

    print("Done.")
finally:
    drone.close()