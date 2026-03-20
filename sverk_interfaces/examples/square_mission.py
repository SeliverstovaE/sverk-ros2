"""
Example: flying a square in the map plane and returning to the start point.

Uses navigate_wait via drone.controll.navigate_wait().
"""

import time
import sverk_interfaces

side = 1.0  # square side length, m
z = 1.0     # flight altitude, m

drone = sverk_interfaces.init(Nodename="example_square_mission")
try:
    # Take current position as the square's zero point (map).
    start = drone.controll.get_telemetry(frame_id="map")
    x0, y0 = start.x, start.y

    # Takeoff / climb above the start point.
    drone.controll.navigate_wait(
        x=x0,
        y=y0,
        z=z,
        yaw=start.yaw,
        speed=0.5,
        frame_id="map",
        auto_arm=True,
        tolerance=0.25,
        timeout=60.0,
    )

    # Square path: (0,0) -> (L,0) -> (L,L) -> (0,L) -> (0,0)
    waypoints = [
        (x0 + side, y0),
        (x0 + side, y0 + side),
        (x0, y0 + side),
        (x0, y0),
    ]

    for i, (x, y) in enumerate(waypoints, start=1):
        print(f"Side {i}: target point ({x:.2f}, {y:.2f}, {z:.2f})")
        drone.controll.navigate_wait(
            x=x,
            y=y,
            z=z,
            yaw=start.yaw,
            speed=0.5,
            frame_id="map",
            auto_arm=False,
            tolerance=0.25,
            timeout=60.0,
        )
        time.sleep(1.0)

    # Land after completing the square.
    drone.controll.land(timeout=10.0)
finally:
    drone.close()