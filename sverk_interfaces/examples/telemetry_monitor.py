"""
Example: telemetry monitoring in a loop.

Periodically requests get_telemetry and prints drone state.
Stop with Ctrl+C.
"""

import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="example_telemetry_monitor")
try:
    while True:
        t = drone.controll.get_telemetry(frame_id="map")
        print(
            f"connected={t.connected} armed={t.armed} mode={t.mode} "
            f"pos=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) yaw={t.yaw:.2f} "
            f"vel=({t.vx:.2f}, {t.vy:.2f}, {t.vz:.2f}) "
            f"lat={t.lat:.6f} lon={t.lon:.6f} alt={t.alt:.2f}"
        )
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Stop by Ctrl+C")
finally:
    drone.close()