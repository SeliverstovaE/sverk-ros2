"""
Examples of working with fmu_calibration_control via drone.fcu:
- normal disarm;
- force disarm;
- Kill Switch;
- sensor calibrations.

WARNING: force_disarm and kill_switch immediately disarm / stop motors.
Use only in safe conditions.
"""

import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="example_safety_and_calibration")
try:
    print("Normal disarm (if drone is on ground)...")
    resp = drone.fcu.disarm()
    print("disarm:", resp.success, resp.message)

    time.sleep(2.0)

    print("Force disarm (force_disarm)...")
    resp = drone.fcu.force_disarm()
    print("force_disarm:", resp.success, resp.message)

    # Calibration examples (must be pre-arm, drone not in flight).
    print("Gyroscope calibration...")
    drone.fcu.calibrate_gyro()
    time.sleep(1.0)

    print("Accelerometer calibration...")
    drone.fcu.calibrate_accel()
    time.sleep(1.0)

    print("Level horizon calibration...")
    drone.fcu.calibrate_level()
    time.sleep(1.0)

    print("If necessary, you can call Kill Switch:")
    print("drone.fcu.kill_switch()  # CAUTION: immediate motor stop")
finally:
    drone.close()