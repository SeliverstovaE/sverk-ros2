#!/usr/bin/env python3
"""
Exercise all offboard services: takeoff, sequential calls with conservative
parameters, then land.

Run (from workspace after sourcing install/setup.bash):
  python3 offboard_control/examples/test_all_services.py
  # or
  ros2 run offboard_control test_all_services.py  # if installed as entry point

Requires: running offboard_control node and (for SITL) simulation.
"""
import rclpy
from rclpy.node import Node
import time
import threading
import math
import sys

from offboard_interfaces.srv import (
    Navigate,
    GetTelemetry,
    SetAltitude,
    SetYaw,
    SetYawRate,
    SetPosition,
    SetVelocity,
    SetAttitude,
    SetRates,
)
from std_srvs.srv import Trigger

# Conservative test timing / limits
TAKEOFF_HEIGHT = 3.0
NAVIGATE_SPEED = 0.4
WAIT_TAKEOFF = 50
WAIT_NAVIGATE = 8
WAIT_STEP = 10
WAIT_ATTITUDE_RATES = 2
SERVICE_TIMEOUT = 10.0


class ServiceTester(Node):
    def __init__(self):
        super().__init__("service_tester")
        self._lock = threading.Lock()

        self._svc = {}
        services = [
            ("navigate", Navigate, "/navigate"),
            ("land", Trigger, "/land"),
            ("get_telemetry", GetTelemetry, "/get_telemetry"),
            ("set_altitude", SetAltitude, "/set_altitude"),
            ("set_yaw", SetYaw, "/set_yaw"),
            ("set_yaw_rate", SetYawRate, "/set_yaw_rate"),
            ("set_position", SetPosition, "/set_position"),
            ("set_velocity", SetVelocity, "/set_velocity"),
            ("set_attitude", SetAttitude, "/set_attitude"),
            ("set_rates", SetRates, "/set_rates"),
        ]
        for name, srv_type, topic in services:
            self._svc[name] = self.create_client(srv_type, topic)

        self.wait_for_all_services()
        self.get_logger().info("All services ready")

    def wait_for_all_services(self):
        for name, client in self._svc.items():
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit(1)
                self.get_logger().info(f"Waiting for {name}...")

    def call(self, name, request):
        with self._lock:
            client = self._svc[name]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT)
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f"{name} failed: {e}")
                    return None
            return None

    def get_telemetry(self):
        req = GetTelemetry.Request()
        resp = self.call("get_telemetry", req)
        if resp and getattr(resp, "connected", False):
            return resp
        return None

    def navigate(self, x, y, z, yaw=0.0, speed=NAVIGATE_SPEED, auto_arm=False, frame_id="body"):
        req = Navigate.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.speed = float(speed)
        req.auto_arm = bool(auto_arm)
        req.frame_id = str(frame_id)
        resp = self.call("navigate", req)
        return resp.success if resp else False

    def land(self):
        req = Trigger.Request()
        resp = self.call("land", req)
        return resp.success if resp else False

    def set_altitude(self, z, frame_id="map"):
        req = SetAltitude.Request()
        req.z = float(z)
        req.frame_id = str(frame_id)
        resp = self.call("set_altitude", req)
        return resp.success if resp else False

    def set_yaw(self, yaw, frame_id="body"):
        req = SetYaw.Request()
        req.yaw = float(yaw)
        req.frame_id = str(frame_id)
        resp = self.call("set_yaw", req)
        return resp.success if resp else False

    def set_yaw_nan(self):
        req = SetYaw.Request()
        req.yaw = float("nan")
        req.frame_id = "body"
        resp = self.call("set_yaw", req)
        return resp.success if resp else False

    def set_yaw_rate(self, yaw_rate):
        req = SetYawRate.Request()
        req.yaw_rate = float(yaw_rate)
        resp = self.call("set_yaw_rate", req)
        return resp.success if resp else False

    def set_position(self, x=0.0, y=0.0, z=0.0, yaw=0.0, frame_id="body", auto_arm=False):
        req = SetPosition.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.frame_id = str(frame_id)
        req.auto_arm = bool(auto_arm)
        resp = self.call("set_position", req)
        return resp.success if resp else False

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw=0.0, frame_id="body", auto_arm=False):
        req = SetVelocity.Request()
        req.vx = float(vx)
        req.vy = float(vy)
        req.vz = float(vz)
        req.yaw = float(yaw)
        req.frame_id = str(frame_id)
        req.auto_arm = bool(auto_arm)
        resp = self.call("set_velocity", req)
        return resp.success if resp else False

    def set_attitude(self, roll=0.0, pitch=0.0, yaw=0.0, thrust=0.5, frame_id="map", auto_arm=False):
        req = SetAttitude.Request()
        req.roll = float(roll)
        req.pitch = float(pitch)
        req.yaw = float(yaw)
        req.thrust = float(thrust)
        req.frame_id = str(frame_id)
        req.auto_arm = bool(auto_arm)
        resp = self.call("set_attitude", req)
        return resp.success if resp else False

    def set_rates(self, roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0, thrust=0.5, auto_arm=False):
        req = SetRates.Request()
        req.roll_rate = float(roll_rate)
        req.pitch_rate = float(pitch_rate)
        req.yaw_rate = float(yaw_rate)
        req.thrust = float(thrust)
        req.auto_arm = bool(auto_arm)
        resp = self.call("set_rates", req)
        return resp.success if resp else False


def log_step(node, msg):
    node.get_logger().info(msg)
    print(msg)


def main():
    rclpy.init()
    node = ServiceTester()

    try:
        print("\n========== Test all offboard services ==========\n")

        # 1. Takeoff
        log_step(node, f">>> 1. Takeoff to {TAKEOFF_HEIGHT} m (navigate, body)")
        if not node.navigate(0.0, 0.0, TAKEOFF_HEIGHT, yaw=0.0, speed=1.0, auto_arm=True, frame_id="body"):
            log_step(node, "ERROR: Takeoff failed")
            return
        log_step(node, f"Waiting {WAIT_TAKEOFF}s for takeoff...")
        time.sleep(WAIT_TAKEOFF)

        # 2. set_altitude (target height in navigate/position-style modes)
        log_step(node, ">>> 2. set_altitude(z=2.0, frame_id='terrain')")
        if node.set_altitude(2.0, "terrain"):
            log_step(node, "set_altitude OK, waiting...")
            time.sleep(WAIT_STEP)
        else:
            log_step(node, "set_altitude call failed (may be OK if no terrain frame)")

        log_step(node, ">>> 2b. set_altitude(z=1.5, frame_id='map')")
        if node.set_altitude(1.5, "map"):
            time.sleep(WAIT_STEP)

        # 3. set_yaw (hold heading; previous motion may continue)
        log_step(node, ">>> 3. set_yaw(yaw=0.3 rad, frame_id='body')")
        if node.set_yaw(0.3, "body"):
            time.sleep(WAIT_STEP)
        log_step(node, ">>> 3b. set_yaw(yaw=0, frame_id='body')")
        node.set_yaw(0.0, "body")
        time.sleep(WAIT_STEP)

        # 4. set_yaw_rate
        log_step(node, ">>> 4. set_yaw_rate(yaw_rate=0.3 rad/s)")
        if node.set_yaw_rate(0.3):
            time.sleep(3.0)
        log_step(node, ">>> 4b. set_yaw(yaw=nan) — stop yaw rate")
        node.set_yaw_nan()
        time.sleep(WAIT_STEP)

        # 5. set_position — hover and small offset
        log_step(node, ">>> 5. set_position(x=0,y=0,z=0, frame_id='body') — hover")
        if node.set_position(0.0, 0.0, 0.0, 0.0, "body"):
            time.sleep(WAIT_STEP)
        log_step(node, ">>> 5b. set_position(x=0.3,y=0,z=0, frame_id='body')")
        if node.set_position(0.3, 0.0, 0.0, 0.0, "body"):
            time.sleep(WAIT_NAVIGATE)
        log_step(node, ">>> 5c. set_position(x=0,y=0,z=0, frame_id='body') — return")
        node.set_position(0.0, 0.0, 0.0, 0.0, "body")
        time.sleep(WAIT_NAVIGATE)

        # 6. set_velocity — short forward then stop
        log_step(node, ">>> 6. set_velocity(vx=0.2, frame_id='body') 2 s")
        if node.set_velocity(0.2, 0.0, 0.0, 0.0, "body"):
            time.sleep(2.0)
        log_step(node, ">>> 6b. set_position(body) — stop")
        node.set_position(0.0, 0.0, 0.0, 0.0, "body")
        time.sleep(WAIT_STEP)

        # 7. set_attitude — brief, small roll/pitch
        log_step(node, ">>> 7. set_attitude(roll=0.05, pitch=0, yaw=0, thrust=0.55) short")
        if node.set_attitude(0.05, 0.0, 0.0, 0.55, "map"):
            time.sleep(WAIT_ATTITUDE_RATES)
        log_step(node, ">>> 7b. Back to set_position hover")
        node.set_position(0.0, 0.0, 0.0, 0.0, "body")
        time.sleep(WAIT_STEP)

        # 8. set_rates — short yaw_rate pulse
        log_step(node, ">>> 8. set_rates(yaw_rate=0.2, thrust=0.5) short")
        if node.set_rates(0.0, 0.0, 0.2, 0.5):
            time.sleep(WAIT_ATTITUDE_RATES)
        log_step(node, ">>> 8b. Back to set_position hover")
        node.set_position(0.0, 0.0, 0.0, 0.0, "body")
        time.sleep(WAIT_STEP)

        # 9. Land
        log_step(node, ">>> 9. Land")
        if node.land():
            log_step(node, "Land command sent, waiting...")
            time.sleep(10)
        else:
            log_step(node, "Land call failed")

        print("\n========== All service tests finished ==========\n")

    except KeyboardInterrupt:
        print("\nInterrupted. Landing...")
        node.land()
        time.sleep(5)
    except Exception as e:
        print(f"Error: {e}")
        try:
            node.land()
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
