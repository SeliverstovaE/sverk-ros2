#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, math, time, subprocess, traceback, shutil, importlib, threading
import os
from threading import Event, Thread, Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped

from px4_msgs.msg import (
    VehicleStatus, SensorCombined, BatteryStatus, TimesyncStatus,
    VehicleOdometry, VehicleLocalPosition, VehicleAttitude, EstimatorStatusFlags,
    FailsafeFlags, DistanceSensor, TelemetryStatus, VehicleControlMode,
)

if sys.stderr.isatty():
    RED, END = "\033[31m", "\033[0m"
else:
    RED = END = ""

thread_local = threading.local()
reports_lock = Lock()


def ff(value, precision=2):
    if value is None:
        return RED + "???" + END
    if isinstance(value, float):
        return ("{:.%df}" % precision).format(value)
    if isinstance(value, int):
        return str(value)
    return str(value)


def warn(text, *args): thread_local.reports.append(("warn", text % args))
def error(text, *args): thread_local.reports.append(("error", text % args))
def info(text, *args): thread_local.reports.append(("info", text % args))


def check(name):
    def inner(fn):
        def wrapper(*args, **kwargs):
            thread_local.reports = []
            node = args[0]
            logger = node.get_logger()
            try:
                fn(*args, **kwargs)
            except Exception:
                tb = traceback.format_exc()
                logger.error(f"[{name}] {RED}ERROR: exception occurred\n{tb}{END}")
                thread_local.reports.append(("error", "exception occurred (see traceback above)"))
            with reports_lock:
                for kind, msg in thread_local.reports:
                    if kind == "info":
                        logger.info(f"[{name}] {msg}")
                    elif kind == "warn":
                        logger.warn(f"[{name}] {RED}WARNING: {msg}{END}")
                    else:
                        logger.error(f"[{name}] {RED}ERROR: {msg}{END}")
                if not thread_local.reports:
                    logger.warn(f"[{name}] {RED}WARNING: check produced no log output{END}")
        return wrapper
    return inner


def import_pytype(pytype_str: str):
    mod_name, cls_name = pytype_str.rsplit(".", 1)
    return getattr(importlib.import_module(mod_name), cls_name)


def quat_to_euler_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def yaw_diff_deg(yaw_a_rad, yaw_b_rad):
    d = math.degrees(yaw_a_rad - yaw_b_rad)
    return (d + 180.0) % 360.0 - 180.0


def px4_nav_state_name(nav_state: int) -> str:
    names = {
        0: "MANUAL",
        1: "ALTCTL",
        2: "POSCTL",
        3: "AUTO_MISSION",
        4: "AUTO_LOITER",
        5: "AUTO_RTL",
        6: "AUTO_LAND",
        7: "ACRO",
        8: "OFFBOARD",
        9: "STAB",
        10: "RATTITUDE",
        11: "AUTO_TAKEOFF",
        12: "AUTO_LANDENGFAIL",
        13: "AUTO_PRECLAND",
        14: "ORBIT",
    }
    try:
        return names.get(int(nav_state), f"NAV_STATE_{int(nav_state)}")
    except Exception:
        return f"NAV_STATE_{nav_state}"


def first_nonzero(seq):
    try:
        for v in seq:
            if abs(float(v)) > 1e-6:
                return float(v)
    except Exception:
        pass
    return None


def _cpu_usage_pct(sample_dt=0.2):
    def read():
        with open("/proc/stat", "r", encoding="utf-8") as f:
            parts = f.readline().split()[1:]
        vals = [int(x) for x in parts[:8]]  # user nice system idle iowait irq softirq steal
        idle = vals[3] + vals[4]
        total = sum(vals)
        return idle, total
    i0, t0 = read()
    time.sleep(sample_dt)
    i1, t1 = read()
    dt, di = (t1 - t0), (i1 - i0)
    return 0.0 if dt <= 0 else 100.0 * (dt - di) / dt


class SelfcheckNode(Node):
    def __init__(self, args):
        super().__init__("selfcheck")
        self.args = args
        self._sub_lock = Lock()

    def wait_for_message(self, msg_type, topic: str, timeout_s: float):
        got = Event()
        box = {"msg": None}

        def cb(msg):
            box["msg"] = msg
            got.set()

        with self._sub_lock:
            sub = self.create_subscription(msg_type, topic, cb, qos_profile_sensor_data)
        try:
            ok = got.wait(timeout_s)
            return box["msg"] if ok else None
        finally:
            with self._sub_lock:
                try: self.destroy_subscription(sub)
                except Exception: pass

    def wait_for_updates(self, msg_type, topic: str, timeout_s: float, min_msgs: int = 2):
        got = Event()
        count = {"n": 0}

        def cb(_msg):
            count["n"] += 1
            if count["n"] >= min_msgs:
                got.set()

        with self._sub_lock:
            sub = self.create_subscription(msg_type, topic, cb, qos_profile_sensor_data)
        try:
            return got.wait(timeout_s)
        finally:
            with self._sub_lock:
                try: self.destroy_subscription(sub)
                except Exception: pass

    def collect_pose_samples(self, topic: str, duration_s: float, max_samples: int = 600):
        samples = []
        stop_at = time.time() + duration_s

        def cb(msg: PoseWithCovarianceStamped):
            now = time.time()
            try:
                t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if t_msg <= 0.0:
                    t_msg = now
            except Exception:
                t_msg = now
            p = msg.pose.pose.position
            samples.append((t_msg, float(p.x), float(p.y), float(p.z)))

        with self._sub_lock:
            sub = self.create_subscription(PoseWithCovarianceStamped, topic, cb, qos_profile_sensor_data)
        try:
            while time.time() < stop_at and len(samples) < max_samples:
                time.sleep(0.01)
        finally:
            with self._sub_lock:
                try: self.destroy_subscription(sub)
                except Exception: pass

        samples.sort(key=lambda s: s[0])
        return samples

    def collect_odometry_pair(self, in_topic: str, out_topic: str, duration_s: float):
        latest = {"in": None, "out": None}
        stop_at = time.time() + duration_s

        def cb_in(msg: VehicleOdometry): latest["in"] = msg
        def cb_out(msg: VehicleOdometry): latest["out"] = msg

        with self._sub_lock:
            sub_in = self.create_subscription(VehicleOdometry, in_topic, cb_in, qos_profile_sensor_data)
            sub_out = self.create_subscription(VehicleOdometry, out_topic, cb_out, qos_profile_sensor_data)
        try:
            while time.time() < stop_at:
                time.sleep(0.02)
        finally:
            with self._sub_lock:
                for sub in (sub_in, sub_out):
                    try: self.destroy_subscription(sub)
                    except Exception: pass
        return latest["in"], latest["out"]

    # Checks

    @check("FCU (PX4 DDS)")
    def check_fcu(self):
        ok = self.wait_for_updates(VehicleStatus, self.args.vehicle_status_topic, timeout_s=self.args.timeout, min_msgs=1)
        if ok: info("VehicleStatus: updated (%s)", self.args.vehicle_status_topic)
        else:
            warn("VehicleStatus: NOT updated (%s)", self.args.vehicle_status_topic)
            return

        bat = self.wait_for_message(BatteryStatus, self.args.battery_topic, self.args.timeout)
        if bat is None:
            warn("BatteryStatus: NOT updated (%s)", self.args.battery_topic)
        else:
            info("BatteryStatus: updated (%s)", self.args.battery_topic)
            voltage = None
            if hasattr(bat, "voltage_v"):
                vv = getattr(bat, "voltage_v")
                if isinstance(vv, (list, tuple)): voltage = first_nonzero(vv)
                else:
                    try: voltage = float(vv)
                    except Exception: voltage = None
            if voltage is not None:
                if self.args.battery_cells and self.args.battery_cells > 0:
                    cell_v = voltage / float(self.args.battery_cells)
                    info("battery voltage: %.2f V (%.2f V/cell, %d cells)", voltage, cell_v, self.args.battery_cells)
                    if cell_v > 4.3 or cell_v < 3.0: warn("incorrect cell voltage: %.2f V/cell", cell_v)
                    elif cell_v < self.args.battery_min_cell_v:
                        warn("low cell voltage: %.2f V/cell (min=%.2f); recharge battery", cell_v, self.args.battery_min_cell_v)
                else:
                    info("battery voltage: %.2f V (set --battery-cells for per-cell checks)", voltage)
            else:
                warn("battery voltage present but could not parse voltage_v")

            # Extra battery checks (fields differ across px4_msgs versions)
            remaining = getattr(bat, "remaining", None)
            if remaining is not None:
                try:
                    rem = float(remaining)
                    if rem <= 1.0:
                        info("battery remaining: %.0f%%", rem * 100.0)
                    else:
                        info("battery remaining: %s", ff(rem, 2))
                    if rem <= 1.0 and rem < self.args.battery_min_remaining:
                        warn("low battery remaining: %.0f%% (min=%.0f%%)", rem * 100.0, self.args.battery_min_remaining * 100.0)
                except Exception:
                    info("battery remaining: unavailable (cannot parse)")

            warning = getattr(bat, "warning", None)
            if warning is not None:
                try:
                    w = int(warning)
                    info("battery warning: %d", w)
                    if w >= self.args.battery_warn_level:
                        warn("battery warning level is %d (threshold=%d)", w, self.args.battery_warn_level)
                except Exception:
                    info("battery warning: unavailable (cannot parse)")

            current_a = getattr(bat, "current_a", None)
            if current_a is not None:
                try:
                    ia = float(current_a)
                    info("battery current: %.2f A", ia)
                    if self.args.battery_max_current_a > 0 and abs(ia) > self.args.battery_max_current_a:
                        warn("battery current is high: %.2f A (max=%.2f A)", ia, self.args.battery_max_current_a)
                except Exception:
                    info("battery current: unavailable (cannot parse)")

            temp = getattr(bat, "temperature", None)
            if temp is not None:
                try:
                    t = float(temp)
                    if t > 200:  # sometimes centi-degC in older stacks
                        t = t / 100.0
                    info("battery temperature: %.1f C", t)
                    if self.args.battery_max_temp_c > 0 and t > self.args.battery_max_temp_c:
                        warn("battery temperature is high: %.1f C (max=%.1f C)", t, self.args.battery_max_temp_c)
                except Exception:
                    info("battery temperature: unavailable (cannot parse)")

        ts = self.wait_for_message(TimesyncStatus, self.args.timesync_topic, self.args.timeout)
        info("TimesyncStatus: %s (%s)", "updated" if ts is not None else "not observed", self.args.timesync_topic)

        fs = self.wait_for_message(FailsafeFlags, self.args.failsafe_topic, self.args.timeout)
        if fs is None:
            info("FailsafeFlags: not observed (%s)", self.args.failsafe_topic)
        else:
            info("FailsafeFlags: updated (%s)", self.args.failsafe_topic)
            active = []
            for attr in dir(fs):
                if attr.startswith("_") or attr == "timestamp": continue
                try: v = getattr(fs, attr)
                except Exception: continue
                if isinstance(v, bool) and v: active.append(attr)
            warn("failsafe flags active: %s", ", ".join(active[:20])) if active else info("failsafe flags: none active")

    @check("TelemetryStatus (PX4 DDS)")
    def check_telemetry_status(self):
        ts = self.wait_for_message(TelemetryStatus, self.args.telemetry_status_topic, self.args.timeout)
        if ts is None:
            warn("TelemetryStatus: NOT updated (%s)", self.args.telemetry_status_topic)
            return
        info("TelemetryStatus: updated (%s)", self.args.telemetry_status_topic)
        for k in ("type", "mode", "rssi", "remote_rssi", "noise", "rxerrors", "fixed", "txbuf", "rate", "heartbeat_time"):
            if hasattr(ts, k):
                try:
                    info("%s: %s", k, str(getattr(ts, k)))
                except Exception:
                    pass

    @check("VehicleControlMode / flight mode (PX4 DDS)")
    def check_vehicle_control_mode(self):
        vs = self.wait_for_message(VehicleStatus, self.args.vehicle_status_topic, self.args.timeout)
        if vs is None:
            warn("VehicleStatus: NOT updated (%s)", self.args.vehicle_status_topic)
        else:
            nav = getattr(vs, "nav_state", None)
            if nav is None:
                info("mode: unavailable (VehicleStatus.nav_state missing)")
            else:
                info("mode: %s (%s)", px4_nav_state_name(nav), str(nav))

        cm = self.wait_for_message(VehicleControlMode, self.args.vehicle_control_mode_topic, self.args.timeout)
        if cm is None:
            warn("VehicleControlMode: NOT updated (%s)", self.args.vehicle_control_mode_topic)
            return
        info("VehicleControlMode: updated (%s)", self.args.vehicle_control_mode_topic)
        # enabled = []
        # for attr in dir(cm):
        #     if not attr.startswith("flag_") or attr == "timestamp":
        #         continue
        #     try:
        #         v = getattr(cm, attr)
        #     except Exception:
        #         continue
        #     if isinstance(v, bool) and v:
        #         enabled.append(attr)
        # info("control flags enabled: %s", ", ".join(sorted(enabled)) if enabled else "none")

    @check("PX4 Local Position (PX4 DDS)")
    def check_px4_local_position(self):
        lp = self.wait_for_message(VehicleLocalPosition, self.args.vehicle_local_position_topic, self.args.timeout)
        if lp is None:
            warn("VehicleLocalPosition: NOT updated (%s)", self.args.vehicle_local_position_topic)
            return
        try:
            x = ff(float(getattr(lp, "x", 0.0)), 3)
            y = ff(float(getattr(lp, "y", 0.0)), 3)
            z = ff(float(getattr(lp, "z", 0.0)), 3)
            vx = ff(float(getattr(lp, "vx", 0.0)), 3)
            vy = ff(float(getattr(lp, "vy", 0.0)), 3)
            vz = ff(float(getattr(lp, "vz", 0.0)), 3)
            heading = ff(float(getattr(lp, "heading", 0.0)), 3)
        except Exception:
            info("VehicleLocalPosition: updated (%s), but could not format numeric fields", self.args.vehicle_local_position_topic)
            return

        xy_valid = getattr(lp, "xy_valid", None)
        z_valid = getattr(lp, "z_valid", None)
        v_xy_valid = getattr(lp, "v_xy_valid", None)
        v_z_valid = getattr(lp, "v_z_valid", None)

        info(
            "VehicleLocalPosition: topic=%s, pos=(%s, %s, %s) m, vel=(%s, %s, %s) m/s, heading=%s rad, valid(xy=%s, z=%s, v_xy=%s, v_z=%s)",
            self.args.vehicle_local_position_topic,
            x, y, z,
            vx, vy, vz,
            heading,
            str(xy_valid), str(z_valid), str(v_xy_valid), str(v_z_valid),
        )

    @check("IMU (PX4 DDS)")
    def check_imu(self):
        ok = self.wait_for_updates(SensorCombined, self.args.imu_topic, timeout_s=self.args.timeout, min_msgs=2)
        info("SensorCombined: updated (%s)", self.args.imu_topic) if ok else warn("SensorCombined: NOT updated (%s)", self.args.imu_topic)

    @check("Attitude (PX4 DDS)")
    def check_attitude(self):
        att = self.wait_for_message(VehicleAttitude, self.args.attitude_topic, self.args.timeout)
        if att is None:
            warn("VehicleAttitude: NOT updated (%s)", self.args.attitude_topic)
            return
        info("VehicleAttitude: updated (%s)", self.args.attitude_topic)
        q = getattr(att, "q", None)
        if q is None or len(q) != 4:
            info("attitude quaternion: unavailable (field q missing/invalid)")
            return
        qw, qx, qy, qz = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        roll, pitch, yaw = quat_to_euler_rpy(qx, qy, qz, qw)
        info("attitude r/p/y: %.2f / %.2f / %.2f deg", math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        max_angle = math.radians(self.args.max_tilt_deg)
        if abs(pitch) > max_angle: warn("pitch is %.2f deg; place copter horizontally or redo level horizon calib", math.degrees(pitch))
        if abs(roll) > max_angle: warn("roll is %.2f deg; place copter horizontally or redo level horizon calib", math.degrees(roll))

    @check("Local position (ArUco)")
    def check_local_position(self):
        ok = self.wait_for_updates(PoseWithCovarianceStamped, self.args.pose_topic, timeout_s=self.args.timeout, min_msgs=2)
        info("PoseWithCovarianceStamped: updated (%s)", self.args.pose_topic) if ok else warn("PoseWithCovarianceStamped: NOT updated (%s)", self.args.pose_topic)

    @check("Velocity estimation (from ArUco pose)")
    def check_velocity(self):
        samples = self.collect_pose_samples(self.args.pose_topic, self.args.velocity_window)
        if len(samples) < max(2, self.args.velocity_min_samples):
            warn("velocity estimate: NOT available (insufficient pose samples from %s)", self.args.pose_topic)
            info("pose sample window: %.2fs (min_samples=%d)", self.args.velocity_window, self.args.velocity_min_samples)
            return
        max_h = max_v = 0.0
        for (t0, x0, y0, z0), (t1, x1, y1, z1) in zip(samples[:-1], samples[1:]):
            dt = t1 - t0
            if dt <= 1e-6: continue
            vx, vy, vz = (x1 - x0) / dt, (y1 - y0) / dt, (z1 - z0) / dt
            max_h = max(max_h, math.hypot(vx, vy))
            max_v = max(max_v, abs(vz))
        info("max horizontal velocity: %s m/s (threshold=%.3f)", ff(max_h, 3), self.args.velocity_threshold)
        info("max vertical velocity: %s m/s (threshold=%.3f)", ff(max_v, 3), self.args.velocity_threshold)
        if max_h > self.args.velocity_threshold: warn("horizontal velocity is %.3f m/s; is the drone staying still?", max_h)
        if max_v > self.args.velocity_threshold: warn("vertical velocity is %.3f m/s; is the drone staying still?", max_v)

    @check("Camera")
    def check_camera(self):
        ok = self.wait_for_updates(Image, self.args.image_topic, timeout_s=self.args.timeout, min_msgs=2)
        info("Image: updated (%s)", self.args.image_topic) if ok else warn("Image: NOT updated (%s)", self.args.image_topic)

    @check("ArUco markers")
    def check_aruco(self):
        try:
            msg_type = import_pytype(self.args.markers_pytype)
        except Exception as e:
            warn("cannot import markers message type: %s", str(e))
            return
        ok = self.wait_for_updates(msg_type, self.args.markers_topic, timeout_s=self.args.timeout, min_msgs=2)
        info("Markers: updated (%s)", self.args.markers_topic) if ok else warn("Markers: NOT updated (%s)", self.args.markers_topic)

    @check("VPE (vision input vs PX4 estimate)")
    def check_vpe(self):
        in_topic = self.args.visual_odom_topic
        info("VPE input topic: %s", in_topic)

        ok_in = self.wait_for_updates(VehicleOdometry, in_topic, timeout_s=self.args.timeout, min_msgs=2)
        if ok_in: info("VPE input odometry: updated (%s)", in_topic)
        else:
            warn("VPE input odometry: NOT updated (%s)", in_topic)
            return

        ok_out = self.wait_for_updates(VehicleOdometry, self.args.vehicle_odometry_topic, timeout_s=self.args.timeout, min_msgs=2)
        if ok_out: info("PX4 odometry output: updated (%s)", self.args.vehicle_odometry_topic)
        else:
            warn("PX4 odometry output: NOT updated (%s)", self.args.vehicle_odometry_topic)
            return

        vin, vout = self.collect_odometry_pair(in_topic, self.args.vehicle_odometry_topic, duration_s=self.args.vpe_compare_window)
        info("VPE compare window: %.2fs", self.args.vpe_compare_window)
        if vin is None or vout is None:
            warn("unable to sample VPE input/output odometry for comparison")
            return

        pin, pout = getattr(vin, "position", None), getattr(vout, "position", None)
        if pin is None or pout is None or len(pin) < 3 or len(pout) < 3:
            warn("odometry position fields unavailable for comparison")
            return

        dx, dy, dz = float(pout[0]) - float(pin[0]), float(pout[1]) - float(pin[1]), float(pout[2]) - float(pin[2])
        horiz, vert = math.hypot(dx, dy), abs(dz)
        info("VPE vs PX4 odom: horiz=%s m, vert=%s m (threshold=%.2f m)", ff(horiz, 3), ff(vert, 3), self.args.vpe_pos_threshold)
        if horiz > self.args.vpe_pos_threshold: warn("VPE/PX4 horizontal inconsistency: %.2f m", horiz)
        if vert > self.args.vpe_pos_threshold: warn("VPE/PX4 vertical inconsistency: %.2f m", vert)

        qin, qout = getattr(vin, "q", None), getattr(vout, "q", None)
        if qin is not None and qout is not None and len(qin) == 4 and len(qout) == 4:
            qw, qx, qy, qz = float(qin[0]), float(qin[1]), float(qin[2]), float(qin[3])
            _, _, yin = quat_to_euler_rpy(qx, qy, qz, qw)
            qw, qx, qy, qz = float(qout[0]), float(qout[1]), float(qout[2]), float(qout[3])
            _, _, yout = quat_to_euler_rpy(qx, qy, qz, qw)
            yd = abs(yaw_diff_deg(yout, yin))
            info("VPE vs PX4 yaw diff: %s deg (threshold=%.2f deg)", ff(yd, 2), self.args.vpe_yaw_threshold_deg)
            if yd > self.args.vpe_yaw_threshold_deg: warn("VPE/PX4 yaw inconsistency: %.2f deg", yd)
        else:
            info("VPE/PX4 yaw diff: unavailable (missing quaternion(s))")

        esf = self.wait_for_message(EstimatorStatusFlags, self.args.estimator_flags_topic, self.args.timeout)
        if esf is None:
            info("EstimatorStatusFlags: not observed (%s)", self.args.estimator_flags_topic)
        else:
            info("EstimatorStatusFlags: updated (%s)", self.args.estimator_flags_topic)
            active = []
            for attr in dir(esf):
                if attr.startswith("_") or attr == "timestamp": continue
                try: v = getattr(esf, attr)
                except Exception: continue
                if isinstance(v, bool) and v and ("fail" in attr or "fault" in attr or "invalid" in attr):
                    active.append(attr)
            warn("estimator status flags indicate issues: %s", ", ".join(active[:20])) if active else info("estimator flags: no failure/fault/invalid flags active")

    @check("SBC health")
    def check_sbc_health(self):
        try:
            total, used, free = shutil.disk_usage("/")
            info("disk free: %.2f GB (total: %.2f GB)", free / (1024**3), total / (1024**3))
            if free < 1024 * 1024 * 1024:
                warn("disk space is less than 1 GB; consider removing logs")
        except Exception as e:
            info("could not check disk free space: %s", str(e))

        flags = (
            (0x4, "system throttled to prevent damage"),
            (0x40000, "system was throttled previously"),
            (0x1, "undervoltage now (power insufficient; flight inadvisable)"),
            (0x10000, "undervoltage occurred previously (power supply insufficient)"),
            (0x2, "frequency capped now (thermal limit)"),
            (0x20000, "frequency capped occurred previously (may overheat)"),
            (0x8, "soft thermal limit now (freq reduced)"),
            (0x80000, "soft thermal limit occurred previously (consider cooling)"),
        )
        try:
            if not os.path.exists("/dev/vchiq"):
                info("vcgencmd get_throttled: skipped (no /dev/vchiq inside container/host)")
                return
            out = subprocess.check_output(["vcgencmd", "get_throttled"], text=True).strip()
            throttle_mask = int(out.split("=")[1], 16)
            info("vcgencmd get_throttled: %s (mask=0x%X)", out, throttle_mask)
            any_flag = False
            for flag, desc in flags:
                if throttle_mask & flag:
                    any_flag = True
                    warn("%s", desc)
            if not any_flag:
                info("throttle flags: none set")
        except OSError:
            info("could not call vcgencmd; not a Raspberry Pi?")
        except Exception as e:
            info("could not parse throttling state: %s", str(e))

    @check("CPU usage")
    def check_cpu_usage(self):
        info("%.1f", _cpu_usage_pct())


def run_checks(node: SelfcheckNode, args):
    checks = []
    if not args.no_fcu: checks.append(node.check_fcu)
    if not args.no_telemetry_status: checks.append(node.check_telemetry_status)
    if not args.no_vehicle_control_mode: checks.append(node.check_vehicle_control_mode)
    if not args.no_px4_local_position: checks.append(node.check_px4_local_position)
    if not args.no_imu: checks.append(node.check_imu)
    if not args.no_attitude: checks.append(node.check_attitude)
    if not args.no_local: checks.append(node.check_local_position)
    if not args.no_velocity: checks.append(node.check_velocity)
    if not args.no_camera: checks.append(node.check_camera)
    if not args.no_aruco: checks.append(node.check_aruco)
    if not args.no_vpe: checks.append(node.check_vpe)
    if not args.no_sbc_health: checks.append(node.check_sbc_health)
    if not args.no_cpu: checks.append(node.check_cpu_usage)

    if args.parallel:
        threads = []
        for fn in checks:
            t = Thread(target=fn)
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
    else:
        for fn in checks:
            fn()


def build_arg_parser():
    import argparse
    p = argparse.ArgumentParser(description="ROS 2 drone selfcheck (PX4 DDS / px4_msgs + ArUco)")
    p.add_argument("--timeout", type=float, default=3.0)
    p.add_argument("--parallel", action="store_true")

    p.add_argument("--vehicle-status-topic", default="/fmu/out/vehicle_status")
    p.add_argument("--imu-topic", default="/fmu/out/sensor_combined")
    p.add_argument("--battery-topic", default="/fmu/out/battery_status")
    p.add_argument("--timesync-topic", default="/fmu/out/timesync_status")
    p.add_argument("--failsafe-topic", default="/fmu/out/failsafe_flags")
    p.add_argument("--estimator-flags-topic", default="/fmu/out/estimator_status_flags")
    p.add_argument("--telemetry-status-topic", default="/fmu/in/telemetry_status")
    p.add_argument("--vehicle-control-mode-topic", default="/fmu/out/vehicle_control_mode")

    p.add_argument("--vehicle-odometry-topic", default="/fmu/out/vehicle_odometry")
    p.add_argument("--vehicle-local-position-topic", default="/fmu/out/vehicle_local_position")
    p.add_argument("--attitude-topic", default="/fmu/out/vehicle_attitude")

    p.add_argument("--visual-odom-topic", default="/fmu/in/vehicle_visual_odometry")
    p.add_argument("--vpe-compare-window", type=float, default=1.5)
    p.add_argument("--vpe-pos-threshold", type=float, default=0.5)
    p.add_argument("--vpe-yaw-threshold-deg", type=float, default=8.0)

    p.add_argument("--pose-topic", default="/aruco_map/pose_cov")
    p.add_argument("--velocity-window", type=float, default=3.0)
    p.add_argument("--velocity-threshold", type=float, default=0.10)
    p.add_argument("--velocity-min-samples", type=int, default=10)

    p.add_argument("--image-topic", default="/camera_1/image_raw")
    p.add_argument("--markers-topic", default="/markers")
    p.add_argument("--markers-pytype", default="aruco_det_loc.msg.MarkerArray")

    p.add_argument("--battery-cells", type=int, default=0)
    p.add_argument("--battery-min-cell-v", type=float, default=3.50)
    p.add_argument("--battery-min-remaining", type=float, default=0.20)
    p.add_argument("--battery-warn-level", type=int, default=2)
    p.add_argument("--battery-max-current-a", type=float, default=0.0)
    p.add_argument("--battery-max-temp-c", type=float, default=0.0)
    p.add_argument("--max-tilt-deg", type=float, default=2.0)

    p.add_argument("--no-fcu", action="store_true")
    p.add_argument("--no-telemetry-status", action="store_true")
    p.add_argument("--no-vehicle-control-mode", action="store_true")
    p.add_argument("--no-px4-local-position", action="store_true")
    p.add_argument("--no-imu", action="store_true")
    p.add_argument("--no-attitude", action="store_true")
    p.add_argument("--no-local", action="store_true")
    p.add_argument("--no-velocity", action="store_true")
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--no-aruco", action="store_true")
    p.add_argument("--no-vpe", action="store_true")
    p.add_argument("--no-sbc-health", action="store_true")
    p.add_argument("--no-cpu", action="store_true")
    return p


def main():
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    node = SelfcheckNode(args)
    node.get_logger().info("Performing selfcheck (ROS 2, PX4 DDS)...")

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop_spin = Event()

    def spin_loop():
        while rclpy.ok() and not stop_spin.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = Thread(target=spin_loop, daemon=True)
    spin_thread.start()

    try:
        run_checks(node, args)
    finally:
        stop_spin.set()
        try: spin_thread.join(timeout=1.0)
        except Exception: pass
        try: executor.remove_node(node)
        except Exception: pass
        try: node.destroy_node()
        except Exception: pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
