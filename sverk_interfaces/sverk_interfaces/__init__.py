import atexit
import math
import threading
import time
from typing import Any, Dict, Optional, Type

import rclpy
from rclpy.node import Node

from offboard_interfaces.srv import (
    Flip,
    GetTelemetry,
    Navigate,
    SetAltitude,
    SetAttitude,
    SetPosition,
    SetRates,
    SetVelocity,
    SetYaw,
    SetYawRate,
)
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

try:
    from led_interfaces.msg import LEDState, LEDStateArray
    from led_interfaces.srv import SetLEDEffect, SetLEDs
    _LED_AVAILABLE = True
except ImportError:
    _LED_AVAILABLE = False
    LEDState = None  # type: ignore[misc, assignment]
    LEDStateArray = None  # type: ignore[misc, assignment]
    SetLEDEffect = None  # type: ignore[misc, assignment]
    SetLEDs = None  # type: ignore[misc, assignment]


_instances_lock = threading.Lock()
_instances: "set[DroneInterfaces]" = set()  # type: ignore[name-defined]
_rclpy_owned_by_library = False


def _normalize_ns(prefix: str) -> str:
    if not prefix:
        return ""
    if not prefix.startswith("/"):
        prefix = "/" + prefix
    return prefix.rstrip("/")


def _service_name(prefix: str, name: str) -> str:
    prefix = _normalize_ns(prefix)
    if prefix:
        return f"{prefix}/{name.lstrip('/')}"
    # Global service name so it does not depend on client namespace
    return "/" + name.lstrip("/")


def _topic_name(prefix: str, name: str) -> str:
    prefix = _normalize_ns(prefix)
    if prefix:
        return f"{prefix}/{name.lstrip('/')}"
    return "/" + name.lstrip("/")


def _ensure_rclpy_init() -> None:
    global _rclpy_owned_by_library
    if not rclpy.ok():
        rclpy.init()
        _rclpy_owned_by_library = True


def _maybe_rclpy_shutdown() -> None:
    """Called when no DroneInterfaces remain and this library started rclpy."""
    global _rclpy_owned_by_library
    if _rclpy_owned_by_library and rclpy.ok():
        try:
            rclpy.shutdown()
        finally:
            _rclpy_owned_by_library = False


def _call_service(
    node: Node,
    client: rclpy.client.Client,
    request: Any,
    timeout: Optional[float],
) -> Any:
    if not client.wait_for_service(timeout_sec=timeout or 5.0):
        raise RuntimeError(f"Service {client.srv_name} is not available")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f"Service {client.srv_name} did not respond within {timeout} s")
    result = future.result()
    if result is None:
        raise RuntimeError(f"Service {client.srv_name} returned an empty result")
    return result


class OffboardControlAPI:
    """High-level access to offboard_control services via drone.controll.*"""

    def __init__(self, node: Node, namespace: str = "") -> None:
        self._node = node
        self._ns = namespace

        self.default_frame_id: str = "map"
        self.default_yaw: float = 0.0
        self.default_speed: float = 0.5
        self.default_auto_arm: bool = False
        self.default_timeout: float = 60.0
        self.default_tolerance: float = 0.2
        self.default_check_interval: float = 0.2

        self.default_flip_axis: str = 'roll'
        self.default_flip_vz: float = 2.0
        self.default_flip_climb_duration: float = 0.5
        self.default_flip_rate: float = 16.0
        self.default_flip_target_angle: float = 6.0
        self.default_flip_thrust: float = 0.1

        # offboard_control service clients
        self._navigate = node.create_client(Navigate, _service_name(self._ns, "navigate"))
        self._land = node.create_client(Trigger, _service_name(self._ns, "land"))
        self._get_telemetry = node.create_client(
            GetTelemetry, _service_name(self._ns, "get_telemetry")
        )
        self._set_altitude = node.create_client(
            SetAltitude, _service_name(self._ns, "set_altitude")
        )
        self._set_yaw = node.create_client(SetYaw, _service_name(self._ns, "set_yaw"))
        self._set_yaw_rate = node.create_client(
            SetYawRate, _service_name(self._ns, "set_yaw_rate")
        )
        self._set_position = node.create_client(
            SetPosition, _service_name(self._ns, "set_position")
        )
        self._set_velocity = node.create_client(
            SetVelocity, _service_name(self._ns, "set_velocity")
        )
        self._set_attitude = node.create_client(
            SetAttitude, _service_name(self._ns, "set_attitude")
        )
        self._set_rates = node.create_client(SetRates, _service_name(self._ns, "set_rates"))
        self._flip = node.create_client(Flip, _service_name(self._ns, "flip"))

    def configure_defaults(
        self,
        *,
        frame_id: Optional[str] = None,
        yaw: Optional[float] = None,
        speed: Optional[float] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None,
        check_interval: Optional[float] = None,
    ) -> None:
        """Change defaults for navigate / navigate_wait.

        Example:
            drone.controll.configure_defaults(
                frame_id="body",
                speed=0.4,
                tolerance=0.3,
            )
        """
        if frame_id is not None:
            self.default_frame_id = frame_id
        if yaw is not None:
            self.default_yaw = float(yaw)
        if speed is not None:
            self.default_speed = float(speed)
        if auto_arm is not None:
            self.default_auto_arm = bool(auto_arm)
        if timeout is not None:
            self.default_timeout = float(timeout)
        if tolerance is not None:
            self.default_tolerance = float(tolerance)
        if check_interval is not None:
            self.default_check_interval = float(check_interval)

    # --- Navigation and flight modes ---

    def navigate(
        self,
        *,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
        speed: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> Navigate.Response:
        """Fly to a waypoint (see offboard_control README.md: navigate service)."""
        # Fill in defaults when arguments are omitted.
        if frame_id is None:
            frame_id = self.default_frame_id
        if yaw is None:
            yaw = self.default_yaw
        if speed is None:
            speed = self.default_speed
        if auto_arm is None:
            auto_arm = self.default_auto_arm
        if timeout is None:
            timeout = self.default_timeout

        req = Navigate.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.speed = float(speed)
        req.frame_id = frame_id
        req.auto_arm = bool(auto_arm)
        return _call_service(self._node, self._navigate, req, timeout)

    def navigate_wait(
        self,
        *,
        x: float,
        y: float,
        z: float,
        yaw: float,
        speed: float,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None,
        check_interval: Optional[float] = None,
    ) -> Navigate.Response:
        """Fly to a waypoint and wait until the goal is reached.

        Assumes get_telemetry(frame_id=...) returns x, y, z in the same frame
        as the navigate arguments.
        """
        # Empty frame_id uses the same default as navigate/get_telemetry.
        effective_frame = frame_id or self.default_frame_id
        eff_timeout = timeout if timeout is not None else self.default_timeout
        eff_tolerance = tolerance if tolerance is not None else self.default_tolerance
        eff_check_interval = (
            check_interval if check_interval is not None else self.default_check_interval
        )

        resp = self.navigate(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            speed=speed,
            frame_id=effective_frame,
            auto_arm=auto_arm,
            timeout=eff_timeout,
        )
        if not resp.success:
            raise RuntimeError(f"navigate_wait: navigate failed: {resp.message}")

        start_t = time.monotonic()
        while True:
            if time.monotonic() - start_t > eff_timeout:
                raise TimeoutError("navigate_wait: timeout waiting for goal")

            telem = self.get_telemetry(frame_id='navigate_target', timeout=2.0)
            dist = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)

            if dist < eff_tolerance:
                return resp

            time.sleep(eff_check_interval)

    def land(self, timeout: Optional[float] = None) -> Trigger.Response:
        """Land (std_srvs/Trigger)."""
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = Trigger.Request()
        return _call_service(self._node, self._land, req, eff_timeout)

    # --- Telemetry ---

    def get_telemetry(
        self, frame_id: Optional[str] = None, timeout: Optional[float] = None
    ) -> GetTelemetry.Response:
        """Request telemetry snapshot."""
        eff_frame = self.default_frame_id if not frame_id else frame_id
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = GetTelemetry.Request()
        req.frame_id = eff_frame
        return _call_service(self._node, self._get_telemetry, req, eff_timeout)

    # --- Altitude and heading ---

    def set_altitude(
        self, z: float, frame_id: Optional[str] = None, timeout: Optional[float] = None
    ) -> SetAltitude.Response:
        eff_frame = frame_id or self.default_frame_id
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetAltitude.Request()
        req.z = float(z)
        req.frame_id = eff_frame
        return _call_service(self._node, self._set_altitude, req, eff_timeout)

    def set_yaw(
        self,
        yaw: Optional[float] = None,
        frame_id: Optional[str] = None,
        timeout: Optional[float] = None,
    ) -> SetYaw.Response:
        eff_yaw = self.default_yaw if yaw is None else float(yaw)
        eff_frame = frame_id or self.default_frame_id
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetYaw.Request()
        req.yaw = eff_yaw
        req.frame_id = eff_frame
        return _call_service(self._node, self._set_yaw, req, eff_timeout)

    def clear_yaw_override(self, timeout: Optional[float] = None) -> SetYaw.Response:
        """Clear yaw / yaw_rate override (yaw=NaN)."""
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetYaw.Request()
        req.yaw = float("nan")
        req.frame_id = "body"
        return _call_service(self._node, self._set_yaw, req, eff_timeout)

    def set_yaw_rate(
        self, yaw_rate: float, timeout: Optional[float] = None
    ) -> SetYawRate.Response:
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetYawRate.Request()
        req.yaw_rate = float(yaw_rate)
        return _call_service(self._node, self._set_yaw_rate, req, eff_timeout)

    # --- Position and velocity ---

    def set_position(
        self,
        *,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetPosition.Response:
        eff_yaw = self.default_yaw if yaw is None else float(yaw)
        eff_frame = frame_id or self.default_frame_id
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetPosition.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = eff_yaw
        req.frame_id = eff_frame
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_position, req, eff_timeout)

    def set_velocity(
        self,
        *,
        vx: float,
        vy: float,
        vz: float,
        yaw: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetVelocity.Response:
        eff_yaw = self.default_yaw if yaw is None else float(yaw)
        eff_frame = frame_id or self.default_frame_id
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetVelocity.Request()
        req.vx = float(vx)
        req.vy = float(vy)
        req.vz = float(vz)
        req.yaw = eff_yaw
        req.frame_id = eff_frame
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_velocity, req, eff_timeout)

    # --- Low-level control ---

    def set_attitude(
        self,
        *,
        roll: float,
        pitch: float,
        yaw: Optional[float] = None,
        thrust: float,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetAttitude.Response:
        eff_yaw = self.default_yaw if yaw is None else float(yaw)
        eff_frame = frame_id or self.default_frame_id
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetAttitude.Request()
        req.roll = float(roll)
        req.pitch = float(pitch)
        req.yaw = float(eff_yaw)
        req.thrust = float(thrust)
        req.frame_id = eff_frame
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_attitude, req, eff_timeout)

    def set_rates(
        self,
        *,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
        thrust: float,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetRates.Response:
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetRates.Request()
        req.roll_rate = float(roll_rate)
        req.pitch_rate = float(pitch_rate)
        req.yaw_rate = float(yaw_rate)
        req.thrust = float(thrust)
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_rates, req, eff_timeout)

    # --- Flip ---

    def flip(
        self,
        *,
        axis: Optional[str] = None,
        vz: Optional[float] = None,
        climb_duration: Optional[float] = None,
        rate: Optional[float] = None,
        target_angle: Optional[float] = None,
        thrust: Optional[float] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> Flip.Response:
        """Flip around roll/pitch/yaw (see README.md: flip)."""

        eff_axis = self.default_flip_axis if axis is None else str(axis)
        eff_vz = self.default_flip_vz if vz is None else float(vz)
        eff_climb_duration = self.default_flip_climb_duration if climb_duration is None else float(climb_duration)
        eff_rate = self.default_flip_rate if rate is None else float(rate)
        eff_target_angle = self.default_flip_target_angle if target_angle is None else float(target_angle)
        eff_thrust = self.default_flip_thrust if thrust is None else float(thrust)
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout

        req = Flip.Request()
        req.axis = eff_axis
        req.vz = eff_vz
        req.climb_duration = eff_climb_duration
        req.rate = eff_rate
        req.target_angle = eff_target_angle
        req.thrust = eff_thrust
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._flip, req, eff_timeout)


class FmuCalibrationAPI:
    """fmu_calibration_control API via drone.fcu.*"""

    def __init__(self, node: Node, namespace: str = "/fmu_calibration_control") -> None:
        self._node = node
        self._ns = _normalize_ns(namespace) or "/fmu_calibration_control"

        self._disarm = node.create_client(
            Trigger, _service_name(self._ns, "disarm")
        )
        self._force_disarm = node.create_client(
            Trigger, _service_name(self._ns, "force_disarm")
        )
        self._kill_switch = node.create_client(
            Trigger, _service_name(self._ns, "kill_switch")
        )

        self._calib_pub = node.create_publisher(
            UInt8, _topic_name(self._ns, "request_calibration"), 10
        )

    # --- Disarm / kill switch ---

    def disarm(self, timeout: float = 5.0) -> Trigger.Response:
        req = Trigger.Request()
        return _call_service(self._node, self._disarm, req, timeout)

    def force_disarm(self, timeout: float = 5.0) -> Trigger.Response:
        req = Trigger.Request()
        return _call_service(self._node, self._force_disarm, req, timeout)

    def kill_switch(self, timeout: float = 5.0) -> Trigger.Response:
        req = Trigger.Request()
        return _call_service(self._node, self._kill_switch, req, timeout)

    # --- Calibration ---

    def request_calibration(self, mode: int) -> None:
        """Publish arbitrary calibration command (see MAV_CMD_PREFLIGHT_CALIBRATION)."""
        msg = UInt8()
        msg.data = int(mode) & 0xFF
        self._calib_pub.publish(msg)

    def calibrate_gyro(self) -> None:
        self.request_calibration(0)

    def calibrate_mag(self) -> None:
        self.request_calibration(1)

    def calibrate_baro(self) -> None:
        self.request_calibration(2)

    def calibrate_temperature(self) -> None:
        self.request_calibration(3)

    def calibrate_accel(self) -> None:
        self.request_calibration(4)

    def calibrate_level(self) -> None:
        self.request_calibration(5)


# LED effect names per led_control docs (README.md)
LED_EFFECTS = (
    "fill",
    "blink",
    "blink_fast",
    "fade",
    "wipe",
    "flash",
    "rainbow",
    "rainbow_fill",
)


class LedAPI:
    """LED strip API (led_control) via drone.led.*.

    Services: /led/set_effect (effect + color), /led/set_leds (per-LED).
    State topic: /led/state (LEDStateArray).
    """

    def __init__(self, node: Node, namespace: str = "led") -> None:
        if not _LED_AVAILABLE:
            raise RuntimeError(
                "led_interfaces is not installed. Add the dependency to use the LED API."
            )
        self._node = node
        self._ns = _normalize_ns(namespace) if namespace else "/led"

        self._set_effect = node.create_client(
            SetLEDEffect, _service_name(self._ns, "set_effect")
        )
        self._set_leds = node.create_client(
            SetLEDs, _service_name(self._ns, "set_leds")
        )

        self.default_timeout: float = 5.0

    def set_effect(
        self,
        effect: str = "fill",
        r: int = 0,
        g: int = 0,
        b: int = 0,
        timeout: Optional[float] = None,
    ) -> Any:
        """Set strip effect and color (/led/set_effect).

        Effects: fill, blink, blink_fast, fade, wipe, flash, rainbow, rainbow_fill.
        Empty string is treated as fill.
        r, g, b are 0–255.
        """
        req = SetLEDEffect.Request()
        req.effect = effect or "fill"
        req.r = int(r) & 0xFF
        req.g = int(g) & 0xFF
        req.b = int(b) & 0xFF
        eff_timeout = timeout if timeout is not None else self.default_timeout
        return _call_service(self._node, self._set_effect, req, eff_timeout)

    def set_leds(
        self,
        leds: list,
        timeout: Optional[float] = None,
    ) -> Any:
        """Set per-LED colors (/led/set_leds).

        leds: list of (index, r, g, b), dicts with index/r/g/b keys,
        or led_interfaces.msg.LEDState. Indices 0 … led_count-1.
        """
        req = SetLEDs.Request()
        req.leds = self._to_led_state_list(leds)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        return _call_service(self._node, self._set_leds, req, eff_timeout)

    def _to_led_state_list(self, leds: list) -> list:
        out = []
        for item in leds:
            if isinstance(item, LEDState):
                out.append(item)
            elif isinstance(item, (tuple, list)) and len(item) >= 4:
                st = LEDState()
                st.index = int(item[0])
                st.r = int(item[1]) & 0xFF
                st.g = int(item[2]) & 0xFF
                st.b = int(item[3]) & 0xFF
                out.append(st)
            elif isinstance(item, dict):
                st = LEDState()
                st.index = int(item["index"])
                st.r = int(item.get("r", 0)) & 0xFF
                st.g = int(item.get("g", 0)) & 0xFF
                st.b = int(item.get("b", 0)) & 0xFF
                out.append(st)
            else:
                raise TypeError(
                    f"Strip element must be LEDState, (index,r,g,b), or dict: {item!r}"
                )
        return out

    def get_state(self, timeout: Optional[float] = None) -> Optional[Any]:
        """Wait for one /led/state message (LEDStateArray).

        Returns LEDStateArray or None on timeout.
        """
        received: list = []
        topic = _topic_name(self._ns, "state")

        def cb(msg: Any) -> None:
            received.append(msg)

        sub = self._node.create_subscription(LEDStateArray, topic, cb, 10)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        deadline = time.monotonic() + eff_timeout
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            return received[0] if received else None
        finally:
            self._node.destroy_subscription(sub)


class GenericServiceFamily:
    """Generic service bundle for extending the API.

    Use via drone.add_family(...); see DroneInterfaces.add_family.
    """

    def __init__(
        self,
        node: Node,
        namespace: str,
        services: Dict[str, Type[Any]],
    ) -> None:
        self._node = node
        self._ns = namespace
        self._clients: Dict[str, rclpy.client.Client] = {}

        for name, srv_type in services.items():
            self._clients[name] = node.create_client(
                srv_type, _service_name(self._ns, name)
            )

    def call(
        self,
        service_name: str,
        request: Any,
        timeout: Optional[float] = None,
    ) -> Any:
        """Low-level call to an arbitrary service in this family.

        Example:

            from my_pkg.srv import MySrv
            fam = drone.add_family(
                name="my_family",
                namespace="/my_ns",
                services={"do_something": MySrv},
            )
            req = MySrv.Request()
            req.foo = 1
            resp = fam.call("do_something", req)
        """
        if service_name not in self._clients:
            raise KeyError(f"Unknown service '{service_name}' in family")
        client = self._clients[service_name]
        return _call_service(self._node, client, request, timeout)


class DroneInterfaces:
    """Main library handle: drone = sverk_interfaces.init(...).

    Attributes:
      - controll: OffboardControlAPI (navigate, land, set_*, etc.)
      - fcu: FmuCalibrationAPI (disarm / force_disarm / kill_switch / calibration)
      - led: LedAPI or None (set_effect, set_leds, get_state) if led_interfaces is installed
    """

    def __init__(
        self,
        node: Node,
        *,
        offboard_namespace: str = "",
        fcu_namespace: str = "/fmu_calibration_control",
        led_namespace: str = "led",
    ) -> None:
        self._node = node
        self._closed = False
        self._families: Dict[str, GenericServiceFamily] = {}

        self.controll = OffboardControlAPI(node, namespace=offboard_namespace)
        self.fcu = FmuCalibrationAPI(node, namespace=fcu_namespace)
        if _LED_AVAILABLE:
            self.led = LedAPI(node, namespace=led_namespace)
        else:
            self.led = None  # type: ignore[assignment]

        with _instances_lock:
            _instances.add(self)

    @property
    def node(self) -> Node:
        """Underlying rclpy node for low-level use."""
        return self._node

    def add_family(
        self,
        *,
        name: str,
        namespace: str,
        services: Dict[str, Type[Any]],
    ) -> GenericServiceFamily:
        """Register a new service family.

        Example:

            from my_pkg.srv import DoThing

            drone = sverk_interfaces.init(Nodename="drone_control")
            my_family = drone.add_family(
                name="my",
                namespace="/my_node",
                services={"do_thing": DoThing},
            )
            req = DoThing.Request()
            req.param = 123
            resp = my_family.call("do_thing", req)
        """
        family = GenericServiceFamily(self._node, namespace, services)
        self._families[name] = family
        setattr(self, name, family)
        return family

    def close(self) -> None:
        """Explicit cleanup (node + rclpy if owned by library)."""
        if self._closed:
            return
        self._closed = True

        try:
            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
        finally:
            with _instances_lock:
                _instances.discard(self)
                # Shutdown rclpy when no instances remain and we started it.
                if not _instances:
                    _maybe_rclpy_shutdown()

    def __del__(self) -> None:  # best-effort cleanup
        try:
            self.close()
        except Exception:
            pass


def init(
    Nodename: Optional[str] = None,
    *,
    node_name: Optional[str] = None,
    offboard_namespace: str = "",
    fcu_namespace: str = "/fmu_calibration_control",
    led_namespace: str = "led",
) -> DroneInterfaces:
    """Initialize the library.

    Examples:

        import sverk_interfaces

        drone = sverk_interfaces.init(Nodename="drone_controll")
        drone.controll.navigate(x=0.0, y=0.0, z=1.5, yaw=0.0,
                                speed=1.0, frame_id="body", auto_arm=True)
        drone.fcu.force_disarm()
        if drone.led:
            drone.led.set_effect("fill", r=255, g=0, b=0)

    Parameters:
      - Nodename / node_name: name of the created rclpy node
      - offboard_namespace: namespace of offboard_control
        (default root → /navigate, /land, ...)
      - fcu_namespace: fmu_calibration_control namespace
        (default /fmu_calibration_control)
      - led_namespace: led_control node namespace (default led → /led/set_effect, /led/set_leds)
    """
    _ensure_rclpy_init()

    name = node_name or Nodename or "sverk_drone_client"
    node = rclpy.create_node(name)
    return DroneInterfaces(
        node,
        offboard_namespace=offboard_namespace,
        fcu_namespace=fcu_namespace,
        led_namespace=led_namespace,
    )


def _atexit_cleanup() -> None:
    # On process exit: destroy nodes and shutdown rclpy if the library started it.
    with _instances_lock:
        instances = list(_instances)
    for inst in instances:
        try:
            inst.close()
        except Exception:
            pass


atexit.register(_atexit_cleanup)


__all__ = [
    "DroneInterfaces",
    "LedAPI",
    "LED_EFFECTS",
    "OffboardControlAPI",
    "FmuCalibrationAPI",
    "GenericServiceFamily",
    "init",
]

