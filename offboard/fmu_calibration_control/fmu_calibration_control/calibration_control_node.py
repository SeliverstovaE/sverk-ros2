#!/usr/bin/env python3
"""
PX4 calibration and quick disarm via /fmu/in/vehicle_command (uXRCE-DDS).
See: https://docs.px4.io/main/en/middleware/dds_topics
     px4_msgs VehicleCommand.msg (VEHICLE_CMD_PREFLIGHT_CALIBRATION=241, VEHICLE_CMD_COMPONENT_ARM_DISARM=400)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand
from std_srvs.srv import Trigger
from std_msgs.msg import UInt8


# MAV_CMD_PREFLIGHT_CALIBRATION, param1 (MAVLink / PX4)
CALIBRATION_GYRO = 0
CALIBRATION_MAG = 1
CALIBRATION_BARO = 2
CALIBRATION_TEMPERATURE = 3  # PX4 PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION
CALIBRATION_ACCEL = 4
CALIBRATION_LEVEL = 5  # accel level / horizon

# Force disarm: ARM_DISARM param2 (bypass not-landed check)
MAV_MODE_FLAG_FORCE_DISARM = 21196


class CalibrationControlNode(Node):
    def __init__(self):
        super().__init__('fmu_calibration_control')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos,
        )

        self._disarm_srv = self.create_service(
            Trigger,
            '/fmu_calibration_control/disarm',
            self._handle_disarm,
        )
        self._force_disarm_srv = self.create_service(
            Trigger,
            '/fmu_calibration_control/force_disarm',
            self._handle_force_disarm,
        )
        self._kill_switch_srv = self.create_service(
            Trigger,
            '/fmu_calibration_control/kill_switch',
            self._handle_kill_switch,
        )

        self._calibrate_sub = self.create_subscription(
            UInt8,
            '/fmu_calibration_control/request_calibration',
            self._handle_calibration_request,
            10,
        )

        self.get_logger().info(
            'fmu_calibration_control: disarm, force_disarm, kill_switch; '
            'topic /fmu_calibration_control/request_calibration (UInt8: 0=gyro,1=mag,2=baro,3=temp,4=accel,5=level)'
        )

    def _timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_pub.publish(msg)

    def _handle_disarm(self, _request, response):
        """Normal disarm (PX4 accepts only when landed)."""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.get_logger().info('Disarm command sent (use force_disarm or kill_switch if "not landed")')
        response.success = True
        response.message = 'Disarm command sent'
        return response

    def _handle_force_disarm(self, _request, response):
        """Force disarm: ARM_DISARM with param2=MAV_MODE_FLAG_FORCE_DISARM (bypass 'not landed')."""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=float(MAV_MODE_FLAG_FORCE_DISARM),
        )
        self.get_logger().info('Force disarm command sent')
        response.success = True
        response.message = 'Force disarm command sent'
        return response

    def _handle_kill_switch(self, _request, response):
        """Kill switch: VEHICLE_CMD_DO_FLIGHTTERMINATION — immediate motor stop (e.g. in air)."""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION,
            param1=1.0,
        )
        self.get_logger().info('Kill switch (flight termination) sent')
        response.success = True
        response.message = 'Kill switch sent'
        return response

    def _handle_calibration_request(self, msg: UInt8):
        """Run preflight calibration: VEHICLE_CMD_PREFLIGHT_CALIBRATION with param1 = type."""
        cal_type = msg.data
        if cal_type > CALIBRATION_LEVEL:
            self.get_logger().warn(f'Unknown calibration type {cal_type}, use 0..5')
            return
        names = ('gyro', 'mag', 'baro', 'temperature', 'accel', 'level')
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_CALIBRATION,
            param1=float(cal_type),
        )
        self.get_logger().info(f'Calibration command sent: {names[cal_type]} (param1={cal_type})')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
