import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l1x import VL53L1X
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import time

class MultiVL53Publisher(Node):
    def __init__(self):
        super().__init__('multi_vl53_node')

        # Default empty array. Empty = single-sensor mode without XSHUT.
        # Allow ROS 2 to change parameter type dynamically
        param_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('xshut_pins', [], param_desc)

        # Read value as a plain Python list (.value)
        pin_numbers = self.get_parameter('xshut_pins').value

        # Guard if ROS 2 returns None
        if pin_numbers is None:
            pin_numbers = []

        # I2C init
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.get_logger().info('I2C bus initialized')
        except Exception as e:
            self.get_logger().error(f'I2C error: {e}')
            return

        self.sensors = []
        self.pubs = []

        # Single auto-detect OR multi-sensor mode
        if not pin_numbers:
            self.get_logger().info('No XSHUT pins: auto-detecting a single sensor...')
            self.init_single_auto_sensor()
        else:
            self.get_logger().info(f'Multi-sensor init for pins: {pin_numbers}')
            # Create publishers for multi mode
            for i in range(len(pin_numbers)):
                topic = f'sensor_{i+1}/range'
                self.pubs.append(self.create_publisher(Float32, topic, 10))
            self.init_multi_sensors(pin_numbers)

        # Poll timer (20 Hz)
        if self.sensors:
            self.timer = self.create_timer(0.05, self.timer_callback)
        else:
            self.get_logger().error('No sensor initialized. Timer not started.')

    def init_single_auto_sensor(self):
        while not self.i2c.try_lock():
            pass
        try:
            addresses = self.i2c.scan()
        finally:
            self.i2c.unlock()

        if not addresses:
            self.get_logger().error('No I2C devices found. Check wiring.')
            return

        self.get_logger().info(f'Devices at addresses: {[hex(a) for a in addresses]}')

        # Try each address for a VL53L1X
        for addr in addresses:
            try:
                self.get_logger().info(f'Trying {hex(addr)}...')
                vl = VL53L1X(self.i2c, address=addr)

                vl.distance_mode = 2  # Long range
                vl.timing_budget = 50
                vl.start_ranging()

                self.sensors.append(vl)
                self.pubs.append(self.create_publisher(Float32, 'sensor_1/range', 10))

                self.get_logger().info(f'Sensor started at {hex(addr)}')
                return
            except Exception as e:
                self.get_logger().debug(f'Device at {hex(addr)} is not VL53L1X or unavailable: {e}')

        self.get_logger().error('Failed to init VL53L1X on any scanned address.')

    def init_multi_sensors(self, pin_numbers):
        xshuts = []

        # Prepare XSHUT pins
        for pin_num in pin_numbers:
            try:
                pin_name = f'D{pin_num}'
                board_pin = getattr(board, pin_name)

                x_pin = DigitalInOut(board_pin)
                x_pin.direction = Direction.OUTPUT
                x_pin.value = False
                xshuts.append(x_pin)
            except AttributeError:
                self.get_logger().error(f'Pin {pin_num} not found on board!')

        time.sleep(0.5)

        base_address = 0x30

        for i, x_pin in enumerate(xshuts):
            x_pin.value = True
            time.sleep(0.1)

            try:
                new_addr = base_address + i
                vl = VL53L1X(self.i2c)
                vl.set_address(new_addr)

                vl.distance_mode = 2
                vl.timing_budget = 50
                vl.start_ranging()

                self.sensors.append(vl)
                self.get_logger().info(f'Sensor {i+1} ready: XSHUT pin={pin_numbers[i]}, addr={hex(new_addr)}')
            except Exception as e:
                self.get_logger().error(f'Failed to start sensor on pin {pin_numbers[i]}: {e}')
                x_pin.value = False

    def timer_callback(self):
        for i, sensor in enumerate(self.sensors):
            try:
                if sensor.data_ready:
                    curr_dist = sensor.distance
                    if curr_dist is not None:
                        msg = Float32()
                        msg.data = float(curr_dist / 100.0)
                        self.pubs[i].publish(msg)
                    sensor.clear_interrupt()
            except Exception as e:
                self.get_logger().warn(f'Sensor {i+1} read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiVL53Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        for sensor in node.sensors:
            try:
                sensor.stop_ranging()
            except:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
