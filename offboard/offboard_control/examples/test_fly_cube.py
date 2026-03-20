#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import threading
from offboard_interfaces.srv import Navigate, GetTelemetry
from std_srvs.srv import Trigger
import sys

# ============================================
# GLOBAL CUBE MISSION PARAMETERS
# ============================================
CUBE_SIDE_LENGTH = 0.5      # Cube edge length (m)
CUBE_HEIGHT = 0.5          # Cube height (m), Z offset between lower and upper face
POINT_DELAY = 5            # Delay between waypoints (s)
TAKEOFF_HEIGHT = 0.5       # Takeoff height (m)
FLIGHT_SPEED = 0.5         # Flight speed (m/s)
# ============================================

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Service clients
        self.navigate_client = self.create_client(Navigate, '/navigate')
        self.land_client = self.create_client(Trigger, '/land')
        self.telemetry_client = self.create_client(GetTelemetry, '/get_telemetry')
        
        # Wait for services
        self.wait_for_services()
        
        self.get_logger().info('All services are ready')
        self._lock = threading.Lock()  # Thread-safe service calls

    def wait_for_services(self):
        """Wait until all services are available."""
        services = [
            (self.navigate_client, 'Navigate'),
            (self.land_client, 'Land'),
            (self.telemetry_client, 'Telemetry')
        ]
        
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit("ROS2 interrupted, exiting...")
                self.get_logger().info(f'{name} service not available, waiting...')

    def call_service(self, client, request):
        """Thread-safe service call."""
        with self._lock:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {str(e)}')
                    return None
            else:
                self.get_logger().error('Service call timed out')
                return None

    def navigate(self, x, y, z, yaw=0.0, speed=0.5, auto_arm=False, frame_id="body"):
        """Send navigate command."""
        request = Navigate.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        request.yaw = float(yaw)
        request.speed = float(speed)
        request.auto_arm = bool(auto_arm)
        request.frame_id = str(frame_id)
        
        response = self.call_service(self.navigate_client, request)
        if response:
            self.get_logger().info(f'Navigate response: success={response.success}, message="{response.message}"')
            return response.success
        return False

    def land(self):
        """Send land command."""
        request = Trigger.Request()
        response = self.call_service(self.land_client, request)
        if response:
            self.get_logger().info(f'Land response: success={response.success}, message="{response.message}"')
            return response.success
        return False

    def get_telemetry(self):
        """Request telemetry."""
        request = GetTelemetry.Request()
        response = self.call_service(self.telemetry_client, request)
        
        if response and response.connected:
            mode = response.mode if hasattr(response, 'mode') else 'UNKNOWN'
            armed = response.armed if hasattr(response, 'armed') else False
            telemetry_str = f"Telemetry: mode={mode}, armed={armed}, "
            telemetry_str += f"x={response.x:.3f}, y={response.y:.3f}, z={response.z:.3f}, "
            telemetry_str += f"yaw={response.yaw:.3f}, vx={response.vx:.3f}, vy={response.vy:.3f}, vz={response.vz:.3f}, "
            telemetry_str += f"voltage={response.voltage:.2f}V"
            self.get_logger().info(telemetry_str)
            
            return {
                'x': response.x,
                'y': response.y,
                'z': response.z,
                'yaw': response.yaw,
                'vx': response.vx,
                'vy': response.vy,
                'vz': response.vz,
                'mode': mode,
                'armed': armed,
                'voltage': response.voltage,
                'connected': True
            }
        else:
            self.get_logger().warn("Drone not connected or telemetry failed")
            return {'connected': False}

def telemetry_loop(node):
    """Background loop: periodic telemetry requests."""
    while rclpy.ok():
        try:
            node.get_telemetry()
            time.sleep(1.0)  # Telemetry once per second
        except Exception as e:
            node.get_logger().error(f'Telemetry error: {str(e)}')
            time.sleep(2.0)  # Back off on error

def main():
    rclpy.init()
    drone = DroneController()
    
    # Start telemetry thread
    telemetry_thread = threading.Thread(target=lambda: telemetry_loop(drone), daemon=True)
    telemetry_thread.start()
    
    try:
        print("\n=== Drone Control Script Started ===")
        print("Press Ctrl+C to stop at any time\n")
        
        # Short settle time
        time.sleep(2)
        
        # 1. Takeoff to target height
        print(f"\n>>> Taking off to {TAKEOFF_HEIGHT} meters")
        success = drone.navigate(0.0, 0.0, TAKEOFF_HEIGHT, yaw=0.0, speed=1.0, auto_arm=True, frame_id="body")
        if not success:
            print("ERROR: Takeoff failed")
            return
        
        # Wait for takeoff
        print("Waiting for takeoff to complete...")
        time.sleep(15)
        
        # 2. Cube trajectory
        print(f"\n>>> Starting cube trajectory flight")
        print(f"Cube parameters: side={CUBE_SIDE_LENGTH}m, height={CUBE_HEIGHT}m")
        print(f"Point delay: {POINT_DELAY}s, Flight speed: {FLIGHT_SPEED}m/s")
        
        # Cube waypoints relative to current pose (body); each step is delta from PREVIOUS pose
        cube_points = [
            # Lower face (at TAKEOFF_HEIGHT)
            (CUBE_SIDE_LENGTH, 0.0, 0.0),      # Forward
            (0.0, CUBE_SIDE_LENGTH, 0.0),      # Left
            (-CUBE_SIDE_LENGTH, 0.0, 0.0),     # Back
            (0.0, -CUBE_SIDE_LENGTH, 0.0),     # Right (close lower square)
            
            # Move to upper face
            (0.0, 0.0, CUBE_HEIGHT),           # Up
            
            # Upper face (TAKEOFF_HEIGHT + CUBE_HEIGHT)
            (CUBE_SIDE_LENGTH, 0.0, 0.0),      # Forward
            (0.0, CUBE_SIDE_LENGTH, 0.0),      # Left
            (-CUBE_SIDE_LENGTH, 0.0, 0.0),     # Back
            (0.0, -CUBE_SIDE_LENGTH, 0.0),     # Right (close upper square)
            
            # Back to start height
            (0.0, 0.0, -CUBE_HEIGHT),          # Down
        ]
        
        # Fly cube waypoints
        for i, (x_rel, y_rel, z_rel) in enumerate(cube_points):
            print(f"\n>>> Moving to point {i+1}/{len(cube_points)}: "
                  f"relative offset x={x_rel:.2f}, y={y_rel:.2f}, z={z_rel:.2f}")
            success = drone.navigate(x_rel, y_rel, z_rel, yaw=0.0, speed=FLIGHT_SPEED, 
                                   auto_arm=False, frame_id="body")
            if not success:
                print(f"ERROR: Failed to reach point {i+1}")
                # Continue mission on failure
                continue
            
            # Wait at waypoint
            print(f"Waiting to reach point {i+1}...")
            time.sleep(POINT_DELAY)
            
        # 3. Land
        print("\n>>> Landing")
        success = drone.land()
        if success:
            print("Landing command sent successfully")
            print("Waiting for landing to complete...")
            time.sleep(10)  # Wait for landing
        else:
            print("ERROR: Landing command failed")
            
        print("\n=== Flight completed successfully ===")
        
    except KeyboardInterrupt:
        print("\n>>> Operation interrupted by user")
        print("Attempting to land...")
        try:
            drone.land()
            time.sleep(5)
        except:
            pass
        
    except Exception as e:
        print(f"ERROR: Unexpected error occurred: {str(e)}")
        print("Attempting emergency landing...")
        try:
            drone.land()
        except:
            pass
            
    finally:
        # Shutdown
        print("\nShutting down...")
        drone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()