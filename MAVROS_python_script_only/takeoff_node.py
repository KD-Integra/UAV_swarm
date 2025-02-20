#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from mavros_msgs.srv import CommandBool      # For arming
from mavros_msgs.srv import SetMode         # For setting mode
from mavros_msgs.srv import CommandTOL      # For takeoff

class TakeoffNode(Node):
    def __init__(self, drone_ns='drone1'):
        super().__init__('takeoff_node')

        # Compose full service name using the drone_ns
        set_mode_srv = f'/{drone_ns}/set_mode'
        arm_srv = f'/{drone_ns}/cmd/arming'
        takeoff_srv = f'/{drone_ns}/cmd/takeoff'

        self.get_logger().info(f"Using namespace: {drone_ns}")

        # Create clients for the required services
        self.set_mode_client = self.create_client(SetMode, set_mode_srv)
        self.arm_client = self.create_client(CommandBool, arm_srv)
        self.takeoff_client = self.create_client(CommandTOL, takeoff_srv)


        # Wait for each service to become available
        self.get_logger().info("Waiting for set_mode service...")
        self.set_mode_client.wait_for_service()
        self.get_logger().info("Waiting for arm service...")
        self.arm_client.wait_for_service()
        self.get_logger().info("Waiting for takeoff service...")
        self.takeoff_client.wait_for_service()

        self.get_logger().info("All services available. Proceeding with commands...")

        # 1. Set mode to GUIDED
        self.set_guided_mode()

        # 2. Arm the drone
        self.arm()

        # 3. Take off to altitude = 5 (meters)
        self.takeoff(altitude=5.0)

    def set_guided_mode(self):
        request = SetMode.Request()
        request.custom_mode = "GUIDED"
        # base_mode is ignored by ArduPilot if custom_mode is set, but set to 0 for safety
        request.base_mode = 0

        self.get_logger().info("Setting mode to GUIDED...")
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info("GUIDED mode set successfully!")
            else:
                self.get_logger().error("Failed to set GUIDED mode")
        else:
            self.get_logger().error("No response from SetMode service")

    def arm(self):
        request = CommandBool.Request()
        request.value = True

        self.get_logger().info("Arming the drone...")
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("Drone armed successfully!")
            else:
                self.get_logger().error(f"Failed to arm: result = {future.result().result}")
        else:
            self.get_logger().error("No response from Arming service")

    def takeoff(self, altitude=5.0):
        request = CommandTOL.Request()
        request.altitude = altitude
        # For ArduCopter SITL, latitude, longitude, yaw can be zero if you just want a straight takeoff
        request.latitude = 0.0
        request.longitude = 0.0
        request.yaw = 0.0
        request.min_pitch = 0.0

        self.get_logger().info(f"Taking off to {altitude} meters...")
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"Takeoff command sent successfully!")
            else:
                self.get_logger().error(f"Failed takeoff: result = {future.result().result}")
        else:
            self.get_logger().error("No response from Takeoff service")


def main(args=None):
    rclpy.init(args=args)

    # Option A: Hard-code the namespace in code
    # node = TakeoffNode(drone_ns='drone1')

    # Option B: Grab the namespace from command-line arguments or a parameter
    import sys
    drone_ns = 'drone1'
    if len(sys.argv) > 1:
        drone_ns = sys.argv[1]
    node = TakeoffNode(drone_ns)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
