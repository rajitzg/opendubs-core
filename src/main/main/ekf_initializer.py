import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandHome

class EKFInitializer(Node):
    def __init__(self):
        super().__init__('ekf_initializer')
        self.client = self.create_client(CommandHome, '/mavros/cmd/set_home')

        self.declare_parameter("use_gps", False)
        self.declare_parameter("home_latitude", 0.0)
        self.declare_parameter("home_longitude", 0.0)
        self.declare_parameter("home_altitude", 0.0)

        request = CommandHome.Request()
        request.current_gps = self.get_parameter("use_gps").value
        request.latitude = self.get_parameter("home_latitude").value
        request.longitude = self.get_parameter("home_longitude").value
        request.altitude = self.get_parameter("home_altitude").value
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.callback)


    def callback(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to call /mavros/cmd/set_home: {exc}")
        else:
            if response.result == 0:
                self.get_logger().info(f"EKF home position set successfully.")
            else:
                self.get_logger().error("Failed to set EKF home position.")
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    ekf_initializer = EKFInitializer()
    try:
        rclpy.spin(ekf_initializer)
    finally:
        ekf_initializer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()