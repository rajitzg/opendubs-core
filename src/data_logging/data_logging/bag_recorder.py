from interfaces.srv import LoggerCommand

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from datetime import datetime
import os
import yaml 
import subprocess
import signal
import shutil

class BagRecorder(Node):
    def __init__(self):
        super().__init__("bag_recorder")

        # Grab parameters
        self.declare_parameter("bag_path", "src/data_logging/rosbags")
        self.declare_parameter("service_topic", "logger_command")
        self.declare_parameter("max_record_duration", 300) # seconds
        self.declare_parameter("log_topics", [""])

        self.bag_path = self.get_parameter("bag_path")\
            .get_parameter_value().string_value
        self.service_topic = self.get_parameter("service_topic")\
            .get_parameter_value().string_value
        self.max_record_duration = self.get_parameter("max_record_duration")\
            .get_parameter_value().integer_value
        log_topics = self.get_parameter("log_topics")\
            .get_parameter_value().string_array_value

        # Remove any empty strings
        self.valid_topics = [t for t in log_topics if t]
        if not self.valid_topics:
            self.get_logger().warning("No valid topics specified for recording.")

        # Ensure bag path exists
        os.makedirs(self.bag_path, exist_ok=True)

        # State
        self.recording = False
        self.rosbag_proc = None

        # Service
        self.create_service(LoggerCommand, self.service_topic, self.service_callback)

        # Watch for runtime parameter changes
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info("Bag recorder ready.")

    # Parameter change callback
    def _on_set_parameters(self, params):
        for param in params:
            if param.name == "log_topics" and param.type_ == Parameter.Type.STRING_ARRAY:
                new_topics = [t for t in param.value if t]
                self.valid_topics = new_topics
                if new_topics:
                    self.get_logger().info(f"Updated log_topics: {new_topics}")
                else:
                    self.get_logger().warning("log_topics updated but no valid topics specified.")
        return SetParametersResult(successful=True)

    # Service callbacks
    def service_callback(self, request, response):
        if request.command == LoggerCommand.Request.START_RECORDING and not self.recording:
            self.start_recording()
            response.success = True
        elif request.command == LoggerCommand.Request.STOP_AND_DISCARD_RECORDING and self.recording:
            self.stop_and_discard_recording()
            response.success = True
        elif request.command == LoggerCommand.Request.STOP_AND_SAVE_RECORDING and self.recording:
            self.stop_and_save_recording()
            self.stop_recording()
            response.success = True
        else:
            self.get_logger().warning("Invalid command or state for recording.")
            response.success = False

        return response

    # Recording logic
    def start_recording(self):
        self.get_logger().info("Starting rosbag recording.")

        command = [
            f"ros2", f"bag", f"record",
            f"--max-bag-duration", f"{self.max_record_duration}",
            f"-o", f"{self.bag_path}/temp",
        ] + self.valid_topics

        self.rosbag_proc = subprocess.Popen(command)
        self.recording = True

    def stop_and_save_recording(self):
        self.stop_recording()
        self.get_logger().info("Saving rosbag recording.")

        # Rename temp bag directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        new_name = f"bag_{timestamp}"

        os.rename(
            os.path.join(self.bag_path, "temp"),
            os.path.join(self.bag_path, new_name)
        )

    def stop_and_discard_recording(self):
        self.stop_recording()
        self.get_logger().info("Discarding rosbag recording.")

        tmp_path = os.path.join(self.bag_path, "temp")
        if os.path.exists(tmp_path):
            shutil.rmtree(tmp_path)

    def stop_recording(self):
        self.get_logger().info("Stopping recording.")
        if self.rosbag_proc:
            self.rosbag_proc.send_signal(signal.SIGINT)
            self.rosbag_proc.wait()
            self.rosbag_proc = None

        self.recording = False


def main(args=None): 
    rclpy.init() 
    node = BagRecorder() 
    
    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        pass 
    finally: 
        node.stop_and_discard_recording()
        node.destroy_node() 
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()