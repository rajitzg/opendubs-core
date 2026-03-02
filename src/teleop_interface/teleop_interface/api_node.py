import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading

from interfaces.srv import LoggerCommand
from teleop_interface.api_router import router as logger_router

# --- FastAPI App ---
app = FastAPI(title="Robot Teleop API", description="REST API for Robot Controls")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the endpoints defined in api_router.py
app.include_router(logger_router)

# Global reference to the ROS 2 node so FastAPI endpoints can use it
ros_node = None


class APINode(Node):
    def __init__(self):
        super().__init__('api_node')

        # --- Parameters ---
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('logger_service_name', 'record_data')

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.logger_service_name = self.get_parameter('logger_service_name').value

        # --- Logger Service Client ---
        self._logger_client = self.create_client(
            LoggerCommand, self.logger_service_name)

        self.get_logger().info('API Node initialized.')

    def send_logger_command(self, command: int) -> bool:
        """Call the LoggerCommand service and wait for the result."""
        if not self._logger_client.service_is_ready():
            self.get_logger().warning(
                f'Logger service "{self.logger_service_name}" not available.')
            return False

        request = LoggerCommand.Request()
        request.command = command
        
        # We must call_async because the ROS 2 executor is spinning in a separate thread.
        # We cannot use spin_until_future_complete because the node is already spinning.
        # We use a threading event to block the FastAPI request until the service finishes.
        future = self._logger_client.call_async(request)
        
        event = threading.Event()
        result_box = []
        
        def _done(f):
            try:
                result_box.append(f.result())
            except Exception:
                result_box.append(None)
            event.set()
            
        future.add_done_callback(_done)
        
        # Wait up to 2.0 seconds for the service response
        event.wait(timeout=2.0)
        
        if result_box and result_box[0] is not None:
             return result_box[0].success
        return False


def ros2_thread_func(node):
    """Function to run the ROS 2 executor in a separate thread."""
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    ros_node = APINode()
    
    # Inject the ROS node into the FastAPI app state so routers can access it
    app.state.ros_node = ros_node

    # Start ROS 2 spinning in a background thread
    ros_thread = threading.Thread(target=ros2_thread_func, args=(ros_node,), daemon=True)
    ros_thread.start()

    try:
        # Run FastAPI on the main thread
        ros_node.get_logger().info(f'Starting FastAPI server on {ros_node.host}:{ros_node.port}')
        
        # NOTE: uvicorn.run when passed 'app' directly cannot take reload=True in all environments easily
        # For a ROS node, we usually just run the app directly
        uvicorn.run(app, host=ros_node.host, port=ros_node.port, log_level="info")
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()