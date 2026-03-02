import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import IntEnum

from mavros_msgs.msg import RCIn
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from interfaces.srv import LoggerCommand


class ControlMode(IntEnum):
    MANUAL = 0
    VELOCITY = 1
    AUTO = 2


class ControllerInterfaceNode(Node):
    def __init__(self):
        super().__init__('controller_interface_node')

        # --- Parameters ---
        # Stick input channels (0-indexed)
        self.declare_parameter('input_ch_fwd', 1)
        self.declare_parameter('input_ch_lat', 0)
        self.declare_parameter('input_ch_yaw', 3)

        # Mode switching channel + PWM thresholds
        self.declare_parameter('input_ch_mode', 4)
        self.declare_parameter('mode_pwm_manual_threshold', 1300)
        self.declare_parameter('mode_pwm_auto_threshold', 1700)

        # Deadband (normalized 0..1)
        self.declare_parameter('input_deadband', 0.05)

        # Watchdog: throttle channel stays at a fixed value when controller is off
        self.declare_parameter('throttle_ch', 2)
        self.declare_parameter('throttle_idle_value', 900)

        # Velocity limits applied in VELOCITY mode (m/s and rad/s)
        # Stick at full deflection (2000 PWM) maps to these max values
        self.declare_parameter('vel_max_linear', 1.0)   # m/s
        self.declare_parameter('vel_max_angular', 1.57)  # rad/s (~90 deg/s)

        self._load_params()

        # --- State ---
        self._current_mode = ControlMode.MANUAL
        self._controller_connected = True  # assume connected until proven otherwise
        self._last_rc_time = self.get_clock().now()

        # --- Subscriptions ---
        rc_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._rc_sub = self.create_subscription(
            RCIn, '/mavros/rc/in', self._rc_callback, rc_qos)

        # --- Publishers ---
        self._cmd_vel_pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)

        # Transient local so ll_control receives the current mode when it starts up
        mode_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_pub = self.create_publisher(Int8, '/teleop/control_mode', mode_pub_qos)

        # Publish initial mode
        self._publish_mode(self._current_mode)

        self.get_logger().info('ControllerInterface node initialized.')

    # -----------------------------------------------------------------------
    # Parameter loading
    # -----------------------------------------------------------------------
    def _load_params(self):
        self._ch_fwd = self.get_parameter('input_ch_fwd').value
        self._ch_lat = self.get_parameter('input_ch_lat').value
        self._ch_yaw = self.get_parameter('input_ch_yaw').value
        self._ch_mode = self.get_parameter('input_ch_mode').value
        self._mode_pwm_manual = self.get_parameter('mode_pwm_manual_threshold').value
        self._mode_pwm_auto = self.get_parameter('mode_pwm_auto_threshold').value
        self._deadband = self.get_parameter('input_deadband').value
        self._throttle_ch = self.get_parameter('throttle_ch').value
        self._throttle_idle = self.get_parameter('throttle_idle_value').value
        self._vel_max_linear = self.get_parameter('vel_max_linear').value
        self._vel_max_angular = self.get_parameter('vel_max_angular').value

    # -----------------------------------------------------------------------
    # RC Callback
    # -----------------------------------------------------------------------
    def _rc_callback(self, msg: RCIn):
        ch = msg.channels
        n = len(ch)

        # --- Controller watchdog ---
        # When the RC transmitter is off, the throttle channel holds the idle value.
        # Any deviation from the idle value means the controller is live.
        if n > self._throttle_ch:
            controller_on = (ch[self._throttle_ch] != self._throttle_idle)
            if controller_on != self._controller_connected:
                self._controller_connected = controller_on
                self.get_logger().info(
                    f'Controller {"connected" if controller_on else "DISCONNECTED - halting cmd_vel."}'
                )

        # Only publish cmd_vel if the controller is live.
        # When disconnected, withholding the topic lets ll_control's cmd_vel_timeout
        # watchdog fire and switch ArduPilot to HOLD.
        if not self._controller_connected:
            return

        # Record the time we last received useful data (always update so ll_control
        # can detect a complete topic dropout as "stale").
        self._last_rc_time = self.get_clock().now()

        # --- Control Mode ---
        if n > self._ch_mode:
            mode_pwm = ch[self._ch_mode]
            if mode_pwm < self._mode_pwm_manual:
                new_mode = ControlMode.MANUAL
            elif mode_pwm > self._mode_pwm_auto:
                new_mode = ControlMode.AUTO
            else:
                new_mode = ControlMode.VELOCITY

            if new_mode != self._current_mode:
                self.get_logger().info(
                    f'Mode changed: {self._current_mode.name} -> {new_mode.name}')
                self._current_mode = new_mode
                self._publish_mode(self._current_mode)

        # --- Build Twist ---
        twist = Twist()
        req_chs = [self._ch_fwd, self._ch_lat, self._ch_yaw]
        if n > max(req_chs):
            norm_fwd = self._normalize(ch[self._ch_fwd])
            norm_lat = self._normalize(ch[self._ch_lat])
            norm_yaw = self._normalize(ch[self._ch_yaw])

            if self._current_mode == ControlMode.VELOCITY:
                # Scale to physical velocity targets (m/s, rad/s)
                twist.linear.x = norm_fwd * self._vel_max_linear
                twist.linear.y = norm_lat * self._vel_max_linear
                twist.angular.z = norm_yaw * self._vel_max_angular
            else:
                # MANUAL / AUTO: keep normalized (-1..1); ll_control converts to PWM
                twist.linear.x = norm_fwd
                twist.linear.y = norm_lat
                twist.angular.z = norm_yaw

        self._cmd_vel_pub.publish(twist)

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------
    def _normalize(self, pwm: int) -> float:
        """Convert PWM (1000..2000) to normalized (-1..1) with deadband."""
        norm = (float(pwm) - 1500.0) / 500.0
        norm = max(-1.0, min(1.0, norm))
        if abs(norm) < self._deadband:
            return 0.0
        return norm

    def _publish_mode(self, mode: ControlMode):
        msg = Int8()
        msg.data = int(mode)
        self._mode_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
