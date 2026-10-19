"""
NexusOS ROS 2 Desktop Bridge
Bridges NexusOS HAL telemetry and motor commands directly to ROS 2 topics for native RViz2 and rqt visualization.
"""

from typing import Optional, Any
from hal.base import RobotTelemetryData, MotorCommandData, TwistData, Vector3D

rclpy: Any = None
Node: Any = None
PoseStamped: Any = None
Twist: Any = None
JointState: Any = None

try:
    import rclpy as _rclpy  # pyright: ignore[reportMissingImports]
    from rclpy.node import Node as _Node  # pyright: ignore[reportMissingImports]
    from geometry_msgs.msg import PoseStamped as _PoseStamped, Twist as _Twist  # pyright: ignore[reportMissingImports]
    from sensor_msgs.msg import JointState as _JointState  # pyright: ignore[reportMissingImports]
    rclpy = _rclpy
    Node = _Node
    PoseStamped = _PoseStamped
    Twist = _Twist
    JointState = _JointState
    ROS2_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    ROS2_AVAILABLE = False


class NexusROS2BridgeNode:
    """
    ROS 2 Bridge Node for NexusOS.
    Publishes NexusOS HAL telemetry to ROS 2 topics (/pose, /joint_states, /cmd_vel)
    allowing instant visualization in RViz2, rqt, and ROS 2 desktop tools.
    """

    def __init__(self, robot_id: str):
        if not ROS2_AVAILABLE or rclpy is None:
            raise RuntimeError(
                "rclpy is not installed in the current environment. "
                "Install ROS 2 (Humble/Iron/Jazzy) to use native RViz2 / rqt desktop visualization."
            )
        rclpy.init()
        self.node = Node(f"nexus_bridge_{robot_id}")
        self.robot_id = robot_id

        # ROS 2 Publishers for RViz2 / rqt
        self.pose_pub = self.node.create_publisher(PoseStamped, f"/{robot_id}/pose", 10)
        self.joint_pub = self.node.create_publisher(JointState, f"/{robot_id}/joint_states", 10)
        self.cmd_pub = self.node.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)

    def publish_telemetry(self, telemetry: RobotTelemetryData):
        """Publish NexusOS HAL telemetry to ROS 2 topics for RViz2."""
        if rclpy is None:
            return

        # 1. Publish PoseStamped for RViz2 3D Robot Trajectory
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(telemetry.pose.position.x)
        pose_msg.pose.position.y = float(telemetry.pose.position.y)
        pose_msg.pose.position.z = float(telemetry.pose.position.z)
        self.pose_pub.publish(pose_msg)

        # 2. Publish JointState for RViz2 Robot Manipulator / Arm Model
        if telemetry.joints and telemetry.joints.names:
            joint_msg = JointState()
            joint_msg.header.stamp = self.node.get_clock().now().to_msg()
            joint_msg.name = telemetry.joints.names
            joint_msg.position = [float(p) for p in telemetry.joints.positions]
            joint_msg.velocity = [float(v) for v in telemetry.joints.velocities]
            self.joint_pub.publish(joint_msg)

        # 3. Spin once to process ROS 2 callbacks
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        self.node.destroy_node()
        if rclpy is not None:
            rclpy.shutdown()
