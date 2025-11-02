import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Imu  # Import the Imu message type
import math
import numpy as np
import json  # Import the json module

# can change one index angle
# chnage the angle of all joints
# can increase or decrease a single joint angle


class ArmCummuteNode(Node):
    def __init__(self, arm_params, arm_angle_control):
        super().__init__("arm_commute_node")
        self.arm_angle_control = arm_angle_control
        # Load parameters first
        self.arm_params = arm_params.get_arm_params()

        # Initialize arm parameters publisher
        self.arm_pub = self.create_publisher(
            JointTrajectoryPoint, self.arm_params["global"]["arm_topic"], 10
        )

        # --- Add IMU Subscriber ---
        self.latest_imu_data = None
        self.imu_sub = self.create_subscription(
            Imu,
            self.arm_params["global"][
                "imu_receive_topic"
            ],  # Get topic name from config
            self.imu_callback,
            10,
        )
        self.get_logger().info(
            f"Subscribing to IMU topic: {self.arm_params['global']['imu_receive_topic']}"
        )
        # --------------------------

        # --- Add yolo object offset Subscriber ---
        self.object_coordinates = {}
        self.yolo_object_offset_sub = self.create_subscription(
            String,
            self.arm_params["global"][
                "yolo_object_offset_receive_topic"
            ],  # Get topic name from config
            self.yolo_object_offset_callback,
            10,
        )
        self.get_logger().info(
            f"Subscribing to IMU topic: {self.arm_params['global']['imu_receive_topic']}"
        )
        # --------------------------

    def imu_callback(self, msg: Imu):
        """Callback function for processing incoming IMU data."""
        # Example: Log the orientation quaternion
        orientation = msg.orientation
        self.latest_imu_data = msg
        # You can add more processing here, e.g., converting quaternion to Euler angles
        # or using linear acceleration/angular velocity data.

    def get_latest_imu_data(self):
        """
        回傳收到的最新 IMU 方向四元數 [x, y, z, w]。
        如果還沒收到過就回傳 None。
        """
        orientation = self.latest_imu_data.orientation
        return [orientation.x, orientation.y, orientation.z, orientation.w]

    def yolo_object_offset_callback(self, msg: String):
        """Callback function for processing incoming YOLO object offset data."""
        try:
            # Extract the JSON string from the message data
            json_string = msg.data
            # Parse the JSON string into a Python list of dictionaries
            object_list = json.loads(json_string)

            # Create a new dictionary mapping labels to coordinates
            new_coordinates = {}
            for item in object_list:
                if isinstance(item, dict) and "label" in item and "offset_flu" in item:
                    label = item["label"]
                    coordinates = item["offset_flu"]
                    # Ensure coordinates are a list of floats
                    if isinstance(coordinates, list) and len(coordinates) == 3:
                        try:
                            float_coords = [float(c) for c in coordinates]
                            new_coordinates[label] = float_coords
                        except (ValueError, TypeError):
                            self.get_logger().warn(
                                f"Invalid coordinate format for label '{label}': {coordinates}"
                            )
                    else:
                        self.get_logger().warn(
                            f"Unexpected coordinate format for label '{label}': {coordinates}"
                        )
                else:
                    self.get_logger().warn(
                        f"Skipping invalid item in JSON list: {item}"
                    )

            # Update the stored coordinates
            self.object_coordinates = new_coordinates
            # self.get_logger().info(
            #     f"Updated object coordinates: {self.object_coordinates}"
            # )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON string: {e}")
            self.get_logger().error(f"Received string: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing YOLO offset message: {e}")

    def get_latest_object_coordinates(self, label: str = None) -> dict:
        """
        回傳解析後的 YOLO 物體偏移字典，
        格式 { label: [x, y, z], … }，
        若還沒收到就回空 dict。
        """
        if label is None:
            # 全部回傳
            return self.object_coordinates
        # 單一物體回傳
        return self.object_coordinates.get(label, None)

    def degrees_to_radians(self, degree_positions):
        """Convert a list of positions from degrees to radians using NumPy

        Args:
            degree_positions (list): Joint positions in degrees

        Returns:
            list: Joint positions in radians
        """
        try:
            # Convert to numpy array, then use np.deg2rad for efficient conversion
            positions_array = np.array(degree_positions, dtype=float)
            radian_positions = np.deg2rad(positions_array).tolist()
            return radian_positions
        except (ValueError, TypeError) as e:
            # Fall back to element-by-element conversion if array conversion fails
            self.get_logger().warn(
                f"Could not convert all values at once: {e}, falling back to individual conversion"
            )
            radian_positions = []
            for pos in degree_positions:
                try:
                    radian_positions.append(float(pos) * math.pi / 180.0)
                except (ValueError, TypeError):
                    self.get_logger().error(f"Invalid angle value: {pos}")
                    radian_positions.append(0.0)
            return radian_positions

    def publish_arm_angle(self):
        """Publish the current arm joint angles"""
        joint_positions = self.arm_angle_control.get_arm_angles()
        msg = JointTrajectoryPoint()
        radian_positions = self.degrees_to_radians(joint_positions)
        msg.positions = radian_positions
        msg.velocities = []
        msg.accelerations = []
        msg.effort = []
        msg.time_from_start.sec = 0
        msg.time_from_start.nanosec = 0
        self.arm_pub.publish(msg)
        self.get_logger().info(f"Publish msg: {msg}")
        # self.get_logger().info(f"Published angles in radians: {radian_positions}")
        # self.get_logger().info(f"Published angles to the topic: {self.arm_params['global']['arm_topic']}")
