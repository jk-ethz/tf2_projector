import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import numpy as np
import transforms3d as t3d

class TransformNode(Node):
    def __init__(self):
        super().__init__("tf2_projector")
        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_subscription = self.create_subscription(
            tf2_msgs.msg.TFMessage,
            '/tf',
            self.tf_callback,
            1
        )
        self.declare_parameter("projection_source_root_frame", rclpy.Parameter.Type.STRING,
                               ParameterDescriptor(description="The common root frame at the top of the tree.")) # map_o3d
        self.declare_parameter("projection_source_frame", rclpy.Parameter.Type.STRING,
                               ParameterDescriptor(description="The frame to use as a projection source.")) # range_sensor_o3d
        self.declare_parameter("projection_target_root_frame",
                               self.get_parameter("projection_source_root_frame").get_parameter_value().string_value,
                               ParameterDescriptor(description="The root frame of the target, usually identical to projection_source_root_frame.")) # map (== map_o3d)
        self.declare_parameter("projection_target_frame", rclpy.Parameter.Type.STRING,
                               ParameterDescriptor(description="The frame to project to.")) # lidar
        self.declare_parameter("projection_target_attachment_frame", rclpy.Parameter.Type.STRING,
                               ParameterDescriptor(description="The frame that should be attached to the root frame by this node to do the projection.")) # odom
        self.transform_last = geometry_msgs.msg.TransformStamped()

    def tf_callback(self, tf_msg: tf2_msgs.msg.TFMessage):
        try:
            time = Time.from_msg(tf_msg.header.stamp)
            T_m_r = self.tf_buffer.lookup_transform(
                self.get_parameter("projection_source_root_frame").get_parameter_value().string_value,
                self.get_parameter("projection_source_frame").get_parameter_value().string_value,
                time
            )
            T_m_r_mat = np.eye(4)
            T_m_r_mat[:3, :3] = t3d.quaternions.quat2mat([
                T_m_r.transform.rotation.w,
                T_m_r.transform.rotation.x,
                T_m_r.transform.rotation.y,
                T_m_r.transform.rotation.z
            ])
            T_m_r_mat[:3, 3] = [
                T_m_r.transform.translation.x,
                T_m_r.transform.translation.y,
                T_m_r.transform.translation.z
            ]
            T_l_o = self.tf_buffer.lookup_transform(
                self.get_parameter("projection_target_frame").get_parameter_value().string_value,
                self.get_parameter("projection_target_attachment_frame").get_parameter_value().string_value,
                time
            )
            T_l_o_mat = np.eye(4)
            T_l_o_mat[:3, :3] = t3d.quaternions.quat2mat([
                T_l_o.transform.rotation.w,
                T_l_o.transform.rotation.x,
                T_l_o.transform.rotation.y,
                T_l_o.transform.rotation.z
            ])
            T_l_o_mat[:3, 3] = [
                T_l_o.transform.translation.x,
                T_l_o.transform.translation.y,
                T_l_o.transform.translation.z
            ]
            T_m_o_mat = np.matmul(T_m_r_mat, T_l_o_mat)
            T_m_o_trans = T_m_o_mat[:3, 3]
            T_m_o_rot = t3d.quaternions.mat2quat(T_m_o_mat[:3, :3])

            T_m_o = geometry_msgs.msg.TransformStamped()
            T_m_o.header.stamp = T_m_r.header.stamp
            T_m_o.header.frame_id = self.get_parameter("projection_target_root_frame").get_parameter_value().string_value
            T_m_o.child_frame_id = self.get_parameter("projection_target_attachment_frame").get_parameter_value().string_value
            T_m_o.transform.translation.x = T_m_o_trans[0]
            T_m_o.transform.translation.y = T_m_o_trans[1]
            T_m_o.transform.translation.z = T_m_o_trans[2]
            T_m_o.transform.rotation.x = T_m_o_rot[1]
            T_m_o.transform.rotation.y = T_m_o_rot[2]
            T_m_o.transform.rotation.z = T_m_o_rot[3]
            T_m_o.transform.rotation.w = T_m_o_rot[0]

            if (
                np.isclose(T_m_o.transform.translation.x, self.transform_last.transform.translation.x) and
                np.isclose(T_m_o.transform.translation.y, self.transform_last.transform.translation.y) and
                np.isclose(T_m_o.transform.translation.z, self.transform_last.transform.translation.z) and
                np.isclose(T_m_o.transform.rotation.x, self.transform_last.transform.rotation.x) and
                np.isclose(T_m_o.transform.rotation.y, self.transform_last.transform.rotation.y) and
                np.isclose(T_m_o.transform.rotation.z, self.transform_last.transform.rotation.z) and
                np.isclose(T_m_o.transform.rotation.w, self.transform_last.transform.rotation.w)
            ):
                return

            self.tf_broadcaster.sendTransform(T_m_o)
            self.transform_last = T_m_o

            self.get_logger().info("Transform ok", once=True)
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Transform lookup failed: {e}", throttle_duration_sec=1.0)
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"Transform extrapolation failed: {e}", throttle_duration_sec=1.0)
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f"Transform connectivity failed: {e}", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
