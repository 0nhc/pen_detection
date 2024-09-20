import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformListener, Buffer
import tf2_ros
import tf_transformations
from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge
import cv2
import numpy as np


class PX100Node(Node):
    def __init__(self):
        super().__init__("px100_node")
        
        self.declare_parameter('timer_frequency', 100.0) # hz
        self._timer_frequency = self.get_parameter('timer_frequency').value
        
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self._main_loop_timer = self.create_timer(1.0/self._timer_frequency, self._main_loop_timer_callback)
        
        
    def _main_loop_timer_callback(self):
        cam2aruco = None
        cam2aruco_received = False
        cam2pen = None
        cam2pen_received = False
        arm2aruco = None
        arm2aruco_received = False
        
        parent_frame = 'camera_color_optical_frame'
        child_frame = 'aruco_marker_frame'
        try:
            now = rclpy.time.Time()
            cam2aruco = self._transform_to_matrix(self._tf_buffer.lookup_transform(
                parent_frame, child_frame, now
            ))
            cam2aruco_received = True
        except Exception as e:
            # self.get_logger().warn(f"Could not transform {from_frame} to {to_frame}: {str(e)}")
            pass
            
        
        parent_frame = 'camera_color_optical_frame'    
        child_frame = 'pen_frame'
        try:
            now = rclpy.time.Time()
            cam2pen = self._transform_to_matrix(self._tf_buffer.lookup_transform(
                parent_frame, child_frame, now
            ))
            cam2pen_received = True
        except Exception as e:
            # self.get_logger().warn(f"Could not transform {from_frame} to {to_frame}: {str(e)}")
            pass
        
        parent_frame = 'px100/base_link'    
        child_frame = 'aruco_marker_frame'
        try:
            now = rclpy.time.Time()
            arm2aruco = self._transform_to_matrix(self._tf_buffer.lookup_transform(
                parent_frame, child_frame, now
            ))
            arm2aruco_received = True
        except Exception as e:
            # self.get_logger().warn(f"Could not transform {from_frame} to {to_frame}: {str(e)}")
            pass
        
        if(cam2aruco_received and cam2pen_received and arm2aruco_received):
            aruco2pen = np.dot(np.linalg.inv(cam2aruco), cam2pen)
            arm2pen = np.dot(arm2aruco, aruco2pen)
            
            x = arm2pen[0,3]
            y = arm2pen[1,3]
            z = arm2pen[2,3] + 0.05
            roll = 0
            pitch = 0
            yaw = np.arctan2(y, x)
            offset = 0.02
            x = x - offset
            y = y - np.tan(yaw)*offset
            quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
            
            grasping_transform = self._publish_tf(float(x), float(y), float(z), 
                                                  quaternion[0],
                                                  quaternion[1],
                                                  quaternion[2],
                                                  quaternion[3],
                                                  "px100/base_link", "grasping_frame")
            
    
    def _transform_to_matrix(self, transform_stamped):
        translation = transform_stamped.transform.translation
        tx, ty, tz = translation.x, translation.y, translation.z
        rotation = transform_stamped.transform.rotation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        matrix = np.identity(4)
        rotation_matrix = tf_transformations.quaternion_matrix([qx, qy, qz, qw])
        matrix[0:3, 0:3] = rotation_matrix[0:3, 0:3]
        matrix[0:3, 3] = [tx, ty, tz]

        return matrix

    def _normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0: 
            return v
        return v / norm
    
    
    def _publish_tf(self, px, py, pz, ox, oy, oz, ow, parent_frame, child_frame):
        current_time = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation.x = ox
        t.transform.rotation.y = oy
        t.transform.rotation.z = oz
        t.transform.rotation.w = ow
        self.tf_broadcaster.sendTransform(t)
        
        return t
        
    
def main(args=None):
    rclpy.init(args=args)
    px100_node = PX100Node()
    rclpy.spin(px100_node)
    px100_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()