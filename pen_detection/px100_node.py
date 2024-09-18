import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformListener, Buffer

from cv_bridge import CvBridge
import cv2
import numpy as np


class PX100Node(Node):
    def __init__(self):
        super().__init__("px100_node")
        
        self.declare_parameter('timer_frequency', 10.0) # hz
        
        self._timer_frequency = self.get_parameter('timer_frequency').value
        
        self.add_on_set_parameters_callback(self._parameter_callback)
        
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        self._main_loop_timer = self.create_timer(1.0/self._timer_frequency, self._main_loop_timer_callback)
        
        
    def _main_loop_timer_callback(self):
        cam2aruco = None
        cam2aruco_received = False
        cam2pen = None
        cam2pen_received = False
        
        from_frame = 'camera_color_optical_frame'
        to_frame = 'aruco_marker_frame'
        try:
            now = rclpy.time.Time()
            cam2aruco: TransformStamped = self._tf_buffer.lookup_transform(
                to_frame, from_frame, now
            )
            cam2aruco_received = True
        except Exception as e:
            self.get_logger().warn(f"Could not transform {from_frame} to {to_frame}: {str(e)}")
            
        from_frame = 'camera_color_optical_frame'
        to_frame = 'pen_frame'
        try:
            now = rclpy.time.Time()
            cam2aruco: TransformStamped = self._tf_buffer.lookup_transform(
                to_frame, from_frame, now
            )
            cam2aruco_received = True
        except Exception as e:
            self.get_logger().warn(f"Could not transform {from_frame} to {to_frame}: {str(e)}")
    
def main(args=None):
    rclpy.init(args=args)
    px100_node = PX100Node()
    rclpy.spin(px100_node)
    px100_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()