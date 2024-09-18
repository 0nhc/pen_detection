import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
import tf2_ros
from tf2_ros import TransformListener, Buffer

from cv_bridge import CvBridge
import cv2
import numpy as np


class PenDetection(Node):
    def __init__(self):
        super().__init__("pen_detectin")
        self._rgb_image = None
        self._rgb_image_received = False
        self._depth_image = None
        self._depth_image_received = False
        self._camera_info = None
        self._camera_intrinsic_matrix = None
        self._camera_info_received = False
        
        self.declare_parameter('timer_frequency', 10.0) # hz
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('lower_hue', 110)
        self.declare_parameter('upper_hue', 150)
        self.declare_parameter('lower_saturation', 125)
        self.declare_parameter('upper_saturation', 255)
        self.declare_parameter('lower_value', 125)
        self.declare_parameter('upper_value', 255)
        self.declare_parameter('m00_threshold', 50000) # num of pixels filtered as PURPLE
        
        self._timer_frequency = self.get_parameter('timer_frequency').value
        self._depth_scale = self.get_parameter('depth_scale').value
        self._lower_hue = self.get_parameter('lower_hue').value
        self._upper_hue = self.get_parameter('upper_hue').value
        self._lower_saturation = self.get_parameter('lower_saturation').value
        self._upper_saturation = self.get_parameter('upper_saturation').value
        self._lower_value = self.get_parameter('lower_value').value
        self._upper_value = self.get_parameter('upper_value').value
        self._m00_threshold = self.get_parameter('m00_threshold').value
        
        self.add_on_set_parameters_callback(self._parameter_callback)
        
        self._rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self._rgb_callback,
            10)
        self._rgb_subscription
        
        self._depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self._depth_callback,
            10)
        self._depth_subscription
        
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._camera_info_callback,
            10)
        self._camera_info_subscription
        
        self._image_publisher = self.create_publisher(Image, '/pen_detection/hsv_result', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        self._main_loop_timer = self.create_timer(1.0/self._timer_frequency, self._main_loop_timer_callback)
        
        # Convert ROS 2 image messages to cv2 format
        self.bridge = CvBridge()
        
        
    def _main_loop_timer_callback(self):
        if(self._rgb_image_received and self._depth_image_received and self._camera_info_received):
            cam2aruco_received = False
            try:
                parent_frame = 'camera_color_optical_frame'
                child_frame = 'aruco_marker_frame'
                now = rclpy.time.Time()
                cam2aruco_rot = self._tf_buffer.lookup_transform(
                    parent_frame, child_frame, now
                ).transform.rotation
                cam2aruco_received = True
            except Exception as e:
                # self.get_logger().warn(f"Could not transform {from_frame} to {to_frame}: {str(e)}")
                pass
            
            if(cam2aruco_received):
                depth_image = self._depth_image
                rgb_image = self._rgb_image
                
                hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
                lower_purple = np.array([self._lower_hue, self._lower_saturation, self._lower_value])  # Lower bound for blue color in HSV
                upper_purple = np.array([self._upper_hue, self._upper_saturation, self._upper_value])
                mask = cv2.inRange(hsv_image, lower_purple, upper_purple) // 255 # Convert from 0/255 to 0/1
                result = cv2.bitwise_and(rgb_image, rgb_image, mask=mask)            
                masked_depth = depth_image * mask 
                
                """
                I asked ChatGPT about how to compute the center position efficiently
                M["m00"] is the sum of pixels that are not zero
                M["m10"] is the sum of every M["m00"] pixel's x coordinate
                M["m01"] is the sum of every M["m00"] pixel's y coordinate
                """
                # Find contours or the center of the mask using moments
                M = cv2.moments(masked_depth)

                # Calculate the centroid (center) if moments are valid
                if M["m00"] >= self._m00_threshold:
                    cX = int(M["m10"] / M["m00"])  # X coordinate of the center
                    cY = int(M["m01"] / M["m00"])  # Y coordinate of the center
                    # Draw a circle (dot) at the center of the mask
                    cv2.circle(result, (cX, cY), 17, (0, 0, 255), -1)
                    
                    # Get the center point's 3D position under the camera depth frame
                    pz = np.mean(masked_depth[masked_depth>0]) * self._depth_scale
                    px = (cX - self._camera_intrinsic_matrix[0,2]) * pz / self._camera_intrinsic_matrix[0,0]
                    py = (cY - self._camera_intrinsic_matrix[1,2]) * pz / self._camera_intrinsic_matrix[1,1]
                    self._publish_tf(px,py,pz, 
                                     cam2aruco_rot.x, 
                                     cam2aruco_rot.y, 
                                     cam2aruco_rot.z, 
                                     cam2aruco_rot.w, 
                                     "camera_color_optical_frame", 
                                     "pen_frame")
                    
                result_msg = self.bridge.cv2_to_imgmsg(result, "bgr8")
                self._image_publisher.publish(result_msg)            
            
        
    def _rgb_callback(self, msg):
        try:
            self._rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._rgb_image_received = True
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
    
    def _depth_callback(self, msg):
        try:
            self._depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            self._depth_image_received = True
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
                
    
    def _camera_info_callback(self, msg):
        try:
            self._camera_info = msg
            self._camera_intrinsic_matrix = np.array(msg.k).reshape(3,3)
            self._camera_info_received = True
                
        except Exception as e:
            self.get_logger().error(f"Failed to process point cloud: {e}")
        
        
    def _parameter_callback(self, params):
        for param in params:
            if param.name == 'lower_hue':
                self._lower_hue = param.value
            elif param.name == 'upper_hue':
                self._upper_hue = param.value
            elif param.name == 'lower_saturation':
                self._lower_saturation = param.value
            elif param.name == 'upper_saturation':
                self._upper_saturation = param.value
            elif param.name == 'lower_value':
                self._lower_value = param.value
            elif param.name == 'upper_value':
                self._upper_value = param.value
        self.get_logger().info(f"Updated HSV range: "
                            f"Hue({self._lower_hue}-{self._upper_hue}), "
                            f"Saturation({self._lower_saturation}-{self._upper_saturation}), "
                            f"Value({self._lower_value}-{self._upper_value})")
        return SetParametersResult(successful=True)
    
    
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
        
    
def main(args=None):
    rclpy.init(args=args)
    pen_detection = PenDetection()
    rclpy.spin(pen_detection)
    pen_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()