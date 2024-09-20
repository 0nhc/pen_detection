import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
import tf2_ros
from tf2_ros import TransformListener, Buffer
import tf_transformations

from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup


class PenDetection(Node):
    def __init__(self):
        super().__init__("px100_grasping")
        
        self.declare_parameter('timer_frequency', 100.0) # hz
        
        self._timer_frequency = self.get_parameter('timer_frequency').value
        
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # self._main_loop_timer = self.create_timer(1.0/self._timer_frequency, self._main_loop_timer_callback)
        self._update_grasping_pose_timer = self.create_timer(1.0/self._timer_frequency, self._update_grasping_pose_timer_callback)
        
        self._grasping_pose = None
        self._grasping_pose_received = False
        
        self._robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        robot_startup()
        self._robot.arm.go_to_sleep_pose()
        self._robot.gripper.release()        
        
        self._init_time_secs = 2.0
        self._init_iterations = self._init_time_secs*self._timer_frequency
        self._init_counter = 0
        
        # self._main()
        
    
    def _update_grasping_pose_timer_callback(self):
        if(self._init_counter == self._init_iterations):
            try:
                # input()
                parent_frame = 'px100/base_link'
                child_frame = 'grasping_frame'
                now = rclpy.time.Time()
                self._grasping_pose = self._tf_buffer.lookup_transform(
                    parent_frame, child_frame, now
                )
                self._grasping_pose_received = True
                self._robot.arm.set_ee_pose_matrix(self._transform_to_matrix(self._grasping_pose))
                self._robot.gripper.set_pressure(1.0)
                self._robot.gripper.grasp(3.0)
                self._robot.arm.go_to_sleep_pose()
                time.sleep(1.0)
                self._robot.gripper.release()
                exit()
            except Exception as e:
                self.get_logger().warn(f"Could not transform {parent_frame} to {child_frame}: {str(e)}")
        else:
            self._init_counter += 1
        
    # def _main_loop_timer_callback(self):
    #     pass
        # self._robot.arm.set_ee_pose_matrix(self._transform_to_matrix(self._grasping_pose))
        # self._robot.gripper.grasp()
        # self._robot.arm.go_to_sleep_pose()
        
        
    def _main(self):
        # while(self._grasping_pose_received != True):
        #     self.get_logger().warn(f"not receiving transfrom")
        # else:
        self._robot.arm.set_ee_pose_matrix(self._transform_to_matrix(self._grasping_pose))
        self._robot.gripper.grasp()
        self._robot.arm.go_to_sleep_pose()
            
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
        
    
def main(args=None):
    rclpy.init(args=args)
    pen_detection = PenDetection()
    rclpy.spin(pen_detection)
    pen_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()