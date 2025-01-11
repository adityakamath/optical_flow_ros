# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rclpy
import numpy as np
from typing import Optional
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Twist, Point, Quaternion, Vector3, TransformStamped, Transform
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

# hard-coded values for PAA5100 and PMW3901 (to be verified for PMW3901)
FOV_DEG = 42.0
RES_PIX = 35

class OpticalFlowPublisher(Node):
    def __init__(self, node_name='optical_flow'):
        super().__init__(node_name)
        self._odom_pub: Optional[Publisher] = None
        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        self._timer: Optional[Timer] = None

        # declare parameters and default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('timer_period', 0.01),
                ('sensor_timeout', 1.0),
                ('parent_frame', 'odom'),
                ('child_frame', 'base_link'),
                ('x_init', 0.0),
                ('y_init', 0.0),
                ('z_height', 0.025),
                ('board', 'paa5100'),
                ('scaler', 5),
                ('spi_nr', 0),
                ('spi_slot', 'front'),
                ('rotation', 0),
                ('publish_tf', True),
            ]
        )
        
        self._pos_x = self.get_parameter('x_init').value
        self._pos_y = self.get_parameter('y_init').value
        self._pos_z = self.get_parameter('z_height').value
        self._scaler = self.get_parameter('scaler').value
        self._dt = self.get_parameter('timer_period').value
        self._sensor = None
        
        self.get_logger().info('Initialized')

    def publish_odom(self):
        if self._odom_pub is not None and self._odom_pub.is_activated:
            try:
                dx, dy = self._sensor.get_motion(timeout=self.get_parameter('sensor_timeout').value)
            except (RuntimeError, AttributeError):
                dx, dy = 0.0, 0.0

            fov = np.radians(FOV_DEG)
            cf = self._pos_z*2*np.tan(fov/2)/(RES_PIX*self._scaler)

            dist_x, dist_y = 0.0, 0.0
            if self.get_parameter('board').value == 'paa5100':
                # Convert data from sensor frame to ROS frame for PAA5100
                # ROS frame: front/back = +x/-x, left/right = +y/-y
                # Sensor frame: front/back = -y/+y, left/right = +x/-x
                dist_x = -1*cf*dy
                dist_y = cf*dx
            elif self.get_parameter('board').value == 'pmw3901':
                # ROS and Sensor frames are assumed to align for PMW3901 based on https://docs.px4.io/main/en/sensor/pmw3901.html#mounting-orientation
                dist_x = cf*dx
                dist_y = cf*dy
            
            self._pos_x += dist_x
            self._pos_y += dist_y

            odom_msg = Odometry(
                header = Header(
                    stamp = self.get_clock().now().to_msg(),
                    frame_id = self.get_parameter('parent_frame').value
                ),
                child_frame_id = self.get_parameter('child_frame').value,
                pose = PoseWithCovariance(
                    pose = Pose(position = Point(x=self._pos_x, y=self._pos_y, z=self._pos_z))
                ),
                twist = TwistWithCovariance(
                    twist = Twist(linear = Vector3(x=dist_x/self._dt, y=dist_y/self._dt, z=0.0))
                ),
            )
            self._odom_pub.publish(odom_msg)

            if self.get_parameter('publish_tf').value is True:
                tf_msg = TransformStamped(
                    header = odom_msg.header,
                    child_frame_id = odom_msg.child_frame_id,
                    transform = Transform(translation = Vector3(x=odom_msg.pose.pose.position.x,
                                                                y=odom_msg.pose.pose.position.y,
                                                                z=odom_msg.pose.pose.position.z)),
                )
                self._tf_broadcaster.sendTransform(tf_msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        sensor_classes = {'pmw3901': PMW3901, 'paa5100': PAA5100}
        SensorClass = sensor_classes.get(self.get_parameter('board').value)

        if SensorClass is not None:
            spi_slots = {'front': BG_CS_FRONT_BCM, 'back': BG_CS_BACK_BCM}
            self._sensor = SensorClass(spi_port=self.get_parameter('spi_nr').value, 
                                        spi_cs_gpio=spi_slots.get(self.get_parameter('spi_slot').value))
            self._sensor.set_rotation(self.get_parameter('rotation').value)

            if self._sensor is not None:
                self._odom_pub = self.create_lifecycle_publisher(Odometry, 'odom', qos_profile=qos_profile_sensor_data)
                self._tf_broadcaster = TransformBroadcaster(self)
                self._timer = self.create_timer(self._dt, self.publish_odom)
            
                self.get_logger().info('Configured')
                return TransitionCallbackReturn.SUCCESS
            else:
                self.get_logger().info('Configuration Failure: Invalid SPI Settings')
                return TransitionCallbackReturn.FAILURE
        else:
            self.get_logger().info('Configuration Failure: Invalid Sensor')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activated')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivated')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.terminate()
        self.get_logger().info('Clean Up Successful')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.terminate()
        self.get_logger().info('Shut Down Successful')
        return TransitionCallbackReturn.SUCCESS
        
    def terminate(self):
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
        if self._odom_pub is not None:
            self.destroy_publisher(self._odom_pub)
        if self._tf_broadcaster is not None:
            del self._tf_broadcaster

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.terminate()
        node.destroy_node()

if __name__ == '__main__':
    main()
