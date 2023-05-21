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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3, TransformStamped
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

class OpticalFlowPublisher(Node):
    def __init__(self, node_name='optical_flow'):
        super().__init__(node_name)
        self._odom_pub: Optional[Publisher] = None
        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        self._timer: Optional[Timer] = None
        
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('timer_period', 0.01)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('x_init', 0.0)
        self.declare_parameter('y_init', 0.0)
        self.declare_parameter('z_height', 0.025)
        self.declare_parameter('theta_init', 0.0)
        self.declare_parameter('board', 'paa5100')
        self.declare_parameter('spi_nr', 0)
        self.declare_parameter('spi_slot', 'front')
        self.declare_parameter('rotation', 0)
        self.declare_parameter('publish_tf', True)
        
        self._pos_x = self.get_parameter('x_init').value
        self._pos_y = self.get_parameter('y_init').value
        self._pos_z = self.get_parameter('z_height').value
        self._theta = self.get_parameter('theta_init').value
        self._dt = self.get_parameter('timer_period').value
        self._sensor = None
        
        self.get_logger().info('Initialized')

    def publish_odom(self):
        if self._odom_pub is not None and self._odom_pub.is_activated:
            try:
                x, y = self._sensor.get_motion(timeout=self.get_parameter('sensor_timeout').value)
            except (RuntimeError, AttributeError):
                x, y = 0.0, 0.0
        
            #TODO: x, y from sensor readings to dx/dy in meters. For now:
            dx = x
            dy = y
            dtheta = np.arctan2(dy, dx)
            
            self._pos_x += dx
            self._pos_y += dy
            self._theta += dtheta

            odom_msg = Odometry()
            odom_msg.header.stamp    = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.get_parameter('parent_frame').value
            odom_msg.child_frame_id  = self.get_parameter('child_frame').value
            odom_msg.pose.pose.position = Point(x=self._pos_x, y=self._pos_y, z=self._pos_z)
            odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=np.sin(self._theta/2.0), w=np.cos(self._theta/2.0))
            odom_msg.twist.twist.linear = Vector3(x=dx/self._dt, y=dy/self._dt, z=0.0)
            odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=dtheta/self._dt)
            
            self._odom_pub.publish(odom_msg)

            if self.get_parameter('publish_tf').value is True:
                tf_msg = TransformStamped()
                tf_msg.header = odom_msg.header
                tf_msg.child_frame_id = odom_msg.child_frame_id
                tf_msg.transform.translation = Vector3(x=odom_msg.pose.pose.position.x,
                                                   y=odom_msg.pose.pose.position.y,
                                                   z=odom_msg.pose.pose.position.z)
                tf_msg.transform.rotation = odom_msg.pose.pose.orientation
            
                self._tf_broadcaster.sendTransform(tf_msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        sensor_classes = {'pwm3901': PMW3901, 'paa5100': PAA5100}
        SensorClass = sensor_classes.get(self.get_parameter('board').value)

        if SensorClass is not None:
            spi_slots = {'front': BG_CS_FRONT_BCM, 'back': BG_CS_BACK_BCM}
            self._sensor = SensorClass(spi_port=self.get_parameter('spi_nr').value, 
                                        spi_cs_gpio=spi_slots.get(self.get_parameter('spi_slot').value))
            self._sensor.set_rotation(self.get_parameter('rotation').value)

            if self._sensor is not None:
                qos_profile = qos_profile_sensor_data
                self._odom_pub = self.create_lifecycle_publisher(Odometry, 
                                                                self.get_parameter('odom_topic').value, 
                                                                qos_profile=qos_profile)
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