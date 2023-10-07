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
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from optical_flow_ros.optical_flow_publisher import OpticalFlowPublisher

def main(args=None):
    rclpy.init(args=args)
    try:
        optical_flow_publisher = OpticalFlowPublisher(node_name='optical_flow')

        executor = SingleThreadedExecutor()
        executor.add_node(optical_flow_publisher)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            optical_flow_publisher.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()