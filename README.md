# pmw3901_ros
ROS 2 node for the [PMW3901](https://shop.pimoroni.com/products/pmw3901-optical-flow-sensor-breakout?variant=27869870358611) optical flow sensor and it's short-range variant [PAA5100](https://shop.pimoroni.com/products/paa5100je-optical-tracking-spi-breakout?variant=39315330170963).

Note: This implementation is a bit over-engineered, as I have been experimenting with ROS 2 [managed/lifecycle](https://design.ros2.org/articles/node_lifecycle.html) nodes, [executors](https://docs.ros.org/en/humble/Concepts/About-Executors.html#executors) and [composition](https://github.com/ros2/examples/blob/rolling/rclpy/executors/examples_rclpy_executors/composed.py) using Python.

## Implementation details

* ```optical_flow_publisher```: This executable uses the [pmw3901-python](https://github.com/pimoroni/pmw3901-python) library to access sensor data over SPI. The delta X and Y measurements from the sensor are converted to 2D odometry data, which is published periodically using a timer as an [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html) message to the ```/odom``` topic and as a [transform broadcast](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-TF2.html) to ```/tf```. The odometry publisher uses the [Sensor Data QoS profile](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-profiles) as default. This implementation is designed as a lifecycle component and can be run individually as well. 

* ```optical_flow_node```: This executable creates an instance of ```optical_flow_publisher``` and runs it using a single threaded executor. 

* ```optical_flow_launch.py```: This is the launch file that launches ```optical_flow_node``` as a  lifecycle node, loads its parameters, and then configures and activates it. The lifecycle node is first initialized, then set to 'configure' from the launch file. When the 'inactive' state is reached, the registered event handler activates the node.

## Parameters

* ```odom_topic```: Odometry topic name (Default: odom)
* ```timer_period```: Timer period in seconds (Default: 0.01)
* ```sensor_timeout```: Sensor timeout in seconds in case of no movement, or sensor failure (Default: 1.0)
* ```parent_frame```: Parent frame for the Odometry message and Transform (Default: odom)
* ```child_frame```: Child frame for the Odometry message and transform (Default: base_link)
* ```x_init```: Initial position in the X axis, in meters (Default: 0.0)
* ```y_init```: Initial position in the Y axis, in meters (Default: 0.0)
* ```z_height```: Height of the sensor from the ground, in meters (Default: 0.025)
* ```board```: Sensor type - pmw3901 or paa5100 (Default: paa5100)
* ```fov_deg```: FOV of the sensor aperture in degrees (Default: 42 for PMW3901/PAA5100)
* ```res_px```: Resolution of the sensor in pixels, assuming the same value in both directions (Default: 35 for PMW3901/PAA5100)
* ```scaler```: Scaling factor, i.e. the sensor value returned for 1 pixel move (Default: 5)
* ```spi_nr```: SPI port number (Default: 0)
* ```spi_slot```: SPI CS pin - front (BCM pin 7 on RPi) or back (BCM pin 8 on RPi) (Default: front)
* ```rotation```: Rotation of the sensor in 90 degree increments - 0, 90, 180, 270 (Default: 270)
* ```publish_tf```: Boolean value to turn transform publisher on/off (Default: true)

## How to use

* Connect the sensor breakout (PMW3901 or PAA5100) to the SPI GPIO pins of a Raspberry Pi device:
  * 3-5V to a 3.3V pin
  * CS to BCM 7
  * SCK to BCM 11
  * MOSI to BCM 10
  * MISO to BCM 9
  * INT to BCM 19
  * GND to any ground pin
* Install the pmw3901-python library: ```sudo pip install pmw3901```
* Clone this repository in a ROS 2 workspace. Check the ```sensor_params.yaml``` file in the config directory, and make any necessary changes.
* Build the package and run the launch file: ```ros2 launch pmw3901_ros optical_flow_launch.py```

## Results

This package was tested using a [PAA5100JE Near Optical Flow sensor](https://shop.pimoroni.com/products/paa5100je-optical-tracking-spi-breakout?variant=39315330170963) from Pimoroni and ROS 2 Humble on two devices:
* A Raspberry Pi 4 (8GB) running Ubuntu 22.04 with a real-time kernel
* A Raspberry Pi Zero 2 W running Ubuntu 22.04 without any kernel modifications
In both cases, the output frequency of 100Hz was achieved. 

## Improvements (To Do)

* Rotations: The publisher assumes that the sensor is placed at a fixed height parallel to the ground (within the range of the respective sensor), and currently does not account for any rotations in any axis. Rotations in x and y directions (roll and pitch) can be accounted for using the explanation [here](https://ardupilot.org/copter/docs/common-mouse-based-optical-flow-sensor-adns3080.html) and in this [thesis report](https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8905295&fileOId=8905299) referenced in the [Bitcraze PMW3901 driver](https://github.com/bitcraze/Bitcraze_PMW3901/tree/master)

* Scaling Factor: For a single pixel move, the sensor returns a value of more than 1. This is the scaling factor or simply *scaler*. This scaler value seems to be proprietary and cannot be found anywhere online, and most references mention an NDA with the sensor manufacturer. This can be calculated by moving the sensor for a known distance and comparing it with the distance measured by the sensor at a fixed height. This still needs to be done once a fixed mount is designed for the sensor. It is also unknown if the scaler value will change for different sensor heights.

* PMW3901: This implementation has been tested with the PAA5100 Near Optical Flow sensor but still needs to be tested with the PMW3901 sensor. There may be some differences such as resolution in pixels. Since the used library defines the raw data length to be 1225 (i.e 35x35), a default resolution of 35 is used. However, some online implementations define the resolution of PMW3901 as 30x30. Another issue is the orientation - for PMW3901, [this](https://docs.px4.io/main/en/sensor/pmw3901.html) document states that the notch on the sensor indicates the back of the board. However, for PAA5100, the frame seems to be rotated 90 degrees counterclockwise. This is accounted for in the software but for both sensors. Both these issues need to be checked with the PMW3901 sensor.

## Challenges

Both these challenges are common for both PMW3901 and PAA5100 boards:

* Secret sauce: the library has a lot of proprietary information which cannot be explained. The manufacturer is apparently very guarded about this information, and the only people who seem to have this information have had to sign an NDA. This includes stuff like the scaling factor to convert from pixel values to distances as well as functionality such as turning the LEDs on and off.

* Off-centered: The sensor breakout from Pimoroni is not centered on the breakout board. Even if this offset is corrected, the aperture of the sensor itself is not centered on the sensor package. Fortunately, Pimoroni has provided [a mechanical drawing](https://cdn.shopify.com/s/files/1/0174/1800/files/39b9173de8970896f2eaa114ef5738eb993a06cc.png?v=1621246728) of the sensor breakout. 
