# serial_stream

ROS package that converts data received via serial communication to ROS topics (std_msgs/Float64). It supports flexible data formats and can be configured with simple settings to handle various sensor data.

## Overview

serial_stream functions as a bridge that parses text data received from serial ports, extracts numerical data according to specified formats, and publishes them as ROS topics. It enables easy integration of various sensor data such as accelerometers, gyroscopes, and thermometers into the ROS environment.

## Features

- Flexible data format support: Compatible with various data formats

## Requirements

- libserial-dev

## Installation

```bash
# Navigate to your workspace
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/sgrsn/stream_data_logger.git

# Install dependencies
sudo apt-get install libserial-dev

# Build
cd ~/catkin_ws
catkin_make
```

## Usage

### Basic Launch

```bash
roslaunch serial_stream log.launch
```

### Launch Example with Custom Settings

If you want to handle three sets of data (time, voltage and temperature), please create a launch file as follows.

```xml
<launch>
  <node name="serial_stream_node" pkg="serial_stream" type="serial_stream_node" output="screen">
    <param name="serial_port" value="/dev/ttyUSB0"/>
    <param name="baud_rate" value="115200"/>
    <param name="data_count" value="3"/>
    <param name="data0_format" value="time: %lf"/>
    <param name="data1_format" value="volt: %lf"/>
    <param name="data2_format" value="temp: %lf"/>
    <remap from="~data0" to="time"/>
    <remap from="~data1" to="voltage"/>
    <remap from="~data2" to="temperature"/>
  </node>
</launch>
```

### Data Format

Serial data needs to be in the following format:

```
time: 1.234, volt: 3.30, temp: 25.2
```

## License

MIT License
