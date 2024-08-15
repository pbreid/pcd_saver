# PCD Saver - ROS Node for Efficient Point Cloud Data Saving

## Overview

`PCD Saver` is a ROS node designed to efficiently capture and save point cloud data published on a ROS topic. The node accumulates incoming point cloud data in memory, retains intensity information, and writes the data to a PCD (Point Cloud Data) file either when memory usage reaches a specified threshold or when the program is terminated. The points are appended to an existing PCD file to prevent data loss.

## Features

- **Efficient Data Accumulation**: Accumulates point cloud data in memory, writing to the file only when necessary.
- **Intensity Retention**: Supports `pcl::PointXYZI` to retain intensity information.
- **File Append Mode**: Appends new point data to the existing PCD file, ensuring no data is lost between flushes.
- **Configurable Parameters**: Easily configure the input topic, output file name, and memory threshold via ROS parameters.
- **Signal Handling**: Ensures data is flushed to disk on program termination (e.g., `Ctrl+C`).

## Prerequisites

- ROS (Robot Operating System) installed and configured.
- PCL (Point Cloud Library) integrated with your ROS setup.

## Installation

1. **Clone the Repository**:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/pbreid/pcd_saver
    ```

2. **Build the Package**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

3. **Source Your Workspace**:
    ```bash
    source devel/setup.bash
    ```

## Usage

### Running the Node

You can run the node using the following command:

```bash
rosrun my_pcl_saver pcd_saver _input_topic:=/your_pointcloud_topic _output_file:=/path/to/output.pcd _max_points:=1000000
