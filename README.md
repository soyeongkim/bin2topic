
# README for bin2topic ROS Node

## Overview
The `bin2topic` ROS node reads binary `.bin` files containing point cloud data from a specified directory, converts them to `sensor_msgs::PointCloud2` messages, and publishes them to a ROS topic.

## Prerequisites
- ROS (Robot Operating System) installed on your system.
- PCL (Point Cloud Library) installed.
- The following ROS packages:
  - `roscpp`
  - `sensor_msgs`
  - `pcl_ros`

## Building the Package

1. **Create a ROS package:**

   If you don't already have a ROS workspace, create one and navigate to the `src` directory:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. **Clone the repository:**

   Clone this repository into the `src` directory of your ROS workspace:
   ```bash
   git clone https://github.com/soyeongkim/bin2topic
   ```

3. **Build the package:**

   Navigate to the root of your ROS workspace and build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. **Source the workspace:**

   Source your ROS workspace to update your environment:
   ```bash
   source devel/setup.bash
   ```

## Running the Node

1. **Set parameters:**

   You can set parameters either by modifying a launch file or directly through the command line. The parameters to set are:
   - `bin2topic/directory` (string): Path to the directory containing the `.bin` files.
   - `bin2topic/frame_id` (string): Frame ID for the published point cloud.
   - `bin2topic/delta_time_sec` (double): Time interval between publishing consecutive point clouds.

2. **Launch the node:**

   You can create a launch file or run the node directly using `rosrun` with the parameters set. For example:

   ```bash
   rosrun bin2topic bin2topic_node _directory:=/path/to/bin/files _frame_id:=base_link _delta_time_sec:=1.0
   ```

## Example Launch File

You can create a launch file `bin2topic.launch` for convenience:

```xml
<launch>
    <node name="bin2topic_node" pkg="bin2topic" type="bin2topic_node" output="screen">
        <param name="bin2topic/directory" value="/path/to/bin/files" />
        <param name="bin2topic/frame_id" value="base_link" />
        <param name="bin2topic/delta_time_sec" value="1.0" />
    </node>
</launch>
```

Run the launch file with:

```bash
roslaunch bin2topic bin2topic.launch
```

## Parameters

- `bin2topic/directory` (`string`): Directory containing the `.bin` files.
- `bin2topic/frame_id` (`string`): Frame ID for the point cloud messages.
- `bin2topic/delta_time_sec` (`double`): Time interval between consecutive point cloud messages.

## Topics

- **Published Topics:**
  - `/bin2topic/pointcloud` (`sensor_msgs::PointCloud2`): Publishes the converted point cloud data.

## Additional Information

### Functionality

- `getSortedBinFiles(const std::string& directory)`: Retrieves and sorts the list of `.bin` files from the specified directory.
- `loadBinFile(const std::string& file_path)`: Loads a `.bin` file into a `pcl::PointCloud<pcl::PointXYZI>` object.
- Main function initializes the ROS node, reads parameters, retrieves the list of `.bin` files, and publishes the point cloud data at the specified rate.

### Error Handling

- Ensure the directory path is correct and contains valid `.bin` files.
- Check that the specified frame ID is valid and matches the frame used in your ROS setup.
- Verify the delta time is set to a reasonable value to avoid publishing data too rapidly or too slowly.

## Conclusion

This node is useful for converting and publishing point cloud data stored in binary files, making it accessible for other ROS nodes and processes. Adjust the parameters as needed to fit your specific use case.

For further information or issues, please refer to the source code or contact the maintainer of the package.
