# ros2/common_interfaces is licensed under the
#               Apache License 2.0
# https://github.com/ros2/common_interfaces/blob/humble/visualization_msgs/msg/MeshFile.msg

# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
