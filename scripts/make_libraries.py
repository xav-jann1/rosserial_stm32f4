#!/usr/bin/env python
#######################################################################################################################
# Software License Agreement (BSD License)
#   Original Copyright: Copyright (c) 2013, Willow Garage, Inc.
#     https://github.com/ros-drivers/rosserial/blob/melodic-devel/rosserial_mbed/src/rosserial_mbed/make_libraries.py
#
#   Modifications by:
#     - Kenta Yonekura (a.k.a. yoneken):
#         https://github.com/yoneken/rosserial_stm32/blob/master/src/rosserial_stm32/make_libraries.py
#
#     - Federica Di Lauro:
#         https://github.com/fdila/rosserial_stm32f7/blob/master/src/rosserial_stm32f7/make_libraries.py
#
#     - Xavier Jannin:
#         this version...
#
#######################################################################################################################


THIS_PACKAGE = "rosserial_stm32f4"

__usage__ = """
make_libraries.py generates the STM32 rosserial library files for STM32CubeIDE.
It requires the location of your STM32CubeIDE project folder.

rosrun rosserial_stm32f4 make_libraries.py <output_path>
"""

# ROS:
import rospkg, rosserial_client
from rosserial_client.make_library import *

# To move files:
import os, shutil


ROS_TO_EMBEDDED_TYPES = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'byte'    :   ('int8_t',            1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'char'    :   ('uint8_t',           1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           8, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          8, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('float',             4, AVR_Float64DataType, []),
    'time'    :   ('ros::Time',         8, TimeDataType, ['ros/time']),
    'duration':   ('ros::Duration',     8, TimeDataType, ['ros/duration']),
    'string'  :   ('char*',             0, StringDataType, []),
    'Header'  :   ('std_msgs::Header',  0, MessageDataType, ['std_msgs/Header'])
}


# Verif params:
if (len(sys.argv) < 2):
    print __usage__
    exit()


## Prepare folder ##

# Get path:
path = sys.argv[1]
if path[-1] == "/": path = path[0:-1]
print "\nExporting to %s" % path

# Delete folder if it already exists:
if os.path.exists(path + "/Inc/ros_lib/"):
    print("Delete old folder:", path + "/Inc/ros_lib/")
    shutil.rmtree(path + "/Inc/ros_lib/")

# Create folder:
os.makedirs(path+"/Inc/ros_lib/")


## Copy ros_lib to destination ##
rospack = rospkg.RosPack()

# Get ros_lib path:
ros_lib_dir = rospack.get_path(THIS_PACKAGE) + "/ros_lib/"

# Copy ros_lib's files in the folder:
files = os.listdir(ros_lib_dir)
for f in files:
    if os.path.isfile(ros_lib_dir + f):
        shutil.copy(ros_lib_dir + f, path + "/Inc/ros_lib/")
rosserial_client_copy_files(rospack, path + "/Inc/ros_lib/")

# Generate messages:
rosserial_generate(rospack, path + "/Inc/ros_lib/", ROS_TO_EMBEDDED_TYPES)


# Move '*.cpp' files in 'Src/' folder:
sourcefiles = os.listdir(path+"/Inc/ros_lib/")
for file in sourcefiles:
    if file.endswith('.cpp'):
        shutil.move(os.path.join(path + "/Inc/ros_lib/", file),
                    os.path.join(path + "/Src/", file))
