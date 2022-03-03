cmake_minimum_required(VERSION 2.8.3)
project(my_turtlebot3_description)

find_package(catkin REQUIRED COMPONENTS roscpp rviz urdf std_msgs)

# Set the build type.  Options are:
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

catkin_package(
    # INCLUDE_DIRS meshes robots urdf
    CATKIN_DEPENDS roscpp std_msgs 
)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)




