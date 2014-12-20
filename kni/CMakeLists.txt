# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(kni)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED )

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(
  FILES
  # TODO: List your msg files here
)
#uncomment if you have defined services
#add_service_files(
  FILES
  # TODO: List your msg files here
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/common/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of common files failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/Base/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of Base library failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/roboop/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of roboop library failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of kinematics library failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/InvKin/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of InvKin library failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/LM/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of LM library failed")
endif(_make_failed)

## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS # TODO add dependencies
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)