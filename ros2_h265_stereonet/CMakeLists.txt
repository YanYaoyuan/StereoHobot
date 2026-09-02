cmake_minimum_required(VERSION 3.5)
project(ros2_h265_stereonet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
set(CMAKE_BUILD_TYPE "Release")

# =================================== set platform ====================================
if(PLATFORM_S100)
  message(STATUS "ros2_h265_stereonet build platform S100")
  add_definitions(-DPLATFORM_S100)
elseif(PLATFORM_S600)
  message(STATUS "ros2_h265_stereonet build platform S600")
  add_definitions(-DPLATFORM_S600)
else()
  message(STATUS "ros2_h265_stereonet build platform X5")
  add_definitions(-DPLATFORM_X5)
endif()

# Cache variable allows the package to be staged separately in a cross-build
# workspace while reusing the parent project's inference sources and ARM64
# OpenCV distribution.
set(STEREONET_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH
    "Path to the parent StereoHobot source tree")
if((PLATFORM_S100 OR PLATFORM_S600) AND
   EXISTS "${STEREONET_ROOT}/standalone/3rdparty/lib_opencv4.5.4/lib/cmake/opencv4/OpenCVConfig.cmake")
  set(OpenCV_DIR
      "${STEREONET_ROOT}/standalone/3rdparty/lib_opencv4.5.4/lib/cmake/opencv4"
      CACHE PATH "ARM64 OpenCV package" FORCE)
endif()

# =================================== find packages ===================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

# FFmpeg
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec
  libavutil
  libswscale
)

# =================================== stereonet parent paths ===========================
set(concurrentqueue_DIR ${STEREONET_ROOT}/3rdparty/concurrentqueue)
set(magic_enum_DIR ${STEREONET_ROOT}/3rdparty/magic_enum/include)
set(bs_thread_pool_DIR ${STEREONET_ROOT}/3rdparty/thread-pool/include)

# =================================== PCL =============================================
set(SYS_ROOT ${CMAKE_SYSROOT})
message(STATUS "SYS_ROOT is ${SYS_ROOT}")

if(DEFINED ENV{ROS_DISTRO})
  set(PREFERRED_ROS_DISTRO $ENV{ROS_DISTRO})
else()
  if(EXISTS "/opt/ros/jazzy")
    set(PREFERRED_ROS_DISTRO jazzy)
  elseif(EXISTS "/opt/ros/humble")
    set(PREFERRED_ROS_DISTRO humble)
  else()
    set(PREFERRED_ROS_DISTRO humble)
  endif()
endif()
message(STATUS "Preferred ROS distribution: ${PREFERRED_ROS_DISTRO}")

if("${PREFERRED_ROS_DISTRO}" STREQUAL "jazzy")
  add_definitions(-DCV_BRIDGE_CPP)
endif()

set(PCL_INCLUDE_DIRS
  ${SYS_ROOT}/usr/include/pcl-1.12/
  ${SYS_ROOT}/usr/include/pcl-1.14/
  ${SYS_ROOT}/usr/include/eigen3/
  /opt/ros/${PREFERRED_ROS_DISTRO}/include/
  /opt/ros/${PREFERRED_ROS_DISTRO}/include/pcl_conversions/
  /opt/ros/${PREFERRED_ROS_DISTRO}/include/pcl_msgs/
)
set(PCL_LIBRARIES pcl_common pcl_filters pcl_io)

# =================================== DNN =============================================
set(DNN_LIBS "")
if(PLATFORM_S100 OR PLATFORM_S600)
  list(APPEND DNN_LIBS dnn hbucp)
else()
  list(APPEND DNN_LIBS dnn cnn_intf hbrt_bayes_aarch64)
endif()

# =================================== include & link dirs =============================
include_directories(
  include
  ${STEREONET_ROOT}/include
  ${SYS_ROOT}/usr/include
  ${SYS_ROOT}/usr/hobot/include
  ${PCL_INCLUDE_DIRS}
  ${concurrentqueue_DIR}
  ${magic_enum_DIR}
  ${bs_thread_pool_DIR}
)

link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/hobot/lib/
  ${SYS_ROOT}/usr/lib/aarch64-linux-gnu/
)

# =================================== build executable ================================
add_executable(stereo_h265_node
  src/stereo_h265_node.cpp
  src/h265_decoder.cpp
  ${STEREONET_ROOT}/src/stereonet_process.cpp
  ${STEREONET_ROOT}/src/timer_utils.cpp
  ${STEREONET_ROOT}/src/img_convert_utils.cpp
  ${STEREONET_ROOT}/src/file_utils.cpp
  ${STEREONET_ROOT}/src/feature_epipolar_align.cpp
  ${STEREONET_ROOT}/src/stereo_rectify.cpp
)

target_compile_features(stereo_h265_node PUBLIC cxx_std_17)

ament_target_dependencies(stereo_h265_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  foxglove_msgs
  ament_index_cpp
)

target_link_libraries(stereo_h265_node
  ${OpenCV_LIBRARIES}
  PkgConfig::LIBAV
  ${DNN_LIBS}
  ${PCL_LIBRARIES}
  alog
  pthread
  OpenMP::OpenMP_CXX
)

# =================================== install =========================================
install(TARGETS stereo_h265_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

set(S100_MODEL
  "${STEREONET_ROOT}/config/dstereo_s100_320_640_352_v2.4.hbm")
if(PLATFORM_S100 AND EXISTS "${S100_MODEL}")
  install(FILES "${S100_MODEL}" DESTINATION share/${PROJECT_NAME}/model)
endif()

ament_package()
