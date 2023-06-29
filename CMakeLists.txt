PROJECT(reference_car)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Reference Car Localization + Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++14")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
FIND_PACKAGE(GTSAMCMakeTools)
FIND_PACKAGE(Torch REQUIRED)

MESSAGE(STATUS "LibTorch location: ${TORCH_INCLUDE_DIRS}")
MESSAGE(STATUS "LibTorch libraries: ${TORCH_LIBRARIES}")

SET(libs roslib roscpp rosbag glog gflags amrl_shared_lib boost_system lua5.1
    pthread)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src)

ROSBUILD_ADD_EXECUTABLE(slam
                        src/slam/landmark_slam.cc)
TARGET_LINK_LIBRARIES(slam shared_library gtsam tbb ${libs})

ROSBUILD_ADD_EXECUTABLE(localize_offline
                        src/slam/localize_offline.cc)
TARGET_LINK_LIBRARIES(localize_offline shared_library gtsam tbb ${libs})

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc)
TARGET_LINK_LIBRARIES(navigation shared_library gtsam ${libs})

ROSBUILD_ADD_EXECUTABLE(localize
                        src/slam/localize_main.cc
                        src/slam/localize.cc)
TARGET_LINK_LIBRARIES(localize shared_library gtsam tbb ${libs})

ROSBUILD_ADD_EXECUTABLE(localizefkd
                        src/localize_fkd/localize_fkd_main.cc
                        src/localize_fkd/localize_fkd.cc
                        src/localize_fkd/model_wrapper.cc)
TARGET_LINK_LIBRARIES(localizefkd shared_library gtsam tbb ${libs} ${TORCH_LIBRARIES})

ROSBUILD_ADD_EXECUTABLE(mapping
                        src/slam/mapping_main.cc
                        src/slam/mapping.cc)
TARGET_LINK_LIBRARIES(mapping shared_library gtsam tbb ${libs})

ROSBUILD_ADD_EXECUTABLE(model_test
                        src/localize_fkd/model_test.cc
                        src/localize_fkd/model_wrapper.cc)
TARGET_LINK_LIBRARIES(model_test shared_library gtsam tbb ${libs} ${TORCH_LIBRARIES})

ADD_EXECUTABLE(eigen_tutorial
               src/eigen_tutorial.cc)
TARGET_LINK_LIBRARIES(eigen_tutorial gtsam tbb)

ADD_EXECUTABLE(torch_test
               src/torch_test.cc)
TARGET_LINK_LIBRARIES(torch_test ${TORCH_LIBRARIES})


ADD_EXECUTABLE(simple_queue_test
               src/navigation/simple_queue_test.cc)
