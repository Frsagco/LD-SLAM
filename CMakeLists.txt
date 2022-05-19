cmake_minimum_required(VERSION 3.5)
project(ld_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


# Skip if Gazebo not present
find_package(gazebo QUIET)
if(NOT gazebo_FOUND)
  message(WARNING "Gazebo not found, proceeding without that simulator.")
  return()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)  
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})


find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(include)

# NanoFLANN
add_library(nanoflann STATIC
  src/front_end/nano_gicp/nanoflann.cpp
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})

# NanoGICP
add_library(nano_gicp STATIC
  src/front_end/nano_gicp/lsq_registration.cpp
  src/front_end/nano_gicp/nano_gicp.cpp
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Odometry Node
add_executable(dlo_odom_node src/front_end/odom_node.cpp src/front_end/odom.cpp)
ament_target_dependencies(dlo_odom_node
  rclcpp
  std_msgs
  )

add_dependencies(dlo_odom_node ${catkin_EXPORTED_TARGETS})
target_compile_options(dlo_odom_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlo_odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp)

# arrivato qua


install(
  DIRECTORY
    launch
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  launch/spawn_turtlebot.py
  DESTINATION lib/${PROJECT_NAME}
)





ament_package()
