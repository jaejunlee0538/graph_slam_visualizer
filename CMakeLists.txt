cmake_minimum_required(VERSION 2.8.3)
project(graph_slam_visualizer)


# check c++11 / c++0x(https://svn.boost.org/trac/boost/ticket/9240)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11  -fext-numeric-literals")
    #'-fext-numeric-literals' : https://svn.boost.org/trac/boost/ticket/9240
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  EdgeSE2.msg
  VertexSE2.msg
  GraphSE2.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES graph_visualizer
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
        include
  ${catkin_INCLUDE_DIRS}
)


add_executable(se2_graph_visualizer src/se2_graph_visualizer_node.cpp)
target_link_libraries(se2_graph_visualizer ${catkin_LIBRARIES})

