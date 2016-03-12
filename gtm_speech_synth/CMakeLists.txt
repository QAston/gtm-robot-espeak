cmake_minimum_required(VERSION 2.8.3)
project(gtm_speech_synth)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gtm_speech_msgs
  actionlib
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gtm_speech_synth
  CATKIN_DEPENDS gtm_speech_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(speech_synthesizer src/speech_synthesizer.cpp)
target_link_libraries(speech_synthesizer ${catkin_LIBRARIES} espeak ${Boost_LIBRARIES})
add_dependencies(speech_synthesizer gtm_speech_msgs_generate_messages)
add_dependencies(speech_synthesizer gtm_speech_msgs_gencfg)
