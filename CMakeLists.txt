##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(intel_ros)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roscpp )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intel_ros
  CATKIN_DEPENDS roscpp 
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

##############################################################################
# Sources
##############################################################################

#add_library(intel_ros VoiceRecognition/VoiceRecognition.cpp)
#target_link_libraries(intel_ros ${catkin_LIBRARIES})

#install(TARGETS intel_ros
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/intel_ros/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")

##############################################################################
# Example
##############################################################################

#add_executable(demo_intel_ros src/main.cpp)
#target_link_libraries(demo_intel_ros intel_ros ${catkin_LIBRARIES})
#install(TARGETS demo_intel_ros 
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(voice VoiceRecognition/VoiceRecognition.cpp)
target_link_libraries(voice ${catkin_LIBRARIES})
##############################################################################
# Test
##############################################################################

#if(CATKIN_ENABLE_TESTING)
#  find_package(catkin COMPONENTS rostest rosunit)
#  include_directories(${GTEST_INCLUDE_DIRS})
#  catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
#  if(TARGET ${PROJECT_NAME}-test)
#    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  endif()
#endif()
