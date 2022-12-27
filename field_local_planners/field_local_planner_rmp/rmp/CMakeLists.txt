cmake_minimum_required(VERSION 2.8.3)
project(rmp)
set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
)

# GTSAM
# NB: The issue with including all GTSAM plugins is that it changes the names of libraries and breaks catkin packages.
# E.g. "librmp" becomes "librmpRelWithDebInfo"
set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable/Disable building with all instructions supported by native architecture (binary may not be portable!)" FORCE)
find_package(GTSAM REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWitDebInfo)
endif()

###################################
## Catkin Specific Configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  #DEPENDS gtsam
)

###########
## Build ##
###########
include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${gtsam_INCLUDE_DIRS}
)

######################################################
add_library(${PROJECT_NAME} SHARED
  src/models/ControlPoint.cpp
  src/models/AckermannModel.cpp
  src/models/DifferentialModel.cpp
  src/rmp.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      gtsam)


##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test test/main.cpp
                                        test/unitTests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
