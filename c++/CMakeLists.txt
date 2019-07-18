cmake_minimum_required(VERSION 2.8.3)
project(gg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp )
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES random
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ../../visibility-polygon/include
        ${catkin_INCLUDE_DIRS}
        {BOOST_INCLUDE_DIRS}
)

SET(visibility-polygon_LIBRARY_OUTPUT_DIRECTORY ${visibility-polygon_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${visibility-polygon_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${visibility-polygon_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${visibility-polygon_RUNTIME_OUTPUT_DIRECTORY})

set(srcs
        src/gazebo_generator.cpp
        )

## Declare a C++ executable
add_executable(${PROJECT_NAME}test src/main.cpp  )


## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
        ${srcs}
        )


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${BOOST_LIBRARIES}
        ${catkin_LIBRARIES}
        /home/bjgilhul/catkin_ws/obstacle_planner/ob1/thirdParty/visibility-polygon/lib/libvisibility-polygon.so
        ${PROJECT_NAME}
        )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
INSTALL(DIRECTORY ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/include
        FILES_MATCHING PATTERN "*.h" )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )




#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ob1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
