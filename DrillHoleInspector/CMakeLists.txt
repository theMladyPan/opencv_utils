cmake_minimum_required(VERSION 3.0.0)

project(DrillHoleInspector VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# The version number.
set (${PROJECT_NAME}_VERSION_MAJOR 0)
set (${PROJECT_NAME}_VERSION_MINOR 1)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find opencv sources and libs
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
find_package(OpenCV REQUIRED)

find_package(Boost 1.45.0 REQUIRED COMPONENTS filesystem regex)

# Set extra Libs
set (EXTRA_LIBS
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES})

# include project sources
include_directories(
    src/
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

set(project_sources
   main.cpp
   #src/noise_filter.cpp
   #src/segmentation.cpp
   #src/pixattr.cpp
)

message(WARNING
    ${EXTRA_LIBS}
    )


# build executable
add_executable(${PROJECT_NAME} ${project_sources})
# link libraries
target_link_libraries (${PROJECT_NAME} ${EXTRA_LIBS})
