cmake_minimum_required(VERSION 2.8)

project(mesh2octo)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)

# specify which version and components you need
find_package(OGRE 1.11 REQUIRED COMPONENTS Bites RTShaderSystem)
# copy resource.cfg next to our binaries where OGRE looks for it
file(COPY ${OGRE_CONFIG_DIR}/resources.cfg DESTINATION ${CMAKE_BINARY_DIR})
find_package(OpenCV REQUIRED)

message(
    "PCL Version:\n"
    ${PCL_VERSION}
    "\nOpenCV version:\n"
    ${OpenCV_VERSION}
    )

include_directories(
        ${PCL_INCLUDE_DIRS}
       # ${OpenCV_INCLUDE_DIRS}
)

set( project_sources
   main.cpp
   utils.cpp
)

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} ${project_sources})

target_link_libraries(
        ${PROJECT_NAME}
        ${PCL_LIBRARIES}
       # ${OpenCV_LIBRARIES}
)

