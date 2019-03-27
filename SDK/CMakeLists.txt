cmake_minimum_required(VERSION 3.1)
#set(CC=gcc-3.3 CXX=gcc-3.3 cmake)


project(camtest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)
find_package(Boost 1.54 REQUIRED COMPONENTS python3)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS}  )

FIND_PACKAGE(PythonLibs REQUIRED)

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})

LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

include_directories("${PROJECT_SOURCE_DIR}/include")

add_library(optorusbcam src/optorusb.cpp src/optorcam.cpp)
target_link_libraries(optorusbcam optorimu)

add_library(optorimu src/optorimu.cpp)

#######

add_library(camtest src/camtest.cpp)
PYTHON_ADD_MODULE(cam src/camtest.cpp src/optorusb.cpp src/optorcam.cpp src/optorimu.cpp )
target_link_libraries(cam camtest optorusbcam optorimu usb-1.0 ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})


FILE(COPY src/cam_run.py src/darknet.py src/util.py  DESTINATION .)
ADD_TEST(NAME 01-cam COMMAND ${PYTHON_EXECUTABLE} cam_run.py)

