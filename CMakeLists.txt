CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

PROJECT(gransac)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin)

IF(MSVC)
  ADD_DEFINITIONS(-DNOMINMAX)
  # Another MSVC quirk. http://stackoverflow.com/questions/6832666/lnk2019-when-including-asio-headers-solution-generated-with-cmake
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP") # For multi-threaded compilation on MSVC
ELSE()
  IF(APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++") # C++11 support on Apple clang
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# C++11 support on GCC
  ENDIF()
ENDIF()

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL REQUIRED COMPONENTS common)
FIND_PACKAGE(GTest REQUIRED)

# OpenMP
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()

catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}
  LIBRARIES line_fit plane_fit
#  CATKIN_DEPENDS
#  DEPENDS
)

INCLUDE_DIRECTORIES(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

ADD_LIBRARY(line_fit src/line_fit.cpp ${HPP_FILES})
TARGET_LINK_LIBRARIES(line_fit ${OpenCV_LIBS})

ADD_LIBRARY(plane_fit src/plane_fit.cpp ${HPP_FILES})
TARGET_LINK_LIBRARIES(plane_fit)

ADD_EXECUTABLE(LineFittingSample test/LineFittingSample.cpp src/line_fit.cpp ${HPP_FILES})
TARGET_LINK_LIBRARIES(LineFittingSample ${OpenCV_LIBS})

ADD_EXECUTABLE(PlaneFittingSample test/PlaneFittingSample.cpp src/plane_fit.cpp ${HPP_FILES})
TARGET_LINK_LIBRARIES(PlaneFittingSample ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES})

CATKIN_ADD_GTEST(test_PlaneModel test/test_PlaneModel.cpp ${HPP_FILES})
TARGET_LINK_LIBRARIES(test_PlaneModel ${OpenCV_LIBS}  ${PCL_COMMON_LIBRARIES} ${GTEST_LIBRARIES})
