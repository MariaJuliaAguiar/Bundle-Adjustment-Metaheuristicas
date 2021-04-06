cmake_minimum_required(VERSION 2.8.3)
project(BAMeta)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS OPENMP FOUND)
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  SET(OpenMP_LIBS ${OpenMP_libomp_LIBRARY})
endif()
find_package(PCL 1.8.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenMP_INCLUDE_DIRS})
include_directories("${CMAKE_SOURCE_DIR}/include/")

SET(HEADERS    include/utils.hpp include/argument.hpp include/benchmark.hpp  include/benchmarkFactory.hpp include/benchmark/fob.hpp include/gwo.hpp include/GWOException.hpp)
add_executable(BAMeta ${HEADERS} src/main.cpp src/utils.cpp src/argument.cc src/benchmark.cc src/benchmarkFactory.cc benchmark/fob.cc src/gwo.cc src/GWOException.cc)
target_compile_options(BAMeta PRIVATE ${OpenMP_FLAGS})
target_link_libraries(BAMeta  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

