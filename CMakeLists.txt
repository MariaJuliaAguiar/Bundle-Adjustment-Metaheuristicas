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

add_executable(BAMeta src/main.cpp)
target_compile_options(BAMeta PRIVATE ${OpenMP_FLAGS})
target_link_libraries(BAMeta  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

