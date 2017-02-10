cmake_minimum_required(VERSION 2.8.3)
project(xm_object)
set(PCL_DIR /usr/local/share/pcl-1.8)
set(OpenCV_DIR /usr/local/share/OpenCV)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11 ")
SET(CMAKE_C_COMPILER mpicc)
SET(CMAKE_CXX_COMPILER mpicxx)

link_directories(
lib
/usr/local/lib
)
FIND_PACKAGE( OpenCV REQUIRED )

find_package(catkin REQUIRED 
  PCL REQUIRED 
  OpenCV REQUIRED 
  xm_msgs
  rospy
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  tf
  message_filters
)
find_package(HDF5)
find_package(FLANN)
 find_package(PkgConfig REQUIRED)
 pkg_check_modules(FLANN REQUIRED flann)

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )


catkin_package(
 INCLUDE_DIRS include src  /usr/local/include
 CATKIN_DEPENDS message_runtime
)

include_directories(
  src
   include
  /usr/local/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${FLANN_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIR}
)
include_directories(MPI_INCLUDE_PATH)

 include_directories(${FLANN_INCLUDE_DIRS})
 if(FLANN_CFLAGS)
   add_definitions(${FLANN_CFLAGS})
 endif(FLANN_CFLAGS)

add_library(Filter src/Filter.cpp)

add_library(Train src/Train.cpp)
add_executable(training   src/training.cpp)
target_link_libraries(training Train ${PCL_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} ${OpenCV_LIBRARIES} ${MPI_LIBRARIES}  )

add_executable(getobject src/getobject.cpp )
target_link_libraries(getobject  Filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  ${MPI_LIBRARIES}  ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
add_dependencies(getobject ${PROJECT_NAME}_genmsg)

add_library(init_recognize src/init_recognize.cpp)
add_executable(recognize   src/recognize.cpp)
target_link_libraries(recognize  Filter  init_recognize  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  ${MPI_LIBRARIES}   ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} libhpdf.so libseetaface.so)
add_dependencies(recognize ${PROJECT_NAME}_genmsg)

link_directories(
  ${catkin_LIBRARIES}
    /usr/local/lib
)


set(CMAKE_BUILD_TYPE Release)
