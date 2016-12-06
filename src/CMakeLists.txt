INCLUDE_DIRECTORIES("../include/iris")
INCLUDE_DIRECTORIES("../include/snopt")
INCLUDE_DIRECTORIES("../include/iris/eigen3")
#
#IF(DEBUG)
#  ADD_DEFINITIONS(-DDEBUG)
#ENDIF(DEBUG)
#
#IF(VERSION_3D)
#  ADD_DEFINITIONS(-DVERSION_3D)
#ENDIF(VERSION_3D)
#
FIND_LIBRARY(IRIS_LIB iris HINTS "../lib/iris")
FIND_LIBRARY(IRIS_GEOMETRY_LIB iris_geometry HINTS "../lib/iris")
#ADD_EXECUTABLE(irisDemo ./iris_demo.cpp)
#TARGET_LINK_LIBRARIES(irisDemo
#  ${IRIS_LIB}
#  ${IRIS_GEOMETRY_LIB})
#
#ADD_EXECUTABLE(snoptDemo ./sntoya.cpp)
FIND_LIBRARY(SNOPT7_LIB snopt7 HINTS "../lib/snopt")
FIND_LIBRARY(SNOPT7_CPP_LIB snopt7_cpp HINTS "../lib/snopt")
#TARGET_LINK_LIBRARIES(snoptDemo
#  ${SNOPT7_CPP_LIB}
#  ${SNOPT7_LIB})
#
#target_compile_features(snoptDemo PRIVATE cxx_range_for)
#
#ADD_EXECUTABLE(uav ./uav.cpp)
#TARGET_LINK_LIBRARIES(uav
#  ${IRIS_LIB}
#  ${IRIS_GEOMETRY_LIB}
#  ${SNOPT7_CPP_LIB}
#  ${SNOPT7_LIB})

ADD_EXECUTABLE(main ./main.cpp ./LargestConvexPolytope.cpp ./util.cpp)
TARGET_LINK_LIBRARIES(main
  ${IRIS_LIB}
  ${IRIS_GEOMETRY_LIB})
#
target_compile_features(main PRIVATE cxx_range_for)

#MESSAGE(STATUS "libsnopt7.so: " ${SNOPT7_LIB})
#MESSAGE(STATUS "libsnopt7_cpp.so: " ${SNOPT7_CPP_LIB})

ADD_SUBDIRECTORY(test test)

