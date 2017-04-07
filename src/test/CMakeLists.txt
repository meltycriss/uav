INCLUDE_DIRECTORIES("../../include/iris")
INCLUDE_DIRECTORIES("../../include/snopt")
INCLUDE_DIRECTORIES("../../include/iris/eigen3")
INCLUDE_DIRECTORIES("..")
INCLUDE_DIRECTORIES("../../include/drake")

IF(DEBUG)
  ADD_DEFINITIONS(-DDEBUG)
ENDIF(DEBUG)

IF(VERSION_3D)
  ADD_DEFINITIONS(-DVERSION_3D)
ENDIF(VERSION_3D)

FIND_LIBRARY(IRIS_LIB iris HINTS "../../lib/iris")
FIND_LIBRARY(IRIS_GEOMETRY_LIB iris_geometry HINTS "../../lib/iris")

#ADD_EXECUTABLE(irisDemo ./iris_demo.cpp)
#TARGET_LINK_LIBRARIES(irisDemo
#  ${IRIS_LIB}
#  ${IRIS_GEOMETRY_LIB})

FIND_LIBRARY(SNOPT7_LIB snopt7 HINTS "../../lib/snopt")
FIND_LIBRARY(SNOPT7_CPP_LIB snopt7_cpp HINTS "../../lib/snopt")

#ADD_EXECUTABLE(snoptDemo ./sntoya.cpp)
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

FIND_LIBRARY(DRAKE_MATH_LIB drakeMath HINTS "../../lib/drake")

#ADD_EXECUTABLE(largestConvexPolytopeTest ./LargestConvexPolytopeTest.cpp ../LargestConvexPolytope.cpp ../util.cpp ../definition.cpp)
#TARGET_LINK_LIBRARIES(largestConvexPolytopeTest
#  ${IRIS_LIB}
#  ${IRIS_GEOMETRY_LIB})
#target_compile_features(largestConvexPolytopeTest PRIVATE cxx_range_for)

#ADD_EXECUTABLE(optimalFormationTest ./OptimalFormationTest.cpp ../LargestConvexPolytope.cpp ../util.cpp ../OptimalFormation.cpp ../definition.cpp)
#TARGET_LINK_LIBRARIES(optimalFormationTest
#  ${IRIS_LIB}
#  ${IRIS_GEOMETRY_LIB}
#  ${SNOPT7_CPP_LIB}
#  ${SNOPT7_LIB}
#  ${DRAKE_MATH_LIB}
#  )
#
#target_compile_features(optimalFormationTest PRIVATE cxx_range_for)


#ADD_EXECUTABLE(absToRelaTest ./AbsToRelaTest.cpp ../LargestConvexPolytope.cpp ../util.cpp ../OptimalFormation.cpp ../definition.cpp)
#TARGET_LINK_LIBRARIES(absToRelaTest
#  ${IRIS_LIB}
#  ${IRIS_GEOMETRY_LIB}
#  ${SNOPT7_CPP_LIB}
#  ${SNOPT7_LIB}
#  ${DRAKE_MATH_LIB}
#  )
#
#target_compile_features(absToRelaTest PRIVATE cxx_range_for)

#ADD_EXECUTABLE(Sphere ./Sphere.cpp)
#FIND_LIBRARY(RVO_LIB RVO HINTS "../../lib/orca")
#TARGET_LINK_LIBRARIES(Sphere ${RVO_LIB})
#
#target_compile_features(Sphere PRIVATE cxx_range_for)

#ADD_EXECUTABLE(Blocks ./Blocks.cpp)
#TARGET_LINK_LIBRARIES(Blocks ${RVO2D_LIB})
#
#target_compile_features(Blocks PRIVATE cxx_range_for)

# -DV2D=ON: planar version
IF(V2D)
  ADD_DEFINITIONS(-DV2D)
  MESSAGE(STATUS "planar version")

  INCLUDE_DIRECTORIES("../../include/orca2d")
  FIND_LIBRARY(RVO2D_LIB RVO HINTS "../../lib/orca2d")

  ADD_EXECUTABLE(uavTest ./UavTest.cpp ../LargestConvexPolytope.cpp ../util.cpp ../OptimalFormation.cpp ../definition.cpp)
  TARGET_LINK_LIBRARIES(uavTest
    ${IRIS_LIB}
    ${IRIS_GEOMETRY_LIB}
    ${SNOPT7_CPP_LIB}
    ${SNOPT7_LIB}
    ${DRAKE_MATH_LIB}
    ${RVO2D_LIB}
    proto
    )
  target_compile_features(uavTest PRIVATE cxx_range_for)
ELSE(V2D)
  INCLUDE_DIRECTORIES("../../include/orca")
  FIND_LIBRARY(RVO3D_LIB RVO HINTS "../../lib/orca")

  ADD_EXECUTABLE(uavTest3D ./UavTest3D.cpp ../LargestConvexPolytope.cpp ../util.cpp ../OptimalFormation.cpp ../definition.cpp)
  TARGET_LINK_LIBRARIES(uavTest3D
    ${IRIS_LIB}
    ${IRIS_GEOMETRY_LIB}
    ${SNOPT7_CPP_LIB}
    ${SNOPT7_LIB}
    ${DRAKE_MATH_LIB}
    ${RVO3D_LIB}
    proto
    )
  target_compile_features(uavTest3D PRIVATE cxx_range_for)

ENDIF(V2D)

ADD_EXECUTABLE(HungarianTest ./HungarianTest.cpp ../util.cpp ../definition.cpp)
TARGET_LINK_LIBRARIES(HungarianTest
  ${IRIS_LIB}
  ${IRIS_GEOMETRY_LIB}
  ${SNOPT7_CPP_LIB}
  ${SNOPT7_LIB}
  ${DRAKE_MATH_LIB}
  ${RVO3D_LIB}
  proto
  )
target_compile_features(HungarianTest PRIVATE cxx_range_for)



