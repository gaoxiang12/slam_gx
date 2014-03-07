MESSAGE(STATUS "Configuring Graphic end..."})

FIND_PACKAGE( OpenCV REQUIRED )
MESSAGE(STATUS "Eigen3 include dir is " ${EIGEN3_INCLUDE_DIR})
MESSAGE(STATUS "G2O include dir is " ${G2O_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

# Build Library
ADD_LIBRARY(graphicend
  ImageReader.cpp GraphicEnd.cpp FeatureGrabber.cpp FeatureManager.cpp)

TARGET_LINK_LIBRARIES( graphicend ${OpenCV_LIBS} yaml-cpp g2o_core )

ADD_LIBRARY(slamend
  Slam_end.cpp
  )
TARGET_LINK_LIBRARIES( slamend
  ${OpenCV_LIBS} yaml-cpp g2o_core
  g2o_types_slam2d g2o_solver_csparse
  g2o_csparse_extension ${CSPARSE_LIBRARY})


# Build test executable
ADD_EXECUTABLE( run_graphic run_graphic_end.cpp )
ADD_EXECUTABLE( run_graphic_and_slam run_graphic_and_slam.cpp)

TARGET_LINK_LIBRARIES( run_graphic graphicend )
TARGET_LINK_LIBRARIES( run_graphic_and_slam graphicend slamend )
