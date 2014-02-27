MESSAGE(STATUS "Configuring Graphic end..."})

FIND_PACKAGE( OpenCV REQUIRED )

# Build Library
ADD_LIBRARY(graphicend
  ImageReader.cpp GraphicEnd.cpp FeatureGrabber.cpp)

TARGET_LINK_LIBRARIES( graphicend ${OpenCV_LIBS} yaml-cpp )

# Build test executable
ADD_EXECUTABLE( run_graphic run_graphic_end.cpp )

TARGET_LINK_LIBRARIES( run_graphic graphicend )
