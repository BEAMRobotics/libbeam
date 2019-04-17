# Define OpenCV::OpenCV target
IF(NOT TARGET OpenCV::OpenCV)
  ADD_LIBRARY(OpenCV::OpenCV INTERFACE IMPORTED)
  SET_TARGET_PROPERTIES(OpenCV::OpenCV PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${OpenCV_LIBS}")
ENDIF()