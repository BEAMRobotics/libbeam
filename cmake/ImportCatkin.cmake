# Temporary fix to create imported target for catkin
# Could be extended in the future to create targets for each package
# we find in catkin.

IF(catkin_FOUND AND NOT TARGET catkin::catkin)
  ADD_LIBRARY(catkin::catkin INTERFACE IMPORTED)
  SET_PROPERTY(TARGET catkin::catkin PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${catkin_INCLUDE_DIRS}")
    #target_link_libraries(catkin::catkin INTERFACE ${catkin_LIBRARIES})
ENDIF()
