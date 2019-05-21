unset(Ladybug_FOUND)
unset(Ladybug_INCLUDE_DIRS)
unset(Ladybug_LIBRARIES)

message(STATUS "Trying to find Ladybug libs")
find_path(Ladybug_INCLUDE_DIRS
  NAMES
    ladybug.h
  PATHS
    /usr/include/ladybug/
    /usr/local/include/ladybug/
)

find_library(Ladybug_LIBRARIES
  NAMES
    ladybug
  PATHS
    /usr/lib/ladybug/
    /usr/local/lib/ladybug/
)

if (Ladybug_INCLUDE_DIRS AND Ladybug_LIBRARIES)
  set(Ladybug_FOUND 1)
  if (NOT TARGET Ladybug::Ladybug)
    add_library(Ladybug::Ladybug INTERFACE IMPORTED)
    set_target_properties(Ladybug::Ladybug PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${Ladybug_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${Ladybug_LIBRARIES}")
  endif ()
endif (Ladybug_INCLUDE_DIRS AND Ladybug_LIBRARIES)