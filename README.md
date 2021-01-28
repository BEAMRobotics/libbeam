# libbeam
[![Build Status](https://travis-ci.com/BEAMRobotics/libbeam.svg?token=zshhVvp9R3DJ7YGGYs6z&branch=master)](https://travis-ci.com/BEAMRobotics/libbeam)

private library for all internal software

## Example:

To use this library inside your program, add the following to your CMakeLists.txt file:


```
FIND_PACKAGE(beam REQUIRED utils [ADD ANY OTHER MODULES NEEDED])
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_node
	[...]
	beam::beam
	[...]
)
```

Then you can include the headers in your .cpp and .hpp files, e.g.:

`#include <beam_utils/math.h>`

## Using libbeam with ROS

When including libbeam in ROS packages, you need to be careful linking to catkin packages. Libbeam defines its own catkin dependencies, and therefore this may override the catkin dependencies you are including in your ROS package. To get around this, for each catkin packages not used in libbeam, you will have to call: 

```
find_package(package)
catkin_package(
	include 
	${package_INCLUDE_DIRS})
target_link_libraries(project_name
	${package_LIBRARIES})
```

For an example implementation, see [example_package](https://github.com/BEAMRobotics/beam_robotics/tree/master/templates/example_package_nodelet) in beam_robotics
