# libbeam
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

`#include <beam/utils/math.hpp>`
