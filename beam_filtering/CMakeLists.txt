PROJECT(beam_filtering)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
  SOURCES
    src/Filter.cpp  
    src/DROR.cpp
    src/ROR.cpp
    src/CropBox.cpp
    src/VoxelDownsample.cpp
    src/Utils.cpp
)

################ tests ##################

add_executable(${PROJECT_NAME}_dror_tests
  tests/DRORTest.cpp
)

target_include_directories(${PROJECT_NAME}_dror_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_dror_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_cropbox_tests
  tests/CropBoxTest.cpp
)

target_include_directories(${PROJECT_NAME}_cropbox_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_cropbox_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_voxeldownsample_tests
  tests/VoxelDownsampleTest.cpp
)

target_include_directories(${PROJECT_NAME}_voxeldownsample_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_voxeldownsample_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_utils_tests
  tests/UtilsTests.cpp
)

target_include_directories(${PROJECT_NAME}_utils_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_utils_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)