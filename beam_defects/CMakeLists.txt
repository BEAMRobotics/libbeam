PROJECT(beam_defects)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::containers
    beam::utils
  SOURCES
    src/Defect.cpp
    src/Crack.cpp
    src/Delam.cpp
    src/Spall.cpp
    src/Corrosion.cpp
    src/defect_functions.cpp
    src/extract_functions.cpp
)

# beam_defects dummy code for testing library
add_executable(${PROJECT_NAME}_main
  src/main.cpp
)
target_include_directories(${PROJECT_NAME}_main
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}
)

# delam tests
add_executable(${PROJECT_NAME}_delam_tests
  tests/get_delam_test.cpp
)

target_include_directories(${PROJECT_NAME}_delam_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_delam_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

# spall tests
add_executable(${PROJECT_NAME}_spall_tests
  tests/get_spall_test.cpp
)

target_include_directories(${PROJECT_NAME}_spall_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_spall_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

# crack tests
add_executable(${PROJECT_NAME}_crack_tests
  tests/get_crack_test.cpp
)

target_include_directories(${PROJECT_NAME}_crack_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_crack_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

# corrosion tests
add_executable(${PROJECT_NAME}_corrosion_tests
  tests/get_corrosion_test.cpp
)

target_include_directories(${PROJECT_NAME}_corrosion_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_corrosion_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

file(COPY tests/test_data
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
