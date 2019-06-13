add_executable(${PROJECT_NAME}_unit_tests
  angles_test.cpp
  math_test.cpp
  utils_tests_main.cpp
)
target_include_directories(${PROJECT_NAME}_unit_tests
  PUBLIC
  ../include
)
target_link_libraries(${PROJECT_NAME}_unit_tests
  Catch2::Catch2
  ${PROJECT_NAME}
)

