# - Functions and macros used in libbeam

INCLUDE(CMakeParseArguments)
INCLUDE(GNUInstallDirs)

# beam_include_directories: Set a module's public include paths so they are
# usable from both the build and install tree
#
# BEAM_INCLUDE_DIRECTORIES(TARGET <INTERFACE|PUBLIC> dir1 [dir2...])
#
# This function expects relative paths, typically "include". It wraps
# TARGET_INCLUDE_DIRECTORIES, using a generator expression for the includes.
#
# See:
# https://cmake.org/cmake/help/v3.4/manual/cmake-buildsystem.7.html#include-directories-and-usage-requirements
# https://stackoverflow.com/a/25681179
FUNCTION(BEAM_INCLUDE_DIRECTORIES TARGET MODE)
    FOREACH(path ${ARGN})
        TARGET_INCLUDE_DIRECTORIES(${TARGET} ${MODE}
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${path}>
            $<INSTALL_INTERFACE:${path}>)
        INSTALL(DIRECTORY ${path}/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    ENDFOREACH()
ENDFUNCTION(BEAM_INCLUDE_DIRECTORIES)

# beam_add_library: Adds a libbeam component library, setting standard install
# destinations and properties
#
# BEAM_ADD_LIBRARY(Name [source1...])
#
# This convenience function wraps ADD_LIBRARY, adding either a normal library
# with the given sources or a header-only INTERFACE library if no sources are
# given.
#
# It calls INSTALL(...) and sets standard paths for binaries and header files,
# and associates the target with the beamTargets export.
#
# This function also sets the output library name to the target name prefixed
# with "beam_". For example, for the name "utils" the library built would be
# "libbeam_utils.a".
FUNCTION(BEAM_ADD_LIBRARY NAME)

    IF(NOT ARGN)
        # No sources given - header-only library
        ADD_LIBRARY(${NAME} INTERFACE)
    ELSE()
        # Normal library, will be static unless BUILD_SHARED_LIBS is ON
        ADD_LIBRARY(${NAME} ${ARGN})
    ENDIF()

    # Remove "beam_" prefix from exported name
    # This means others can link to beam::utils instead of beam::beam_utils
    STRING(REGEX REPLACE "^beam_" "" unprefixed_name ${NAME})
    SET_TARGET_PROPERTIES(${NAME} PROPERTIES EXPORT_NAME ${unprefixed_name})

    # Add an alias so internal target_link_libraries can also use the namespace
    # The benefit is newer CMake will immediately give an error on typos
    ADD_LIBRARY(beam::${unprefixed_name} ALIAS ${NAME})

    # Add this target to the all-inclusive "beam" library
    SET_PROPERTY(TARGET beam APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${NAME})

    # Add install destinations
    # The variables used here are defined by GNUInstallDirs
    INSTALL(TARGETS ${NAME} EXPORT beamTargets
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    # Note INCLUDES DESTINATION does not actually install anything; it only sets
    # paths on the exported targets. Due to this quirk we also call INSTALL in
    # BEAM_INCLUDE_DIRECTORIES.
ENDFUNCTION(BEAM_ADD_LIBRARY)


# beam_check_module: Declares an optional libbeam component library, and
# returns from the script if it cannot or should not be built.
#
# BEAM_CHECK_MODULE(Name [DEPENDS target1...])
#
# This macro is meant to be called from the CMakeLists script defining the
# libbeam component. It:
#  - Adds a user option BUILD_name, which is ON by default but can be disabled
#  - Checks that each target listed after DEPENDS exists
#
# If the user option is disabled or one of the DEPENDS does not exist, this
# macro returns from the calling script (returning to the top-level CMake
# assuming it was called from a subdirectory). In either case, it prints a
# message whether the component is being built, and the reason if not.
#
# Note this is a macro (which does not have its own scope), so RETURN() exits
# the calling script.
MACRO(BEAM_CHECK_MODULE NAME)

    # Define the arguments this macro accepts
    SET(options "")
    SET(one_value_args "")
    SET(multi_value_args "DEPENDS")
    CMAKE_PARSE_ARGUMENTS(BEAM_COMPONENT
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    # Add a user option
    OPTION(BUILD_${NAME} "Build the ${NAME} library" ON)

    IF(NOT ${BUILD_${NAME}})
        MESSAGE(STATUS "Not building ${NAME}: Disabled by user")
        RETURN()
    ENDIF()

    # Check that each of the given DEPENDS (if any) exists as a target
    #[[FOREACH(dep IN LISTS BEAM_COMPONENT_DEPENDS)
        IF(NOT TARGET ${dep})
            MESSAGE(STATUS "Not building ${NAME}: Requires ${dep}")
            RETURN()
        ENDIF()
    ENDFOREACH()]]

    MESSAGE(STATUS "Building ${NAME}")

ENDMACRO(BEAM_CHECK_MODULE)

# beam_add_module: Does everything needed to add a typical libbeam component
# library.
#
# BEAM_ADD_MODULE(Name
#                [DEPENDS target1...]
#                [SOURCES source1...])
#
# This macro does the equivalent of the following:
#
# BEAM_CHECK_MODULE(Name DEPENDS <depends>)
# BEAM_ADD_LIBRARY(Name <sources>)
# BEAM_INCLUDE_DIRECTORIES(Name PUBLIC "include")
# TARGET_LINK_LIBRARIES(Name PUBLIC <depends>)
#
# If no SOURCES are given (for a header-only library) INTERFACE is used instead
# of PUBLIC.
MACRO(BEAM_ADD_MODULE NAME)

    # Define the arguments this macro accepts
    SET(options "")
    SET(one_value_args "")
    SET(multi_value_args DEPENDS SOURCES)
    CMAKE_PARSE_ARGUMENTS(BEAM_ADD_MODULE
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    # Check if the module should be built, based on options and dependencies
    BEAM_CHECK_MODULE(${NAME} DEPENDS ${BEAM_ADD_MODULE_DEPENDS})

    IF(BEAM_ADD_MODULE_SOURCES)
        SET(link_type PUBLIC)
    ELSE()
        # No sources given - header-only library
        SET(link_type INTERFACE)
    ENDIF()

    BEAM_ADD_LIBRARY(${NAME} ${BEAM_ADD_MODULE_SOURCES})

    # Use these headers when building, and make clients use them
    BEAM_INCLUDE_DIRECTORIES(${NAME} ${link_type} "include")

    # Depend on these modules and external libraries, and make clients use them
    TARGET_LINK_LIBRARIES(${NAME} ${link_type} ${BEAM_ADD_MODULE_DEPENDS})
ENDMACRO(BEAM_ADD_MODULE)

# beam_add_gtest: Add a gtest target
#
# BEAM_ADD_GTEST(Name [DISABLED] src1 [src2...])
#
# The test will be added to the tests run by "make test", unless DISABLED is
# given. It will be linked against the needed gtest libraries. Any other links
# can be made separately with the target_link_libraries command.
FUNCTION(BEAM_ADD_GTEST NAME)

    # Define the arguments this function accepts
    SET(options DISABLED)
    SET(one_value_args "")
    SET(multi_value_args "")
    CMAKE_PARSE_ARGUMENTS(BEAM_ADD_GTEST
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    # Build the test executable using the given sources
    ADD_EXECUTABLE(${NAME} ${BEAM_ADD_GTEST_UNPARSED_ARGUMENTS})

    # Link gtest libraries including one providing main()
    TARGET_LINK_LIBRARIES(${NAME} GTest::Main)

    # Put the test executable in the gtests/ directory
    SET_TARGET_PROPERTIES(${NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/gtests)

    # Build this test on "make tests"
    ADD_DEPENDENCIES(tests ${NAME})

    IF(NOT BEAM_ADD_GTEST_DISABLED)
        # Add the executable as a test, so it runs with "make test"
        ADD_TEST(NAME ${NAME} COMMAND ${NAME})
    ENDIF()

ENDFUNCTION(BEAM_ADD_GTEST)