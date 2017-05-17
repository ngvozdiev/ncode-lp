cmake_minimum_required(VERSION 2.8.7)

# If there is already a target called ncode_common someone has already
# added this project. Will ignore.
if(TARGET ncode_lp)
  return()
endif(TARGET ncode_lp)

project(NCodeLP)
option(NCODE_LP_DISABLE_TESTS "If tests should be compiled or not" ON)
option(NCODE_LP_DEBUG "A debug build" OFF)
option(NCODE_LP_ASAN "Compile with ASAN on" OFF)
option(NCODE_LP_TSAN "Compile with TSAN on" OFF)

set(NCODE_LP_BASE_FLAGS "-g -std=c++11 -pedantic-errors -Winit-self -Woverloaded-virtual -Wuninitialized -Wall -Wextra -fno-exceptions")
set(NCODE_LP_BASE_LD_FLAGS "")
if (NCODE_LP_ASAN)
   set(NCODE_LP_BASE_FLAGS "${NCODE_LP_BASE_FLAGS} -fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls")
   set(NCODE_LP_BASE_LD_FLAGS "${NCODE_LP_BASE_LD_FLAGS} -fsanitize=address")
endif()
if (NCODE_LP_TSAN)
   set(NCODE_LP_BASE_FLAGS "${NCODE_LP_BASE_FLAGS} -fsanitize=thread -fno-omit-frame-pointer -fno-optimize-sibling-calls")
   set(NCODE_LP_BASE_LD_FLAGS "${NCODE_LP_BASE_LD_FLAGS} -fsanitize=thread")
endif()

if (NCODE_LP_DEBUG)
  set(NCODE_LP_BASE_FLAGS "${NCODE_LP_BASE_FLAGS} -O0 -fno-omit-frame-pointer")
else()
  set(NCODE_LP_BASE_FLAGS "${NCODE_LP_BASE_FLAGS} -O3 -march=native -DNDEBUG")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NCODE_LP_BASE_FLAGS}")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${NCODE_LP_BASE_LD_FLAGS}")

if (NOT NCODE_LP_DISABLE_TESTS)
   include(CTest)
   add_subdirectory(external/googletest)
   macro(add_test_exec name src_file deps)
     add_executable(${name} ${src_file})
     target_link_libraries(${name} gtest gmock_main ${deps} ${ARGN})
     add_test(NAME ${name} COMMAND ${name})
   endmacro(add_test_exec)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_extensions)

# Need a linear optimizer. Will look for both GLPK and CPLEX.
find_package(GLPK)
find_package(CPLEX)

# Check which optimization package is found
set(FORCE_GLPK false CACHE BOOL "If GLPK and CPLEX are found, by default we build against CPLEX. Setting this to true forces to build against GLPK")
if(CPLEX_FOUND)
    if(GLPK_FOUND AND FORCE_GLPK)
        set(OPTIMIZER_INCLUDE_DIRS ${GLPK_INCLUDE_DIR})
        set(OPTIMIZER_LIBRARIES ${GLPK_LIBRARY})
	set(OPTIMIZER "GLPK")
    else()
        set(OPTIMIZER_INCLUDE_DIRS ${CPLEX_INCLUDE_DIRS})
	if(APPLE)
	    set(OPTIMIZER_LIBRARIES -Wl,-force_load ${CPLEX_LIBRARIES})
	elseif(UNIX)
	    set(OPTIMIZER_LIBRARIES -Wl,-whole-archive ${CPLEX_LIBRARIES} -Wl,-no-whole-archive)
	endif()
	   set(OPTIMIZER "CPLEX")
    endif()
else()
    if(GLPK_FOUND)
        set(OPTIMIZER_INCLUDE_DIRS ${GLPK_INCLUDE_DIR})
        set(OPTIMIZER_LIBRARIES ${GLPK_LIBRARY})
	set(OPTIMIZER "GLPK")
    else()
        message(FATAL_ERROR "No LP optimizer found at all!")
    endif()
endif()

configure_file(
  "${PROJECT_SOURCE_DIR}/src/config.h.in"
  "${PROJECT_BINARY_DIR}/ncode_config.h")

if (NOT TARGET ncode_common)
   EXECUTE_PROCESS(COMMAND "git" "clone" "https://github.com/ngvozdiev/ncode-common.git" "${CMAKE_BINARY_DIR}/external/ncode_common")
   add_subdirectory(${CMAKE_BINARY_DIR}/external/ncode_common)
endif()

if (NOT TARGET ncode_net)
   EXECUTE_PROCESS(COMMAND "git" "clone" "https://github.com/ngvozdiev/ncode-net.git" "${CMAKE_BINARY_DIR}/external/ncode_net")
   add_subdirectory(${CMAKE_BINARY_DIR}/external/ncode_net)
endif()

include_directories(${CMAKE_SOURCE_DIR}/external ${PROJECT_BINARY_DIR} ${OPTIMIZER_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/external)

set(LP_HEADER_FILES src/lp.h src/mc_flow.h src/demand_matrix.h)
add_library(ncode_lp STATIC src/lp.cc src/mc_flow.cc src/demand_matrix.cc)
target_link_libraries(ncode_lp ncode_net ${OPTIMIZER_LIBRARIES})

if (NOT NCODE_LP_DISABLE_TESTS)
  add_test_exec(lp_test src/lp_test.cc ncode_lp)
  add_test_exec(lp_mc_flow_test src/mc_flow_test.cc ncode_lp)
  add_test_exec(lp_demand_matrix_test src/demand_matrix_test.cc ncode_lp)
endif()
