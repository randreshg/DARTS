# Runtime tests (opt-in: -DDARTS_BUILD_RUNTIME_TESTS=ON).
#
# unit_<x>.cpp builds darts-unit-<x> (ctest label "unit"); rt_<x>.cpp builds
# darts-rt-<x> (label "runtime", run serially because each one starts a
# pinned Runtime). Exit code 4 means the host or the process affinity mask
# does not have the shape a test needs; ctest reports it as skipped.
#
#   ctest -L unit
#   ctest -L runtime          (on an otherwise idle node)

set(DARTS_RUNTIME_TESTS
    rt_directed.cpp
    rt_sticky_place.cpp
    unit_numa_topology.cpp
    rt_numa_paired.cpp
    rt_node_place.cpp
    rt_sibling_steal.cpp
    rt_su_barrier.cpp
    rt_persistent_root.cpp
    unit_rearm.cpp
    unit_resident_table.cpp
    rt_resident.cpp
    rt_wake.cpp)

# rt_poll_hint counts real lock acquisitions and idle back-off sleeps: every
# pthread_mutex_lock and usleep reference in the test and in libdarts goes
# through its __wrap_ functions (GNU ld / lld --wrap, Linux only).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  list(APPEND DARTS_RUNTIME_TESTS rt_poll_hint.cpp)
endif()

foreach(src ${DARTS_RUNTIME_TESTS})
  get_filename_component(stem ${src} NAME_WE)
  string(REPLACE "_" "-" name "darts-${stem}")
  add_executable(${name} ${src})
  target_link_libraries(${name} darts)
  add_test(NAME ${name} COMMAND ${name})
  if(stem MATCHES "^unit_")
    set_tests_properties(${name} PROPERTIES LABELS "unit" TIMEOUT 300 SKIP_RETURN_CODE 4)
  else()
    set_tests_properties(${name} PROPERTIES LABELS "runtime" RUN_SERIAL TRUE TIMEOUT 900 SKIP_RETURN_CODE 4)
  endif()
endforeach()

if(TARGET darts-rt-poll-hint)
  target_link_libraries(darts-rt-poll-hint "-Wl,--wrap=pthread_mutex_lock" "-Wl,--wrap=usleep")
endif()
