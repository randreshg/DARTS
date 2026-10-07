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
    rt_sticky_place.cpp)

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
