add_library(a_star_search SHARED
  a_star_search.cc
)

pybind11_add_module(a_star
  python_interface.cc
)
target_link_libraries(a_star PRIVATE a_star_search)

if(BUILD_TESTING)
  add_executable(test_same_node_path test_same_node_path.cc)
  target_link_libraries(test_same_node_path PRIVATE a_star_search)
  add_test(NAME test_same_node_path COMMAND test_same_node_path)
endif()
