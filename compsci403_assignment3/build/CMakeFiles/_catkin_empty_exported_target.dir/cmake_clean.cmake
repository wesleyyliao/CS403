file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/compsci403_assignment3/msg"
  "../src/compsci403_assignment3/srv"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/_catkin_empty_exported_target.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
