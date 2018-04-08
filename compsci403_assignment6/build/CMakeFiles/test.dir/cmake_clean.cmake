file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/compsci403_assignment6/msg"
  "../src/compsci403_assignment6/srv"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
