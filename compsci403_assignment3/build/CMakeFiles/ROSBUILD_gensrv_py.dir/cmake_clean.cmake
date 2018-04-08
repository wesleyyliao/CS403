file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/compsci403_assignment3/msg"
  "../src/compsci403_assignment3/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/compsci403_assignment3/srv/__init__.py"
  "../src/compsci403_assignment3/srv/_FitMinimalPlaneSrv.py"
  "../src/compsci403_assignment3/srv/_TransformPointSrv.py"
  "../src/compsci403_assignment3/srv/_FitBestPlaneSrv.py"
  "../src/compsci403_assignment3/srv/_FindInliersSrv.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
