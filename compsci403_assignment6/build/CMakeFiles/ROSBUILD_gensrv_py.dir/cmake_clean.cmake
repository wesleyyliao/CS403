file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/compsci403_assignment6/msg"
  "../src/compsci403_assignment6/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/compsci403_assignment6/srv/__init__.py"
  "../src/compsci403_assignment6/srv/_ExtendNodeSrv.py"
  "../src/compsci403_assignment6/srv/_RRTPlanSrv.py"
  "../src/compsci403_assignment6/srv/_RandomConfigSrv.py"
  "../src/compsci403_assignment6/srv/_BuildRRTSrv.py"
  "../src/compsci403_assignment6/srv/_CheckExtensionSrv.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
