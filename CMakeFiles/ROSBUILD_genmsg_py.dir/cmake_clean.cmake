FILE(REMOVE_RECURSE
  "msg_gen"
  "src/robolink/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/robolink/msg/__init__.py"
  "src/robolink/msg/_RobolinkInfo.py"
  "src/robolink/msg/_RobolinkControl.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
