file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/k2_client/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/k2_client/msg/__init__.py"
  "../src/k2_client/msg/_JointOrientationAndType.py"
  "../src/k2_client/msg/_Activities.py"
  "../src/k2_client/msg/_Audio.py"
  "../src/k2_client/msg/_Lean.py"
  "../src/k2_client/msg/_Expressions.py"
  "../src/k2_client/msg/_BodyArray.py"
  "../src/k2_client/msg/_Appearance.py"
  "../src/k2_client/msg/_Body.py"
  "../src/k2_client/msg/_JointPositionAndState.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
