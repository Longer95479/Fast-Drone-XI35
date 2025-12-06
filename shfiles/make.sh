# only build quadrotor_msgs package
catkin_make --pkg quadrotor_msgs -DCMAKE_BUILD_TYPE=Release
# build all packages
catkin_make -DCMAKE_BUILD_TYPE=Release
# build all packages with compile_commands.json
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release