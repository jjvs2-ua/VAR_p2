execute_process(COMMAND "/home/jose/Escritorio/VAR/p2/build/ros_gazebo_gym/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jose/Escritorio/VAR/p2/build/ros_gazebo_gym/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
