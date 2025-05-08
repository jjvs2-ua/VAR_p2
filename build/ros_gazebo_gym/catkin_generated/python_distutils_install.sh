#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jose/Escritorio/VAR/p2/src/ros_gazebo_gym"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jose/Escritorio/VAR/p2/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jose/Escritorio/VAR/p2/install/lib/python3/dist-packages:/home/jose/Escritorio/VAR/p2/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jose/Escritorio/VAR/p2/build" \
    "/usr/bin/python3" \
    "/home/jose/Escritorio/VAR/p2/src/ros_gazebo_gym/setup.py" \
    egg_info --egg-base /home/jose/Escritorio/VAR/p2/build/ros_gazebo_gym \
    build --build-base "/home/jose/Escritorio/VAR/p2/build/ros_gazebo_gym" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jose/Escritorio/VAR/p2/install" --install-scripts="/home/jose/Escritorio/VAR/p2/install/bin"
