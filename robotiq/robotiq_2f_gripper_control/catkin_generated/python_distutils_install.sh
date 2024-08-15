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

echo_and_run cd "/home/xj/Documents/catkin_ws/src/robotiq/robotiq_2f_gripper_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/xj/Documents/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/xj/Documents/catkin_ws/install/lib/python2.7/dist-packages:/home/xj/Documents/catkin_ws/src/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/xj/Documents/catkin_ws/src" \
    "/usr/bin/python2" \
    "/home/xj/Documents/catkin_ws/src/robotiq/robotiq_2f_gripper_control/setup.py" \
     \
    build --build-base "/home/xj/Documents/catkin_ws/src/robotiq/robotiq_2f_gripper_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/xj/Documents/catkin_ws/install" --install-scripts="/home/xj/Documents/catkin_ws/install/bin"
