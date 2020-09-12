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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/rahul/catkin_ws/src/joint_state_publisher-kinetic-devel/joint_state_publisher"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/rahul/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/rahul/catkin_ws/install/lib/python2.7/dist-packages:/home/rahul/catkin_ws/build/joint_state_publisher/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/rahul/catkin_ws/build/joint_state_publisher" \
    "/usr/bin/python2" \
    "/home/rahul/catkin_ws/src/joint_state_publisher-kinetic-devel/joint_state_publisher/setup.py" \
    build --build-base "/home/rahul/catkin_ws/build/joint_state_publisher" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/rahul/catkin_ws/install" --install-scripts="/home/rahul/catkin_ws/install/bin"
