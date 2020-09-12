#!/bin/sh

DESTDIR=/home/rahul/catkin_ws/build/nlopt/nlopt_install make install

cp -r /home/rahul/catkin_ws/build/nlopt/nlopt_install//home/rahul/catkin_ws/install/* /home/rahul/catkin_ws/devel/.private/nlopt/
