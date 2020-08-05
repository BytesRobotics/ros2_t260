#!/bin/bash

echo "Please refer to https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md for any issue"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y -qq install librealsense2 librealsense2-utils librealsense2-dev librealsense2-dbg
