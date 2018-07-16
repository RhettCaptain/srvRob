#! /bin/sh
mkdir ~/srv_rob_conf
#SRC_PATH=$(cd `dirname $0`;cd ..; pwd)
CONF_PATH=~/srv_rob_conf
cp initPose.xml ${CONF_PATH}
cp laser_conf.yaml ${CONF_PATH}
echo "source ~/develop/ws_srv_rob/devel/setup.bash" >> ~/.bashrc
