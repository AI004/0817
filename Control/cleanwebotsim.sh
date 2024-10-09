#!/bin/bash

set -o errexit #exit on error

root=$(pwd)
build_dir=${root}/build/
#build main
cd ${build_dir}
sudo rm -r -f ./*

cd -
# cd ${root}/bin
# sudo rm -r -f ./*

# cd -
# cd ${root}/webotsim_old/controllers/webots_interface
# sudo rm webots_interface

# cd -
cd ${root}/Webots/controllers/mpc_controller
sudo rm mpc_controller

cd -