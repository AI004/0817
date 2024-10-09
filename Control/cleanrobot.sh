#!/bin/bash

set -o errexit #exit on error

root=$(pwd)
build_dir=${root}/build/
#build main

cd ${build_dir}
sudo rm -r -f ./*

cd -
cd ${root}/build
sudo rm -r -f ./*

cd -
# pushd ${root}/webotsim_old/controllers/webots_interface
# sudo rm webots_interface

# popd
# pushd ${root}/webotsim/controllers/webots_interface
# sudo rm webots_interface

# popd