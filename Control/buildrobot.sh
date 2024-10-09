#!/bin/bash

set -o errexit #exit on error

root=$(pwd)

build_dir=${root}/build
bin_dir=${root}/bin
if [ ! -d "$build_dir" ];then
mkdir $build_dir
echo "build create succeed!"
else
echo "build already exist!"
fi
if [ ! -d "$bin_dir" ];then
mkdir $bin_dir
echo "bin create succeed!"
else
echo "bin already exist!"
fi
#build main
# pushd ${build_dir}
cd ${build_dir}
# sudo rm -r -f ./*
cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${root} \
    -Dwebots_sim_def=false \
    -Drealrobot=true ..
make -j8
make install
# popd
cd -