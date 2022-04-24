#!/bin/bash
export CUDA_BIN_PATH=/usr/local/cuda-9.2/bin
export CUDA_PATH=/usr/local/cuda-9.2
mkdir -p build_x86;cd build_x86; cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TARGET=Executable ../;make -j4

cd ..;mkdir -p bin/x86_64-linux
cp  build_x86/bin/* bin/x86_64-linux/
