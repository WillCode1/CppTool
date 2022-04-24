#!/bin/bash
if [ $# -eq 1 ]; then
    build_target=$1
else
    build_target=Executable
fi

export CUDA_BIN_PATH=/usr/local/cuda-10.0/bin
export CUDA_PATH=/usr/local/cuda-10.0
mkdir -p build_agx;
cd build_agx;
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TARGET=$build_target -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-V5L-AGX.cmake -DVIBRANTE_PDK:STRING=/work/DriveSDK/drive-t186ref-linux ../;make -j4
