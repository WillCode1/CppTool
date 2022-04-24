#!/bin/bash
if [ $# -eq 1 ]; then
    build_target=$1
else
    build_target=Executable
fi

export CUDA_BIN_PATH=/usr/local/cuda-9.2/bin
export CUDA_PATH=/usr/local/cuda-9.2
mkdir -p build_px2;cd build_px2; cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TARGET=$build_target -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-V5L.cmake -DVIBRANTE_PDK:STRING=/work/DriveSDK/drive-t186ref-linux ../;make -j4
