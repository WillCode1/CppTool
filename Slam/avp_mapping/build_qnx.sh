#!/bin/bash
if [ $# -eq 1 ]; then
    build_target=$1
else
    build_target=Executable
fi

export CUDA_BIN_PATH=/usr/local/cuda-10.1/bin
export CUDA_PATH=/usr/local/cuda-10.1
mkdir -p build_qnx;cd build_qnx; cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TARGET=$build_target -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-V5Q.cmake -DVIBRANTE_PDK:STRING=/work/drivePDK_qnx/drive-t186ref-qnx ../;make -j4
