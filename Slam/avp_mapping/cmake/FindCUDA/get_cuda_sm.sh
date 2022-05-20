#!/bin/bash
#
# Prints the compute capability of all CUDA device installed
# on the system usable with the nvcc `--gpu-code=` flag

timestamp=$(date +%s.%N)
gcc_binary=${CMAKE_CXX_COMPILER:-$(which c++)}
cuda_root=${CUDA_DIR:-/usr/local/cuda}
CUDA_INCLUDE_DIRS=${CUDA_INCLUDE_DIRS:-${cuda_root}/include}
CUDA_CUDART_LIBRARY=${CUDA_CUDART_LIBRARY:-${cuda_root}/lib64/libcudart.so}
generated_binary="/tmp/cuda-compute-version-helper-$$-$timestamp"

# create a 'here document' that is code we compile and use to probe the card
source_code="$(cat << EOF
#include <cuda_runtime_api.h>
#include <sstream>
#include <iostream>

int main() {
  auto device_count = int{};
  auto status = cudaGetDeviceCount(&device_count);
  if (status != cudaSuccess) {
    std::cerr << "cudaGetDeviceCount() failed: " << cudaGetErrorString(status)
              << std::endl;
    return -1;
  }
  if (!device_count) {
    std::cerr << "No cuda devices found" << std::endl;
    return -1;
  }

  std::stringstream flag;
  flag << "--gpu-code=";

  for (auto i = 0; i < device_count; ++i) {
    if (i != 0)
      flag << ",";

    auto prop = cudaDeviceProp{};
    status = cudaGetDeviceProperties(&prop, i);
    if (status != cudaSuccess) {
      std::cerr
          << "cudaGetDeviceProperties() for device ${device_index} failed: "
          << cudaGetErrorString(status) << std::endl;
      return -1;
    }

    flag << "sm_" << prop.major * 10 + prop.minor;
  }

  std::cout << flag.str() << std::endl;
  return 0;
}
EOF
)"
echo "$source_code" | $gcc_binary -std=c++11 -x c++ -I"$CUDA_INCLUDE_DIRS" -o "$generated_binary" - -x none "$CUDA_CUDART_LIBRARY"

# probe the card and cleanup
$generated_binary
ret_code=$?
rm $generated_binary
exit $ret_code
