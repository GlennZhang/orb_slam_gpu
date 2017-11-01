#include <cuda/Cuda.hpp>
#include "cuda_runtime.h"  
#include "device_launch_parameters.h"  
#include "helper_cuda.h"
#include <helper_functions.h>

namespace ORB_SLAM2 { namespace cuda {
  void deviceSynchronize() {
    checkCudaErrors( cudaDeviceSynchronize() );
  }
} }
