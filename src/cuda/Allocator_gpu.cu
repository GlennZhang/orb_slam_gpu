#include <cassert>
#include "cuda_runtime.h"  
#include "device_launch_parameters.h"  
#include "helper_cuda.h"
#include <helper_functions.h>
#include <cuda/Allocator.hpp>


#ifdef __cplusplus
#define INITIALIZER(f) \
        static void f(void); \
        struct f##_t_ { f##_t_(void) { f(); } }; static f##_t_ f##_; \
        static void f(void)
#elif defined(_MSC_VER)
#pragma section(".CRT$XCU",read)
#define INITIALIZER2_(f,p) \
        static void f(void); \
        __declspec(allocate(".CRT$XCU")) void (*f##_)(void) = f; \
        __pragma(comment(linker,"/include:" p #f "_")) \
        static void f(void)
#ifdef _WIN64
#define INITIALIZER(f) INITIALIZER2_(f,"")
#else
#define INITIALIZER(f) INITIALIZER2_(f,"_")
#endif
#else
#define INITIALIZER(f) \
        static void f(void) __attribute__((constructor)); \
        static void f(void)
#endif





namespace ORB_SLAM2 { namespace cuda {

size_t Allocator::getPitch(size_t widthSize){
    return 128 + widthSize - widthSize%128;
}

bool Allocator::allocate(cv::cuda::GpuMat* mat, int rows, int cols, size_t elemSize)
{
    if (rows > 1 && cols > 1)
    {
        mat->step = getPitch(elemSize * cols);
        checkCudaErrors(cudaMallocManaged(&mat->data, mat->step * rows));
    }
    else
    {
        // Single row or single column must be continuous
        checkCudaErrors(cudaMallocManaged(&mat->data, elemSize * cols * rows));
        mat->step = elemSize * cols;
    }

    mat->refcount = (int*) new int();

    return true;
}

void Allocator::free(cv::cuda::GpuMat* mat)
{
    checkCudaErrors(cudaFree(mat->datastart));
    delete mat->refcount;
}

cv::cuda::GpuMat::Allocator * gpu_mat_allocator;

} }


namespace {
  using namespace ORB_SLAM2;

  //void __attribute__((constructor)) init() {
  //static void init(){
  INITIALIZER(init){
    // Setup GPU Memory Management
    cuda::gpu_mat_allocator = new cuda::Allocator();
    // cv::cuda::GpuMat::setDefaultAllocator(cuda::gpu_mat_allocator);
  }
}
