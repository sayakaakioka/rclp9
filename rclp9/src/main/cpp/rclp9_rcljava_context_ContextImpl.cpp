#include <iostream>

#include "rcl/context.h"
#include "rcl/error_handling.h"
#include "rcl/init.h"

#include "rclp9_rcljava_context_ContextImpl.h"

JNIEXPORT void JNICALL Java_rclp9_rcljava_context_ContextImpl_nativeDispose(JNIEnv *, jclass, jlong handle){
  if (handle == 0) {
    return;
  }

  rcl_context_t * context = reinterpret_cast<rcl_context_t *>(handle);
  rcl_ret_t ret = rcl_context_fini(context);
  if (ret != RCL_RET_OK) {
    std::cout << "Failed to destroy context: " << rcl_get_error_string().str;
    rcl_reset_error();
    // TODO: throw exception
  }
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_context_ContextImpl_nativeInit(JNIEnv *, jclass, jlong context_handle){
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to init context options: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw exception
        return;
    }

    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(context_handle);
    ret = rcl_init(0, nullptr, &init_options, context);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to init context: " << rcl_get_error_string().str;
        rcl_ret_t ignored_ret = rcl_init_options_fini(&init_options);
        (void)ignored_ret;
        rcl_reset_error();
        // TODO: throw exception
        return;
    }

    rcl_ret_t fini_ret = rcl_init_options_fini(&init_options);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to init context: " << rcl_get_error_string().str;
        rcl_ret_t ignored_ret = rcl_shutdown(context);
        (void)ignored_ret;
        rcl_reset_error();
        // TODO: throw exception
        return;
    }
}

JNIEXPORT jboolean JNICALL Java_rclp9_rcljava_context_ContextImpl_nativeIsValid(JNIEnv *, jclass, jlong context_handle){
    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(context_handle);
    return rcl_context_is_valid(context);
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_context_ContextImpl_nativeShutdown(JNIEnv *, jclass, jlong context_handle){
    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(context_handle);
    if(!rcl_context_is_valid(context)){
        return;
    }

    rcl_ret_t ret = rcl_shutdown(context);
    if(ret != RCL_RET_OK){
        std::cout << "Failed to shutdown context: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw exception
    }
}