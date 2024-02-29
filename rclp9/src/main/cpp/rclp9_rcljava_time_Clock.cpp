#include <iostream>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rclp9_rcljava_time_Clock.h"

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_time_Clock_nativeCreateClockHandle(JNIEnv *, jclass){
    rcl_clock_type_t clock_type = RCL_SYSTEM_TIME;
    rcl_clock_t * clock = static_cast<rcl_clock_t *>(malloc(sizeof(rcl_clock_t)));
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rcl_ret_t ret = rcl_clock_init(clock_type, clock, &allocator);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to init clock: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
        return 0;
    }

    return reinterpret_cast<jlong>(clock);
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_time_Clock_nativeDispose(JNIEnv *, jclass, jlong clock_handle){
    if(clock_handle == 0){
        return;
    }

    rcl_clock_t * clock = reinterpret_cast<rcl_clock_t *>(clock_handle);
    rcl_ret_t ret = rcl_clock_fini(clock);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to destroy clock: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}