#include <iostream>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rclp9_rcljava_time_WallClockTimerImpl.h"

JNIEXPORT void JNICALL Java_rclp9_rcljava_time_WallClockTimerImpl_nativeCallTimer(JNIEnv *, jclass, jlong wall_timer_handle){
    rcl_timer_t * timer = reinterpret_cast<rcl_timer_t *>(wall_timer_handle);
    rcl_ret_t ret = rcl_timer_call(timer);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to call timer: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_time_WallClockTimerImpl_nativeDispose(JNIEnv *, jclass, jlong wall_clock_timer_handle){
    if (wall_clock_timer_handle == 0) {
        return;
    }

    rcl_timer_t * timer = reinterpret_cast<rcl_timer_t *>(wall_clock_timer_handle);
    rcl_ret_t ret = rcl_timer_fini(timer);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to destroy timer: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT jboolean JNICALL Java_rclp9_rcljava_time_WallClockTimerImpl_nativeIsReady(JNIEnv *, jclass, jlong wall_timer_handle){
    rcl_timer_t * timer = reinterpret_cast<rcl_timer_t *>(wall_timer_handle);

    bool is_ready;
    rcl_ret_t ret = rcl_timer_is_ready(timer, &is_ready);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to check timer ready: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: trhow an exception
    }

    return is_ready;
}
