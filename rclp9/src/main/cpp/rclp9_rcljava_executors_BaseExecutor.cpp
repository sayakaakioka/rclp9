#include <iostream>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rclp9_rcljava_executors_BaseExecutor.h"

JNIEXPORT void JNICALL Java_rclp9_rcljava_executors_BaseExecutor_nativeDisposeWaitSet(JNIEnv *, jclass, jlong wait_set_handle){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_ret_t ret = rcl_wait_set_fini(wait_set);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to destroy timer: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_executors_BaseExecutor_nativeGetZeroInitializedWaitSet(JNIEnv *, jclass){
    rcl_wait_set_t * wait_set = static_cast<rcl_wait_set_t *>(malloc(sizeof(rcl_wait_set_t)));
    *wait_set = rcl_get_zero_initialized_wait_set();
    return reinterpret_cast<jlong>(wait_set);
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_executors_BaseExecutor_nativeWait(JNIEnv *, jclass, jlong wait_set_handle, jlong timeout){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_ret_t ret = rcl_wait(wait_set, timeout);
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
        std::cout << "Failed to wait on wait set: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_executors_BaseExecutor_nativeWaitSetAddTimer(JNIEnv *, jclass, jlong wait_set_handle, jlong timer_handle){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_timer_t * timer = reinterpret_cast<rcl_timer_t *>(timer_handle);
    rcl_ret_t ret = rcl_wait_set_add_timer(wait_set, timer, nullptr);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to add timer to wait set: " << rcl_get_error_string().str;
    rcl_reset_error();
    // TODO: throw an exception
  }

}

JNIEXPORT void JNICALL Java_rclp9_rcljava_executors_BaseExecutor_nativeWaitSetClear(JNIEnv *, jclass, jlong wait_set_handle){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_ret_t ret = rcl_wait_set_clear(wait_set);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to clear wait set: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_executors_BaseExecutor_nativeWaitSetInit(JNIEnv *, jclass, jlong wait_set_handle, jlong context_handle, jint number_of_subscriptions, jint number_of_guard_conditions, jint number_of_timers, jint number_of_clients, jint number_of_services, jint number_of_events){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(context_handle);
    rcl_ret_t ret = rcl_wait_set_init(wait_set, number_of_subscriptions, number_of_guard_conditions, number_of_timers, number_of_clients, number_of_services, number_of_events, context, rcl_get_default_allocator());

    if (ret != RCL_RET_OK) {
        std::cout << "Failed to initialize wait set: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT jboolean JNICALL Java_rclp9_rcljava_executors_BaseExecutor_nativeWaitSetTimerIsReady(JNIEnv *, jclass, jlong wait_set_handle, jlong index){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    return wait_set->timers[index] != nullptr;
}