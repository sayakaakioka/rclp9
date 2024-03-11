#include <iostream>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rclp9_rcljava_executor_SingleThreadedExecutor.h"

using convert_from_java_signature = void * (*)(jobject, void *);
using convert_to_java_signature = jobject (*)(void *, jobject);
using destroy_ros_message_signature = void (*)(void *);

JNIEXPORT void JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeDisposeWaitSet(JNIEnv *, jclass, jlong wait_set_handle){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_ret_t ret = rcl_wait_set_fini(wait_set);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to destroy timer: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeGetZeroInitializedWaitSet(JNIEnv *, jclass){
    rcl_wait_set_t * wait_set = static_cast<rcl_wait_set_t *>(malloc(sizeof(rcl_wait_set_t)));
    *wait_set = rcl_get_zero_initialized_wait_set();
    return reinterpret_cast<jlong>(wait_set);
}

JNIEXPORT jobject JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeTake(JNIEnv * env, jclass, jlong subscriber_handle, jclass jmessage_class){
    rcl_subscription_t * subscriber = reinterpret_cast<rcl_subscription_t *>(subscriber_handle);

    jmethodID jfrom_mid = env->GetStaticMethodID(jmessage_class, "getFromJavaConverter", "()J");
    jlong jfrom_java_converter = env->CallStaticLongMethod(jmessage_class, jfrom_mid);

    jmethodID jdestructor_mid = env->GetStaticMethodID(jmessage_class, "getDestructor", "()J");
    jlong jdestructor_handle = env->CallStaticLongMethod(jmessage_class, jdestructor_mid);

    convert_from_java_signature convert_from_java = reinterpret_cast<convert_from_java_signature>(jfrom_java_converter);
    destroy_ros_message_signature destroy_ros_message = reinterpret_cast<destroy_ros_message_signature>(jdestructor_handle);

    jmethodID jconstructor = env->GetMethodID(jmessage_class, "<init>", "()V");
    jobject jmsg = env->NewObject(jmessage_class, jconstructor);
    void * taken_msg = convert_from_java(jmsg, nullptr);

    rcl_ret_t ret = rcl_take(subscriber, taken_msg, nullptr, nullptr);
    if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
        destroy_ros_message(taken_msg);
        std::cout << "Failed to take from a subscription: " << rcl_get_error_string().str;
        rcl_reset_error();
        // throw an exception
        return nullptr;
    }

    if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
        jmethodID jto_mid = env->GetStaticMethodID(jmessage_class, "getToJavaConverter", "()J");
        jlong jto_java_converter = env->CallStaticLongMethod(jmessage_class, jto_mid);

        convert_to_java_signature convert_to_java = reinterpret_cast<convert_to_java_signature>(jto_java_converter);
        jobject jtaken_msg = convert_to_java(taken_msg, nullptr);
        destroy_ros_message(taken_msg);
        return jtaken_msg;
    }
    destroy_ros_message(taken_msg);
    return nullptr;
  }

JNIEXPORT void JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeWait(JNIEnv *, jclass, jlong wait_set_handle, jlong timeout){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_ret_t ret = rcl_wait(wait_set, timeout);
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
        std::cout << "Failed to wait on wait set: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeWaitSetAddSubscriber(JNIEnv * env, jclass, jlong wait_set_handle, jlong subscriber_handle){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_subscription_t * subscriber = reinterpret_cast<rcl_subscription_t *>(subscriber_handle);

    rcl_ret_t ret = rcl_wait_set_add_subscription(wait_set, subscriber, nullptr);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to add subscription to wait set: " << rcl_get_error_string().str;
        rcl_reset_error();
        // throw an exception
    }
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeWaitSetAddTimer(JNIEnv *, jclass, jlong wait_set_handle, jlong timer_handle){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_timer_t * timer = reinterpret_cast<rcl_timer_t *>(timer_handle);
    rcl_ret_t ret = rcl_wait_set_add_timer(wait_set, timer, nullptr);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to add timer to wait set: " << rcl_get_error_string().str;
    rcl_reset_error();
    // TODO: throw an exception
  }

}

JNIEXPORT void JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeWaitSetClear(JNIEnv *, jclass, jlong wait_set_handle){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_ret_t ret = rcl_wait_set_clear(wait_set);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to clear wait set: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeWaitSetInit(JNIEnv *, jclass, jlong wait_set_handle, jlong context_handle, jint number_of_subscriptions, jint number_of_guard_conditions, jint number_of_timers, jint number_of_clients, jint number_of_services, jint number_of_events){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(context_handle);
    rcl_ret_t ret = rcl_wait_set_init(wait_set, number_of_subscriptions, number_of_guard_conditions, number_of_timers, number_of_clients, number_of_services, number_of_events, context, rcl_get_default_allocator());

    if (ret != RCL_RET_OK) {
        std::cout << "Failed to initialize wait set: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}

JNIEXPORT jboolean JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeWaitSetSubscriberIsReady(JNIEnv *, jclass, jlong wait_set_handle, jlong index){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    return wait_set->subscriptions[index] != nullptr;
}

JNIEXPORT jboolean JNICALL Java_rclp9_rcljava_executor_SingleThreadedExecutor_nativeWaitSetTimerIsReady(JNIEnv *, jclass, jlong wait_set_handle, jlong index){
    rcl_wait_set_t * wait_set = reinterpret_cast<rcl_wait_set_t *>(wait_set_handle);
    return wait_set->timers[index] != nullptr;
}