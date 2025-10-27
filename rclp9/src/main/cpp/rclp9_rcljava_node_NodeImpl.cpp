#include <iostream>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include <rclx/rcl_timer.hpp>
#include "rclp9_rcljava_node_NodeImpl.h"

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_node_NodeImpl_nativeCreatePublisherHandle(JNIEnv * env, jclass, jlong node_handle, jclass jmessage_class, jstring jtopic, jlong qos_profile_handle){
    jmethodID mid = env->GetStaticMethodID(jmessage_class, "getTypeSupport", "()J");
    jlong jts = env->CallStaticLongMethod(jmessage_class, mid);

    const char * topic = env->GetStringUTFChars(jtopic, 0);
    rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);
    rosidl_message_type_support_t * ts = reinterpret_cast<rosidl_message_type_support_t *>(jts);

    rcl_publisher_t * publisher = static_cast<rcl_publisher_t *>(malloc(sizeof(rcl_publisher_t)));
    *publisher = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

    rmw_qos_profile_t * qos_profile = reinterpret_cast<rmw_qos_profile_t *>(qos_profile_handle);
    publisher_ops.qos = *qos_profile;

    rcl_ret_t ret = rcl_publisher_init(publisher, node, ts, topic, &publisher_ops);
    env->ReleaseStringUTFChars(jtopic, topic);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to create publisher: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
        return 0;
    }

    return reinterpret_cast<jlong>(publisher);
}

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_node_NodeImpl_nativeCreateSubscriberHandle(JNIEnv * env, jclass, jlong node_handle, jclass jmessage_class, jstring jtopic, jlong qos_profile_handle){
    jmethodID mid = env->GetStaticMethodID(jmessage_class, "getTypeSupport", "()J");
    jlong jts = env->CallStaticLongMethod(jmessage_class, mid);
    const char * topic = env->GetStringUTFChars(jtopic, 0);
    rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);
    rosidl_message_type_support_t * ts = reinterpret_cast<rosidl_message_type_support_t *>(jts);

    rcl_subscription_t * subscriber = static_cast<rcl_subscription_t *>(malloc(sizeof(rcl_subscription_t)));
    *subscriber = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscriber_ops = rcl_subscription_get_default_options();

    rmw_qos_profile_t * qos_profile = reinterpret_cast<rmw_qos_profile_t *>(qos_profile_handle);
    subscriber_ops.qos = *qos_profile;

    rcl_ret_t ret = rcl_subscription_init(subscriber, node, ts, topic, &subscriber_ops);
    env->ReleaseStringUTFChars(jtopic, topic);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to create subscription: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
        return 0;
    }

    jlong jsubscriber = reinterpret_cast<jlong>(subscriber);
    return jsubscriber;
  }

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_node_NodeImpl_nativeCreateTimerHandle(JNIEnv *, jclass, jlong clock_handle, jlong context_handle, jlong timer_period){
    rcl_clock_t * clock = reinterpret_cast<rcl_clock_t *>(clock_handle);
    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(context_handle);
    rcl_timer_t * timer = static_cast<rcl_timer_t *>(malloc(sizeof(rcl_timer_t)));
    *timer = rcl_get_zero_initialized_timer();

    //rcl_ret_t ret = rcl_timer_init(timer, clock, context, timer_period, NULL, rcl_get_default_allocator());
    // rcl_ret_t ret = rcl_timer_init2(timer, clock, context, timer_period, NULL, rcl_get_default_allocator(), true);
    rcl_ret_t ret = rclx::timer_init(timer, clock, context, timer_period, NULL, rcl_get_default_allocator(), true);    

    if (ret != RCL_RET_OK) {
        std::cout << "Failed to create timer: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
        return 0;
    }

    return reinterpret_cast<jlong>(timer);
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_node_NodeImpl_nativeDispose(JNIEnv *, jclass, jlong node_handle){
    if (node_handle == 0) {
        return;
    }

    rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);
    rcl_ret_t ret = rcl_node_fini(node);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to destroy node: " << rcl_get_error_string().str;
        rcl_reset_error();
        // throw an exception
    }
}

