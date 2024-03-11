#include <iostream>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rclp9_rcljava_subscriber_SubscriberImpl.h"

JNIEXPORT void JNICALL Java_rclp9_rcljava_subscriber_SubscriberImpl_nativeDispose(JNIEnv *, jclass, jlong node_handle, jlong subscriber_handle){
    if (subscriber_handle == 0) {
        return;
    }

    if (node_handle == 0) {
        return;
    }

    rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);
    rcl_subscription_t * subscriber = reinterpret_cast<rcl_subscription_t *>(subscriber_handle);
    rcl_ret_t ret = rcl_subscription_fini(subscriber, node);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to destroy subscription: " << rcl_get_error_string().str;
        rcl_reset_error();
        // throw an exception
    }
}