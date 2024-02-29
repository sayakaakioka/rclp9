#include <iostream>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rclp9_rcljava_publisher_PublisherImpl.h"

using convert_from_java_signature = void * (*)(jobject, void *);
using destroy_ros_message_signature = void (*)(void *);

JNIEXPORT void JNICALL Java_rclp9_rcljava_publisher_PublisherImpl_nativeDispose(JNIEnv *, jclass, jlong node_handle, jlong publisher_handle){
    if (publisher_handle == 0 || node_handle == 0) {
        return;
    }

    rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);
    rcl_publisher_t * publisher = reinterpret_cast<rcl_publisher_t *>(publisher_handle);
    rcl_ret_t ret = rcl_publisher_fini(publisher, node);

    if (ret != RCL_RET_OK) {
        std::cout << "Failed to destroy publisher: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }

}

JNIEXPORT void JNICALL Java_rclp9_rcljava_publisher_PublisherImpl_nativePublish(JNIEnv * env, jclass, jlong publisher_handle, jlong jmsg_destructor_handle, jobject jmsg){
    rcl_publisher_t * publisher = reinterpret_cast<rcl_publisher_t *>(publisher_handle);
    jclass jmessage_class = env->GetObjectClass(jmsg);
    jmethodID mid = env->GetStaticMethodID(jmessage_class, "getFromJavaConverter", "()J");
    jlong jfrom_java_converter = env->CallStaticLongMethod(jmessage_class, mid);

    convert_from_java_signature convert_from_java = reinterpret_cast<convert_from_java_signature>(jfrom_java_converter);
    void * raw_ros_message = convert_from_java(jmsg, nullptr);
    rcl_ret_t ret = rcl_publish(publisher, raw_ros_message, nullptr);

    destroy_ros_message_signature destroy_ros_message = reinterpret_cast<destroy_ros_message_signature>(jmsg_destructor_handle);
    destroy_ros_message(raw_ros_message);

    if (ret != RCL_RET_OK) {
        std::cout << "Failed to publish: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw an exception
    }
}