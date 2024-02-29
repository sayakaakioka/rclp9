#include <iostream>
#include <string>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/context.h"
#include "rcl/node.h"
#include "rmw/rmw.h"

#include "rclp9_rcljava_RCLJava.h"

#define RCLJAVA_QOS_SET_RMW_TIME(qos_profile, policy_name, seconds, nanos)              \
    do                                                                                  \
    {                                                                                   \
        if (seconds < 0 || nanos < 0)                                                   \
        {                                                                               \
            std::cout << "seconds and nanos must be positive or zero: " << #policy_name;     \
            free(qos_profile);                                                          \
            return 0;                                                                   \
        }                                                                               \
        qos_profile->policy_name.sec = static_cast<uint64_t>(seconds);                  \
        qos_profile->policy_name.nsec = static_cast<uint64_t>(nanos);                   \
    } while (0)

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_RCLJava_nativeConvertQoSProfileToHandle(JNIEnv *, jclass, jint history, jint depth, jint reliability, jint durability, jlong deadline_sec, jint deadline_nanos, jlong lifespan_sec, jint lifespan_nanos, jint liveliness, jlong liveliness_lease_sec, jint liveliness_lease_nanos, jboolean avoidROSNamespaceConventions){
    rmw_qos_profile_t * qos_profile = static_cast<rmw_qos_profile_t *>(malloc(sizeof(rmw_qos_profile_t)));
    qos_profile->history = static_cast<rmw_qos_history_policy_t>(history);
    qos_profile->depth = depth;
    qos_profile->reliability = static_cast<rmw_qos_reliability_policy_t>(reliability);
    qos_profile->durability = static_cast<rmw_qos_durability_policy_t>(durability);
    RCLJAVA_QOS_SET_RMW_TIME(qos_profile, deadline, deadline_sec, deadline_nanos);
    RCLJAVA_QOS_SET_RMW_TIME(qos_profile, lifespan, lifespan_sec, lifespan_nanos);
    qos_profile->liveliness = static_cast<rmw_qos_liveliness_policy_t>(liveliness);
    RCLJAVA_QOS_SET_RMW_TIME(qos_profile, liveliness_lease_duration, liveliness_lease_sec, liveliness_lease_nanos);
    qos_profile->avoid_ros_namespace_conventions = avoidROSNamespaceConventions;
    return reinterpret_cast<jlong>(qos_profile);
}

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_RCLJava_nativeCreateContextHandle (JNIEnv *, jclass){
    rcl_context_t * context = static_cast<rcl_context_t *>(malloc(sizeof(rcl_context_t)));
    *context = rcl_get_zero_initialized_context();
    jlong context_handle = reinterpret_cast<long>(context);
    return context_handle;
}

bool parse_arguments(JNIEnv * env, jobject cli_args, rcl_arguments_t * arguments){
    jclass java_util_ArrayList = env->FindClass("java/util/ArrayList");
    jmethodID java_util_ArrayList_size = env->GetMethodID(java_util_ArrayList, "size", "()I");
    jmethodID java_util_ArrayList_get = env->GetMethodID(
    java_util_ArrayList, "get", "(I)Ljava/lang/Object;");

    *arguments = rcl_get_zero_initialized_arguments();
    jint argc = env->CallIntMethod(cli_args, java_util_ArrayList_size);
    std::vector<const char *> argv(argc);
    for (jint i = 0; i < argc; ++i) {
        jstring element = static_cast<jstring>(env->CallObjectMethod(cli_args, java_util_ArrayList_get, i));
        argv[i] = env->GetStringUTFChars(element, nullptr);
        env->DeleteLocalRef(element);
    }

    rcl_ret_t ret = rcl_parse_arguments(argc, &argv[0], rcl_get_default_allocator(), arguments);
    for (jint i = 0; i < argc; ++i) {
        jstring element = static_cast<jstring>(env->CallObjectMethod(cli_args, java_util_ArrayList_get, i));
        env->ReleaseStringUTFChars(element, argv[i]);
        env->DeleteLocalRef(element);
    }

    if (ret != RCL_RET_OK) {
        std::cout << "Failed to parse node arguments: " << rcl_get_error_string().str;
        rcl_reset_error();
        // TODO: throw exception
        return false;
    }

    return true;
}

JNIEXPORT jlong JNICALL Java_rclp9_rcljava_RCLJava_nativeCreateNodeHandle(JNIEnv * env, jclass, jstring jnode_name, jstring jnamespace, jlong context_handle, jobject cli_args){
    const char * node_name_tmp = env->GetStringUTFChars(jnode_name, 0);
    std::string node_name(node_name_tmp);
    env->ReleaseStringUTFChars(jnode_name, node_name_tmp);

    const char * namespace_tmp = env->GetStringUTFChars(jnamespace, 0);
    std::string namespace_(namespace_tmp);
    env->ReleaseStringUTFChars(jnamespace, namespace_tmp);

    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(context_handle);
    rcl_node_t * node = static_cast<rcl_node_t *>(malloc(sizeof(rcl_node_t)));
    if (node == nullptr) {
        std::cout << "Failed to allocate a node.";
        // TODO: throw an exception
        return 0;
    }
    *node = rcl_get_zero_initialized_node();

    rcl_arguments_t arguments;
        if (!parse_arguments(env, cli_args, &arguments)) {
        free(node);
        return 0;
    }

    rcl_node_options_t options = rcl_node_get_default_options();
    rcl_ret_t ret = rcl_node_init(node, node_name.c_str(), namespace_.c_str(), context, &options);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to create node: " << rcl_get_error_string().str;
        rcl_reset_error();
        free(node);
        if (rcl_arguments_fini(&arguments) != RCL_RET_OK) {
            // TODO: throw an exception
        }
        return 0;
    }

    ret = rcl_arguments_fini(&arguments);
    if (ret != RCL_RET_OK) {
        std::cout << "Failed to cleanup after creating node: " << rcl_get_error_string().str;
        rcl_reset_error();
        if (rcl_node_fini(node) != RCL_RET_OK) {
            // TODO: throw an exception
        }
        free(node);
        return 0;
    }
    return reinterpret_cast<jlong>(node);
}

JNIEXPORT void JNICALL Java_rclp9_rcljava_RCLJava_nativeDisposeQoSProfile(JNIEnv *, jclass, jlong qos_profile_handle){
    rmw_qos_profile_t * qos_profile = reinterpret_cast<rmw_qos_profile_t *>(qos_profile_handle);
    free(qos_profile);
}

JNIEXPORT jstring JNICALL Java_rclp9_rcljava_RCLJava_nativeGetRMWIdentifier(JNIEnv * env, jclass){
    const char * rmw_implementation_identifier = rmw_get_implementation_identifier();
    return env->NewStringUTF(rmw_implementation_identifier);
}

