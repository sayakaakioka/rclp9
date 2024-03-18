#include "builtin_interfaces/msg/time.h"
#include "std_msgs/msg/header.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "std_msgs_msg_Header.h"




#ifdef __cplusplus
extern "C" {
#endif

JavaVM * g_vm = nullptr;

jclass _jstd_msgs__msg__Header_class_global = nullptr;
jmethodID _jstd_msgs__msg__Header_constructor_global = nullptr;


jclass _jbuiltin_interfaces__msg__Time_class_global = nullptr;
jmethodID _jbuiltin_interfaces__msg__Time_constructor_global = nullptr;


jmethodID _jbuiltin_interfaces__msg__Time_from_java_converter_global = nullptr;
using _jbuiltin_interfaces__msg__Time_from_java_signature = builtin_interfaces__msg__Time * (*)(jobject, builtin_interfaces__msg__Time *);
jlong _jbuiltin_interfaces__msg__Time_from_java_converter_ptr_global = 0;
_jbuiltin_interfaces__msg__Time_from_java_signature _jbuiltin_interfaces__msg__Time_from_java_function = nullptr;

jmethodID _jbuiltin_interfaces__msg__Time_to_java_converter_global = nullptr;
using _jbuiltin_interfaces__msg__Time_to_java_signature = jobject (*)(builtin_interfaces__msg__Time *, jobject);
jlong _jbuiltin_interfaces__msg__Time_to_java_converter_ptr_global = 0;
_jbuiltin_interfaces__msg__Time_to_java_signature _jbuiltin_interfaces__msg__Time_to_java_function = nullptr;

#ifdef __cplusplus
}
#endif

std_msgs__msg__Header * std_msgs_Header__convert_from_java(jobject _jmessage_obj, std_msgs__msg__Header * ros_message){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

    if (ros_message == nullptr) {
        ros_message = std_msgs__msg__Header__create();
    }

    auto _jfield_stamp_fid = env->GetFieldID(_jstd_msgs__msg__Header_class_global, "stamp", "Lbuiltin_interfaces/msg/Time;");
    jobject _jfield_stamp_obj = env->GetObjectField(_jmessage_obj, _jfield_stamp_fid);

    if (_jfield_stamp_obj != nullptr) {
        ros_message->stamp = *_jbuiltin_interfaces__msg__Time_from_java_function(_jfield_stamp_obj, nullptr);
    }
    env->DeleteLocalRef(_jfield_stamp_obj);

    auto _jfield_frame_id_fid = env->GetFieldID(_jstd_msgs__msg__Header_class_global, "frame_id", "Ljava/lang/String;");
    jstring _jvalueframe_id = static_cast<jstring>(env->GetObjectField(_jmessage_obj, _jfield_frame_id_fid));
    if (_jvalueframe_id != nullptr) {
        const char * _strframe_id = env->GetStringUTFChars(_jvalueframe_id, 0);
        rosidl_runtime_c__String__assign(
        &ros_message->frame_id, _strframe_id);
        env->ReleaseStringUTFChars(_jvalueframe_id, _strframe_id);
    }

    return ros_message;
}

jobject std_msgs_Header__convert_to_java(std_msgs__msg__Header * _ros_message, jobject _jmessage_obj){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

    if (_jmessage_obj == nullptr) {
        _jmessage_obj = env->NewObject(_jstd_msgs__msg__Header_class_global, _jstd_msgs__msg__Header_constructor_global);
    }

    auto _jfield_stamp_fid = env->GetFieldID(_jstd_msgs__msg__Header_class_global, "stamp", "Lbuiltin_interfaces/msg/Time;");
    jobject _jfield_stamp_obj = _jbuiltin_interfaces__msg__Time_to_java_function(&(_ros_message->stamp), nullptr);
    env->SetObjectField(_jmessage_obj, _jfield_stamp_fid, _jfield_stamp_obj);

    auto _jfield_frame_id_fid = env->GetFieldID(_jstd_msgs__msg__Header_class_global, "frame_id", "Ljava/lang/String;");
    if (_ros_message->frame_id.data != nullptr) {
        env->SetObjectField(_jmessage_obj, _jfield_frame_id_fid, env->NewStringUTF(_ros_message->frame_id.data));
    }

    return _jmessage_obj;
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM * vm, void *){
    if (g_vm == nullptr) {
        g_vm = vm;
    }

    JNIEnv * env;
    if (g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) != JNI_OK) {
        return JNI_ERR;
    } else {
        auto _jstd_msgs__msg__Header_class_local = env->FindClass("std_msgs/msg/Header");
        _jstd_msgs__msg__Header_class_global = static_cast<jclass>(env->NewGlobalRef(_jstd_msgs__msg__Header_class_local));
        env->DeleteLocalRef(_jstd_msgs__msg__Header_class_local);
        _jstd_msgs__msg__Header_constructor_global = env->GetMethodID(_jstd_msgs__msg__Header_class_global, "<init>", "()V");

        auto _jbuiltin_interfaces__msg__Time_class_local = env->FindClass("builtin_interfaces/msg/Time");
        _jbuiltin_interfaces__msg__Time_class_global = static_cast<jclass>(env->NewGlobalRef(_jbuiltin_interfaces__msg__Time_class_local));
        env->DeleteLocalRef(_jbuiltin_interfaces__msg__Time_class_local);
        _jbuiltin_interfaces__msg__Time_constructor_global = env->GetMethodID(_jbuiltin_interfaces__msg__Time_class_global, "<init>", "()V");

        _jbuiltin_interfaces__msg__Time_from_java_converter_global = env->GetStaticMethodID(_jbuiltin_interfaces__msg__Time_class_global, "getFromJavaConverter", "()J");
        _jbuiltin_interfaces__msg__Time_from_java_converter_ptr_global = env->CallStaticLongMethod(_jbuiltin_interfaces__msg__Time_class_global, _jbuiltin_interfaces__msg__Time_from_java_converter_global);

        _jbuiltin_interfaces__msg__Time_from_java_function = reinterpret_cast<_jbuiltin_interfaces__msg__Time_from_java_signature>(_jbuiltin_interfaces__msg__Time_from_java_converter_ptr_global);
        _jbuiltin_interfaces__msg__Time_to_java_converter_global = env->GetStaticMethodID(_jbuiltin_interfaces__msg__Time_class_global, "getToJavaConverter", "()J");
        _jbuiltin_interfaces__msg__Time_to_java_converter_ptr_global = env->CallStaticLongMethod(_jbuiltin_interfaces__msg__Time_class_global, _jbuiltin_interfaces__msg__Time_to_java_converter_global);

        _jbuiltin_interfaces__msg__Time_to_java_function = reinterpret_cast<_jbuiltin_interfaces__msg__Time_to_java_signature>(_jbuiltin_interfaces__msg__Time_to_java_converter_ptr_global);
    }

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM * vm, void *){
    JNIEnv * env;
    if (g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) == JNI_OK) {
        if (_jstd_msgs__msg__Header_class_global != nullptr) {
            env->DeleteGlobalRef(_jstd_msgs__msg__Header_class_global);
            _jstd_msgs__msg__Header_class_global = nullptr;
            _jstd_msgs__msg__Header_constructor_global = nullptr;
        }

        if (_jbuiltin_interfaces__msg__Time_class_global != nullptr) {
            env->DeleteGlobalRef(_jbuiltin_interfaces__msg__Time_class_global);
            _jbuiltin_interfaces__msg__Time_class_global = nullptr;
            _jbuiltin_interfaces__msg__Time_constructor_global = nullptr;
            _jbuiltin_interfaces__msg__Time_from_java_converter_global = nullptr;
            _jbuiltin_interfaces__msg__Time_from_java_converter_ptr_global = 0;
            _jbuiltin_interfaces__msg__Time_from_java_function = nullptr;

            _jbuiltin_interfaces__msg__Time_to_java_converter_global = nullptr;
            _jbuiltin_interfaces__msg__Time_to_java_converter_ptr_global = 0;
            _jbuiltin_interfaces__msg__Time_to_java_function = nullptr;
        }
    }
}



JNIEXPORT jlong JNICALL Java_std_1msgs_msg_Header_getDestructor(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(std_msgs__msg__Header__destroy);
}

JNIEXPORT jlong JNICALL Java_std_1msgs_msg_Header_getFromJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(&std_msgs_Header__convert_from_java);
}

JNIEXPORT jlong JNICALL Java_std_1msgs_msg_Header_getToJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(std_msgs_Header__convert_to_java);
}

JNIEXPORT jlong JNICALL Java_std_1msgs_msg_Header_getTypeSupport(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header));
}