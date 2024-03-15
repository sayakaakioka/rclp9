#include "std_msgs/msg/string.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "std_msgs_msg_String.h"

#ifdef __cplusplus
extern "C" {
#endif

JavaVM *g_vm = nullptr;
jclass _jstd_msgs__msg__String_class_global = nullptr;
jmethodID _jstd_msgs__msg__String_constructor_global = nullptr;

#ifdef __cplusplus
}
#endif

std_msgs__msg__String * std_msgs_String__convert_from_java(jobject _jmessage_obj, std_msgs__msg__String * ros_message){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);


    if (ros_message == nullptr) {
        ros_message = std_msgs__msg__String__create();
    }

    auto _jfield_data_fid = env->GetFieldID(_jstd_msgs__msg__String_class_global, "data", "Ljava/lang/String;");
    jstring _jvaluedata = static_cast<jstring>(env->GetObjectField(_jmessage_obj, _jfield_data_fid));
    if (_jvaluedata != nullptr) {
        const char * _strdata = env->GetStringUTFChars(_jvaluedata, 0);
        //rosidl_generator_c__String__assign(&ros_message->data, _strdata);
        rosidl_runtime_c__String__assign(&ros_message->data, _strdata);
        env->ReleaseStringUTFChars(_jvaluedata, _strdata);
    }

    return ros_message;
}

jobject std_msgs_String__convert_to_java(std_msgs__msg__String * _ros_message, jobject _jmessage_obj){
  JNIEnv * env = nullptr;
  g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

  if (_jmessage_obj == nullptr) {
    _jmessage_obj = env->NewObject(_jstd_msgs__msg__String_class_global, _jstd_msgs__msg__String_constructor_global);
  }

  auto _jfield_data_fid = env->GetFieldID(_jstd_msgs__msg__String_class_global, "data", "Ljava/lang/String;");
  if (_ros_message->data.data != nullptr) {
    env->SetObjectField(_jmessage_obj, _jfield_data_fid, env->NewStringUTF(_ros_message->data.data));
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
        auto _jstd_msgs__msg__String_class_local = env->FindClass("std_msgs/msg/String");
        _jstd_msgs__msg__String_class_global = static_cast<jclass>(env->NewGlobalRef(_jstd_msgs__msg__String_class_local));
        env->DeleteLocalRef(_jstd_msgs__msg__String_class_local);
        _jstd_msgs__msg__String_constructor_global = env->GetMethodID(_jstd_msgs__msg__String_class_global, "<init>", "()V");
    }
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM * vm, void *){
    JNIEnv * env;
    if (g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) == JNI_OK) {
        if (_jstd_msgs__msg__String_class_global != nullptr) {
            env->DeleteGlobalRef(_jstd_msgs__msg__String_class_global);
            _jstd_msgs__msg__String_class_global = nullptr;
            _jstd_msgs__msg__String_constructor_global = nullptr;
        }
    }
}


JNIEXPORT jlong JNICALL Java_std_1msgs_msg_String_getDestructor(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(std_msgs__msg__String__destroy);
}

JNIEXPORT jlong JNICALL Java_std_1msgs_msg_String_getFromJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(&std_msgs_String__convert_from_java);
}

JNIEXPORT jlong JNICALL Java_std_1msgs_msg_String_getToJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(std_msgs_String__convert_to_java);
}

JNIEXPORT jlong JNICALL Java_std_1msgs_msg_String_getTypeSupport(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String));
}

