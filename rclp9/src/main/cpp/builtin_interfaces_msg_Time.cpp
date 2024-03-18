#include "builtin_interfaces/msg/time.h"

#include "builtin_interfaces_msg_Time.h"

#ifdef __cplusplus
extern "C" {
#endif

JavaVM * g_vm = nullptr;

jclass _jbuiltin_interfaces__msg__Time_class_global = nullptr;
jmethodID _jbuiltin_interfaces__msg__Time_constructor_global = nullptr;


jclass _jjava__lang__Integer_class_global = nullptr;
jmethodID _jjava__lang__Integer_constructor_global = nullptr;
jmethodID _jjava__lang__Integer_value_global = nullptr;

#ifdef __cplusplus
}
#endif

builtin_interfaces__msg__Time * builtin_interfaces_Time__convert_from_java(jobject _jmessage_obj, builtin_interfaces__msg__Time * ros_message){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

    if (ros_message == nullptr) {
        ros_message = builtin_interfaces__msg__Time__create();
    }

    auto _jfield_sec_fid = env->GetFieldID(_jbuiltin_interfaces__msg__Time_class_global, "sec", "I");
    ros_message->sec = env->GetIntField(_jmessage_obj, _jfield_sec_fid);

    auto _jfield_nanosec_fid = env->GetFieldID(_jbuiltin_interfaces__msg__Time_class_global, "nanosec", "I");
    ros_message->nanosec = env->GetIntField(_jmessage_obj, _jfield_nanosec_fid);

    return ros_message;
}

jobject builtin_interfaces_Time__convert_to_java(builtin_interfaces__msg__Time * _ros_message, jobject _jmessage_obj){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

    if (_jmessage_obj == nullptr) {
        _jmessage_obj = env->NewObject(_jbuiltin_interfaces__msg__Time_class_global, _jbuiltin_interfaces__msg__Time_constructor_global);
    }

    auto _jfield_sec_fid = env->GetFieldID(_jbuiltin_interfaces__msg__Time_class_global, "sec", "I");
    env->SetIntField(_jmessage_obj, _jfield_sec_fid, _ros_message->sec);
    auto _jfield_nanosec_fid = env->GetFieldID(_jbuiltin_interfaces__msg__Time_class_global, "nanosec", "I");
    env->SetIntField(_jmessage_obj, _jfield_nanosec_fid, _ros_message->nanosec);

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
        auto _jbuiltin_interfaces__msg__Time_class_local = env->FindClass("builtin_interfaces/msg/Time");
        _jbuiltin_interfaces__msg__Time_class_global = static_cast<jclass>(env->NewGlobalRef(_jbuiltin_interfaces__msg__Time_class_local));
        env->DeleteLocalRef(_jbuiltin_interfaces__msg__Time_class_local);
        _jbuiltin_interfaces__msg__Time_constructor_global = env->GetMethodID(_jbuiltin_interfaces__msg__Time_class_global, "<init>", "()V");

        auto _jjava__lang__Integer_class_local = env->FindClass("java/lang/Integer");
        _jjava__lang__Integer_class_global = static_cast<jclass>(env->NewGlobalRef(_jjava__lang__Integer_class_local));
        env->DeleteLocalRef(_jjava__lang__Integer_class_local);
        _jjava__lang__Integer_constructor_global = env->GetMethodID(_jjava__lang__Integer_class_global, "<init>", "(I)V");
        _jjava__lang__Integer_value_global = env->GetMethodID(_jjava__lang__Integer_class_global, "intValue", "()I");
    }

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM * vm, void *){
    JNIEnv * env;

    if (g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) == JNI_OK) {
        if (_jbuiltin_interfaces__msg__Time_class_global != nullptr) {
            env->DeleteGlobalRef(_jbuiltin_interfaces__msg__Time_class_global);
            _jbuiltin_interfaces__msg__Time_class_global = nullptr;
            _jbuiltin_interfaces__msg__Time_constructor_global = nullptr;
        }

        if (_jjava__lang__Integer_class_global != nullptr) {
            env->DeleteGlobalRef(_jjava__lang__Integer_class_global);
            _jjava__lang__Integer_class_global = nullptr;
            _jjava__lang__Integer_constructor_global = nullptr;
            _jjava__lang__Integer_value_global = nullptr;
        }
    }
}


JNIEXPORT jlong JNICALL Java_builtin_1interfaces_msg_Time_getDestructor(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(builtin_interfaces__msg__Time__destroy);
}

JNIEXPORT jlong JNICALL Java_builtin_1interfaces_msg_Time_getFromJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(&builtin_interfaces_Time__convert_from_java);
}

JNIEXPORT jlong JNICALL Java_builtin_1interfaces_msg_Time_getToJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(builtin_interfaces_Time__convert_to_java);
}

JNIEXPORT jlong JNICALL Java_builtin_1interfaces_msg_Time_getTypeSupport(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(ROSIDL_GET_MSG_TYPE_SUPPORT(builtin_interfaces, msg, Time));
}

