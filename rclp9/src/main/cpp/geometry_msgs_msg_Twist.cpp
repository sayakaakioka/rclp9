#include "geometry_msgs/msg/twist.h"
#include "geometry_msgs/msg/vector3.h"

#include "geometry_msgs_msg_Twist.h"

#ifdef __cplusplus
extern "C" {
#endif

JavaVM * g_vm = nullptr;

jclass _jgeometry_msgs__msg__Twist_class_global = nullptr;
jmethodID _jgeometry_msgs__msg__Twist_constructor_global = nullptr;

jclass _jgeometry_msgs__msg__Vector3_class_global = nullptr;
jmethodID _jgeometry_msgs__msg__Vector3_constructor_global = nullptr;

jmethodID _jgeometry_msgs__msg__Vector3_from_java_converter_global = nullptr;
using _jgeometry_msgs__msg__Vector3_from_java_signature = geometry_msgs__msg__Vector3 * (*)(jobject, geometry_msgs__msg__Vector3 *);
jlong _jgeometry_msgs__msg__Vector3_from_java_converter_ptr_global = 0;
_jgeometry_msgs__msg__Vector3_from_java_signature _jgeometry_msgs__msg__Vector3_from_java_function = nullptr;

jmethodID _jgeometry_msgs__msg__Vector3_to_java_converter_global = nullptr;
using _jgeometry_msgs__msg__Vector3_to_java_signature = jobject (*)(geometry_msgs__msg__Vector3 *, jobject);
jlong _jgeometry_msgs__msg__Vector3_to_java_converter_ptr_global = 0;
_jgeometry_msgs__msg__Vector3_to_java_signature _jgeometry_msgs__msg__Vector3_to_java_function = nullptr;

#ifdef __cplusplus
}
#endif

geometry_msgs__msg__Twist * geometry_msgs_Twist__convert_from_java(jobject _jmessage_obj, geometry_msgs__msg__Twist * ros_message){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);
    
    if (ros_message == nullptr) {
        ros_message = geometry_msgs__msg__Twist__create();
    }
    
    auto _jfield_linear_fid = env->GetFieldID(_jgeometry_msgs__msg__Twist_class_global, "linear", "Lgeometry_msgs/msg/Vector3;");
    jobject _jfield_linear_obj = env->GetObjectField(_jmessage_obj, _jfield_linear_fid);
    if (_jfield_linear_obj != nullptr) {
        ros_message->linear = *_jgeometry_msgs__msg__Vector3_from_java_function(_jfield_linear_obj, nullptr);
    }
    env->DeleteLocalRef(_jfield_linear_obj);
    
    auto _jfield_angular_fid = env->GetFieldID(_jgeometry_msgs__msg__Twist_class_global, "angular", "Lgeometry_msgs/msg/Vector3;");
    jobject _jfield_angular_obj = env->GetObjectField(_jmessage_obj, _jfield_angular_fid);
    if (_jfield_angular_obj != nullptr) {
        ros_message->angular = *_jgeometry_msgs__msg__Vector3_from_java_function(_jfield_angular_obj, nullptr);
    }
    env->DeleteLocalRef(_jfield_angular_obj);
    
    return ros_message;
}

jobject geometry_msgs_Twist__convert_to_java(geometry_msgs__msg__Twist * _ros_message, jobject _jmessage_obj){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);
    
    if (_jmessage_obj == nullptr) {
        _jmessage_obj = env->NewObject(_jgeometry_msgs__msg__Twist_class_global, _jgeometry_msgs__msg__Twist_constructor_global);
    }
    
    auto _jfield_linear_fid = env->GetFieldID(_jgeometry_msgs__msg__Twist_class_global, "linear", "Lgeometry_msgs/msg/Vector3;");
    jobject _jfield_linear_obj = _jgeometry_msgs__msg__Vector3_to_java_function(&(_ros_message->linear), nullptr);
    env->SetObjectField(_jmessage_obj, _jfield_linear_fid, _jfield_linear_obj);
    
    auto _jfield_angular_fid = env->GetFieldID(_jgeometry_msgs__msg__Twist_class_global, "angular", "Lgeometry_msgs/msg/Vector3;");
    jobject _jfield_angular_obj = _jgeometry_msgs__msg__Vector3_to_java_function(&(_ros_message->angular), nullptr);
    env->SetObjectField(_jmessage_obj, _jfield_angular_fid, _jfield_angular_obj);
    
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
        auto _jgeometry_msgs__msg__Twist_class_local = env->FindClass("geometry_msgs/msg/Twist");
        _jgeometry_msgs__msg__Twist_class_global = static_cast<jclass>(env->NewGlobalRef(_jgeometry_msgs__msg__Twist_class_local));
        env->DeleteLocalRef(_jgeometry_msgs__msg__Twist_class_local);
        _jgeometry_msgs__msg__Twist_constructor_global = env->GetMethodID(_jgeometry_msgs__msg__Twist_class_global, "<init>", "()V");
        
        auto _jgeometry_msgs__msg__Vector3_class_local = env->FindClass("geometry_msgs/msg/Vector3");
        _jgeometry_msgs__msg__Vector3_class_global = static_cast<jclass>(env->NewGlobalRef(_jgeometry_msgs__msg__Vector3_class_local));
        env->DeleteLocalRef(_jgeometry_msgs__msg__Vector3_class_local);
        _jgeometry_msgs__msg__Vector3_constructor_global = env->GetMethodID(_jgeometry_msgs__msg__Vector3_class_global, "<init>", "()V");
        
        _jgeometry_msgs__msg__Vector3_from_java_converter_global 
            = env->GetStaticMethodID(_jgeometry_msgs__msg__Vector3_class_global, "getFromJavaConverter", "()J");
        _jgeometry_msgs__msg__Vector3_from_java_converter_ptr_global 
            = env->CallStaticLongMethod(_jgeometry_msgs__msg__Vector3_class_global, _jgeometry_msgs__msg__Vector3_from_java_converter_global);
        _jgeometry_msgs__msg__Vector3_from_java_function 
            = reinterpret_cast<_jgeometry_msgs__msg__Vector3_from_java_signature>(_jgeometry_msgs__msg__Vector3_from_java_converter_ptr_global);
            
        _jgeometry_msgs__msg__Vector3_to_java_converter_global 
            = env->GetStaticMethodID(_jgeometry_msgs__msg__Vector3_class_global, "getToJavaConverter", "()J");
        _jgeometry_msgs__msg__Vector3_to_java_converter_ptr_global 
            = env->CallStaticLongMethod(_jgeometry_msgs__msg__Vector3_class_global, _jgeometry_msgs__msg__Vector3_to_java_converter_global);
        _jgeometry_msgs__msg__Vector3_to_java_function 
            = reinterpret_cast<_jgeometry_msgs__msg__Vector3_to_java_signature>(_jgeometry_msgs__msg__Vector3_to_java_converter_ptr_global);
    }
    
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM * vm, void *){
    JNIEnv * env;
    if (g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) == JNI_OK) {
        if (_jgeometry_msgs__msg__Twist_class_global != nullptr) {
            env->DeleteGlobalRef(_jgeometry_msgs__msg__Twist_class_global);
            _jgeometry_msgs__msg__Twist_class_global = nullptr;
            _jgeometry_msgs__msg__Twist_constructor_global = nullptr;
        }
    
        if (_jgeometry_msgs__msg__Vector3_class_global != nullptr) {
            env->DeleteGlobalRef(_jgeometry_msgs__msg__Vector3_class_global);
            _jgeometry_msgs__msg__Vector3_class_global = nullptr;
            _jgeometry_msgs__msg__Vector3_constructor_global = nullptr;

            _jgeometry_msgs__msg__Vector3_from_java_converter_global = nullptr;
            _jgeometry_msgs__msg__Vector3_from_java_converter_ptr_global = 0;
            _jgeometry_msgs__msg__Vector3_from_java_function = nullptr;

            _jgeometry_msgs__msg__Vector3_to_java_converter_global = nullptr;
            _jgeometry_msgs__msg__Vector3_to_java_converter_ptr_global = 0;
            _jgeometry_msgs__msg__Vector3_to_java_function = nullptr;
        }
    }
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Twist_getDestructor(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(geometry_msgs__msg__Twist__destroy);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Twist_getFromJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(&geometry_msgs_Twist__convert_from_java);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Twist_getToJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(geometry_msgs_Twist__convert_to_java);
}


JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Twist_getTypeSupport (JNIEnv *, jclass){
    return reinterpret_cast<jlong>(ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist));
}