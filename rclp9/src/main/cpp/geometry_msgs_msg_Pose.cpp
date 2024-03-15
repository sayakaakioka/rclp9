#include "geometry_msgs/msg/pose.h"
#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/quaternion.h"

#include "geometry_msgs_msg_Pose.h"

#ifdef __cplusplus
extern "C" {
#endif

JavaVM * g_vm = nullptr;

jclass _jgeometry_msgs__msg__Pose_class_global = nullptr;
jmethodID _jgeometry_msgs__msg__Pose_constructor_global = nullptr;

jclass _jgeometry_msgs__msg__Point_class_global = nullptr;
jmethodID _jgeometry_msgs__msg__Point_constructor_global = nullptr;

jmethodID _jgeometry_msgs__msg__Point_from_java_converter_global = nullptr;
using _jgeometry_msgs__msg__Point_from_java_signature = geometry_msgs__msg__Point * (*)(jobject, geometry_msgs__msg__Point *);
jlong _jgeometry_msgs__msg__Point_from_java_converter_ptr_global = 0;
_jgeometry_msgs__msg__Point_from_java_signature _jgeometry_msgs__msg__Point_from_java_function = nullptr;

jmethodID _jgeometry_msgs__msg__Point_to_java_converter_global = nullptr;
using _jgeometry_msgs__msg__Point_to_java_signature = jobject (*)(geometry_msgs__msg__Point *, jobject);
jlong _jgeometry_msgs__msg__Point_to_java_converter_ptr_global = 0;
_jgeometry_msgs__msg__Point_to_java_signature _jgeometry_msgs__msg__Point_to_java_function = nullptr;

jclass _jgeometry_msgs__msg__Quaternion_class_global = nullptr;
jmethodID _jgeometry_msgs__msg__Quaternion_constructor_global = nullptr;

jmethodID _jgeometry_msgs__msg__Quaternion_from_java_converter_global = nullptr;
using _jgeometry_msgs__msg__Quaternion_from_java_signature = geometry_msgs__msg__Quaternion * (*)(jobject, geometry_msgs__msg__Quaternion *);
jlong _jgeometry_msgs__msg__Quaternion_from_java_converter_ptr_global = 0;
_jgeometry_msgs__msg__Quaternion_from_java_signature _jgeometry_msgs__msg__Quaternion_from_java_function = nullptr;

jmethodID _jgeometry_msgs__msg__Quaternion_to_java_converter_global = nullptr;
using _jgeometry_msgs__msg__Quaternion_to_java_signature = jobject (*)(geometry_msgs__msg__Quaternion *, jobject);
jlong _jgeometry_msgs__msg__Quaternion_to_java_converter_ptr_global = 0;
_jgeometry_msgs__msg__Quaternion_to_java_signature _jgeometry_msgs__msg__Quaternion_to_java_function = nullptr;

#ifdef __cplusplus
}
#endif

geometry_msgs__msg__Pose * geometry_msgs_Pose__convert_from_java(jobject _jmessage_obj, geometry_msgs__msg__Pose * ros_message){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);
    
    if (ros_message == nullptr) {
        ros_message = geometry_msgs__msg__Pose__create();
    }
    
    auto _jfield_position_fid = env->GetFieldID(_jgeometry_msgs__msg__Pose_class_global, "position", "Lgeometry_msgs/msg/Point;");
    jobject _jfield_position_obj = env->GetObjectField(_jmessage_obj, _jfield_position_fid);
    if (_jfield_position_obj != nullptr) {
        ros_message->position = *_jgeometry_msgs__msg__Point_from_java_function(_jfield_position_obj, nullptr);
    }
    env->DeleteLocalRef(_jfield_position_obj);
    
    auto _jfield_orientation_fid = env->GetFieldID(_jgeometry_msgs__msg__Pose_class_global, "orientation", "Lgeometry_msgs/msg/Quaternion;");
    jobject _jfield_orientation_obj = env->GetObjectField(_jmessage_obj, _jfield_orientation_fid);
    if (_jfield_orientation_obj != nullptr) {
        ros_message->orientation = *_jgeometry_msgs__msg__Quaternion_from_java_function(_jfield_orientation_obj, nullptr);
    }
    env->DeleteLocalRef(_jfield_orientation_obj);
    
    return ros_message;
}

jobject geometry_msgs_Pose__convert_to_java(geometry_msgs__msg__Pose * _ros_message, jobject _jmessage_obj){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);
    
    if (_jmessage_obj == nullptr) {
        _jmessage_obj = env->NewObject(_jgeometry_msgs__msg__Pose_class_global, _jgeometry_msgs__msg__Pose_constructor_global);
    }
    
    auto _jfield_position_fid = env->GetFieldID(_jgeometry_msgs__msg__Pose_class_global, "position", "Lgeometry_msgs/msg/Point;");
    jobject _jfield_position_obj = _jgeometry_msgs__msg__Point_to_java_function(&(_ros_message->position), nullptr);
    env->SetObjectField(_jmessage_obj, _jfield_position_fid, _jfield_position_obj);
    
    auto _jfield_orientation_fid = env->GetFieldID(_jgeometry_msgs__msg__Pose_class_global, "orientation", "Lgeometry_msgs/msg/Quaternion;");
    jobject _jfield_orientation_obj = _jgeometry_msgs__msg__Quaternion_to_java_function(&(_ros_message->orientation), nullptr);
    env->SetObjectField(_jmessage_obj, _jfield_orientation_fid, _jfield_orientation_obj);
    
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
        auto _jgeometry_msgs__msg__Pose_class_local = env->FindClass("geometry_msgs/msg/Pose");
        _jgeometry_msgs__msg__Pose_class_global = static_cast<jclass>(env->NewGlobalRef(_jgeometry_msgs__msg__Pose_class_local));
        env->DeleteLocalRef(_jgeometry_msgs__msg__Pose_class_local);
        _jgeometry_msgs__msg__Pose_constructor_global = env->GetMethodID(_jgeometry_msgs__msg__Pose_class_global, "<init>", "()V");
        
        auto _jgeometry_msgs__msg__Point_class_local = env->FindClass("geometry_msgs/msg/Point");
        _jgeometry_msgs__msg__Point_class_global = static_cast<jclass>(env->NewGlobalRef(_jgeometry_msgs__msg__Point_class_local));
        env->DeleteLocalRef(_jgeometry_msgs__msg__Point_class_local);
        _jgeometry_msgs__msg__Point_constructor_global = env->GetMethodID(_jgeometry_msgs__msg__Point_class_global, "<init>", "()V");
        
        _jgeometry_msgs__msg__Point_from_java_converter_global 
            = env->GetStaticMethodID(_jgeometry_msgs__msg__Point_class_global, "getFromJavaConverter", "()J");
        _jgeometry_msgs__msg__Point_from_java_converter_ptr_global 
            = env->CallStaticLongMethod(_jgeometry_msgs__msg__Point_class_global, _jgeometry_msgs__msg__Point_from_java_converter_global);
        _jgeometry_msgs__msg__Point_from_java_function 
            = reinterpret_cast<_jgeometry_msgs__msg__Point_from_java_signature>(_jgeometry_msgs__msg__Point_from_java_converter_ptr_global);
            
        _jgeometry_msgs__msg__Point_to_java_converter_global 
            = env->GetStaticMethodID(_jgeometry_msgs__msg__Point_class_global, "getToJavaConverter", "()J");
        _jgeometry_msgs__msg__Point_to_java_converter_ptr_global 
            = env->CallStaticLongMethod(_jgeometry_msgs__msg__Point_class_global, _jgeometry_msgs__msg__Point_to_java_converter_global);
        _jgeometry_msgs__msg__Point_to_java_function 
            = reinterpret_cast<_jgeometry_msgs__msg__Point_to_java_signature>(_jgeometry_msgs__msg__Point_to_java_converter_ptr_global);
            
        auto _jgeometry_msgs__msg__Quaternion_class_local = env->FindClass("geometry_msgs/msg/Quaternion");
        _jgeometry_msgs__msg__Quaternion_class_global = static_cast<jclass>(env->NewGlobalRef(_jgeometry_msgs__msg__Quaternion_class_local));
        env->DeleteLocalRef(_jgeometry_msgs__msg__Quaternion_class_local);
        _jgeometry_msgs__msg__Quaternion_constructor_global = env->GetMethodID(_jgeometry_msgs__msg__Quaternion_class_global, "<init>", "()V");
        
        _jgeometry_msgs__msg__Quaternion_from_java_converter_global 
            = env->GetStaticMethodID(_jgeometry_msgs__msg__Quaternion_class_global, "getFromJavaConverter", "()J");
        _jgeometry_msgs__msg__Quaternion_from_java_converter_ptr_global 
            = env->CallStaticLongMethod(_jgeometry_msgs__msg__Quaternion_class_global, _jgeometry_msgs__msg__Quaternion_from_java_converter_global);
        _jgeometry_msgs__msg__Quaternion_from_java_function 
            = reinterpret_cast<_jgeometry_msgs__msg__Quaternion_from_java_signature>(_jgeometry_msgs__msg__Quaternion_from_java_converter_ptr_global);
            
        _jgeometry_msgs__msg__Quaternion_to_java_converter_global 
            = env->GetStaticMethodID(_jgeometry_msgs__msg__Quaternion_class_global, "getToJavaConverter", "()J");
        _jgeometry_msgs__msg__Quaternion_to_java_converter_ptr_global 
            = env->CallStaticLongMethod(_jgeometry_msgs__msg__Quaternion_class_global, _jgeometry_msgs__msg__Quaternion_to_java_converter_global);
        _jgeometry_msgs__msg__Quaternion_to_java_function 
            = reinterpret_cast<_jgeometry_msgs__msg__Quaternion_to_java_signature>(_jgeometry_msgs__msg__Quaternion_to_java_converter_ptr_global);
    }
    
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM * vm, void *){
    JNIEnv * env;
    if (g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) == JNI_OK) {
        if (_jgeometry_msgs__msg__Pose_class_global != nullptr) {
            env->DeleteGlobalRef(_jgeometry_msgs__msg__Pose_class_global);
            _jgeometry_msgs__msg__Pose_class_global = nullptr;
            _jgeometry_msgs__msg__Pose_constructor_global = nullptr;
        }
    
        if (_jgeometry_msgs__msg__Point_class_global != nullptr) {
            env->DeleteGlobalRef(_jgeometry_msgs__msg__Point_class_global);
            _jgeometry_msgs__msg__Point_class_global = nullptr;
            _jgeometry_msgs__msg__Point_constructor_global = nullptr;

            _jgeometry_msgs__msg__Point_from_java_converter_global = nullptr;
            _jgeometry_msgs__msg__Point_from_java_converter_ptr_global = 0;
            _jgeometry_msgs__msg__Point_from_java_function = nullptr;

            _jgeometry_msgs__msg__Point_to_java_converter_global = nullptr;
            _jgeometry_msgs__msg__Point_to_java_converter_ptr_global = 0;
            _jgeometry_msgs__msg__Point_to_java_function = nullptr;
        }

        if (_jgeometry_msgs__msg__Quaternion_class_global != nullptr) {
            env->DeleteGlobalRef(_jgeometry_msgs__msg__Quaternion_class_global);
            _jgeometry_msgs__msg__Quaternion_class_global = nullptr;
            _jgeometry_msgs__msg__Quaternion_constructor_global = nullptr;

            _jgeometry_msgs__msg__Quaternion_from_java_converter_global = nullptr;
            _jgeometry_msgs__msg__Quaternion_from_java_converter_ptr_global = 0;
            _jgeometry_msgs__msg__Quaternion_from_java_function = nullptr;

            _jgeometry_msgs__msg__Quaternion_to_java_converter_global = nullptr;
            _jgeometry_msgs__msg__Quaternion_to_java_converter_ptr_global = 0;
            _jgeometry_msgs__msg__Quaternion_to_java_function = nullptr;
        }
    }
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Pose_getDestructor(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(geometry_msgs__msg__Pose__destroy);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Pose_getFromJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(&geometry_msgs_Pose__convert_from_java);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Pose_getToJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(geometry_msgs_Pose__convert_to_java);
}


JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Pose_getTypeSupport (JNIEnv *, jclass){
    return reinterpret_cast<jlong>(ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose));
}