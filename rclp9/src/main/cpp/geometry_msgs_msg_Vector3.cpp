#include "geometry_msgs/msg/vector3.h"

#include "geometry_msgs_msg_Vector3.h"

#ifdef __cplusplus
extern "C" {
#endif

JavaVM * g_vm = nullptr;

jclass _jgeometry_msgs__msg__Vector3_class_global = nullptr;
jmethodID _jgeometry_msgs__msg__Vector3_constructor_global = nullptr;


jclass _jjava__lang__Double_class_global = nullptr;
jmethodID _jjava__lang__Double_constructor_global = nullptr;
jmethodID _jjava__lang__Double_value_global = nullptr;

#ifdef __cplusplus
}
#endif

geometry_msgs__msg__Vector3 * geometry_msgs_Vector3__convert_from_java(jobject _jmessage_obj, geometry_msgs__msg__Vector3 * ros_message){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);
    
    if (ros_message == nullptr) {
        ros_message = geometry_msgs__msg__Vector3__create();
    }
    
    auto _jfield_x_fid = env->GetFieldID(_jgeometry_msgs__msg__Vector3_class_global, "x", "D");
    ros_message->x = env->GetDoubleField(_jmessage_obj, _jfield_x_fid);
    
    auto _jfield_y_fid = env->GetFieldID(_jgeometry_msgs__msg__Vector3_class_global, "y", "D");
    ros_message->y = env->GetDoubleField(_jmessage_obj, _jfield_y_fid);
    
    auto _jfield_z_fid = env->GetFieldID(_jgeometry_msgs__msg__Vector3_class_global, "z", "D");
    ros_message->z = env->GetDoubleField(_jmessage_obj, _jfield_z_fid);
    
    return ros_message;
}

jobject geometry_msgs_Vector3__convert_to_java(geometry_msgs__msg__Vector3 * _ros_message, jobject _jmessage_obj){
  JNIEnv * env = nullptr;
  g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

  if (_jmessage_obj == nullptr) {
    _jmessage_obj = env->NewObject(_jgeometry_msgs__msg__Vector3_class_global, _jgeometry_msgs__msg__Vector3_constructor_global);
  }

  auto _jfield_x_fid = env->GetFieldID(_jgeometry_msgs__msg__Vector3_class_global, "x", "D");
  env->SetDoubleField(_jmessage_obj, _jfield_x_fid, _ros_message->x);
  
  auto _jfield_y_fid = env->GetFieldID(_jgeometry_msgs__msg__Vector3_class_global, "y", "D");
  env->SetDoubleField(_jmessage_obj, _jfield_y_fid, _ros_message->y);
  
  auto _jfield_z_fid = env->GetFieldID(_jgeometry_msgs__msg__Vector3_class_global, "z", "D");
  env->SetDoubleField(_jmessage_obj, _jfield_z_fid, _ros_message->z);

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
        auto _jgeometry_msgs__msg__Vector3_class_local = env->FindClass("geometry_msgs/msg/Vector3");
        _jgeometry_msgs__msg__Vector3_class_global = static_cast<jclass>(env->NewGlobalRef(_jgeometry_msgs__msg__Vector3_class_local));
        env->DeleteLocalRef(_jgeometry_msgs__msg__Vector3_class_local);
        
        _jgeometry_msgs__msg__Vector3_constructor_global = env->GetMethodID(_jgeometry_msgs__msg__Vector3_class_global, "<init>", "()V");
        auto _jjava__lang__Double_class_local = env->FindClass("java/lang/Double");
        _jjava__lang__Double_class_global = static_cast<jclass>(env->NewGlobalRef(_jjava__lang__Double_class_local));
        env->DeleteLocalRef(_jjava__lang__Double_class_local);
        
        _jjava__lang__Double_constructor_global = env->GetMethodID(_jjava__lang__Double_class_global, "<init>", "(D)V");
        _jjava__lang__Double_value_global = env->GetMethodID(_jjava__lang__Double_class_global, "doubleValue", "()D");
    }
    
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM * vm, void *){
    JNIEnv * env;
    if (g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) == JNI_OK) {
        if (_jgeometry_msgs__msg__Vector3_class_global != nullptr) {
            env->DeleteGlobalRef(_jgeometry_msgs__msg__Vector3_class_global);
            _jgeometry_msgs__msg__Vector3_class_global = nullptr;
            _jgeometry_msgs__msg__Vector3_constructor_global = nullptr;
        }
        
        if (_jjava__lang__Double_class_global != nullptr) {
            env->DeleteGlobalRef(_jjava__lang__Double_class_global);
            _jjava__lang__Double_class_global = nullptr;
            _jjava__lang__Double_constructor_global = nullptr;
            _jjava__lang__Double_value_global = nullptr;
        }
    }
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Vector3_getDestructor(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(geometry_msgs__msg__Vector3__destroy);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Vector3_getFromJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(&geometry_msgs_Vector3__convert_from_java);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Vector3_getToJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(geometry_msgs_Vector3__convert_to_java);
}


JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_Vector3_getTypeSupport (JNIEnv *, jclass){
    return reinterpret_cast<jlong>(ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3));
}