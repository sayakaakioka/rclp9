#include "geometry_msgs/msg/pose.h"
#include "std_msgs/msg/header.h"
#include "geometry_msgs/msg/pose_stamped.h"

#include "geometry_msgs_msg_PoseStamped.h"

#ifdef __cplusplus
extern "C" {
#endif

JavaVM * g_vm = nullptr;

jclass _jgeometry_msgs__msg__PoseStamped_class_global = nullptr;
jmethodID _jgeometry_msgs__msg__PoseStamped_constructor_global = nullptr;


jclass _jstd_msgs__msg__Header_class_global = nullptr;
jmethodID _jstd_msgs__msg__Header_constructor_global = nullptr;

jmethodID _jstd_msgs__msg__Header_from_java_converter_global = nullptr;
using _jstd_msgs__msg__Header_from_java_signature = std_msgs__msg__Header * (*)(jobject, std_msgs__msg__Header *);
jlong _jstd_msgs__msg__Header_from_java_converter_ptr_global = 0;
_jstd_msgs__msg__Header_from_java_signature _jstd_msgs__msg__Header_from_java_function = nullptr;

jmethodID _jstd_msgs__msg__Header_to_java_converter_global = nullptr;
using _jstd_msgs__msg__Header_to_java_signature = jobject (*)(std_msgs__msg__Header *, jobject);
jlong _jstd_msgs__msg__Header_to_java_converter_ptr_global = 0;
_jstd_msgs__msg__Header_to_java_signature _jstd_msgs__msg__Header_to_java_function = nullptr;
jclass _jgeometry_msgs__msg__Pose_class_global = nullptr;
jmethodID _jgeometry_msgs__msg__Pose_constructor_global = nullptr;

jmethodID _jgeometry_msgs__msg__Pose_from_java_converter_global = nullptr;
using _jgeometry_msgs__msg__Pose_from_java_signature = geometry_msgs__msg__Pose * (*)(jobject, geometry_msgs__msg__Pose *);
jlong _jgeometry_msgs__msg__Pose_from_java_converter_ptr_global = 0;
_jgeometry_msgs__msg__Pose_from_java_signature _jgeometry_msgs__msg__Pose_from_java_function = nullptr;

jmethodID _jgeometry_msgs__msg__Pose_to_java_converter_global = nullptr;
using _jgeometry_msgs__msg__Pose_to_java_signature = jobject (*)(geometry_msgs__msg__Pose *, jobject);
jlong _jgeometry_msgs__msg__Pose_to_java_converter_ptr_global = 0;
_jgeometry_msgs__msg__Pose_to_java_signature _jgeometry_msgs__msg__Pose_to_java_function = nullptr;

#ifdef __cplusplus
}
#endif

geometry_msgs__msg__PoseStamped * geometry_msgs_PoseStamped__convert_from_java(jobject _jmessage_obj, geometry_msgs__msg__PoseStamped * ros_message){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

    if (ros_message == nullptr) {
        ros_message = geometry_msgs__msg__PoseStamped__create();
    }

    auto _jfield_header_fid = env->GetFieldID(_jgeometry_msgs__msg__PoseStamped_class_global, "header", "Lstd_msgs/msg/Header;");
    jobject _jfield_header_obj = env->GetObjectField(_jmessage_obj, _jfield_header_fid);
    if (_jfield_header_obj != nullptr) {
        ros_message->header = *_jstd_msgs__msg__Header_from_java_function(_jfield_header_obj, nullptr);
    }
    env->DeleteLocalRef(_jfield_header_obj);

    auto _jfield_pose_fid = env->GetFieldID(_jgeometry_msgs__msg__PoseStamped_class_global, "pose", "Lgeometry_msgs/msg/Pose;");
    jobject _jfield_pose_obj = env->GetObjectField(_jmessage_obj, _jfield_pose_fid);
    if (_jfield_pose_obj != nullptr) {
        ros_message->pose = *_jgeometry_msgs__msg__Pose_from_java_function(_jfield_pose_obj, nullptr);
    }
    env->DeleteLocalRef(_jfield_pose_obj);

    return ros_message;
}

jobject geometry_msgs_PoseStamped__convert_to_java(geometry_msgs__msg__PoseStamped * _ros_message, jobject _jmessage_obj){
    JNIEnv * env = nullptr;
    g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

    if (_jmessage_obj == nullptr) {
        _jmessage_obj = env->NewObject(_jgeometry_msgs__msg__PoseStamped_class_global, _jgeometry_msgs__msg__PoseStamped_constructor_global);
    }

    auto _jfield_header_fid = env->GetFieldID(_jgeometry_msgs__msg__PoseStamped_class_global, "header", "Lstd_msgs/msg/Header;");
    jobject _jfield_header_obj = _jstd_msgs__msg__Header_to_java_function(&(_ros_message->header), nullptr);
    env->SetObjectField(_jmessage_obj, _jfield_header_fid, _jfield_header_obj);

    auto _jfield_pose_fid = env->GetFieldID(_jgeometry_msgs__msg__PoseStamped_class_global, "pose", "Lgeometry_msgs/msg/Pose;");
    jobject _jfield_pose_obj = _jgeometry_msgs__msg__Pose_to_java_function(&(_ros_message->pose), nullptr);
    env->SetObjectField(_jmessage_obj, _jfield_pose_fid, _jfield_pose_obj);

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
        auto _jgeometry_msgs__msg__PoseStamped_class_local = env->FindClass("geometry_msgs/msg/PoseStamped");
        _jgeometry_msgs__msg__PoseStamped_class_global = static_cast<jclass>(env->NewGlobalRef(_jgeometry_msgs__msg__PoseStamped_class_local));
        env->DeleteLocalRef(_jgeometry_msgs__msg__PoseStamped_class_local);
        _jgeometry_msgs__msg__PoseStamped_constructor_global = env->GetMethodID(_jgeometry_msgs__msg__PoseStamped_class_global, "<init>", "()V");

        auto _jstd_msgs__msg__Header_class_local = env->FindClass("std_msgs/msg/Header");
        _jstd_msgs__msg__Header_class_global = static_cast<jclass>(env->NewGlobalRef(_jstd_msgs__msg__Header_class_local));
        env->DeleteLocalRef(_jstd_msgs__msg__Header_class_local);
        _jstd_msgs__msg__Header_constructor_global = env->GetMethodID(_jstd_msgs__msg__Header_class_global, "<init>", "()V");

        _jstd_msgs__msg__Header_from_java_converter_global = env->GetStaticMethodID(_jstd_msgs__msg__Header_class_global, "getFromJavaConverter", "()J");
        _jstd_msgs__msg__Header_from_java_converter_ptr_global = env->CallStaticLongMethod(_jstd_msgs__msg__Header_class_global, _jstd_msgs__msg__Header_from_java_converter_global);
        _jstd_msgs__msg__Header_from_java_function = reinterpret_cast<_jstd_msgs__msg__Header_from_java_signature>(_jstd_msgs__msg__Header_from_java_converter_ptr_global);

        _jstd_msgs__msg__Header_to_java_converter_global = env->GetStaticMethodID(_jstd_msgs__msg__Header_class_global, "getToJavaConverter", "()J");
        _jstd_msgs__msg__Header_to_java_converter_ptr_global = env->CallStaticLongMethod(_jstd_msgs__msg__Header_class_global, _jstd_msgs__msg__Header_to_java_converter_global);
        _jstd_msgs__msg__Header_to_java_function = reinterpret_cast<_jstd_msgs__msg__Header_to_java_signature>(_jstd_msgs__msg__Header_to_java_converter_ptr_global);

        auto _jgeometry_msgs__msg__Pose_class_local = env->FindClass("geometry_msgs/msg/Pose");
        _jgeometry_msgs__msg__Pose_class_global = static_cast<jclass>(env->NewGlobalRef(_jgeometry_msgs__msg__Pose_class_local));
        env->DeleteLocalRef(_jgeometry_msgs__msg__Pose_class_local);
        _jgeometry_msgs__msg__Pose_constructor_global = env->GetMethodID(_jgeometry_msgs__msg__Pose_class_global, "<init>", "()V");
        _jgeometry_msgs__msg__Pose_from_java_converter_global = env->GetStaticMethodID(_jgeometry_msgs__msg__Pose_class_global, "getFromJavaConverter", "()J");

        _jgeometry_msgs__msg__Pose_from_java_converter_ptr_global = env->CallStaticLongMethod(_jgeometry_msgs__msg__Pose_class_global, _jgeometry_msgs__msg__Pose_from_java_converter_global);
        _jgeometry_msgs__msg__Pose_from_java_function = reinterpret_cast<_jgeometry_msgs__msg__Pose_from_java_signature>(_jgeometry_msgs__msg__Pose_from_java_converter_ptr_global);

        _jgeometry_msgs__msg__Pose_to_java_converter_global = env->GetStaticMethodID(_jgeometry_msgs__msg__Pose_class_global, "getToJavaConverter", "()J");
        _jgeometry_msgs__msg__Pose_to_java_converter_ptr_global = env->CallStaticLongMethod(_jgeometry_msgs__msg__Pose_class_global, _jgeometry_msgs__msg__Pose_to_java_converter_global);
        _jgeometry_msgs__msg__Pose_to_java_function = reinterpret_cast<_jgeometry_msgs__msg__Pose_to_java_signature>(_jgeometry_msgs__msg__Pose_to_java_converter_ptr_global);
    }

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM * vm, void *){
    JNIEnv * env;
    if (g_vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) == JNI_OK) {
        if (_jgeometry_msgs__msg__PoseStamped_class_global != nullptr) {
            env->DeleteGlobalRef(_jgeometry_msgs__msg__PoseStamped_class_global);
            _jgeometry_msgs__msg__PoseStamped_class_global = nullptr;
            _jgeometry_msgs__msg__PoseStamped_constructor_global = nullptr;
        }

        if (_jstd_msgs__msg__Header_class_global != nullptr) {
            env->DeleteGlobalRef(_jstd_msgs__msg__Header_class_global);
            _jstd_msgs__msg__Header_class_global = nullptr;
            _jstd_msgs__msg__Header_constructor_global = nullptr;
            _jstd_msgs__msg__Header_from_java_converter_global = nullptr;
            _jstd_msgs__msg__Header_from_java_converter_ptr_global = 0;
            _jstd_msgs__msg__Header_from_java_function = nullptr;

            _jstd_msgs__msg__Header_to_java_converter_global = nullptr;
            _jstd_msgs__msg__Header_to_java_converter_ptr_global = 0;
            _jstd_msgs__msg__Header_to_java_function = nullptr;
        }

        if (_jgeometry_msgs__msg__Pose_class_global != nullptr) {
            env->DeleteGlobalRef(_jgeometry_msgs__msg__Pose_class_global);
            _jgeometry_msgs__msg__Pose_class_global = nullptr;
            _jgeometry_msgs__msg__Pose_constructor_global = nullptr;
            _jgeometry_msgs__msg__Pose_from_java_converter_global = nullptr;
            _jgeometry_msgs__msg__Pose_from_java_converter_ptr_global = 0;
            _jgeometry_msgs__msg__Pose_from_java_function = nullptr;

            _jgeometry_msgs__msg__Pose_to_java_converter_global = nullptr;
            _jgeometry_msgs__msg__Pose_to_java_converter_ptr_global = 0;
            _jgeometry_msgs__msg__Pose_to_java_function = nullptr;
        }
    }
}


JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_PoseStamped_getDestructor(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(geometry_msgs__msg__PoseStamped__destroy);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_PoseStamped_getFromJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(&geometry_msgs_PoseStamped__convert_from_java);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_PoseStamped_getToJavaConverter(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(geometry_msgs_PoseStamped__convert_to_java);
}

JNIEXPORT jlong JNICALL Java_geometry_1msgs_msg_PoseStamped_getTypeSupport(JNIEnv *, jclass){
    return reinterpret_cast<jlong>(ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped));
}