#include <string>

#include "rcl/context.h"
#include "rcl/error_handling.h"
#include "rcl/init.h"

#ifdef __cplusplus
extern "C" {
#endif

    void nativeDispose(long handle);
    void nativeInit(long handle);
    bool nativeIsValid(long handle);
    void nativeShutdown(long handle);

#ifdef __cplusplus
}
#endif

void nativeDispose(long handle){
    if(handle == 0){
        // already destroyed
        return;
    }

    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(handle);
    rcl_ret_t ret = rcl_context_fini(context);
    if(ret != RCL_RET_OK){
        std::string msg = "Failed to destroy context: " + std::string(rcl_get_error_string().str);
        rcl_reset_error();
        // TODO: throw exception
    }
}

void nativeInit(long handle){
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
    if(ret != RCL_RET_OK){
        std::string msg = "Failed to init context options: " + std::string(rcl_get_error_string().str);
        rcl_reset_error();
        // TODO: throw exception
        return;
    }

    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(handle);
    ret = rcl_init(0, nullptr, &init_options, context);
    if(ret != RCL_RET_OK){
        std::string msg = "Failed to init context: " + std::string(rcl_get_error_string().str);
        rcl_ret_t ignored_ret = rcl_init_options_fini(&init_options);
        (void)ignored_ret;
        rcl_reset_error();
        // TODO: throw exception
        return;
    }

    rcl_ret_t fini_ret = rcl_init_options_fini(&init_options);
    if(fini_ret != RCL_RET_OK){
        std::string msg = "Failed to init context: " + std::string(rcl_get_error_string().str);
        rcl_ret_t ignored_ret = rcl_shutdown(context);
        (void)ignored_ret;
        rcl_reset_error();
        // TODO: throw exception
        return;
    }
}

bool nativeIsValid(long handle){
    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(handle);
    return rcl_context_is_valid(context);
}

void nativeShutdown(long handle){
    rcl_context_t * context = reinterpret_cast<rcl_context_t *>(handle);
    if(!rcl_context_is_valid(context)){
        return;
    }

    rcl_ret_t ret = rcl_shutdown(context);
    if(ret != RCL_RET_OK){
        std::string msg = "Failed to shutdown context: " + std::string(rcl_get_error_string().str);
        rcl_reset_error();
        // TODO: throw exception
    }
}