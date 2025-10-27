#pragma once
#include <cstdint>
#include <rcl/timer.h>
#include <rcutils/allocator.h>

namespace rclx {
inline rcl_ret_t timer_init(
    rcl_timer_t * timer,
    rcl_clock_t * clock,
    rcl_context_t * context,
    int64_t timer_period,
    rcl_timer_callback_t callback,
    rcutils_allocator_t allocator,
    bool autostart
) {
#ifdef HAS_RCL_TIMER_INIT2
    return ::rcl_timer_init2(
        timer,
        clock,
        context,
        timer_period,
        callback,
        allocator,
        autostart
    );
#else
    (void) autostart;
    return ::rcl_timer_init(
        timer,
        clock,
        context,
        timer_period,
        callback,
        allocator
        
    );
#endif
}
}