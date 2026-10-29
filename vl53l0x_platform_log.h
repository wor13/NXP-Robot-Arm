#ifndef APP_VL53L0X_PLATFORM_LOG_H
#define APP_VL53L0X_PLATFORM_LOG_H
#include <string.h>
/* ST calls these macros at function entry/exit. Disable its verbose internal
 * tracing; main.c still prints application diagnostics. ... accepts any extra
 * arguments. (void)0 is a C expression that deliberately does nothing.
 */
#define _LOG_FUNCTION_START(module, ...) ((void)0)
#define _LOG_FUNCTION_END(module, ...) ((void)0)
#define _LOG_FUNCTION_END_FMT(module, ...) ((void)0)
/* ST's own call sites supply fixed, sufficiently sized destination buffers. */
#define VL53L0X_COPYSTRING(destination, source) strcpy((destination), (source))
#endif
