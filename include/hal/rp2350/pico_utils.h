
#ifndef UTILS_H
#define UTILS_H

#include <cstdint>
#include <pico/time.h>
#include <stdio.h>
#include <stdarg.h>
#include <pico/critical_section.h>

namespace utils
{
    typedef absolute_time_t time_t;

    // Logging levels for debug and error messages.
    // NONE: No logging
    // ERROR: Error messages
    // WARN: Warnings
    // INFO: Informational messages
    // DEBUG: Debug messages
    // The higher the level, the more verbose the logging.
    typedef enum LogLevel
    {
        LOG_LEVEL_NONE = 0,
        LOG_LEVEL_ERROR = 1,
        LOG_LEVEL_WARN = 2,
        LOG_LEVEL_INFO = 3,
        LOG_LEVEL_DEBUG = 4,
    } LogLevel;

    inline time_t now()
    {
        return get_absolute_time();
    }
    inline bool hasElapsed_us(time_t start, time_t duration)
    {
        return absolute_time_diff_us(start, get_absolute_time()) >= duration;
    }
    inline bool hasElapsed_ms(time_t start, time_t duration)
    {
        return absolute_time_diff_us(start, get_absolute_time()) >= duration * 1000;
    }
    inline bool hasElapsed_s(time_t start, time_t duration)
    {
        return absolute_time_diff_us(start, get_absolute_time()) >= duration * 1000000;
    }
    inline void sleep_ms(uint32_t ms)
    {
        ::sleep_ms(ms);
    }

    inline bool isDebugEnabled = false; // Flag to enable or disable debug messages, regardless of the current log level.
    inline LogLevel currentLogLevel = LOG_LEVEL_NONE;
    inline critical_section_t cs; // Not static: static would give each translation unit its own uninitialized copy

    inline void init_critical_section()
    {
        critical_section_init(&cs);
    }

    inline void enter_critical_section()
    {
        critical_section_enter_blocking(&cs);
    }

    inline void exit_critical_section()
    {
        critical_section_exit(&cs);
    }

    inline void debug_enable(bool enable)
    {
        isDebugEnabled = enable;
    }

    inline void set_log_level(LogLevel level)
    {
        currentLogLevel = level;
    }

    inline void debug_printf(const char *format, ...)
    {
        if (!isDebugEnabled) return;
        if (currentLogLevel < LOG_LEVEL_DEBUG) return;
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }

    inline void info_printf(const char *format, ...)
    {
        if (!isDebugEnabled) return;
        if (currentLogLevel < LOG_LEVEL_INFO) return;
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }

    inline void warn_printf(const char *format, ...)
    {
        if (!isDebugEnabled) return;
        if (currentLogLevel < LOG_LEVEL_WARN) return;
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }

    inline void error_printf(const char *format, ...)
    {
        if (!isDebugEnabled) return;
        if (currentLogLevel < LOG_LEVEL_ERROR) return;
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
}

#endif // UTILS_H
