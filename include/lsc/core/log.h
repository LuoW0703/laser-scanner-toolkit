#pragma once

#include <cstdlib>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

namespace lsc {

enum class LogLevel { TRACE = 0, DEBUG, INFO, WARN, ERROR, FATAL };

inline const char* logLevelName(LogLevel level) {
    switch (level) {
        case LogLevel::TRACE: return "TRACE";
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARN: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
    }
    return "UNKNOWN";
}

inline LogLevel parseLogLevel(const char* value) {
    if (value == nullptr) return LogLevel::INFO;

    const std::string level(value);
    if (level == "TRACE") return LogLevel::TRACE;
    if (level == "DEBUG") return LogLevel::DEBUG;
    if (level == "INFO") return LogLevel::INFO;
    if (level == "WARN" || level == "WARNING") return LogLevel::WARN;
    if (level == "ERROR") return LogLevel::ERROR;
    if (level == "FATAL") return LogLevel::FATAL;
    return LogLevel::INFO;
}

inline LogLevel defaultLogLevel() {
#ifdef _MSC_VER
    char* value = nullptr;
    size_t len = 0;
    if (_dupenv_s(&value, &len, "LSC_LOG_LEVEL") != 0 || value == nullptr) {
        return LogLevel::INFO;
    }
    const LogLevel level = parseLogLevel(value);
    std::free(value);
    return level;
#else
    return parseLogLevel(std::getenv("LSC_LOG_LEVEL"));
#endif
}

class Logger {
public:
    inline static LogLevel minLevel = defaultLogLevel();
    inline static std::ostream* output = &std::clog;

    Logger(LogLevel level, const char* tag) : level_(level), tag_(tag) {}

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    Logger(Logger&& other) noexcept
        : level_(other.level_), tag_(other.tag_), stream_(std::move(other.stream_)), active_(other.active_) {
        other.active_ = false;
    }

    ~Logger() {
        if (active_ && level_ >= minLevel && output != nullptr) {
            *output << "[" << logLevelName(level_) << "][" << tag_ << "] " << stream_.str() << '\n';
        }
    }

    template <typename T>
    Logger& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

private:
    LogLevel level_;
    const char* tag_;
    std::ostringstream stream_;
    bool active_ = true;
};

} // namespace lsc

#define LSC_TRACE(tag) lsc::Logger(lsc::LogLevel::TRACE, tag)
#define LSC_DEBUG(tag) lsc::Logger(lsc::LogLevel::DEBUG, tag)
#define LSC_INFO(tag) lsc::Logger(lsc::LogLevel::INFO, tag)
#define LSC_WARN(tag) lsc::Logger(lsc::LogLevel::WARN, tag)
#define LSC_ERROR(tag) lsc::Logger(lsc::LogLevel::ERROR, tag)
#define LSC_FATAL(tag) lsc::Logger(lsc::LogLevel::FATAL, tag)
