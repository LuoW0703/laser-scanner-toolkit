#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace lsc {

enum class ErrorCode {
    OK = 0,
    IO_ERROR = 100,
    INVALID_ARGUMENT = 200,
    NUMERICAL_ERROR = 300,
    DETECTION_FAILED = 400,
    INSUFFICIENT_DATA = 500,
};

struct Error {
    ErrorCode code = ErrorCode::OK;
    std::string message;
};

template <typename T>
class Result {
public:
    static Result ok(T value) { return Result(std::move(value)); }
    static Result err(Error error) { return Result(std::move(error)); }

    bool isOk() const { return std::holds_alternative<T>(data_); }
    bool isErr() const { return std::holds_alternative<Error>(data_); }

    const T& value() const {
        if (isErr()) {
            throw std::logic_error("Result::value() called on error");
        }
        return std::get<T>(data_);
    }

    T&& moveValue() {
        if (isErr()) {
            throw std::logic_error("Result::moveValue() called on error");
        }
        return std::move(std::get<T>(data_));
    }

    const Error& error() const {
        if (isOk()) {
            throw std::logic_error("Result::error() called on ok");
        }
        return std::get<Error>(data_);
    }

private:
    explicit Result(T value) : data_(std::move(value)) {}
    explicit Result(Error error) : data_(std::move(error)) {}

    std::variant<T, Error> data_;
};

} // namespace lsc
