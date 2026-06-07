#pragma once

#include <opencv2/core.hpp>

namespace lsc::hal {

class ICamera {
public:
    virtual ~ICamera() = default;

    virtual bool open() = 0;
    virtual cv::Mat grab() = 0;
    virtual void close() = 0;
};

} // namespace lsc::hal
