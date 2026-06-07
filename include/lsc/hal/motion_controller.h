#pragma once

namespace lsc::hal {

class IMotionController {
public:
    virtual ~IMotionController() = default;

    virtual bool open() = 0;
    virtual bool home() = 0;
    virtual bool moveTo(double positionMm) = 0;
    virtual double currentPosition() const = 0;
    virtual void close() = 0;
};

} // namespace lsc::hal
