#pragma once

#include <iostream>
#include <chrono>
#include <HandBase.hpp>
#include <XRBase.hpp>
#include <MatrixUtils.hpp>

#include <Eigen/Dense>

class HandGestureDetector
{
public:
    HandGestureDetector(const XRBase::XRType& type_);
    HandGestureDetector();
    ~HandGestureDetector();

    void Init(const XRBase::XRType& type_);

    bool IsOkGesture(const HandBase::HandData& data);

    bool IsThumbsUpGesture(const HandBase::HandData& data);

    bool IsPinchGesture(const HandBase::HandData& data);

private:

    bool IsOkGestureByVisionPro(const HandBase::HandData& data);

    bool IsThumbsUpGestureByVisionPro(const HandBase::HandData& data);

    bool IsPinchGestureByVisionPro(const HandBase::HandData& data);

    XRBase::XRType type;

    std::chrono::steady_clock::time_point lastUpdateTime;

    // OK Gesture threshold


    // Thumbs Up Gesture threshold

    // VisionPro
    std::vector<size_t> thumbFingerJointsIndex;
    std::vector<size_t> indexFingerJointsIndex;
    std::vector<size_t> middleFingerJointsIndex;
    std::vector<size_t> ringFingerJointsIndex;
    std::vector<size_t> littleFingerJointsIndex;

};
