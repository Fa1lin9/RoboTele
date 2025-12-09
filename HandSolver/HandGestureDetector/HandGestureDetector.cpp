#include <HandGestureDetector/HandGestureDetector.hpp>

HandGestureDetector::HandGestureDetector(const XRBase::XRType& type_){
//    this->type = type_;
    this->Init(type_);
}

HandGestureDetector::HandGestureDetector(){

}


HandGestureDetector::~HandGestureDetector(){

}

void HandGestureDetector::Init(const XRBase::XRType& type_){
    this->type = type_;

    // VisionPro Configuration
    this->thumbFingerJointsIndex = std::vector<size_t>(XRBase::VisionProConfig::ThumbFingerJointsIndex.begin(),
                                                       XRBase::VisionProConfig::ThumbFingerJointsIndex.end());
    this->indexFingerJointsIndex = std::vector<size_t>(XRBase::VisionProConfig::IndexFingerJointsIndex.begin(),
                                                       XRBase::VisionProConfig::IndexFingerJointsIndex.end());
    this->middleFingerJointsIndex = std::vector<size_t>(XRBase::VisionProConfig::MiddleFingerJointsIndex.begin(),
                                                        XRBase::VisionProConfig::MiddleFingerJointsIndex.end());
    this->ringFingerJointsIndex = std::vector<size_t>(XRBase::VisionProConfig::RingFingerJointsIndex.begin(),
                                                      XRBase::VisionProConfig::RingFingerJointsIndex.end());
    this->littleFingerJointsIndex = std::vector<size_t>(XRBase::VisionProConfig::LittleFingerJointsIndex.begin(),
                                                        XRBase::VisionProConfig::LittleFingerJointsIndex.end());

}

bool HandGestureDetector::IsOkGesture(const HandBase::HandData& data){

    switch (this->type) {
    case XRBase::XRType::VisionPro:
        return this->IsOkGestureByVisionPro(data);
    default:
        break;
    }
}

bool HandGestureDetector::IsThumbsUpGesture(const HandBase::HandData& data){

    switch (this->type) {
    case XRBase::XRType::VisionPro:
        return this->IsThumbsUpGestureByVisionPro(data);
    default:
        break;
    }
}

bool HandGestureDetector::IsPinchGesture(const HandBase::HandData& data){

    switch (this->type) {
    case XRBase::XRType::VisionPro:
        return this->IsPinchGestureByVisionPro(data);
    default:
        break;
    }
}

bool HandGestureDetector::IsOkGestureByVisionPro(const HandBase::HandData& data){
    auto handPositions = data.handPositions;
    auto handGesture = data.handGesture;

    // threshold
    double angleThreshold = 165.0;
    double distThreshold = 0.02;// 2cm

    // Middle Finger 11-10-14
    double middleAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->middleFingerJointsIndex[1]],
                                handPositions[this->middleFingerJointsIndex[0]],
                                handPositions[this->middleFingerJointsIndex[4]]);
    std::cout<<"[HandGestureDetector::IsOkGestureByVisionPro] middleAngle: "<<middleAngle<<std::endl;
    bool middleFlag = middleAngle < 180.0 && middleAngle > angleThreshold;

    // Ring Finger 16-15-19
    double ringAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->ringFingerJointsIndex[1]],
                                handPositions[this->ringFingerJointsIndex[0]],
                                handPositions[this->ringFingerJointsIndex[4]]);
    std::cout<<"[HandGestureDetector::IsOkGestureByVisionPro] ringAngle: "<<ringAngle<<std::endl;
    bool ringFlag = ringAngle < 180.0 && ringAngle > angleThreshold;

    // Little Finger 21-20-23
    double littleAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->littleFingerJointsIndex[1]],
                                handPositions[this->littleFingerJointsIndex[0]],
                                handPositions[this->littleFingerJointsIndex[4]]);
    std::cout<<"[HandGestureDetector::IsOkGestureByVisionPro] littleAngle: "<<littleAngle<<std::endl;
    bool littleFlag = littleAngle < 180.0 && littleAngle > angleThreshold;

    // Thumb and Index Finger
    double dist = MatrixUtils::CalL2Dist(handPositions[this->indexFingerJointsIndex[4]],
                                         handPositions[this->thumbFingerJointsIndex[3]]);
    bool distFlag = dist < distThreshold;

    if( middleFlag && ringFlag && littleFlag && distFlag ){
        return true;
    }else{
        return false;
    }
}

bool HandGestureDetector::IsThumbsUpGestureByVisionPro(const HandBase::HandData& data){
    auto handPositions = data.handPositions;

    // threshold
    double angleThreshold = 5.0;
    double distThreshold = 0.015;

    double thumbAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->thumbFingerJointsIndex[0]],
                                handPositions[this->thumbFingerJointsIndex[1]],
                                handPositions[this->thumbFingerJointsIndex[3]]);
    bool thumbFlag = thumbAngle < angleThreshold;

    double dist = MatrixUtils::CalPoint2PlaneDist({handPositions[this->middleFingerJointsIndex[0]],
                                                   handPositions[this->indexFingerJointsIndex[1]],
                                                   handPositions[this->littleFingerJointsIndex[1]]},
                                                   handPositions[this->thumbFingerJointsIndex[3]]);
    bool distFalg = dist < distThreshold;

    if( thumbFlag && distFalg ){
        return true;
    }else{
        return false;
    }
}

bool HandGestureDetector::IsPinchGestureByVisionPro(const HandBase::HandData& data){
    // threshold
    double pinchThreshold = 0.012;

    bool pinchFlag = data.handGesture.pinchValue < pinchThreshold;

    if(pinchFlag){
        return true;
    }else{
        return false;
    }
}

