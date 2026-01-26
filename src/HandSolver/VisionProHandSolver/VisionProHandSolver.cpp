#include <VisionProHandSolver/VisionProHandSolver.hpp>

VisionProHandSolver::VisionProHandSolver(const HandSolver::BasicConfig &config_){
    this->handDof = config_.handDof;
    this->Init();
}

VisionProHandSolver::~VisionProHandSolver(){


}

Eigen::VectorXd VisionProHandSolver::SolveSingleHand(const HandBase::HandData& data,
                                                     const HandBase::HandType& type){
    std::vector<double> ret;
    auto handPositions = data.handPositions;

    // Thumb Finger 1-4-6
//    double thumbAngle =
//            MatrixUtils::CalVecAngle(  handPositions[this->thumbFingerJointsIndex[0]],
//                                handPositions[this->indexFingerJointsIndex[1]],
//                                handPositions[this->thumbFingerJointsIndex[3]]);
//    ret.push_back(thumbAngle);

//    // Thumb Finger 2-3-4
//    double thumbAngle =
//            MatrixUtils::CalVecAngle(  handPositions[this->thumbFingerJointsIndex[1]],
//                                handPositions[this->thumbFingerJointsIndex[2]],
//                                handPositions[this->thumbFingerJointsIndex[3]]);
//    std::cout << "Original Thumb Finger's Angle: " << thumbAngle << std::endl;
//    thumbAngle = this->fingersUpperBound[0] - thumbAngle;
//    ret.push_back(thumbAngle);

    // Thumb Finger 1-2-4
    double thumbAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->thumbFingerJointsIndex[0]],
                                handPositions[this->thumbFingerJointsIndex[1]],
                                handPositions[this->thumbFingerJointsIndex[3]]);
//    std::cout << "Original Thumb Finger's Angle: " << thumbAngle << std::endl;
    thumbAngle = this->fingersUpperBound[0] - thumbAngle;
    ret.push_back(thumbAngle);

    // Index Finger 6-5-9
    double indexAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->indexFingerJointsIndex[1]],
                                handPositions[this->indexFingerJointsIndex[0]],
                                handPositions[this->indexFingerJointsIndex[4]]);
    ret.push_back(indexAngle);

    // Middle Finger 11-10-14
    double middleAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->middleFingerJointsIndex[1]],
                                handPositions[this->middleFingerJointsIndex[0]],
                                handPositions[this->middleFingerJointsIndex[4]]);
    ret.push_back(middleAngle);

    // Ring Finger 16-15-19
    double ringAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->ringFingerJointsIndex[1]],
                                handPositions[this->ringFingerJointsIndex[0]],
                                handPositions[this->ringFingerJointsIndex[4]]);
    ret.push_back(ringAngle);

    // Little Finger 21-20-24
    double littleAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->littleFingerJointsIndex[1]],
                                handPositions[this->littleFingerJointsIndex[0]],
                                handPositions[this->littleFingerJointsIndex[4]]);
    ret.push_back(littleAngle);

    // Thumb Finger Rotation 6-3-21
    double thumbRotAngle =
            MatrixUtils::CalVecAngle(  handPositions[this->indexFingerJointsIndex[1]], // 6
                                    handPositions[this->thumbFingerJointsIndex[2]], // 3
                                    handPositions[this->littleFingerJointsIndex[1]]); // 21
    thumbRotAngle = 180.0 - thumbRotAngle;
//    std::cout<<"Thumb Rotation: "<<thumbRotAngle<<std::endl;
    ret.push_back(thumbRotAngle);

    // Clamp
    for(size_t i=0;i<ret.size();i++){
//        std::cout<<"Original Angle: "<<ret[i]<<std::endl;
        ret[i] = std::min(std::max(ret[i], this->fingersLowerBound[i]), this->fingersUpperBound[i]);
//        std::cout<<"Clamped Angle: "<<ret[i]<<std::endl;
    }

    // Temp print for HandGestureDetector
//    double dist = MatrixUtils::CalPoint2PlaneDist({handPositions[this->middleFingerJointsIndex[0]],
//                                                   handPositions[this->indexFingerJointsIndex[1]],
//                                                   handPositions[this->littleFingerJointsIndex[1]]},
//                                                   handPositions[this->thumbFingerJointsIndex[3]]);
//    std::cout<<"Dist: "<<dist<<std::endl;

    // map
    return this->MapXR2Hand(ret, type);
}

Eigen::VectorXd VisionProHandSolver::SolveDualHand(const HandBase::DualHandData& data)
{
    auto leftHandAngle = this->SolveSingleHand(data.leftHandData,data.type);
    auto rightHandAngle = this->SolveSingleHand(data.rightHandData,data.type);

    Eigen::VectorXd ret(leftHandAngle.size() + rightHandAngle.size());
    ret << leftHandAngle, rightHandAngle;

//    std::cout<<"[VisionProHandSolver::SolveDualHand] Result: "<<std::endl;
//    std::cout<<ret<<std::endl;

    // Filter the data
    filter.AddData(ret);
    ret = filter.GetFilteredData();

    return ret;
}

std::vector<double> VisionProHandSolver::GetLowerBound()
{
    return this->fingersLowerBound;
}

std::vector<double> VisionProHandSolver::GetUpperBound()
{
    return this->fingersUpperBound;
}

std::vector<std::string> VisionProHandSolver::GetFingersName()
{
    return this->fingersName;
}

void VisionProHandSolver::Init(){
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

    // Original Angle Limits
    this->fingersLowerBound = {0,
                               45,
                               45,
                               45,
                               45,
                               30};
    this->fingersUpperBound = {36,
                               170,
                               170,
                               170,
                               170,
                               110};

    this->fingersName = {
        "Thumb",
        "Index",
        "Middle",
        "Ring",
        "Little",
        "ThumbRot",
    };

    // Filter
    this->filter.Init(std::vector<double>{0.6, 0.2, 0.2}, this->handDof);
}

Eigen::VectorXd VisionProHandSolver::MapXR2Hand(const std::vector<double>& angles,
                                                const HandBase::HandType& type){
    Eigen::VectorXd ret;
    switch (type) {
    case HandBase::HandType::ROHand:
        ret.resize(HandBase::ROHandConfig::NumFingers);
        for(size_t i=0;i<HandBase::ROHandConfig::NumFingers;i++){
            double targetLow    = HandBase::ROHandConfig::FingersLowerBound[i];
            double targetHigh = HandBase::ROHandConfig::FingersUpperBound[i];

            double currentLow = this->fingersLowerBound[i];
            double currentHigh = this->fingersUpperBound[i];

            ret(i) =
                    targetLow + (angles[i] - currentLow)/(currentHigh - currentLow) * (targetHigh - targetLow);
        }
        return ret;
    case HandBase::HandType::Revo2Hand:
        ret.resize(HandBase::ROHandConfig::NumFingers);
        for(size_t i=0;i<HandBase::ROHandConfig::NumFingers;i++){
            double targetLow    = HandBase::Revo2HandConfig::FingersLowerBound[i];
            double targetHigh   = HandBase::Revo2HandConfig::FingersUpperBound[i];

            double currentLow = this->fingersLowerBound[i];
            double currentHigh = this->fingersUpperBound[i];

            ret(i) =
                    targetLow + (angles[i] - currentLow)/(currentHigh - currentLow) * (targetHigh - targetLow);
        }
        return ret;
    default:
        throw std::invalid_argument("[VisionProHandSolver::Map] Plz provide valid HandType! ");
    }


}
