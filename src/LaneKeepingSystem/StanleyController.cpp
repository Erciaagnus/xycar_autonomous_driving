// Copyright (C) 2023 Grepp CO.
// All rights reserved.

#include "LaneKeepingSystem/StanleyController.hpp"

namespace Xycar {

template <typename PREC>
void StanleyController<PREC>::calculateSteeringAngle(PREC crossTrackError, PREC headingError, PREC velocity)
{
    // Calculate the cross-track error (cte) compensation
    PREC alpha = std::atan2(-mGain * crossTrackError, velocity);

    // Calculate the desired heading angle
    PREC desiredHeading = this->normalizeAngle(headingError) + alpha;

    // Calculate the steering angle using the desired heading and look-ahead distance
    PREC steeringAngle = std::atan2(2 * mLookAheadDistance * std::sin(desiredHeading), velocity);

    this->mResult = steeringAngle;
}
template <typename PREC>
PREC StanleyController<PREC>::normalizeAngle(PREC angle) const {
    // normalize the angle to range of [-π, π]
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

template class StanleyController<float>;
template class StanleyController<double>;
} // namespace Xycar
