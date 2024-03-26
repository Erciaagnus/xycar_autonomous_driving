/**
 * @file LaneKalmanFilter.cpp
 * @author JeongHyeok Lim (henricus0973@korea.ac.kr)
 * @brief Kalman Filter c++ file
 * @version 1.0
 * @date 2024-03-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "LaneKeepingSystem/LaneKalmanFilter.hpp"
namespace Xycar {

template <typename PREC>
void LaneKalmanFilter<PREC>::predict(const Eigen::Vector2d& u)
{
    mX = mF * mX + mB * u;
    mP = mF * mP * mF.transpose() + mQ;
}

template <typename PREC>
void LaneKalmanFilter<PREC>::update(const Eigen::Vector2d& z)
{
    Eigen::Vector2d y = z - mH * mX;
    Eigen::Matrix2d S = mH * mP * mH.transpose() + mR;
    Eigen::Matrix<double, 2, 2> K = mP * mH.transpose() * S.inverse();

    mX = mX + K * y;
    mP = (Eigen::Matrix2d::Identity() - K * mH) * mP;
}

template class LaneKalmanFilter<float>;
template class LaneKalmanFilter<double>;
} // namespace Xycar