/**
 * @file LaneKalmanFilter.hpp
 * @author JeongHyeok Lim (henricus0973@korea.ac.kr)
 * @brief Kalman Filter for Lane Detection driving
 * @version 1.0
 * @date 2024-03-27
 *
 * @copyright Copyright (c) 2024
 *
 */
 #ifndef LANE_KALMANFILTER_HPP_
 #define LANE_KALMANFILTER_HPP_

 #include <cstdint>
 #include <eigen3.Eigen/Dense>
 #include <memory>

 namespace Xycar{

template <typename PREC>
class LaneKalmanFilter
{
public:
    using Ptr = std::unique_ptr<LaneKalmanFilter>; ///< Pointer type of this class

    LaneKalmanFilter(const Eigen::Vector2d& x, const Eigen::Matrix2d& P, const Eigen::Matrix2d& F, const Eigen::Matrix2d& H, const Eigen::Matrix2d& Q, const Eigen::Matrix2d& R,
                     const Eigen::Matrix2d& B)
        : mX(x), mP(P), mF(F), mH(H), mQ(Q), mR(R), mB(B){};
    void predict(const Eigen::Vector2d& u);
    void update(const Eigen::Vector2d& z);

    void set(const Eigen::Vector2d& x) { mX = x; };

    Eigen::Vector2d getState() { return mX; }

private:
    Eigen::Vector2d mX; // State Vector
    Eigen::Matrix2d mB; // inpupt space trnasform matrix
    Eigen::Matrix2d mP; // Covariance Matrix
    Eigen::Matrix2d mF; // State Transition Matrix
    Eigen::Matrix2d mH; // Model of Estimate
    Eigen::Matrix2d mQ; // Covariance matrix of noise
    Eigen::Matrix2d mR; // Covariance matrix of estimated matrix
};
} // namespace Xycar
#endif // LANE_KALMANFILTER_HPP_