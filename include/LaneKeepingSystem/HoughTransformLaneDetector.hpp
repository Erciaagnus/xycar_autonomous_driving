/**
 * @file HoughTransformLaneDetector.hpp
 * @author JeongHyeok Lim (henricus0973@korea.ac.kr)
 * @brief Hough Transform Lane Detector class header file. Detect lane and estimate the position using kalman filter.
 * @version 1.2
 * @date 2024-03027
 */
#ifndef HOUGH_TRANSFORM_LANE_DETECTOR_HPP_
#define HOUGH_TRANSFORM_LANE_DETECTOR_HPP_

#include <iostream>
#include <memory>
#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

namespace Xycar {

{
    LEFT = 0,  ///< Line direction LEFT
    RIGHT = 1, ///< Line direction RIGHT
};


enum HoughIndex : uint8_t
{
    x1 = 0, ///< First point x
    y1 = 1, ///< First point y
    x2 = 2, ///< Second point x
    y2 = 3, ///< Second point y
};

using Line = cv::Vec4i;               ///< Line between two points
using Lines = std::vector<Line>;      ///< Vector of Lines
using Indices = std::vector<int32_t>; ///< Indices of lines

template <typename PREC>
class HoughTransformLaneDetector final
{
public:
    using Ptr = std::unique_ptr<HoughTransformLaneDetector>; ///< Pointer type of this class
    using KalmanFilterPtr = typename LaneKalmanFilter<PREC>::Ptr;

    static constexpr double kHoughRho = 4.0;                  ///< Distance resolution of the accumulator in pixels.
    static constexpr double kHoughTheta = CV_PI / 180.0;      ///< Angle resolution of the accumulator in radians. If C++20, CV_PI should be replaced with std::numbers::pi
    static constexpr int32_t kDebugLineWidth = 2;             ///< Thickness of lines for debugging
    static constexpr int32_t kDebugRectangleHalfWidth = 5;    ///< Half ot width of rectangle for debugging
    static constexpr int32_t kDebugRectangleStartHeight = 15; ///< Start height of rectangle for debugging
    static constexpr int32_t kDebugRectangleEndHeight = 25;   ///< End height of rectangle for debugging
    static inline const cv::Scalar kRed = { 0, 0, 255 };      ///< Scalar values of Red
    static inline const cv::Scalar kGreen = { 0, 255, 0 };    ///< Scalar values of Green
    static inline const cv::Scalar kBlue = { 255, 0, 0 };     ///< Scalar values of Blue

    HoughTransformLaneDetector(const YAML::Node& config) { setConfiguration(config); }
    // YAML의 파라미터로부터 받아옴.
    // 칼만 필터를 이용해 차선 위치 설정

    // 왼쪽 차선 위치 설정
    void setLeftLanePosition(int32_t lanePosition) {
        Eigen::Vector2d inputVector; // Eigen 라이브러리에서 inputVector 생성
        inputVector << static_cast<PREC>(lanePosition), 0.f; // lanePosition 변수를 PREC 타입으로 변환, 벡터 두 번째 요소로 0.0 사용. 부동 소수점
        mLeftKalmanFilter->set(inputVector); // mLeft칼만의 set 메서드를 호출해 2차원 벡터를 칼만필터에 설정함
        // set은 칼만필터의 초기상태 및 새로운 측정값을 설정하는 데 사용됨. 차선의 위치를 상태변수로 받음.
    };

    // 오른쪽 차선 위치 설정, 왼쪽 차선과 동일
    void setRightLanePosition(int32_t lanePosition) {
        Eigen::Vector2d inputVector;
        inputVector << static_cast<PREC>(lanePosition), 0.f;
        mRightKalmanFilter->set(inputVector);
    };
     // getLanePosition 메서드는 주어진 이미지 내에서 차선 위치 검출 및 위치를 나타내는 두 정수 쌍 반환
    std::pair<int32_t, int32_t> getLanePosition(const cv::Mat& image, bool runUpdate, std::string detection);

    // 입력 벡터를 기반으로 위치를 예측하고 그 예측된 위치를 나타내는 두 정수 쌍을 반환한다. 칼만필터를 이용해 차선 다음 위치를 예측할 때 사용한다.
    std::pair<int32_t, int32_t> predictLanePosition(Eigen::Vector2d& inputVector);

    void copyDebugFrame(const cv::Mat& image) { image.copyTo(mDebugFrame); }

    /**
     * @brief Draw the position rectangles on debug image
     *
     * @param[in] leftPositionX Left x position for drawing rectangular
     * @param[in] rightPositionX Right x position for drawing rectangular
     * @param[in] estimatedPositionX Estimated x position from moving average filter
     */
    void drawRectangles(int32_t leftPositionX, int32_t rightPositionX, int32_t estimatedPositionX);

    /**
     * @brief Get the Debug Frame object pointer
     *
     * @return Return frame for debuging
     */
    const cv::Mat& getDebugFrame() const { return mDebugFrame; };

    void setPrevLinePosition(int32_t leftPosition, int32_t rightPosition);
    std::pair<int32_t, int32_t> getPrevLinePosition();
    bool getStopLineStatus();

private:
    /**
     * @brief Set the parameters from config file
     *
     * @param[in] config Configuration including parameters for detector
     */
    void setConfiguration(const YAML::Node& config);

    /**
     * @brief Divide lines into left and right line index vector
     *
     * @param[in] lines Line vectors from Hough Transform
     * @return Left line index vector, right line index vector
     */
    std::pair<std::vector<std::vector<int>>, bool> divideLines(const Lines& lines);

    /**
     * @brief Get the line positions x
     *
     * @param[in] lines Computed all Hough lines
     * @param[in] lineIndices Left or right line index vector
     * @param[in] direction Left or right lane
     * @return Average x position
     */
    int32_t getLinePositionX(const Lines& lines, const Indices& lineIndices, Direction direction);

    /**
     * @brief Get the vaiables of line equation (First-order polynomial equation, y = mx + b)
     *
     * @param[in] lines Left and right lines to get slope(m) and intercept(b)
     * @param[in] lineIndices Candidate left or right indices of lines
     * @return Slope and intercept of 'y = mx + b' to determine hough line
     */
    std::pair<PREC, PREC> getLineParameters(const Lines& lines, const Indices& lineIndices);

    /**
     * @brief Draw the lines on debug image
     *
     * @param[in] lines Left and right lines
     * @param[in] leftLineIndex Left indices among lines
     * @param[in] rightLineIndex Right indices among lines
     */
    void drawLines(const Lines& lines, const Indices& leftLineIndices, const Indices& rightLineIndices);


// Private 정의
private:
    int32_t mCannyEdgeLowThreshold;  ///< Low threshold for Canny edge
    int32_t mCannyEdgeHighThreshold; ///< High threshold for Canny edge
    int32_t mHoughThreshold;         ///< Accumulator threshold parameter. Only those lines are returned that get enough votes
    int32_t mHoughMinLineLength;     ///< Minimum line length. Line segments shorter than that are rejected.
    int32_t mHoughMaxLineGap;        ///< Maximum allowed gap between points on the same line to link them.
    PREC mHoughClusterDistance;
    PREC mHoughRadian;
    PREC mHoughLineSlopeRange; ///< Slope range to limit Hough lines

    // Image parameters
    int32_t mImageWidth;     ///< The width of the image
    int32_t mImageHeight;    ///< The height of the image
    int32_t mROIStartHeight; ///< The height of the offset for debugging
    int32_t mROIHeight;      ///< Height of ROI

    int32_t mContinusThreshold;

    // Debug Image and flag
    cv::Mat mDebugFrame; ///< The frame for debugging
    bool mDebugging;     ///< Debugging or not
    bool mStopDetected;

    PREC mPrevLeftPosition;
    PREC mPrevRightPosition;

    KalmanFilterPtr mLeftKalmanFilter;
    KalmanFilterPtr mRightKalmanFilter;

    PREC mCalculateDistance(PREC x1, PREC y1, PREC x2, PREC y2);
};
} // namespace Xycar
#endif // HOUGH_TRANSFORM_LANE_DETECTOR_HPP_