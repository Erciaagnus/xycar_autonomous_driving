/**
 * @file PIDController.hpp
 * @author JeongHyeok Lim (henricus0973@korea.ac.kr)
 * @brief PID Controller Class header file
 * @version 1.2
 * @date 2024-03-27
 */
#ifndef BINARY_FILTER_HPP_
#define BINARY_FILTER_HPP_

#include <deque>
#include <iostream>
#include <memory>
#include <vector>

namespace Xycar {

template <typename PREC>
class BinaryFilter
{
public:
    using Ptr = std::unique_ptr<BinaryFilter>; ///< Pointer type of this class

    BinaryFilter(uint32_t sampleSize, PREC prior);

    /**
     * @brief Add new data to filter
     *
     * @param[in] newSample New position to be used in filtering
     */
    void addSample(bool newSample); // 이 함수를 통해 샘플을 추가함, 내부적으로 필터링 알고리즘 수행

    /**
     * @brief Get the filtered data
     *
     * @return Result of weighted moving average filtering
     */
    const PREC getResult() const { return mFilteringResult; } // 필터링 결과를 얻음.

private:
    const PREC mPrior; // 필터링 과정에서 사용되는 사전 확률. 생성자를 통해 초기화되며 이후 변경 불가
    const PREC mSampleSize; // 필터링에 사용될 샘플 크기

    PREC mFilteringResult; // 현재까지의 필터링 결과를 저장함

    std::deque<bool> mSamples; ///< Deque including values of samples

    PREC update(uint32_t sampleSize);
};
} // namespace Xycar
#endif // BINARY_FILTER_HPP_