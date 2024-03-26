#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <yaml-cpp/yaml.h> //Included yaml header file.

namespace Xycar {
/**
 * @brief PID Controller Class
 * @tparam PREC Precision of data
 */

template <typename PREC>
class PIDController
{
public:
    using Ptr = std::unique_ptr<PIDController>; ///< Pointer type of this class

    // Constructor that takes three parameters for PID values
    PIDController(PREC kp, PREC ki, PREC kd);

    // Function to compute the control output
    PREC getControlOutput(PREC error);

private:
    const PREC Kp,Ki, Kd; // Define PID Coefficients as "Constant"
    PREC integral; // Define I, D, P term
    PREC prev_error;
    PREC derivative;
};

// template <typename PREC>
// PIDController<PREC>::PIDController(PREC kp, PREC ki, PREC kd)
//     : kp_(kp), ki_(ki), kd_(kd) {
// }

// template <typename PREC>
// PREC PIDController<PREC>::getControlOutput(PREC error) {
//     return (kp_ * error); // Simplified example
// }

} // namespace Xycar
#endif // PID_CONTROLLER_HPP_
