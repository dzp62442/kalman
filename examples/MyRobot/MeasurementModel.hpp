#ifndef KALMAN_EXAMPLES_MyRobot_MEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_MyRobot_MEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

namespace KalmanExamples
{
namespace MyRobot
{

/**
 * @brief 观测向量，到地标（原点）的距离和朝向角度
 * @param T Numeric scalar type
 */
template<typename T>
class Measurement : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Measurement, T, 2)
    
    //! 到地标的距离
    static constexpr size_t D = 0;
    
    //! 与地标的朝向夹角
    static constexpr size_t ALPHA = 1;
    
    T d()       const { return (*this)[ D ]; }
    T alpha()       const { return (*this)[ ALPHA ]; }
    
    T& d()      { return (*this)[ D ]; }
    T& alpha()      { return (*this)[ ALPHA ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class MeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, Measurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  KalmanExamples::MyRobot::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::MyRobot::Measurement<T> M;
    
    /**
     * @brief Constructor
     */
    MeasurementModel()
    {
        // Setup noise jacobian. As this one is static, we can define it once
        // and do not need to update it dynamically
        this->H.setIdentity();
        this->V.setIdentity();
    }
    
    /**
     * @brief 观测方程
     * @param [in] x 当前时刻系统状态
     * @returns 预测的传感器测量值
     */
    M h(const S& x) const
    {
        M measurement;
        
        // Robot position as (x,y)-vector
        // This uses the Eigen template method to get the first 2 elements of the vector
        Kalman::Vector<T, 2> position = x.template head<2>();
        
        // 机器人到地标的距离
        measurement.d() = std::sqrt( position.dot(position) );
        
        // 机器人相对于地标的朝向
        T tmp = std::atan2( position[1], position[0] );
        measurement.alpha() = tmp - x.theta();
        
        return measurement;
    }

protected:
    
    /**
     * @brief 观测方程关于状态向量的雅各比矩阵
     * @param x 当前系统状态，在此处进行线性化
     */
    void updateJacobians( const S& x )
    {
        // H = dh/dx (Jacobian of measurement function w.r.t. the state)
        this->H.setZero();
        
        // Robot position as (x,y)-vector
        // This uses the Eigen template method to get the first 2 elements of the vector
        Kalman::Vector<T, 2> position = x.template head<2>();
        
        // Distances
        T d2 = position.dot(position);
        T d = std::sqrt(d2);
        
        this->H( M::D, S::X ) = position[0] / d;
        this->H( M::D, S::Y ) = position[1] / d;
        
        this->H( M::ALPHA, S::X ) = - position[1] / d2;
        this->H( M::ALPHA, S::Y ) = position[0] / d2;
        this->H( M::ALPHA, S::THETA ) = -1;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif