#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace KalmanExamples
{
namespace MyRobot
{

/**
 * @brief 状态向量(x,y,theta)
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(State, T, 3)
    
    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! Orientation
    static constexpr size_t THETA = 2;
    
    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T theta()   const { return (*this)[ THETA ]; }
    
    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& theta()  { return (*this)[ THETA ]; }
};

/**
 * @brief 控制向量：速度，角速度(单位时间的角度变化量)
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Control, T, 2)
    
    //! Velocity
    static constexpr size_t V = 0;
    //! Angular Rate (Orientation-change)
    static constexpr size_t DTHETA = 1;
    
    T v()       const { return (*this)[ V ]; }
    T dtheta()  const { return (*this)[ DTHETA ]; }
    
    T& v()      { return (*this)[ V ]; }
    T& dtheta() { return (*this)[ DTHETA ]; }
};

/**
 * @brief 系统模型，从某时刻到下一时刻的状态转移
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
	typedef KalmanExamples::MyRobot::State<T> S;  // State type shortcut definition
    typedef KalmanExamples::MyRobot::Control<T> C;  // Control type shortcut definition
    
    /**
     * @brief 非线性状态转移函数
     * @param [in] x 当前时刻系统状态
     * @param [in] u 输入的控制向量
     * @returns 预测的下一时刻系统状态
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;
        
        // New orientation given by old orientation plus orientation change
        auto newOrientation = x.theta() + u.dtheta();
        // Re-scale orientation to [-pi/2 to +pi/2]
        
        x_.theta() = newOrientation;
        
        // New x-position given by old x-position plus change in x-direction
        // Change in x-direction is given by the cosine of the (new) orientation
        // times the velocity
        x_.x() = x.x() + std::cos( newOrientation ) * u.v();
        x_.y() = x.y() + std::sin( newOrientation ) * u.v();
        
        // Return transitioned state vector
        return x_;
    }
    
protected:
    /**
     * @brief 使用当前状态来更新系统的状态转移函数f的雅各比矩阵
     * @param x 当前系统状态，在此处线性化
     */
    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setZero();
        
        this->F( S::X, S::X ) = 1;
        this->F( S::X, S::THETA ) = -std::sin( x.theta() + u.dtheta() ) * u.v();
        
        this->F( S::Y, S::Y ) = 1;
        this->F( S::Y, S::THETA ) = std::cos( x.theta() + u.dtheta() ) * u.v();
        
        this->F( S::THETA, S::THETA ) = 1;
        
        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif