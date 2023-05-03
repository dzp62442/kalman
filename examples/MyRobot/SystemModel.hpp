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
    
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
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
    
    static constexpr size_t V = 0;
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
	typedef KalmanExamples::MyRobot::State<T> S;
    typedef KalmanExamples::MyRobot::Control<T> C;
    /**
     * @brief 非线性状态转移函数
     * @param [in] x 当前时刻系统状态
     * @param [in] u 输入的控制向量
     * @returns 预测的下一时刻系统状态
     */
    S f(const S& x, const C& u) const
    {
        S x_;  //! 下一时刻的状态向量预测值
        
        auto newOrientation = x.theta() + u.dtheta();        
        x_.theta() = newOrientation;
        
        x_.x() = x.x() + std::cos( newOrientation ) * u.v();
        x_.y() = x.y() + std::sin( newOrientation ) * u.v();
        
        return x_;
    }
    
protected:
    /**
     * @brief 使用当前状态来更新系统的状态转移函数f的雅各比矩阵
     * @param x 当前系统状态，在此处线性化
     */
    void updateJacobians( const S& x, const C& u )
    {
        // 关于状态的雅各比矩阵 F = df/dx
        this->F.setZero();
        
        this->F( S::X, S::X ) = 1;
        this->F( S::X, S::THETA ) = -std::sin( x.theta() + u.dtheta() ) * u.v();
        
        this->F( S::Y, S::Y ) = 1;
        this->F( S::Y, S::THETA ) = std::cos( x.theta() + u.dtheta() ) * u.v();
        
        this->F( S::THETA, S::THETA ) = 1;
        
        // 关于噪声的雅各比矩阵 W = df/dw
        this->W.setIdentity();
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif