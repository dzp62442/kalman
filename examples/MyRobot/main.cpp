#define _USE_MATH_DEFINES
#include <cmath>

#include "SystemModel.hpp"
#include "MeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>

using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef MyRobot::State<T> State;
typedef MyRobot::Control<T> Control;
typedef MyRobot::SystemModel<T> SystemModel;

typedef MyRobot::Measurement<T> Measurement;
typedef MyRobot::MeasurementModel<T> MeasurementModel;

int main(int argc, char** argv)
{
    // 状态向量真实值
    State x;
    x.setZero();
    
    Control u;  // 控制输入
    SystemModel sys;  // 系统模型
    MeasurementModel mm;  // 观测模型
    
    // 生成随机噪声
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);  // 服从均值为0，标准差为1的正态分布
    
    // 纯预测器，没有观测更新
    Kalman::ExtendedKalmanFilter<State> predictor;
    // 扩展卡尔曼滤波器
    Kalman::ExtendedKalmanFilter<State> ekf;
    
    // 初始化滤波器
    predictor.init(x);
    ekf.init(x);
    
    T systemNoise = 0.1;  // 状态向量的噪声标准差
    T orientationNoise = 0.05;  // 观测向量的朝向分量的噪声标准差
    T distanceNoise = 0.05;  // 观测向量的距离分量的噪声标准差
    
    // Simulate for 100 steps
    const size_t N = 100;
    for(size_t i = 1; i <= N; i++)
    {
        // 控制向量输入
        // u.v() = 1. + std::sin( T(2) * T(M_PI) / T(N) );
        // u.dtheta() = std::sin( T(2) * T(M_PI) / T(N) ) * (1 - 2*(i > 50));
        u.v() = 1. + std::sin( T(2) * T(M_PI) / T(N) );
        u.dtheta() = std::sin( T(2) * T(M_PI) / T(N) );
        
        // 仿真系统模型
        x = sys.f(x, u);
        
        // 引入过程噪声Wk
        x.x() += systemNoise*noise(generator);
        x.y() += systemNoise*noise(generator);
        x.theta() += systemNoise*noise(generator);
        
        // 预测当前时刻系统状态
        auto x_pred = predictor.predict(sys, u);
        auto x_ekf = ekf.predict(sys, u);

        {
            // 仿真观测模型
            Measurement measure = mm.h(x);
            
            // 引入观测噪声Vk
            measure.d() += distanceNoise * noise(generator);
            measure.alpha() += orientationNoise * noise(generator);
            
            // 更新EKF
            x_ekf = ekf.update(mm, measure);
        }
        
        // 以csv格式打印输出
        std::cout   << x.x() << "," << x.y() << "," << x.theta() << ","
                    << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta()  << ","
                    << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta()
                    << std::endl;
    }
    
    return 0;
}
