
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
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
    // 状态向量真实值（无噪声）
    State x;
    x.setZero();
    
    // Control input
    Control u;
    // System model
    SystemModel sys;
    
    // Measurement model
    MeasurementModel mm;
    
    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);
    
    // Some filters for estimation
    // Pure predictor without measurement updates
    Kalman::ExtendedKalmanFilter<State> predictor;
    // Extended Kalman Filter
    Kalman::ExtendedKalmanFilter<State> ekf;
    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);
    
    // Init filters with true system state
    predictor.init(x);
    ekf.init(x);
    ukf.init(x);
    
    // Standard-Deviation of noise added to all state vector components during state transition
    T systemNoise = 0.01;
    // Standard-Deviation of noise added to all measurement vector components in orientation measurements
    T orientationNoise = 0.025;
    // Standard-Deviation of noise added to all measurement vector components in distance measurements
    T distanceNoise = 0.025;
    
    // Simulate for 100 steps
    const size_t N = 100;
    for(size_t i = 1; i <= N; i++)
    {
        // 控制向量输入
        u.v() = 1. + std::sin( T(2) * T(M_PI) / T(N) );
        u.dtheta() = std::sin( T(2) * T(M_PI) / T(N) ) * (1 - 2*(i > 50));
        
        // Simulate system
        x = sys.f(x, u);
        
        // Add noise: Our robot move is affected by noise (due to actuator failures)
        x.x() += systemNoise*noise(generator);
        x.y() += systemNoise*noise(generator);
        x.theta() += systemNoise*noise(generator);
        
        // Predict state for current time-step using the filters
        auto x_pred = predictor.predict(sys, u);
        auto x_ekf = ekf.predict(sys, u);
        auto x_ukf = ukf.predict(sys, u);

        {
            // We can measure the position every 10th step
            Measurement measure = mm.h(x);
            
            // Measurement is affected by noise as well
            measure.d() += distanceNoise * noise(generator);
            measure.alpha() += orientationNoise * noise(generator);
            
            // Update EKF
            x_ekf = ekf.update(mm, measure);
            
            // Update UKF
            x_ukf = ukf.update(mm, measure);
        }
        
        // Print to stdout as csv format
        std::cout   << x.x() << "," << x.y() << "," << x.theta() << ","
                    << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta()  << ","
                    << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta()  << ","
                    << x_ukf.x() << "," << x_ukf.y() << "," << x_ukf.theta()
                    << std::endl;
    }
    
    return 0;
}
