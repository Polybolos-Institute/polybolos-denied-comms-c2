#include "filters/kalman_filter.h"
#include "fusion/data_association.h"
#include "sim/scenario_generator.h"
#include <chrono>
#include <iostream>
#include <iomanip>
using namespace Polybolos::Fusion;
using namespace Polybolos::Fusion::Sim;
int main() {
    std::cout << "=== Polybolos Sensor Fusion Benchmark ===" << std::endl;
    std::cout << std::endl;
    
    // Benchmark 1: EKF Performance
    {
        std::cout << "Benchmark 1: EKF Predict + Update (1000 iterations)" << std::endl;
        
        ExtendedKalmanFilter ekf;
        ExtendedKalmanFilter::MeasurementVector z;
        z << 1000.0, 0.0, 50.0, 500.0, 0.0, 1.0;
        ekf.initialize(z, 0.0);
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < 1000; i++) {
            ekf.predict(0.01);  // 10ms predict
            
            ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
                ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
            ExtendedKalmanFilter::MeasurementVector z_update;
            z_update << 1000.0 + i*0.5, 0.0, 50.0, 500.0, 0.0, 1.0;
            ekf.update(z_update, R);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        double avg_us = elapsed.count() / 1000.0;
        double avg_ms = avg_us / 1000.0;
        
        std::cout << "  Total time: " << elapsed.count() << " µs" << std::endl;
        std::cout << "  Avg per cycle: " << std::fixed << std::setprecision(3) << avg_us << " µs (" << avg_ms << " ms)" << std::endl;
        std::cout << "  Rate: " << 1000000.0 / avg_us << " cycles/sec" << std::endl;
        std::cout << std::endl;
    }
    
    // Benchmark 2: UKF Performance
    {
        std::cout << "Benchmark 2: UKF Predict + Update (1000 iterations)" << std::endl;
        
        UnscentedKalmanFilter ukf;
        ExtendedKalmanFilter::MeasurementVector z;
        z << 1000.0, 0.0, 50.0, 500.0, 0.0, 1.0;
        ukf.initialize(z, 0.0);
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < 1000; i++) {
            ukf.predict(0.01);
            
            ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
                ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
            ExtendedKalmanFilter::MeasurementVector z_update;
            z_update << 1000.0 + i*0.5, 0.0, 50.0, 500.0, 0.0, 1.0;
            ukf.update(z_update, R);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        double avg_us = elapsed.count() / 1000.0;
        
        std::cout << "  Total time: " << elapsed.count() << " µs" << std::endl;
        std::cout << "  Avg per cycle: " << std::fixed << std::setprecision(3) << avg_us << " µs" << std::endl;
        std::cout << std::endl;
    }
    
    // Benchmark 3: Scenario generation
    {
        std::cout << "Benchmark 3: Synthetic Scenario Generation (30 sec @ 100Hz)" << std::endl;
        
        ScenarioConfig config = PredefinedScenarios::straightLineCrossing();
        ScenarioGenerator gen(config);
        
        auto start = std::chrono::high_resolution_clock::now();
        auto output = gen.generateScenario();
        auto end = std::chrono::high_resolution_clock::now();
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "  Generated " << output.measurements.size() << " measurements" << std::endl;
        std::cout << "  Ground truth points: " << output.ground_truth.size() << std::endl;
        std::cout << "  Time: " << elapsed.count() << " ms" << std::endl;
        std::cout << std::endl;
    }
    
    std::cout << "Benchmarks complete." << std::endl;
    return 0;
}
