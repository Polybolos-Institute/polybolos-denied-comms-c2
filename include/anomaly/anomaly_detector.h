#pragma once

#include <string>

#include "fusion/multi_sensor_fusion.h"

namespace Polybolos {
namespace Fusion {

/// Week 1 stub: spoofing / anomaly flags on observations.
struct AnomalyResult {
    bool is_anomaly = false;
    double score = 0.0;
    std::string reason;
};

class AnomalyDetector {
public:
    AnomalyResult evaluate(const SensorObservation& obs) const;
};

}  // namespace Fusion
}  // namespace Polybolos
