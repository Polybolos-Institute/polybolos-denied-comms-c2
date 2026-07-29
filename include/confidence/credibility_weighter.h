#pragma once

#include "fusion/multi_sensor_fusion.h"

namespace Polybolos {
namespace Fusion {

/// Week 1 stub: per-sensor credibility weights.
class CredibilityWeighter {
public:
    double weight(SensorObservation::Type type, double raw_score) const;
    void noteAgreement(SensorObservation::Type type, bool agrees);

private:
    double radar_w_ = 1.0;
    double rf_w_ = 1.0;
    double optical_w_ = 1.0;
};

}  // namespace Fusion
}  // namespace Polybolos
