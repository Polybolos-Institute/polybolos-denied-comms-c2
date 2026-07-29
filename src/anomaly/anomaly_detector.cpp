#include "anomaly/anomaly_detector.h"

#include <cmath>

namespace Polybolos {
namespace Fusion {

AnomalyResult AnomalyDetector::evaluate(const SensorObservation& obs) const {
    AnomalyResult r;
    if (obs.sensor_type == SensorObservation::Type::RADAR) {
        if (obs.radar.range < 0.0 || obs.radar.range > 1.0e7) {
            r.is_anomaly = true;
            r.score = 1.0;
            r.reason = "radar_range_out_of_bounds";
            return r;
        }
    }
    if (obs.sensor_credibility < 0.05) {
        r.is_anomaly = true;
        r.score = 1.0 - obs.sensor_credibility;
        r.reason = "credibility_floor";
        return r;
    }
    r.score = 0.0;
    return r;
}

}  // namespace Fusion
}  // namespace Polybolos
