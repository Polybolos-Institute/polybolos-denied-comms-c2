#include "confidence/credibility_weighter.h"

#include <algorithm>

namespace Polybolos {
namespace Fusion {

double CredibilityWeighter::weight(SensorObservation::Type type,
                                   double raw_score) const {
    double w = 1.0;
    switch (type) {
        case SensorObservation::Type::RADAR:
            w = radar_w_;
            break;
        case SensorObservation::Type::RF:
            w = rf_w_;
            break;
        case SensorObservation::Type::OPTICAL:
            w = optical_w_;
            break;
    }
    return w * raw_score;
}

void CredibilityWeighter::noteAgreement(SensorObservation::Type type,
                                        bool agrees) {
    auto& w = (type == SensorObservation::Type::RADAR)    ? radar_w_
            : (type == SensorObservation::Type::RF)       ? rf_w_
                                                          : optical_w_;
    if (agrees) {
        w = std::min(1.0, w + 0.01);
    } else {
        w = std::max(0.1, w - 0.05);
    }
}

}  // namespace Fusion
}  // namespace Polybolos
