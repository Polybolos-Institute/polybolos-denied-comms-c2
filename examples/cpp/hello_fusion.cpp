#include "fusion/multi_sensor_fusion.h"

#include <iostream>

int main() {
    Polybolos::Fusion::MultiSensorFusion fusion;
    Polybolos::Fusion::SensorObservation obs;
    obs.sensor_type = Polybolos::Fusion::SensorObservation::Type::RADAR;
    obs.radar.range = 500.0;
    fusion.addObservation(obs);
    std::cout << "tracks=" << fusion.getFusedTracks().size() << "\n";
    return 0;
}
