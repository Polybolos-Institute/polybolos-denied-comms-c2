#include "bayesian/bayesian_network.h"
#include "fusion/multi_sensor_fusion.h"

#include <cstdlib>
#include <iostream>

int main() {
    using namespace Polybolos::Fusion;

    BayesianNetwork net;
    BayesianNode presence;
    presence.id = "target_present";
    presence.states = {"yes", "no"};
    net.addNode(presence);

    const auto belief = net.queryBelief("target_present");
    if (belief.size() != 2) {
        std::cerr << "FAIL: expected 2 prior states\n";
        return 1;
    }

    MultiSensorFusion fusion;
    SensorObservation obs;
    obs.sensor_type = SensorObservation::Type::RADAR;
    obs.timestamp = 0.0;
    obs.radar.range = 1200.0;
    obs.radar.bearing = 45.0;
    fusion.addObservation(obs);
    fusion.processCycle(0.05);

    if (fusion.getFusedTracks().empty()) {
        std::cerr << "FAIL: expected at least one fused track after cycle\n";
        return 1;
    }

    std::cout << "PASS fusion_smoke_test\n";
    return 0;
}
