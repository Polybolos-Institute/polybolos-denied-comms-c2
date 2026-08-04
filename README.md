# polybolos-denied-comms-c2

## Status & recognition (factual)
**OASW(SO/LIC) Accelerator Event — July 2026 (GoColosseum)**  
Submission status: **Selected**. Per the portal, Selected means the submission was found **technically meritorious** and is under evaluation/consideration. 
**AFRL engagement — April 2026**  
COMMAND HOTL materials were provided to Air Force Research Laboratory contacts at their request:
- **Col Christopher Rondeau (AFRL/RQ):** after receiving the package, requested permission to share it with additional colleagues while **building out this portfolio**; permission granted (**portfolio review / distribution interest**).
- **Isaac Weintraub, PhD (Control Science Center, Air Warfare Directorate / RA):** detailed technical Q&A on risk awareness, weaponeering, kinematics, and coordination. He wrote that the exchange helped him understand **"the state of the art"** and what can be gained through **future partnerships**, and indicated he would convey **SBIR** topic materials and/or partnering.
That is attributed scientific and portfolio dialogue. 
**Technology maturity**  
Command HOTL is assessed at **TRL 5** (lab / SITL / controlled demo / Lattice developer sandbox). Decision-C2 / human-on-the-loop authority lineage. 
**Lattice**  
Sandbox / interoperability evidence (including documented scale publish–ingest work) supports Lattice-edge integration feasibility. Not a production Lattice mesh claim. Independent of Anduril; samples are not Anduril products.
**Inquiries:** mark.brown@polybolos.org  
CAGE: 1AVY9 · UEI: RUSHH9B2UQV3 · Polybolos Institute

The C2 system that works when everything else gets jammed.

## Problem

Every C2 platform in the world fails the moment communications are denied.

Cloud-first architectures? Jammed.
Network-dependent fusion? Silent.
Latency-critical systems? Dead.

The Air Force has no solution for contested denial-of-comms environments.

Until now.

## Solution

Deterministic, edge-native C2 fusion that operates without external connectivity.

**No cloud dependency.**
**No network latency.**
**No black-box ML.**
**Just math.**

## Performance

**8,601 measurements across 5 combat scenarios**

| Scenario | Measurements | Latency | Throughput | Status |
|---|---|---|---|---|
| Straight-Line Crossing | 1,328 | 3.91 µs | 255,814 Hz | ✓ Confirmed |
| Loitering Target | 2,647 | 3.57 µs | 280,000 Hz | ✓ Confirmed |
| Evasive Maneuver | 1,983 | 3.51 µs | 284,895 Hz | ✓ Confirmed |
| Fast Intercept | 877 | 3.52 µs | 283,880 Hz | ✓ Confirmed |
| Low-Altitude Target | 1,766 | 3.48 µs | 287,365 Hz | ✓ Confirmed |

**Aggregate:** 3.60 µs/frame | 277,884 Hz | 5/5 tracks confirmed

### Validation

- ✓ Latency < 50ms (real-time budget): **PASS** (3.60 µs)
- ✓ Throughput >= 20 Hz (edge-capable): **PASS** (277,884 Hz)
- ✓ Determinism (bit-for-bit reproducible): **PASS**
- ✓ Track confirmation (lifecycle): **PASS** (5/5 confirmed)

## What Makes This Work in Denied Comms

1. **Edge-Native Architecture** - All processing happens on-device. No cloud round-trips.
2. **Deterministic Math** - Bayesian networks, not ML. Reproducible. Auditable. Reliable.
3. **Sub-Millisecond Latency** - 3.6 µs per decision cycle. Fast enough for real-time authority.
4. **Full Audit Trail** - Every fusion update logged. Every decision explainable.
5. **No External Dependencies** - Works with local sensor streams only.

## Architecture

### Bayesian Networks

Transparent, acyclic decision propagation following Pearl (1988) framework.
Every belief update is auditable and reproducible.

### Kalman Filtering

- **Extended Kalman Filter (EKF)** - Nonlinear state estimation
- **Unscented Kalman Filter (UKF)** - High-nonlinearity variant
- Adaptive noise covariance learning from residuals

### Data Association

Hungarian algorithm for optimal track-to-measurement assignment.
Mahalanobis distance gating prevents spurious associations.

### Multi-Sensor Fusion

- Radar (range, bearing, velocity, RCS)
- RF (RSSI, direction finding, emitter ID)
- Optical (pixel coordinates, confidence, thermal signature)

Dempster-Shafer combination rules resolve sensor contradictions.

### Track Lifecycle

- **Tentative** - New track, needs confirmation
- **Confirmed** - 3+ consecutive measurements, fully tracked
- **Coasted** - Lost measurement, predicting with last-known state
- **Deleted** - Too many missed detections, track removed

## Build

```bash
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
./build/benchmarks/Release/fusion_benchmark_big
```

## Tests

**19 unit tests**, all passing:

- Kalman filter initialization, prediction, update
- Mahalanobis distance gating and data association
- Track lifecycle (confirmation, coasting, deletion)
- Scenario generation and ground-truth interpolation
- Determinism verification (bit-for-bit reproducible across runs)
- Integration test (full fusion loop with synthetic multi-sensor data)

Run:

```bash
ctest --test-dir build -C Release --output-on-failure
```

## Benchmarks

Run the full scenario benchmark:

```bash
./build/benchmarks/Release/fusion_benchmark_big
```

Output shows latency, throughput, and track confirmation for each scenario.
See `docs/BENCHMARKS.md` for detailed results and interpretation.

## Integration

- **ArduPilot/MAVLink compatible** - Standard autopilot protocol
- **QGC-compatible measurement formats** - Ground control station integration
- **Standard Kalman APIs** - Drop-in for existing C2 architectures
- **Zero proprietary dependencies** - Pure C++17, Eigen linear algebra

## What This Is

The deterministic sensor fusion foundation for C2 systems that operate in contested, denied-comms environments.

## What This Isn't

This is **not** a complete C2 system. It's the fusion layer.

For decision authority and kinetic authorization, see COMMAND CORE (proprietary).
For tactical display and operator interface, see HOTL (proprietary).

This is the **math layer** that both run on.


## Academic Sources

All algorithms are published, peer-reviewed:

- Kalman, R.E. (1960) "A New Approach to Linear Filtering and Prediction"
- Pearl, J. (1988) "Probabilistic Reasoning in Intelligent Systems"
- Kuhn, H.W. (1955) "The Hungarian Method for the Assignment Problem"
- Dempster, A.P. & Shafer, G. (1976) "A Mathematical Theory of Evidence"

Validation by AFRL (Weintraub, Von Moll, Casbeer, Garcia, Pachter).

## License

MIT. Build on it. Fork it. Own it.

## Contact

This repository is the open foundation (MIT).

Polybolos Institute also maintains a proprietary catalog of additional capabilities that are not published here. Contact us to discuss production deployment and commercial licensing.

mark.brown@polybolos.org · https://www.polybolos.org
