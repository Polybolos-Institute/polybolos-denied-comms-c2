# Contributing to polybolos-denied-comms-c2

## Philosophy

This is a defense research project. Contributions should prioritize:

1. **Correctness** - Math must be verified, tests must pass, determinism must hold.
2. **Auditability** - Every decision must be explainable and reproducible.
3. **Simplicity** - Complex code is a liability in C2. Prefer clarity over optimization.
4. **Standards** - Follow published algorithms (no proprietary tricks).

## How to Contribute

### Reporting Issues

Create an issue with:

- Clear description of the problem
- Steps to reproduce
- Expected vs. actual behavior
- Benchmark impact (latency, throughput)

### Submitting Code

1. **Fork the repo**
2. **Create a branch** (`feature/your-feature` or `fix/your-fix`)
3. **Write tests** - New code requires unit tests. All tests must pass.
4. **Verify determinism** - Run the same test 10 times. Results must be identical.
5. **Run benchmarks** - Measure latency/throughput impact before and after.
6. **Submit a PR** with:
 - Clear description of changes
 - Benchmark results
 - Test results
 - References to academic sources (if applicable)

### Coding Standards

- **C++17** - Use modern features, but keep code readable.
- **Eigen for linear algebra** - Don't reinvent matrix operations.
- **No randomness in core loops** - Determinism is non-negotiable.
- **Document assumptions** - Comments should explain *why*, not *what*.

### Testing Requirements

All PRs must:

- Pass all 19 unit tests
- Pass determinism verification (bit-for-bit reproducibility)
- Maintain or improve benchmark latency
- Include new tests for new functionality

```bash
ctest --test-dir build -C Release --output-on-failure
./build/benchmarks/Release/fusion_benchmark_big
```

### Commit Messages

- Clear, descriptive subject line (imperative mood)
- Reference issues or PRs if applicable
- Include benchmark impact if relevant

Example:

```
Add UKF covariance correction for high-nonlinearity scenarios

Implements sigma-point adaptation per Nobre & Gee (2019)
Latency impact: +1.2 µs/cycle (acceptable for UKF variant)
All tests passing, determinism verified

Closes #42
```

## Code Review

All PRs require:

- Functional correctness (tests pass)
- Performance impact (benchmarks acceptable)
- Documentation (if user-facing)
- Determinism verification (if core fusion)

## Questions?

Open an issue or contact mark.brown@polybolos.org.
