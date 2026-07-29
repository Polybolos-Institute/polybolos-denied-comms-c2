## GoogleTest 1.14.0

Place sources at `third_party/googletest` (contains top-level `CMakeLists.txt`).

```powershell
Invoke-WebRequest -Uri https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip -OutFile gtest.zip
Expand-Archive gtest.zip -DestinationPath .
Rename-Item googletest-1.14.0 googletest
```

## Eigen 3.4.0

Place sources at `third_party/eigen` (must contain `Eigen/Core`).

```powershell
Invoke-WebRequest -Uri https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip -OutFile eigen.zip
Expand-Archive eigen.zip -DestinationPath .
Rename-Item eigen-3.4.0 eigen
```

If absent, CMake falls back to FetchContent (may fail on some Windows network filesystems).
