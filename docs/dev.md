# Development Guide

This document is the developer-focused workflow reference for `orbslam3_ros2`.

> [!IMPORTANT]
> This is the `1.0` development baseline for local build/test/gate workflows.

## Prerequisites

- ROS 2 Humble installed
- Workspace root exported as `COLCON_WS`

```bash
export COLCON_WS=$HOME/my_ros2_ws
cd "$COLCON_WS"
```

> [!INFO]
> Install Conan with one of the following methods:

```bash
python3 -m pip install --user conan
```

```bash
uv tool install conan
```

## Build Workflow

```bash
conan install src/orbslam3_ros2 \
  -pr:h src/orbslam3_ros2/conan/profiles/linux_x86_64 \
  -pr:b src/orbslam3_ros2/conan/profiles/linux_x86_64 \
  -of build/conan \
  -b missing

source /opt/ros/humble/setup.bash
colcon build --packages-select orbslam3_ros2 \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/build/conan/conan_toolchain.cmake \
    -DCMAKE_PREFIX_PATH=$PWD/build/conan \
    -DCMAKE_BUILD_TYPE=Release
```

On arm64/aarch64, replace both profiles above with `src/orbslam3_ros2/conan/profiles/linux_armv8`.

## Build Via Script

`scripts/build_colcon.py` wraps `colcon build` for this package.

Recommended mode (Ninja + Conan toolchain):

```bash
source /opt/ros/humble/setup.bash
python3 src/orbslam3_ros2/scripts/build_colcon.py \
  --ws "$COLCON_WS" \
  --pkg orbslam3_ros2 \
  --ninja-conan
```

What `--ninja-conan` does:

- Runs `conan install` into `build/conan` (unless `--skip-conan` is set).
- Auto-picks profile by arch (`linux_x86_64` or `linux_armv8`) unless `--profile` is provided.
- Builds with `-G Ninja`, `-DCMAKE_TOOLCHAIN_FILE=.../build/conan/conan_toolchain.cmake`, and `-DCMAKE_PREFIX_PATH=.../build/conan`.
- Sets conservative build parallelism by default (`CMAKE_BUILD_PARALLEL_LEVEL=min(4, cpu/2)`).

Standard build:

```bash
source /opt/ros/humble/setup.bash
python3 src/orbslam3_ros2/scripts/build_colcon.py \
  --ws "$COLCON_WS" \
  --pkg orbslam3_ros2
```

Edge-device conservative build (single-worker + no testing):

```bash
source /opt/ros/humble/setup.bash
python3 src/orbslam3_ros2/scripts/build_colcon.py \
  --ws "$COLCON_WS" \
  --pkg orbslam3_ros2 \
  --ninja-conan \
  --edge
```

`--edge` additionally enforces:

- `Release` build type.
- single-job build (`CMAKE_BUILD_PARALLEL_LEVEL=1`, `MAKEFLAGS=-j1`, `NINJAFLAGS=-j1`).
- sequential colcon executor and `--parallel-workers 1`.
- `BUILD_TESTING=OFF` and `-O2 -g0` compile flags.

Clean workspace build artifacts first:

```bash
python3 src/orbslam3_ros2/scripts/build_colcon.py \
  --ws "$COLCON_WS" \
  --pkg orbslam3_ros2 \
  --clean
```

## Rebuild Local Conan Recipes

When editing recipes under `src/orbslam3_ros2/conan/recipes/*`:

1. Recreate the changed recipe package.
2. Re-run `conan install` for the workspace graph.
3. Rebuild package with colcon.

Example (g2o recipe):

```bash
python3 src/orbslam3_ros2/scripts/conan_rebuild.py \
  --profile src/orbslam3_ros2/conan/profiles/linux_x86_64 \
  --cwd src/orbslam3_ros2 \
  --mode create \
  --recipe-dir src/orbslam3_ros2/conan/recipes/g2o_orbslam3
```

## Test Workflow

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

colcon test --packages-select orbslam3_ros2
colcon test-result --all --verbose
```

## Local Gate Workflow

Smoke gate:

```bash
python3 src/orbslam3_ros2/scripts/run_e2e.py \
  --ws "$COLCON_WS" \
  --dataset fr1_xyz \
  --smoke \
  --artifact-dir /tmp/orbslam3_e2e_artifacts_smoke
```

Full gate:

```bash
python3 src/orbslam3_ros2/scripts/run_e2e.py \
  --ws "$COLCON_WS" \
  --dataset fr1_xyz \
  --full \
  --artifact-dir /tmp/orbslam3_e2e_artifacts_full
```

Gate artifacts:

- `<artifact-dir>/manifest.json`
- `<artifact-dir>/test_summary.json`
- runtime diagnostics (`ldd_orbslam3_core_node.txt`, `conan_graph_info.txt`)

## Known Issues We Hit (And Fixes)

> [!IMPORTANT]
> Profile drift caused unstable dependency behavior before 1.0.  
> Fix: use explicit arch profiles for host+build (`src/orbslam3_ros2/conan/profiles/linux_x86_64` or `src/orbslam3_ros2/conan/profiles/linux_armv8`) and keep Eigen ABI safety flags there.

> [!INFO]
> Symptom: gate fails with `invalid_config=true` in `manifest.json`.  
> Cause: playback window too short (`play_seconds < min_play_seconds_required`).  
> Fix: increase `--max-frames` / `--fps` or set longer `ORBSLAM3_E2E_PLAY_SECONDS`.

> [!INFO]
> Symptom: runtime check reports `libg2o.so` not resolved from Conan cache.  
> Fix: rerun `conan install` with the matching profile (`linux_x86_64` or `linux_armv8`), rebuild with colcon, and re-source `install/local_setup.bash`.

> [!INFO]
> Symptom: split launch config confusion (`rgbd.yaml` vs preset catalog).  
> Fix: `config/rgbd.yaml` is runtime base; `config/rgbd_split.yaml` is preset catalog used for override tuning.

## C++ Comment Contract Standard

Use Google style consistency + Doxygen contracts + Linus-style rationale.

Core rules:

- Comments must explain `why`, assumptions, and failure behavior; avoid restating code flow.
- Write short, direct, concrete sentences. Avoid words like "obviously" and "simply".
- Keep API comments as contracts (`pre/post`, timing, QoS/transport, failure mode).

Doxygen levels:

- File level: require `@file` + `@brief` + `Contract`/`Rationale`.
- Class level: require `@brief` + invariants.
- Public methods: require `@param`, `@return`, and at least one of `@pre`, `@post`, `@note`, `@warning`.

Inline and block rules:

- Inline `//` comments are only for non-obvious pitfalls or required constraints.
- Block rationale should use short bullets with explicit trade-offs.
- `TODO/FIXME` must include owner + exit condition.

```cpp
// TODO(rvl): Replace polling with event-based wait once rclcpp exposes XYZ.
// Exit: test_e2e_lifecycle_bag no longer needs min_play_seconds_required.
```

Project vocabulary:

- `gate`: local machine-checkable spec.
- `contract`: external behavior guarantee.
- `deterministic`: behavior independent of RMW/timing variance.
- `cycle`: one smoke/full run iteration.

High-value mandatory comment points in this repo:

- Why each topic QoS is fixed (and where defaults are unsafe).
- Why lifecycle timing controls interface readiness.
- How `manifest.json` and `test_summary.json` fields decide gate pass/fail.

Function template:

```cpp
/**
 * @brief <One-line responsibility>.
 *
 * @param <name> <meaning>.
 * @return <meaning>.
 *
 * @pre <required condition>.
 * @post <guarantee>.
 * @note <threading / timing / QoS implications>.
 * @warning <common misuse / footgun>.
 */
```

Block template:

```cpp
// Rationale:
// - <why 1>
// - <why 2>
// Trade-off:
// - <what we lose by this choice>
```

## Useful Scripts

- `scripts/run_e2e.py`: run local release gates and emit manifest.
- `scripts/conan_rebuild.py`: rebuild Conan recipe and verify required flags in build outputs.
- `scripts/verify_runtime.py`: check runtime shared library resolution.
- `scripts/build_colcon.py`: convenience wrapper for colcon build.
