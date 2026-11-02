# HRLH

Code and data for **Milk-Run Routing and Three-Dimensional Loading with Stacking Compatibility and Optional Demand: An Efficient Multiobjective Hybrid Heuristic**.

The repository contains HRLH-ILS, HRLH-ALNS, HRLH-HGS, the complete Gurobi routing/loading/scheduling model, 300 anonymized benchmark instances, and the paper's run-level experimental results. Solvers are implemented in C++; Python scripts provide data conversion and experiment utilities.

## Contents

```text
CMakeLists.txt
config/hrlh.ini
include/
src/
scripts/convert_instances.py
scripts/run_solver.py
scripts/smoke_test.py
scripts/generate_scenarios.py
scripts/reproduce_tables.py
Instances/Benchmark_*.xlsx
Instances/Example_Instance.json
Experiment_Results/Experiment_Results.xlsx
```

## Requirements and build

- C++20 compiler and CMake 3.24 or later.
- Gurobi C++ SDK with a valid user-provided license; the SDK is not bundled.
- Python 3.10 or later and `openpyxl`:

```sh
python -m pip install openpyxl
```

Set `GUROBI_HOME` to the SDK installation. Build from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGUROBI_HOME="$GUROBI_HOME"
cmake --build build --config Release --parallel
```

In PowerShell, replace `"$GUROBI_HOME"` with `"$env:GUROBI_HOME"`.

The build was tested on Windows x64 with MSVC 19.51 and CMake 4.2.3; other platforms have not been validated. Use an ASCII-only checkout/build path if CMake encounters path errors. The runner adds `GUROBI_HOME/bin` to the runtime search path.

Build targets are `hrlh_ils`, `hrlh_alns`, `hrlh_hgs`, and `hrlh_exact`.

## Run an instance

```sh
python scripts/run_solver.py --instance Instances/Example_Instance.json --algorithm ILS --seconds 60 --seed 205062346 --output run_outputs/example_ils.json
python scripts/smoke_test.py --seconds 3 --exact
```

`--algorithm` accepts `ILS`, `ALNS`, `HGS`, or `Gurobi`. Use a new output filename for each run. `--build` selects a build directory and `--config` selects an INI file. Explicit `--seed` and `--seconds` override the corresponding INI values; other settings remain unchanged.

The supplied `config/hrlh.ini` contains the paper's algorithm settings. Heuristic budgets include fresh Gurobi stack preprocessing. Preprocessing and exact solves use one thread.

Exact comparison settings are G60 (60 s), G3600 (3,600 s), and Gseq (3,600 s total, with objective-stage allocations of 1,800/1,200/600 s). Run Gseq with:

```sh
python scripts/run_solver.py --instance Instances/Example_Instance.json --algorithm Gurobi --seconds 3600 --pass-limits 1800 1200 600 --output run_outputs/example_gseq.json
```

Saved results support recomputing the paper's analyses. Recollecting convergence traces, efficiency timings and initialization-preserving final-intensification ablations requires instrumentation not included here. Parameter configuration records provide search ranges and selected values, not tuning trial histories.

The implementation includes modified PyVRP and HGS components.
