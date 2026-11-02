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

## Instance and output formats

Join the instance relations on `case_id`.

| Workbook / sheet | Key | Contents |
|---|---|---|
| Benchmark_Metadata / Metadata | case_id | Scale, fleet, dimensions, costs, counts, generation parameters |
| Benchmark_Nodes / Nodes | case_id, node_id | Service times and plant delivery deadlines |
| Benchmark_Parts / Parts | case_id, item_id | Supplier, demand type, pallet/rack flag, priority, dimensions, grouping attributes |
| Benchmark_Distances / Distances_01–08 | case_id, node_i, node_j | Directed distances and travel times |
| Benchmark_Compatibility / Compatibility | case_id, item_m, item_b | Rack-pair compatibility in both directions, including zero entries |

The converter combines all eight distance sheets. Geometry is in millimetres, distance in kilometres, and service times, deadlines and departure objective `F` in minutes. Runtime is in seconds. `C` uses the benchmark's monetary units, `P` is a priority score, and `K` is the number of used vehicles.

`is_optional=true` identifies optional racks; all other items are mandatory. `is_pallet=true` identifies pallets. Rotation is horizontal. `dock_id` and `size_class` are anonymous grouping attributes. `stack_mode` values 0/1/2/3 denote no stacking/same material/same container/same footprint. Explicit `q_mb` and `q_bm` values define compatibility. Generation attributes, including `max_stack_layers` and compatibility probability, support sensitivity input generation.

Convert and run a benchmark instance:

```sh
python scripts/convert_instances.py --case case_134_medium_S11 --output run_inputs/case_134.json
python scripts/run_solver.py --instance run_inputs/case_134.json --algorithm ILS --seconds 60 --output run_outputs/case_134_ils.json
```

JSON inputs contain metadata and `suppliers`, `items`, `arcs`, and `stack_pairs` arrays. The runner handles native input conversion.

Output JSON contains `feasible`, `objective`, `runtime`, `stop_reason`, `vehicle_count`, `validation_ok`, `cost_components`, `solution`, and `case_id`. Objectives are ordered lexicographically: minimize transportation cost `C`, maximize collected optional priority `P`, then maximize the sum of latest feasible departure times `F`. If no solution is available, `objective=null`. Cost components are fixed vehicle and distance costs.

Heuristic solutions undergo independent and physical validation. `solver_returned` denotes a normal heuristic return, not proven optimality. Exact-solver runtime and status come from Gurobi; physical validation is reported separately from incumbent existence.

## Experimental results

| Worksheet | Records | Contents |
|---|---:|---|
| Algorithm_Comparison | 9,000 | 300 instances × three algorithms × ten seeds |
| Gurobi | 900 | 300 instances × three settings, with bounds and objective-stage proof fields |
| Ablation | 9,000 | Three disabled-module variants; Full uses the ILS comparison records |
| Convergence_Events | 2,760 | Incumbent improvement events |
| Convergence_Phases | 3,300 | Phase transitions |
| Efficiency_B | 50,100 | Single-construction timing repetitions |
| Efficiency_B_Instances | 5,010 | Instance medians and diagnostics |
| Efficiency_C | 300 | Skyline and total-runtime measurements |
| Efficiency_Settings | 13 | Timing and sampling rules |
| Sensitivity | 3,900 | 30 instances × 13 scenarios × ten seeds |
| Parameter_Settings | 4 | Parameter domains and selected values |

The Gurobi sheet includes the planned comparison grid, not 900 successful solves. Unavailable objectives, bounds and runtimes are left blank, not replaced by zeros.

Table 4 uses benchmark metadata; Table 5 uses Parameter_Settings; Tables 6–7 use Algorithm_Comparison and Gurobi; Table 8 additionally uses Ablation. Convergence figures use the event/phase records, and efficiency and sensitivity figures use their corresponding worksheets.

Recompute tables and plotting data:

```sh
python scripts/reproduce_tables.py --output reproduced
python scripts/reproduce_tables.py --output reproduced_with_figure_data --figures
```

`--figures` exports plotting data, not images. Convergence grids are reconstructed by carrying events forward at their recorded display times. BKS is the best complete lexicographic tuple across the three heuristics and three Gurobi settings. Comparison tolerances are 0.00001 for C/P and 0.001 for F. Table 8's profit panel averages selected best-run scores over instances with matching best transportation costs. Seeds are repeated runs within an instance, not independent instances.

## Experiment settings and sensitivity scenarios

The ten seeds are:

```text
205062346 1335958857 1940480144 1685070621 407557420
1947553710 499779373 1747588940 1919697709 1791432104
```

Convergence runs use 600 s; comparison, ablation, sensitivity and efficiency-C runs use 60 s including preprocessing. Time-limited stochastic outcomes and timings depend on hardware, compiler, solver environment and system load.

Generate sensitivity inputs:

```sh
python scripts/generate_scenarios.py --case case_065_small_S08 --output run_inputs/sensitivity_case_065
```

The scenario seed is 2026102501. Compatibility levels retain 60/70/80/90/100% of baseline compatible pairs; optional quantities and deadline slack use 60/80/100/120/140%. The three 100% levels share BASE, yielding 13 scenarios. Added racks inherit physical attributes from templates within the same supplier and receive unique IDs. New compatibility entries follow the recorded generation rules; existing entries and scenario nesting are preserved.

Saved results support recomputing the paper's analyses. Recollecting convergence traces, efficiency timings and initialization-preserving final-intensification ablations requires instrumentation not included here. Parameter configuration records provide search ranges and selected values, not tuning trial histories.

The implementation includes modified PyVRP and HGS components.
