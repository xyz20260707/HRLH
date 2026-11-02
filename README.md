# HRLH: paper replication attachment

Implementation and benchmark data for **Milk-Run Routing and Three-Dimensional Loading with Stacking Compatibility and Optional Demand: An Efficient Multiobjective Hybrid Heuristic**.

This attachment contains HRLH-ILS, HRLH-ALNS and HRLH-HGS, the complete Gurobi routing/loading/scheduling model, 300 anonymized benchmark instances, and the run-level experimental data used in the paper. Time-limited stochastic runs on other hardware need not reproduce identical solutions.

The three algorithms and the complete exact model are implemented in C++. The five Python scripts provide input conversion, executable launching, scenario generation, table reproduction and smoke testing.

No project license has been selected by the authors. This repository does not grant an open-source license to the authors' original code or data. A license decision is needed before presenting it as a licensed open-source release. Gurobi is neither bundled nor licensed by this repository.

## Contents

```text
config/hrlh.ini                         final full-algorithm settings
include/, src/                          required C++ implementation
CMakeLists.txt                          single build entry
scripts/convert_instances.py            relational workbooks to input JSON
scripts/run_solver.py                   portable single-instance runner
scripts/smoke_test.py                   short executable checks
scripts/generate_scenarios.py           the 13 nested sensitivity scenarios
scripts/reproduce_tables.py             paper tables and optional figure-data CSVs
Instances/Benchmark_*.xlsx              normalized benchmark relations
Instances/Example_Instance.json         complete runnable small instance
Experiment_Results/Experiment_Results.xlsx
```

The benchmark contains 300 anonymized instances; no separate real-world case dataset is included.

## Requirements and build

- C++20 compiler, CMake 3.24 or later, Python 3.10 or later.
- Gurobi C++ SDK and a valid user-provided license. Gurobi runs use one thread.
- `openpyxl` for reading the supplied Excel files: `python -m pip install openpyxl`.

From the repository root, with `GUROBI_HOME` set to the SDK installation, the configure-and-build command is:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGUROBI_HOME="$GUROBI_HOME" && cmake --build build --config Release --parallel
```

PowerShell equivalent, after setting `$env:GUROBI_HOME` to your installation:

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGUROBI_HOME="$env:GUROBI_HOME"; if ($LASTEXITCODE -eq 0) { cmake --build build --config Release --parallel }
```

This package was tested with Windows x64, MSVC 19.51 and CMake 4.2.3. Other platforms have not been validated. Some Windows CMake builds fail when the current directory contains non-ASCII characters; use an ASCII-only checkout/build location or invoke CMake from an ASCII-only current directory. The Gurobi runtime library must be discoverable; the Python runner prepends `GUROBI_HOME/bin` when that environment variable is set.

Fresh MSVC configurations select separate Release and Debug Gurobi C++ libraries. `GUROBI_CPP` remains an explicit single-library override; if reusing a build directory with an older cached override, clear it with `cmake -U GUROBI_CPP -S . -B build` before switching configurations.

The targets are `hrlh_ils`, `hrlh_alns`, `hrlh_hgs`, and `hrlh_exact`. Build artifacts are not part of the attachment.

## Run an example

```sh
python scripts/run_solver.py --instance Instances/Example_Instance.json --algorithm ILS --seconds 60 --seed 205062346 --output run_outputs/example_ils.json
python scripts/run_solver.py --instance Instances/Example_Instance.json --algorithm ALNS --seconds 60 --seed 205062346 --output run_outputs/example_alns.json
python scripts/run_solver.py --instance Instances/Example_Instance.json --algorithm HGS --seconds 60 --seed 205062346 --output run_outputs/example_hgs.json
python scripts/run_solver.py --instance Instances/Example_Instance.json --algorithm Gurobi --seconds 60 --output run_outputs/example_gurobi.json
python scripts/smoke_test.py --seconds 3 --exact
```

Use a new output filename for each invocation. `--build` selects another build directory. `--config` selects another INI file; explicit `--seed` and `--seconds` override only their corresponding INI settings. Other settings remain unchanged. The four tuned parameters are `routing.feasible_descent_no_improvement_limit=60000`, `packing.packing_iters=3`, `packing.mutation_swap_ratio=0.50`, and `routing.deep_packing_retry_iters=45`. The final stage is controlled by `packing.enable_optional_demand_profit_intensification` and `packing.optional_profit_intensification_budget_fraction`; packing guided route repair is controlled by `routing.enable_packing_guided_route_repair`.

The heuristic budget includes fresh Gurobi stack preprocessing. No preprocessed solution cache is supplied or used by the runner. Gurobi preprocessing and the exact model use one thread. The exact comparison settings are `G60` (60 s), `G3600` (3,600 s), and `Gseq` (3,600 s global, three objective passes allocated 1,800/1,200/600 s):

```sh
python scripts/run_solver.py --instance Instances/Example_Instance.json --algorithm Gurobi --seconds 3600 --pass-limits 1800 1200 600 --output run_outputs/example_gseq.json
```

Do not run these long experiments as part of a smoke test. The exact model uses tight time bounds and lexicographic objective priorities.

## Input relations and units

Each data worksheet has one header row. Join all relations on `case_id`. The distance relation is partitioned into eight identically structured worksheets within one workbook; the converter unions them in worksheet order. The other instance workbooks each have one worksheet.

| Workbook / sheet | Primary key | Meaning |
|---|---|---|
| Benchmark_Metadata / Metadata | case_id | Scale, supplier/depot IDs, fleet, vehicle dimensions, cost coefficients, item counts and generation parameters |
| Benchmark_Nodes / Nodes | case_id, node_id | Supplier service times and plant delivery deadlines |
| Benchmark_Parts / Parts | case_id, item_id | Supplier, mandatory/optional status, pallet/rack flag, priority, dimensions, anonymous grouping IDs and packaging attributes |
| Benchmark_Distances / Distances_01–08 | case_id, node_i, node_j | Complete directed distance and travel-time relation, partitioned by rows without dropping records |
| Benchmark_Compatibility / Compatibility | case_id, item_m, item_b | Both directions of each original rack-pair compatibility entry, including zero entries |

Geometry supplied to the solver is in **millimetres**; distance is in **kilometres**; supplier service times, deadlines and departure objective `F` are in **minutes**. Runtime fields explicitly suffixed `_s` are seconds. Monetary coefficients retain the benchmark's cost units. `P` is a priority score, not money. `K` is the number of used vehicles.

`is_optional=true` identifies optional racks; all other items are mandatory. `is_pallet=true` identifies a pallet. Rotation is horizontal only in the model. `dock_id` and `size_class` are anonymous numeric attributes preserving the original solver grouping/comparison behavior. `stack_mode` is 0/1/2/3 for no stacking/same material/same container/same footprint. The explicit `q_mb` and `q_bm` compatibility values are authoritative; legacy descriptive packaging/rotation fields do not override the mathematical model. `max_stack_layers` and the recorded compatibility probability are retained for reproducing sensitivity input generation.

Reconstruct one complete solver input:

```sh
python scripts/convert_instances.py --case case_134_medium_S11 --output run_inputs/case_134.json
python scripts/run_solver.py --instance run_inputs/case_134.json --algorithm ILS --seconds 60 --output run_outputs/case_134_ils.json
```

The converter reconstructs the solver input fields from the anonymized relational workbooks without numeric loss.

The JSON input combines a metadata record with `suppliers`, `items`, `arcs`, and `stack_pairs` arrays. `Example_Instance.json` is a complete, directly runnable example, not a schema stub. Temporary native `.dat` input is generated by `run_solver.py`, so users need not supply another undocumented input format.

## Outputs

Every run returns parseable JSON with `feasible`, `objective`, `runtime`, and `stop_reason`. A feasible `objective` contains `C` (minimize transportation cost), `P` (maximize collected optional priority), and `F` (maximize the sum of latest feasible departures), in that lexicographic order. An unavailable solution has `objective=null`, never zero-filled objectives.

Additional fields include `vehicle_count`, `validation_ok`, `cost_components`, `solution`, and `case_id`. Transportation cost is decomposed into fixed vehicle and distance costs. Heuristic solutions are checked by both the independent checker and the physical validator. A heuristic `solver_returned` stop reason reports a normal return, not a proof of optimality. For the exact solver, `runtime` is Gurobi's reported optimization runtime and `stop_reason` is its status; a time-limited incumbent is not labeled optimal. Exact physical validation is returned separately from incumbent existence.

For a nondefault preprocessing path that omits a material witness, the export layer recovers the saved stack assignments without rerouting or repacking. It reports an error if the stored assignments cannot supply a nonempty stack for each returned footprint; it does not substitute another packing. Exact-model item records use `is_optional` and `optional_score`.

## Formal result workbook and paper mapping

| Worksheet | Rows | Purpose |
|---|---:|---|
| Algorithm_Comparison | 9,000 | 300 cases × ILS/ALNS/HGS × ten seeds; objectives, cost components and runtime |
| Gurobi | 900 | 300 cases × three comparison settings; incumbent status, accepted objectives, bounds/gaps and objective-pass proof fields |
| Ablation | 9,000 | Three disabled-module variants × 300 cases × ten seeds; Full is reused from ILS, not duplicated |
| Convergence_Events | 2,760 | Saved incumbent improvement events |
| Convergence_Phases | 3,300 | Phase transition records |
| Efficiency_B | 50,100 | All single-construction timing repetitions |
| Efficiency_B_Instances | 5,010 | Per-input diagnostics and medians |
| Efficiency_C | 300 | Complete HRLH-ILS runs, inclusive Skyline time and total-runtime ratio |
| Efficiency_Settings | 14 | Timing boundaries, size distribution, sampling and averaging rules |
| Sensitivity | 3,900 | Both stages combined, 30 cases × 13 scenarios × ten seeds |
| Parameter_Settings | 4 | Search ranges and selected settings reported in Table 5 |

The `Gurobi` worksheet preserves the complete planned comparison grid. It must not be interpreted as 900 successful or even 900 actually executed solves. Some original records were not run or found no feasible solution. Their objective/bound/runtime cells remain unavailable as recorded. The accepted tolerance-corrected objective values in the supplied final workbook are authoritative; original validation flags are retained separately for traceability. `原记录验证通过` denotes validation accepted in the original record; `用户容差复核通过` denotes acceptance after the authors' tolerance review. No older historical BKS is imported.

Paper mapping: Table 4 uses `Benchmark_Metadata`; Table 5 uses `Parameter_Settings`; Tables 6–7 and exact optimality counts use `Algorithm_Comparison` + `Gurobi`; Table 8 uses those two sheets plus `Ablation`; the efficiency time-share table uses `Efficiency_C`. Convergence figures use the phase/event records; the construction-time figure uses `Efficiency_B`; sensitivity panels use `Sensitivity`.

`convergence_grid()` in the reproduction script reconstructs all 300 × 601 observations by carrying forward events at their recorded display times, retaining unavailable initial objectives as blanks.

```sh
python scripts/reproduce_tables.py --output reproduced
python scripts/reproduce_tables.py --output reproduced_with_figure_data --figures
```

Generated CSVs are derived outputs and are deliberately not pre-stored in the repository. `--figures` exports plotting data, not figure images. The frozen BKS is selected from the six published comparison methods only. Reporting comparisons use the paper's existing numerical tolerances: 0.00001 for `C`/`P` and 0.001 for `F`. The script uses one complete lexicographic result, not independently mixed objective minima/maxima. Table 8's profit panel averages the selected best-run profit over cases whose two compared best-run costs match; it is not the average of all ten seed profits. Ten seeds within a case are repeated runs, not ten independent benchmark instances.

The formal seed list is:

```text
205062346 1335958857 1940480144 1685070621 407557420
1947553710 499779373 1747588940 1919697709 1791432104
```

The convergence budget is 600 s; comparison, ablation, sensitivity and efficiency-C budgets are 60 s including preprocessing. New runtime measurements and time-limited search results depend on CPU load, compiler, solver version/license and wall-clock stopping.

## Sensitivity scenario inputs

```sh
python scripts/generate_scenarios.py --case case_065_small_S08 --output run_inputs/sensitivity_case_065
```

This reproduces the original input-generation procedure with scenario seed 2026102501. The common `BASE` scenario is reused. Compatibility levels retain 60/70/80/90/100% of original compatible pairs; optional quantity and deadline-slack levels are 60/80/100/120/140%. Added racks receive unique IDs, clone their template's physical attributes within the same supplier, and obtain new compatibility entries from the recorded generator law. Existing pair values and nesting are preserved. No original random draw is guessed from the observed compatibility matrix.

The package supports building the algorithms, reconstructing all base and published sensitivity inputs, and recomputing the paper's analyses from saved observations. Recollecting convergence event traces, timing probes or the initialization-preserving final-intensification ablation requires research instrumentation that is not included; the observations needed to verify their reported analyses are supplied. Parameter tuning data comprise the selected settings and search ranges, not the trial histories.

## Citation and upstream components

The implementation includes modified PyVRP and HGS components. The Gurobi SDK is an external dependency and is not bundled.

