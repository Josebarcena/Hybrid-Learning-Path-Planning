# Hybrid Pathfinding — TFM starter (C++20)

First milestone: **A*** (optimal baseline) and **Weighted A*** (bounded-suboptimal baseline). **ARA* and RL are not yet implemented.**

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Run

```bash
./build/pathfinder maps/example.txt 0 0 9 5
./build/pathfinder maps/example.txt 0 0 9 5 1.5
```

On Windows with Visual Studio, executable may be `build/Release/pathfinder.exe`.

Maps: `.` traversable, `#` blocked. Coordinates are 0-based `(x,y)` with `(0,0)` top-left. Movement: 4-connected, unit cost. Manhattan heuristic. No diagonal moves.

Metrics: wall-clock runtime in milliseconds, expanded nodes, generated queue entries, peak queue entries, and **estimated peak bytes for algorithm buffers only**. This is not RSS or total process memory; benchmark full process memory separately. For fair timing, run many repetitions in Release mode, with identical maps, warmup and hardware.

Optimality: A* with weight=1 is optimal for this model. Weighted A* with weight>=1 may return suboptimal paths; for admissible consistent h and this implementation, its first solution has cost <= weight * optimum. It is **not** an anytime implementation.

Next: randomized map generator with fixed seeds; Dijkstra reference tests; proper benchmark harness; ARA* with OPEN/CLOSED/INCONS; only then RL integration.

## Reproducible benchmark (milestone 2)

The new `pathfinding_benchmark` executable generates deterministic random maps
with sizes 50, 100, 250, 500 and obstacle probabilities 10%, 20%, 30%.
Start and goal are always traversable. Unreachable maps are **retained** in the
CSV (not silently discarded). Each map is validated against an independent
breadth-first search (BFS) oracle, which gives the optimal cost for 4-connected
unit-cost grids. A* optimality and Weighted A*'s `weight * optimal` bound are
checked automatically. Every reported path is also validated.

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\pathfinding_benchmark.exe results\benchmark.csv 10 5 500
```

Arguments: `output.csv maps_per_configuration timed_repetitions max_side`.
For a quick smoke test use `results\smoke.csv 1 2 50`.

CSV rows contain the map's seed, reachability, optimal BFS cost, weight,
repetition, solution cost, relative error, runtime, expansions, generated queue
entries, maximum queue size, and **estimated** peak buffer bytes. Empty cost/error
fields mean no route exists. Memory is not measured process RSS. Runtime excludes
map generation and BFS verification, but includes A* initialization and path
reconstruction. One untimed warm-up is executed per map/weight; results are
individual runs, not aggregates. For analysis, group by map and weight, summarize
median/p95 runtime, compare paired maps, and report reachable vs unreachable
separately. Run on a quiet machine in Release mode; do not interpret microsecond
single-run differences as stable speedups.

**Limitations:** generated maps are Bernoulli obstacle fields, not representative
of all real maps; benchmark weights are not learned; this is not yet ARA* or RL.

## Milestone 3 — ARA* (Anytime Repairing A*)

ARA* is implemented using persistent `g`, `OPEN`, `CLOSED`, and `INCONS` data.
It decreases the inflation factor and **reuses search information** rather than
restarting A* for each weight. Run on Windows:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\pathfinding_anytime.exe maps\example.txt 0 0 9 5 3 0.5
```

Arguments after coordinates: `initial_weight decrement time_limit_ms memory_limit_bytes`.
Zero for either budget means no limit. Each completed phase records weight,
incumbent route cost, cumulative expansions, elapsed time and estimated buffers.
A phase with no incumbent does not yield a snapshot. `optimality_proven=1`
means the incumbent is optimal (or infeasibility has been established) under
this static, unit-cost grid model. The final reported incumbent may have been
found during an interrupted phase; only completed phases are recorded.

**Limitations:** time limits are checked between node expansions, not hard
real-time deadlines; memory limits are checked using an approximate buffer
accounting and do not cap the allocator or process RSS. Memory is checked at
expansion boundaries, and a phase can stop partway through. This version
implements static-map ARA*, not dynamic replanning or reinforcement learning.
The existing CSV benchmark still compares A* and Weighted A*; add ARA* runs
on the same seeds before drawing performance conclusions about ARA*.

## Benchmark ARA* (v0.4)

Build and test (Visual Studio / Windows):

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\pathfinding_benchmark_anytime.exe results\anytime_smoke.csv 1 2 50
.\build\Release\pathfinding_benchmark_anytime.exe results\anytime.csv 10 5 500
```

`pathfinding_benchmark_anytime` uses the same deterministic map seeds as the earlier benchmark. The CSV contains `AStar` baseline rows, `ARAStar` phase snapshots (phase >= 1), `ARAStar` final rows (phase=0), and `ARAStarBudget` final rows (phase=0). For budget rows `weight_or_budget_ms` is the time limit (0.1, 1, 5, 10 milliseconds); for other rows it is the heuristic weight. Time measurements are machine-dependent. A time limit is checked during expansion, not enforced by an external watchdog. An interrupted run may return `found=0` even when the oracle knows a path exists. `estimated_peak_bytes` is not process RSS and does not account for all allocations.

The ARA* snapshot path cost is now derived from the actual reconstructed path length, avoiding a mismatch when predecessor g-values improve during reuse of search state. This change should be reviewed alongside the ARA* invariants before treating the implementation as a formal verification of the published algorithm.

## v0.5: external phase controller (experimental)

`pf::AnytimeSession` is a persistent ARA* state machine. Call `advance(weight)`
with non-increasing weights (>= 1.0), inspect `observe()` and `best_path()`.
The session reuses g-values, OPEN, CLOSED and INCONS; it does not restart search.
`advance` completes a full phase and is not interruptible within that phase.
Time and memory are observations, not hard limits. No RL agent is trained yet.

Build and run:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\pathfinding_session_demo.exe maps\example.txt
```

The new randomized regression test checks 60 maps against optimal A* and
validates path connectivity and reported costs.

## v0.6: session instrumentation and sparse rebuild

The session now tracks algorithm-only cumulative time (excluding caller I/O between `advance` calls). Per-phase timings separate OPEN rebuild from search; `rebuild_candidates` and `phase_expanded` expose actual work. OPEN rebuild iterates tracked active and INCONS nodes rather than scanning all grid cells. These are experimental timings, not process RSS or a guarantee of optimal runtime.

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\pathfinding_session_demo.exe maps\example.txt
```

## Experimental tabular RL controller (v0.7)

`pathfinding_rl_benchmark` trains a lightweight tabular TD/Q-learning controller
for `AnytimeSession`. Actions are stop, reduce weight by 0.25, or reduce by 1.0.
The state uses only online observable features (weight, solution cost proxy,
recent improvement, and expansion activity). Training uses A* as an oracle for
reward only; evaluation uses disjoint random seeds and never exposes optimal
cost to the policy. The Q-table is small and fixed-size. Baselines: stop after
first solution, jump directly to weight 1, and fixed decrements of 0.5.

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\\build\\Release\\pathfinding_rl_benchmark.exe results\\rl_smoke.csv 2 2
.\\build\\Release\\pathfinding_rl_benchmark.exe results\\rl.csv 40 20
```

CSV columns include held-out map identifiers, policy, optimal cost (evaluation
oracle for reporting only), route quality, time, estimated tracked memory,
expansions, decisions, and proof status. The estimated memory excludes the
Q-table, environment storage, and process RSS. The timing excludes agent
training and map construction, and includes the session's phase work but not
agent decision overhead; do not interpret it as end-to-end RL latency. This is
an exploratory prototype, not evidence that RL outperforms fixed schedules.
The current fixed policy is a simple baseline; further work should add a
no-learning adaptive baseline, train/validation/test splits, and end-to-end
memory/time instrumentation.

## v0.8: RL multiobjetivo y costes de decisión

`pathfinding_rl_benchmark` mantiene la interfaz de la v0.7:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\pathfinding_rl_benchmark.exe results\rl_v08_smoke.csv 2 2
.\build\Release\pathfinding_rl_benchmark.exe results\rl_v08.csv 40 20
```

Cambios: estado discretizado por peso, elongación de ruta respecto a Manhattan, mejora observada, expansiones recientes y tiempo; recompensa terminal con error relativo, tiempo normalizado y memoria estimada; penalización por acción; quinto baseline `quality_proxy` (umbral heurístico del 5 % sobre Manhattan, **no es una cota certificada de error**). Se exportan `planner_ms`, `decision_ms`, `wall_ms`, `planner_peak_bytes`, `estimated_total_bytes`, `stop_actions`, `small_actions` y `large_actions`.

**Limitaciones importantes:** `estimated_total_bytes` añade los bytes estáticos de la tabla Q solo al agente, pero no es RSS ni pico real de asignación del proceso; `wall_ms` incluye construcción de sesión, instrumentación y decisiones, no el coste de entrenamiento ni de calcular el óptimo de referencia. Las observaciones no usan el óptimo; el entrenamiento sí lo utiliza como señal de recompensa. Los tiempos en microsegundos deben contrastarse con repeticiones y orden de políticas aleatorizado. El estado usa distancia Manhattan como proxy, no como garantía de calidad. Los mapas de entrenamiento y evaluación tienen semillas distintas, pero comparten familias de tamaños/densidades.


## v0.9 — lower bound, marginal feedback, action trace

`PhaseObservation` exposes an admissible lower bound based on the minimum `g+h`
over OPEN and INCONS, capped by incumbent cost, and a certified gap proxy.
The bound is computed from the current search frontier (not the A* oracle).
The RL state adds a discretized certified gap; training adds marginal reward
for reducing the gap and penalizes per-phase search time. The final training
reward still uses optimal cost from A* (training-only).

The RL benchmark now also writes `<csv_stem>_trace.csv`, recording every
action and the before/after gap, route cost, expansions and phase time.
This allows auditing whether STOP/refinement decisions make sense.

The admissible bound is specific to the current static four-neighbor unit-cost
model; the bound's validity should be tested on adversarial maps before
claiming formal certificates. Timing and estimated memory are not full
process-level resource accounting. Compare paired maps and multiple seeds.

## v1.0: evaluación reproducible

La CLI del benchmark RL acepta ahora `replicate` e `include_500` (0/1), además de
la ruta CSV y el número de mapas de entrenamiento/evaluación por configuración.
Cada réplica utiliza semillas diferentes tanto para entrenar como para evaluar;
las cinco políticas comparten exactamente los mismos mapas dentro de cada réplica.
Los mapas 500x500 se reservan para evaluación **fuera de distribución** (no se
utilizan durante el entrenamiento). La réplica se añade a los CSV.

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\pathfinding_rl_benchmark.exe results\rl_v10_run0.csv 40 20 0 1
.\build\Release\pathfinding_rl_benchmark.exe results\rl_v10_run1.csv 40 20 1 1
.\build\Release\pathfinding_rl_benchmark.exe results\rl_v10_run2.csv 40 20 2 1
python analysis\paired_analysis.py results\rl_v10_run0.csv results\rl_v10_run1.csv results\rl_v10_run2.csv
```

El análisis calcula diferencias **emparejadas** por mapa, intervalos bootstrap
sobre la mediana de la diferencia, fracción de mapas donde RL gana, resumen por
tamaño y recuento de políticas no dominadas en tres objetivos (tiempo, error,
memoria estimada). El intervalo bootstrap es descriptivo; no elimina el ruido
de ejecución ni reemplaza repeticiones con control de temperatura y carga.

Limitaciones: `estimated_total_bytes` no es una medición de RSS/heap real; los
mapas de alta densidad pueden ser inalcanzables; la Q-table se contabiliza
completa en RL; `wall_ms` incluye costes de sesión pero no la generación de
mapas ni el A* oráculo. La certificación de cotas requiere justificación formal
adicional. La validación fuera de distribución se limita a tamaños 500x500.

## v1.1 — Ablaciones reproducibles

El ejecutable admite un séptimo argumento `ablation`: `0` = modelo completo,
`1` = elimina la cota `certified_gap` del estado (la sustituye por una constante),
`2` = elimina la recompensa marginal (mantiene la recompensa terminal).
Los tres modos **entrenan un agente nuevo**, usando exactamente las mismas semillas
por réplica. No hay consulta al coste óptimo en la selección de acciones; se utiliza
solamente en la recompensa de entrenamiento y en la evaluación.

Ejemplo con una réplica completa (Windows, PowerShell):

```powershell
.\build\Release\pathfinding_rl_benchmark.exe results\v11_full_0.csv 40 20 0 1 0
.\build\Release\pathfinding_rl_benchmark.exe results\v11_nogap_0.csv 40 20 0 1 1
.\build\Release\pathfinding_rl_benchmark.exe results\v11_nomarginal_0.csv 40 20 0 1 2
python analysis\ablation_analysis.py results\v11_full_0.csv results\v11_nogap_0.csv results\v11_nomarginal_0.csv
```

Repetir con réplicas `1` y `2` y analizar los archivos correspondientes.
La instrumentación `estimated_total_bytes` es una **estimación de estructuras**,
no RSS ni pico de memoria real; incluye el tamaño estático de la tabla Q pero no
todos los costes del asignador. `wall_ms` incluye la inicialización de la sesión,
pero no la generación del mapa, el cálculo de A* de referencia ni el entrenamiento.
Los tiempos de submilisegundo deben medirse con repeticiones y control de ruido
para sustentar conclusiones sólidas. Los intervalos bootstrap sobre mapas no
sustituyen la variabilidad entre agentes entrenados con semillas distintas.

## v1.2: algorithm verification and memory probe

The new `pathfinding_verification_tests` test exercises 1,050 deterministic random maps
(2, 3, 5, 10, 25, 50, 100 square cells; 5 densities; 30 seeds), and checks
5,250 phase observations against optimal A*: path adjacency, obstacle avoidance,
path length versus reported cost, nonincreasing incumbent, admissible lower bound,
certified-gap upper bound, and claimed optimality. The test uncovered a mutable
parent-chain bug in earlier versions. v1.2 retains immutable predecessor snapshots
for each improved g-value, so returned paths remain consistent with their reported
cost. This increases memory usage and changes the runtime profile: **v1.0/v1.1
benchmark timings and memory estimates must not be reported as v1.2 results.**

A separate `pathfinding_memory_probe` executable reports tracked algorithm-buffer
peak and operating-system process peak resident memory/working set. These are
not interchangeable: the latter includes the runtime, input grid, allocator,
and other process allocations, and is a *process lifetime* peak, not per-phase
memory. Run the probe in a fresh process per configuration and repeat it:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\pathfinding_memory_probe.exe 500 20 20261110
.\build\Release\pathfinding_memory_probe.exe 500 30 20261111
```

On Unix-like systems use `./build/pathfinding_memory_probe`. Do not interpret
`estimated_peak_bytes` as exact heap usage: it approximates tracked container
capacities, and the OS peak is a broader process-level measurement. The RL agent's
Q-table is not allocated in this standalone probe; for full-system memory analysis
an RL-inclusive process-isolated probe is still required. Formal proof of all
ARA* invariants remains separate from these randomized regression checks.
