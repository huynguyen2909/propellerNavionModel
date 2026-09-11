# t6c_landing_gear

T-6C Texan II ground-reaction / landing gear force model: spring-damper
normal force, Pacejka cornering, and a globally-solved Coulomb friction
model (Projected Gauss-Seidel over all gears at once), ported from JSBSim's
`FGLGear` + `FGAccelerations::CalculateFrictionForces`.

## Files

| File | Role |
|---|---|
| [`t6c_landing_gear.hpp`](t6c_landing_gear.hpp) / [`.cpp`](t6c_landing_gear.cpp) | The library: contact geometry, normal force, friction solve |
| [`t6c_landing_gear_params.yaml`](t6c_landing_gear_params.yaml) | T-6C's real numbers, human-readable (data only -- not parsed by any code here) |
| [`main.cpp`](main.cpp) | Everything the library deliberately leaves out: your own EOM, a drop-test scenario, and a real-time-paced demo -- 3D + live charts by default (SDL2/OpenGL), or `--console`/`--stream` text output |
| [`live_server.py`](live_server.py) | Optional 4th way to view it: launches `./main_exe --stream` and serves a live browser dashboard over Server-Sent Events (stdlib only) |

## How the library is organized (read this before the code)

The library plays only the `FGLGear` half of JSBSim's ground-reaction
pipeline. It deliberately does **not** sum forces, does **not** know about
gravity/aero/propulsion, and does **not** integrate the equations of motion
-- that split mirrors real JSBSim, where `FGLGear` only supplies normal
force + friction bounds and `FGAccelerations` (the 6DOF/EOM) is the one
place that sums every subsystem's force and runs the friction solve.

| Step | Function | What it does |
|---|---|---|
| 1 | `LandingGearModel::UpdateGears(state, controls, dt)` | Per gear: contact detection, compression, spring-damper normal force, steering, wheel slip angle. Returns `normal_force_body_n`/`normal_moment_body_nm` (gravity excluded) plus a `multipliers` list (direction/lever-arm/friction-cone bounds per gear currently on the ground) |
| 2 | *(your own EOM)* | Sum `force_pre = external_force + normal_force_body_n + gravity_body` and `moment_pre` the same way |
| 3 | `SolveFriction(multipliers, force_pre, moment_pre, v, omega, mass, inertia, inertia_inv, dt, max_iter, tol)` | Free function, zero landing-gear knowledge. Converts `force_pre`/`moment_pre` to accelerations (F=ma, Part 10.2 implicit-velocity stabilization) and runs Projected Gauss-Seidel over every gear's multipliers at once (braking one wheel affects the others through this shared solve) |
| 4 | `LandingGearModel::ApplyFrictionResult(gear, friction)` | Writes `total_force_body_n`/`total_moment_body_nm` (normal + friction, still gravity-excluded), fills per-gear friction diagnostics, caches each gear's lambda as next step's warm start |

Read `t6c_landing_gear.cpp` in that same order: `UpdateGear()` (per-gear
geometry/normal-force, called by step 1), then `SolveFriction()` (step 3),
then `LandingGearModel::UpdateGears()`/`ApplyFrictionResult()` (the two
methods that tie it together). Everything above `LandingGearModel` in the
file (`BodyToNed`, `ComputeSteerAngle`, `BuildMultipliers`, ...) is a small
internal helper used by exactly one of those steps.

## How main.cpp is organized

`main.cpp` is the "6DOF" side of the split above, plus a scenario to drive
it -- none of it is part of the library, all of it is ordinary rigid-body
code any caller would write for itself. It's commented with the same step
numbers as the code:

1. `MakeT6CParams()` -- hand-transcribed from `t6c_landing_gear_params.yaml`.
2. `BodyToNed()` / `GravityBody()` -- generic Euler-angle rotation and
   gravity-into-body-axes math.
3. `StepOnce()` -- the library's steps 1-4 above in sequence, then
   integrates velocity/attitude/position one step forward (semi-implicit
   Euler).
4. `DropTestScenario` / `MakeInitialState()` / `ControlsForTime()` -- one
   named scenario, every parameter explicit: a drop from ~2.97 m AGL with
   forward + lateral ("crab") speed and 8 deg of initial roll (deliberately
   exaggerated so RIGHT_MLG, NOSE_LG, and LEFT_MLG touch down at 3 visibly
   different times -- see each one announced individually in the console
   output), a nose-steer command that switches on shortly after NOSE_LG
   touches down, and a full-brake command once all 3 gears are down.
5. `RunOnce()` -- runs `StepOnce()` for 10 s of simulated time, in a loop
   **paced to the wall clock** (`std::this_thread::sleep_until` against
   absolute targets computed from a single start time, so the loop's own
   processing time never accumulates drift) -- 1 simulated second takes 1
   real second. Used by `--console` (prints a live human-readable table plus
   `>>` event announcements -- each gear's touchdown is detected the instant
   its own `weight_on_wheels` flips, at full dt=0.001s resolution,
   independent of how often the table itself is printed) and `--stream`
   (one JSON line per frame instead -- `PrintJsonFrame()` -- for
   `live_server.py` to forward to a browser).
6. `main()` -- picks a mode from `argv`: `--console`, `--stream`, or (no
   argument) `RunVisualization()`.
7. `DrawBox()` / `DrawGroundGrid()` / `DrawAircraft()` / `RunVisualization()`
   -- the default native view: a resizable SDL2/OpenGL window with a chase
   camera, fuselage box, and 3 wheel boxes (NOSE=red/LEFT=blue/RIGHT=yellow,
   same colors used everywhere else in this file) placed at each gear's
   `location_body_m` offset by `-compression_m` along body Z -- the exact
   formula `UpdateGear()` uses internally, so the wheel visibly sinks into
   the gear well as the strut compresses. The whole aircraft's transform is
   built directly from `BodyToNed()`'s rotation matrix (via a fixed NED->GL
   basis change), so rendered attitude can't drift out of sync with the
   physics.
8. `ChartPanel` / `RunVisualization()`'s 2nd window -- a live-updating
   12-panel chart grid running in parallel with the 3D window (same
   physics step feeds both), covering every logged signal. There's no
   SDL2_ttf on hand for in-canvas text, so panels are identified by a
   legend printed to the console once at startup (`PrintChartLegend()`)
   plus consistent coloring (gear-identity colors match the 3D window;
   other panels use a fixed red/green/blue "3-component" palette).

## Build & run

Needs a C++17 compiler, Eigen3, and SDL2 + OpenGL/GLU headers:

```bash
sudo apt install libeigen3-dev libsdl2-dev libgl1-mesa-dev libglu1-mesa-dev
```

```bash
cd T6C_landing_gear
g++ -std=c++17 -Wall -Wextra -I. -I/usr/include/eigen3 -pthread \
    main.cpp t6c_landing_gear.cpp -o main_exe -lSDL2 -lGL -lGLU
```

### 3D + live charts (default)

```bash
./main_exe
```

Opens **2 windows in parallel**, both fed by the same physics step so
they're always in sync:

- **"T-6C Drop Test -- 3D"**: a chase-camera view of the fuselage + 3
  color-coded wheels (NOSE=red, LEFT=blue, RIGHT=yellow) touching down,
  compressing, steering, and braking on a grass grid. The window title
  shows live `t`/AGL/steer/brake.
- **"T-6C Drop Test -- Charts"**: a 12-panel grid (altitude, vz, attitude,
  total force, total moment, per-gear compression/normal-force/roll-friction
  /side-friction, nose steer-vs-slip, position drift, control commands)
  drawn live as the run progresses. There's no text-rendering library on
  hand, so panels aren't labeled in-canvas -- **read the legend the program
  prints to the console at startup** (which panel is which, and what each
  color means) before the windows grab your attention.

Both windows run in real time (1 simulated second = 1 real second, ~10 s
total). Press **R** in either window to reset and replay the exact same
run; **Escape** or closing either window quits both.

If `SDL_Init` fails (no display available, e.g. a headless SSH session),
it prints a message and exits -- use `--console` or `--stream` instead.

### Console (text only)

```bash
./main_exe --console
```

Prints the scenario description first (drop height, speeds, initial roll,
when steer/brake engage), then a live-updating table: `t`, wheel AGL,
vertical speed, total vertical force, NOSE_LG compression, nose steer angle,
brake command, and per-gear weight-on-wheels -- interleaved with `>>` lines
announcing each gear's touchdown and control events the instant they occur:

```
>> t=0.728  RIGHT_MLG touchdown
>> t=0.777  NOSE_LG touchdown
>> t=0.847  LEFT_MLG touchdown (+0.119s after RIGHT_MLG)
>> t=0.847  all 3 gears down
>> t=0.978  nose steer command engaged (0.50)
>> t=1.347  brakes engaged (1.00)
```

At the end it prompts `Nhan Enter de xem lai (reset), hoac go 'q' roi Enter
de thoat:` -- press **Enter** to reset and replay, or **`q`** + Enter to quit.

### Live web dashboard (optional, 4th way to view it)

```bash
python3 live_server.py
# open http://localhost:8765/ , click "Chay mo phong"
```

`live_server.py` (stdlib only, no pip install) launches `./main_exe
--stream` as a subprocess -- same binary and scenario, printing one JSON
line per frame instead of a table -- and forwards each line live to the
browser over Server-Sent Events. The page draws 11 labeled charts (it has
real HTML/CSS text, unlike the SDL windows) as the data arrives. Click
"Chay mo phong" again after it ends to reset and replay.
