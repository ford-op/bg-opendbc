# Hosting the BluePilot lateral

A host is an opendbc fork (sunnypilot/opendbc today) with this library at `opendbc/bluepilot_lateral/` and the hook patch applied, plus the openpilot on top of it that fills the library's inputs each frame (bp-sunnypilot today; BP7 later). This page is what a host provides and what it gets back.

## Mounting the library

A host mounts this repo as a git submodule at `opendbc/bluepilot_lateral/` inside its opendbc, so the Python imports (`opendbc.bluepilot_lateral`) and the safety `#include "opendbc/bluepilot_lateral/safety/ford.h"` resolve the same in every host. A plain folder at the same path (a subtree) works too. Updating is a bump of the submodule's commit. The hook patch below carries every file bg-opendbc adds or changes outside the library, so once bg-opendbc mounts the library it also carries the mount's `.gitmodules` entry.

## The hook patch

`tools/make_hook_patch.sh` prints it: every file outside the library that bg-opendbc changes from `master`, minus bg-opendbc's own `.github/` (its CI). It fails unless the edited files are exactly the non-`.github` entries of `.github/bluepilot-hooks.txt`.

| file | what the hook does |
|---|---|
| `opendbc/car/ford/carcontroller.py` | Builds one `BluePilotLateral(CP, CP_SP, packer, CAN)`, the sunnypilot adapter (`hosts/sunnypilot/lateral.py`, below). Each frame `bp.update(CC_SP, frame)` says whether BluePilot lateral is active; if it is, `bp.step_lateral` sends the steering message at `STEER_STEP` (and returns the curvature to pass back next time) and `bp.step_lka` sends Lane_Assist_Data1 at `LKA_STEP`. Otherwise the stock path runs. |
| `opendbc/car/structs.py` | Imports `LateralInputs` from `host_types.py` and adds `CarControlSP.lateralInputs`. |
| `opendbc/sunnypilot/car/interfaces.py` | Calls `init_ford_safety_param_sp(CP, CP_SP, params_dict)` (`hosts/sunnypilot/safety_param.py`) at car init: the startup safety bits below. |
| `opendbc/safety/modes/ford.h` | Includes `safety/ford.h` and makes one call per hook site: counter and quality checks for the pinion message, the yaw measurement skipped while the pinion source is on, `ford_bp_rx_hook` (pinion measurement), `ford_bp_lka_tx` (angle-mode state from Lane_Assist_Data1), `ford_bp_lmc_tx` in place of the stock steering check when `BP_LATERAL` is set (the stock check runs unchanged when it isn't), and the pinion RX check list from `ford_bp_init`. |
| `opendbc/safety/tests/test_bluepilot_lateral.py` | New: hands the library's `safety/tests/` to upstream's test discovery, so upstream's `test.sh` and `mutation.py` run the BluePilot safety tests. |

## Every frame: inputs

`CarControlSP.lateralInputs` (`host_types.LateralInputs`):

| fields | from |
|---|---|
| `pathYawRate`, `pathX`, `pathY` | `modelV2.orientationRate.z`, `modelV2.position.x`, `.y` (33 points) |
| `laneLineLeftX/Y`, `laneLineRightX/Y` | `modelV2.laneLines[1]`, `[2]` |
| `laneLineProbs`, `laneLineStds` | `modelV2.laneLineProbs`, `.laneLineStds` |
| `laneChangeState`, `laneChangeDirection` | `modelV2.meta` |
| `steerRatio`, `stiffnessFactor`, `roll`, `angleOffsetDeg` | the learned vehicle parameters (`vehicleParameters`, paramsd) |
| `lateralDelay` | `lateralDelay.lateralDelay` |
| `valid` | true only when both the model and the vehicle parameters have been seen and every field above is filled |

Each path array has `host_types.MODEL_PATH_POINTS` (33) entries at the times in `host_types.MODEL_T_IDXS` (openpilot's `ModelConstants.T_IDXS`): the core interpolates over that grid. On a frame that isn't valid, or whose arrays aren't the full shape or contain a non-finite value, the library keeps the previous inputs.

## Every frame: settings

Every key in `opendbc.bluepilot_lateral.core.values_ext.BP_LATERAL_PARAMS` (the per-frame settings in `params/manifest.py`; hosts import the list from there), every frame, as (key, value) with the value as bytes (sunnypilot: `CarControlSP.params` entries): bools `"1"`/`"0"`, numbers as text. A missing, empty or unparsable value gives the manifest default. A host publishes its own registry default for a key nobody has set, so a host can carry its own defaults (BP7 uses 3.0 for `LC_PID_gain_UI_curv`). `python -m opendbc.bluepilot_lateral.params.manifest` prints the keys as `params_keys.h` lines.

## At startup: the safety bits

`values_ext.ford_safety_param(CP, params_dict)` reads the startup settings (`disable_BP_lat_UI`, `FordPrefSteerAngleCurvature`) and returns the bits. The host sends them to the panda in its second safety param (sunnypilot: the adapter ORs them into `CP_SP.safetyParam`):

| bit | |
|---|---|
| 5 `BP_LATERAL` | the panda runs the BluePilot steering checks; the car controller sends the 4-signal messages only when it's set |
| 0 `STEER_ANGLE_CURVATURE`, 1-4 geometry index | the pinion angle replaces the yaw rate as the curvature measurement, on platforms with a row in the pinion table |

The panda keeps what it was given at init: changing `disable_BP_lat_UI` while driving takes full effect at the next init.

## Reading status

`BluePilotLateral.status()` returns `host_types.LateralStatus`, this frame's values for a BluePilot UI, under the names BP7's `controllerStateBP` publisher reads off its car controller:

| field | |
|---|---|
| `lateralUncertainty` | from whichever strategy ran |
| `angleRateLimited` | angle mode: the path angle soft rate limit bit this frame |
| `curvatureRateLimited` | angle mode: the equivalent curvature would have been rate-limited |
| `curvatureDeviationLimited` | either mode: the current-curvature error clip constrained the command |
| `humanTurnLateralPaused` | angle mode: lateral sent inactive during a manual turn |
| `stallBlipActive` | angle mode: the brief mode-0 pulse after a post-override stall |
| `disable_BP_lat_UI` | BluePilot lateral off: the setting, or the panda bit not set at init |
| `primary_lateral_control` | 0 = curvature, 1 = angle |

The four angle-mode values read false outside angle mode. bp-sunnypilot doesn't read status; it has no BluePilot UI.

## What the library uses from the host's opendbc

Every host is an opendbc fork, so the library uses these directly:

- `opendbc.car`: `structs`, `DT_CTRL`, `ACCELERATION_DUE_TO_GRAVITY`, `scale_rot_inertia_per_mass`, `scale_tire_stiffness_per_mass`, `common.pid.PIDController`, `vehicle_model.VehicleModel`, `lateral` (`AngleSteeringLimits`, `apply_std_steer_angle_limits`, `ISO_LATERAL_ACCEL`), `ford.fordcan.CanBus`, `ford.values` (`CarControllerParams`, `FordFlags`).
- The safety headers use `modes/ford.h`'s Ford constants and upstream's safety helpers. `safety/lateral.h` and `can/fordcan_ext.py` copy four upstream functions, pinned by `tests/test_bluepilot_upstream_copies.py`, so a host where one changed fails that test.
- The safety tests use upstream's test harness: `opendbc.safety.tests.test_ford`, `common` and `libsafety.libsafety_py`.

## The core's API, and the sunnypilot adapter

The core takes only the library's types:

- `core.carcontroller_ext.BluePilotLateral(CP, safety_param, packer, CAN)`: `safety_param` is the startup bits (`ford_safety_param`).
- `update(inputs, params, frame)`: `inputs` is a `host_types.LateralInputs`, `params` a `params.param_store.ParamStore` built from the frame's (key, value) settings. Then `step_lateral`, `step_lka` and `status()` as above.

Nothing under `core/`, `can/`, `params/` or `safety/` imports a host (`tests/test_core_imports.py`). `hosts/sunnypilot/` is the one adapter:

- `lateral.py`: a `BluePilotLateral` subclass with sunnypilot's types, `(CP, CP_SP, packer, CAN)` and `update(CC_SP, frame)`. It passes `CP_SP.safetyParam`, `CC_SP.lateralInputs` and the `CC_SP.params` entries as (key, value) pairs.
- `safety_param.py`: `init_ford_safety_param_sp(CP, CP_SP, params_dict)` ORs `ford_safety_param` into `CP_SP.safetyParam`. Its own module, so car init imports only `values_ext`.

Another host writes its own adapter against the same API.

## Running the tests

- Python tests: `python -m unittest discover -s opendbc/bluepilot_lateral/tests -t .`
- Safety tests: upstream's `opendbc/safety/tests/test.sh` runs them through the loader. Coverage of the BluePilot headers and `modes/ford.h`: `safety/tests/bluepilot_coverage.sh`; mutation: `safety/tests/bluepilot_mutation.py`.
- On a clean sunnypilot/opendbc checkout: `tools/test_on_host.sh <checkout>`, from a bg-opendbc checkout. It copies the library as committed and applies the hook patch to a temporary clone, then runs both.
