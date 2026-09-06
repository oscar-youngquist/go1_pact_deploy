# Position-only controller on Jetson Orin

Initial source port from simulation commit `2fb76e4` onto hardware `main` at
`3c5311c`. This port has not been configured, compiled, or executed on the host
machine. Compilation and physical-robot validation are pending on the Orin.

## Configuration and launch

Place the exported position-only TorchScript model at:

```text
models/position/May18_21-30-18_pos_100hz_spec_ite3000.pt
```

Alternatively, change `policy_name` in `params/position_config.yaml`. Relative
model paths resolve against this repository's `models/` directory; absolute
paths are accepted. Model weights are not included in this port.

On the Orin, use the existing deployment build procedure. For an existing
`build` directory, the commands from the repository root are:

```bash
cmake -S . -B build
cmake --build build -j2
cd build
./pact_deploy ../params/position_config.yaml
```

The YAML selects `ctrl_type: position`. Running `./pact_deploy` without an
argument from `build/` retains the existing PACT configuration. Unknown
controller types are rejected. No MuJoCo dependencies or simulation targets
are added; the existing Orin LibTorch/CUDA and arm64 SDK configuration is kept.

## Controller contract

- Calls `forward(obs, obs_history)` during startup validation and control.
- Requires a finite `[1, 12]` position-action output. PACT's 24-output tensor
  is rejected; `act_inference` is not used by the position controller.
- Defaults to `[1, 45]` observations and `[1, 900]` history (20 frames), with
  the same field order as the simulation position controller. Only exports
  trained with trailing zero torque inputs should use the supported 57-field
  mode; it is not the legacy computed-torque observation format.
- Matches hardware PACT's Unitree remote signs: `+ly, -lx, -rx` for forward,
  lateral, and yaw commands. Joint order is FR, FL, RR, RL, three joints per leg.
- Keeps the simulation position model's shared PACT hyperparameters: 100 Hz
  inference, 0.25 position-action scale, 30/0.6 control gains, 60/5 standing
  gains, normalization, clipping, command limits, and standing pose. EMA stays
  inactive, as in the current PACT and simulation position controllers.
- Sends clipped joint-position targets and zero desired velocity/feedforward
  torque through the existing Unitree motor-command path. PD actuation remains
  on the robot. Reset clears history and previous actions.

The hardware state machine, UDP threads, estimator, existing logging, and
`PositionLimit`/`PowerProtect` calls are retained. Estimator configuration uses
main's hardware sensor values, including the 100-count airbag contact threshold.
Model loading and the initial forward validation happen before UDP loops start.

Use the existing physical-robot test procedure on the Orin. Confirm the exported
input shapes, controller gains, joint ordering, and joystick directions during
that validation. The existing controls are START to initialize, L1+R2 to stand,
L1+A to enter policy control, and L1+B to enter damping.
