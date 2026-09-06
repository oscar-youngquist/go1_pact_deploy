# Position-only simulation

`include/posistion_controller.hpp` defines `PositionController`. It calls the
exported TorchScript method `forward(obs, obs_history)` and requires a
finite `[1, 12]` tensor of position actions. The exported model contains the
network architecture and its learned weights, including PACT-style context
encoding; the C++ controller handles observations and joint targets.

Set `policy_name` in `params/position_config.yaml` to your single-head export.
Relative model filenames resolve inside the outer simulation repository's
`models` directory; absolute model paths are also accepted. The default
`position_policy.pt` is a placeholder, not a supplied checkpoint. The older
Apr02 model uses a different architecture and torque-feedback observations.

The default observation has 45 elements, in this order: 3 commands, 3 projected
gravity values, 3 angular velocities, 12 joint offsets from the standing pose,
12 joint velocities, and 12 previous clipped position actions. A 20-frame
history includes the current frame as its newest entry. Set `num_single_obs`
to 57 only if the export was trained with 12 additional zero torque inputs.
This does not emulate the older `pos` controller's computed torque feedback.

The configuration copies PACT's shared hyperparameters: 100 Hz policy updates,
30/0.6 control PD gains, 60/5 standing gains, 0.25 position-action scaling,
command limits, normalization, clipping, standing pose, and joint limits.
As in PACT, EMA is inactive. Position actions become clipped joint-position
targets; desired velocities and feedforward torques are always zero. MuJoCo
applies the same PD position-control path already used by the legacy controller.

From the outer simulation repository:

```bash
cmake -S . -B build
cmake --build build --target pact_sim -j2
build/bin/pact_sim Configs/position_mujoco_sim_config.yaml
```

The command uses `ctrl_type: position`. Existing `pact` and legacy `pos` modes
remain available. Running without arguments still selects PACT. Joystick
controls and the Kalman estimator are unchanged; no CSV logging is added.

The deterministic controller check needs neither a trained model nor a joystick:

```bash
cmake --build build --target position_controller_test -j2
build/bin/position_controller_test go1_pact_deploy/params/position_config.yaml
```
