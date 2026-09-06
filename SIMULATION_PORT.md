# MuJoCo port of main

Reviewed `origin/main` through `3c5311c` against `mujoco_sim` at `f8e23b6`.

Retained changes:

- Main's PACT model selection, lateral command limit (0.5), and EMA settings.
  EMA settings remain inactive because the controller uses raw clipped actions.
- Configurable `foot_airbag_contact_lower` in the existing estimator YAML,
  retaining the MuJoCo threshold of 15 N instead of hardware's 100 airbag counts.
- An optional `pact_policy_smoke` build target for checking policy inference,
  simulation command signs, finite outputs, and joint limits. Its build was
  interrupted at the user's request; the test has not been run.

Policy history fixes, direct action clipping, the separate basic controller,
and configurable command limits were already present on this branch. The
simulation's existing Kalman filter, physics timestep, joystick conventions,
position controller, and YAML library remain in use.

CSV logging and its dedicated tests were removed at the user's request.
Hardware-only changes were not imported. The source has not been rebuilt since
removing logging; existing build artifacts may still contain the earlier code.
