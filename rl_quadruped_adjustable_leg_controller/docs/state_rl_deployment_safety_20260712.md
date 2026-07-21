# StateRL deployment safety contract (2026-07-12 audit)

This patch is an offline-reviewed source change. It has not been copied to an
onboard computer, has not activated a controller, and has not driven a motor.

## Why it exists

Both 2026-07-07 bags contain a 59--70 ms interval immediately after the policy2
switch where all commanded positions, Kp and Kd are zero. The old `enter()`
initialized `output_dof_pos_`, but the control loop wrote a default-constructed
`robot_command_` before asynchronous inference returned. The same bags also show
continuous revolute target-limit violations, including FL hip targets around
2.53--2.55 rad while the audited source URDF specifies +/-1.22173 rad.

## Command handoff

`enter()` now snapshots measured joint positions and atomically publishes a
complete current-position hold with the configured RL Kp/Kd before it enables
the inference thread. The first complete inference replaces that command under
a mutex. Every entry/exit increments an epoch, so an inference that began in an
old state cannot commit after a switch. Invalid/non-finite commands are not
written; the previous hardware-interface values are preserved and the FSM asks
for passive. Re-entry also takes an inference mutex before resetting observation
history, so an old inference cannot race the reset. No delay or first-inference
sleep is used.

Before touching any ros2_control interface, `setCommand()` validates all 16
q/dq/Kp/Kd/tau tuples, including finite values, positive Kp, non-negative Kd,
torque bounds and q inside the effective physical/policy target intersection.
Only after the entire frame passes does a second loop write it, so an invalid
late joint cannot leave a mixed half-frame on the interfaces.
The command mutex remains held through the second write pass, so lifecycle exit
cannot invalidate an epoch between validation and interface writes.

## Target limits

`physical_dof_pos_lower` and `physical_dof_pos_upper` are mandatory YAML fields.
Missing, non-finite, mis-sized, inverted or default-incompatible limits stop RL
configuration. Optional `output_dof_pos_lower/upper` values are a policy-specific
envelope; StateRL intersects them with the mandatory physical envelope on every
inference.

The deployment transpose contract requires exactly `rows=4` legs and `cols=4`
joint types. Every array using that path must contain exactly 16 entries.
Validation occurs before the first index operation, preventing alternate-shape
mis-mapping, short-vector out-of-bounds access and long-vector truncation.

The adjustable-leg controller passes its resolved `joints` parameter into each
StateRL instance. StateRL compares all 16 names and positions against the only
ordering supported by the YAML transpose contract and fails construction on any
mismatch. Logs and the manifest use the verified controller-provided order, not
an unverified hard-coded declaration.

The target-envelope values come from this source tree's `xacro/const.xacro`,
`xacro/leg.xacro` and generated robot description:

- hip: `[-1.2217304763960306, 1.2217304763960306]` rad;
- thigh: `[-1.5708, 3.4907]` rad;
- calf: `[-2.775073510670984, -0.6457718232379019]` rad;
- box: `[0.0, 0.06]` m.

These are not asserted to be the final onboard limits. Verify each value against
the exact robot description and motor-coordinate convention before motor
activation. Policy names and gains are deliberately machine-specific. The
tracked default profile contains only backward-compatible defaults and shared
safety fields; each machine's selected policies and gains remain local and
must not overwrite one another. Checked-in named profiles such as
`rl_policy_e7700` are explicit MuJoCo profiles, not defaults for the real robot.

## Bag-visible provenance and safety topics

`/rl_deployment_manifest` is a reliable volatile string, published on entry and
republished periodically. Volatile durability avoids late recorders receiving
two cached manifests from the separate policy1/policy2 StateRL publishers.
It identifies policy1 versus policy2, the exact selected YAML key, resolved model
and config paths, deterministic FNV-1a-64 fingerprints plus byte counts, build
date, joint order, gains, scales, defaults, action clips and target envelopes.
FNV-1a is explicitly labelled and is not SHA256; archive SHA256 externally when
creating a release artifact.

All debug publishers are per-StateRL instance; no mutable static publisher is
shared by the two inference threads.

`/rl_command_safety_debug` layout (Float32MultiArray schema 1):

1. schema version;
2. FSM state enum;
3. command epoch;
4. running;
5. command valid;
6. inference command valid;
7. hold active;
8. deployment fault;
9. command sequence;
10. committed inference count for this entry;
11. target-clamp event count;
12. latest max clamp delta;
13. lifetime max clamp delta for this entry;
14. first-inference commit latency in ms (`-1` until committed).

`/rl_target_clamp_debug` is published for every clamp event. Its layout is
schema, state enum, command sequence, inference sequence, event count, max
delta, raw targets[16], clamped targets[16]. The inference sequence makes clamp
fraction and consecutive-event duration reconstructable without assuming a fixed
loop rate or relying on throttled log text.

## Required pre-onboard checks

1. Merge the code into the exact onboard checkout; retain the onboard
   machine-local `config.yaml`, gains and explicitly selected
   `model_name`/`model_name_policy2`.
2. Independently verify the 16 physical target bounds and joint order.
3. Build and run the controller against a no-motor mock hardware interface.
4. Switch fixed-stand -> policy1 and fixed-stand -> policy2 repeatedly; assert no
   sample has zero/invalid Kp/Kd, hold is visible before inference, and stale
   epochs never commit.
5. Record all three topics above and compare the manifest gains/paths/fingerprints
   to `/joint_commands` before any locomotion command.
6. Treat sustained target clamping as policy failure, not successful protection.

Controller deactivation calls the current FSM state's `exit()` before releasing
interfaces. StateRL exit invalidates the command epoch and waits on the inference
barrier, so no inference remains active across deactivation.

Local build validation alone is not authorization for a real-robot test.

## Offline validation completed

- The three related packages (`arcdog_adjustable_leg_description`,
  `mujoco_simulator` and `rl_quadruped_adjustable_leg_controller`) build
  successfully with `BUILD_TESTING=ON`.
- The five targeted deployment/MuJoCo contract files pass 37 tests in the
  IsaacLab Python environment. In a system Python without PyTorch, the single
  TorchScript execution test skips while the source-contract tests still run.
- Python launch/test/tool syntax, shell syntax, XML parsing and all checked-in
  TorchScript input/output shape and finite-output checks pass.
- No launch file, controller node, hardware interface or motor was started by
  these checks.

An executable mock-hardware switch test is still required before copying this
patch to an onboard system. The current repository does not provide a no-motor
fixture that exercises the complete FSM and ros2_control command interfaces.
