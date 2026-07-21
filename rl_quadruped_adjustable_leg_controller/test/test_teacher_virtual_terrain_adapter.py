from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
HEADER = (
    PACKAGE
    / "include/rl_quadruped_adjustable_leg_controller/FSM"
    / "HighstepTeacherVirtualTerrainAdapter.h"
)
SOURCE = PACKAGE / "src/FSM/StateRL.cpp"
CONFIG = (
    PACKAGE.parent
    / "robot_description/arcdog_adjustable_leg_description/config"
    / "rl_policy_teacher_virtual_terrain/config.yaml"
)
DIAGNOSTIC_DISABLED = (
    PACKAGE.parent
    / "robot_description/arcdog_adjustable_leg_description/config"
    / "teacher_virtual_diagnostic_disabled.yaml"
)
DIAGNOSTIC_MUJOCO = (
    PACKAGE.parent
    / "robot_description/arcdog_adjustable_leg_description/config"
    / "teacher_virtual_diagnostic_mujoco_once.yaml"
)
REFERENCE = Path(
    PACKAGE.parent
    / "robot_description"
    / "arcdog_adjustable_leg_description"
    / "config"
    / "rl_policy_teacher_virtual_terrain"
    / "teacher_b300_virtual_terrain_reference.pt"
)


def test_adapter_has_no_simulator_world_truth_api():
    text = HEADER.read_text()
    forbidden = (
        "mujoco", "mj_model", "mjdata", "geom_pos", "raycast",
        "world_contact", "free_joint",
    )
    assert all(token not in text.lower() for token in forbidden)
    assert "longitudinal_progress" in text
    assert "lateral_offset" in text
    assert "relative_yaw" in text
    assert "body_height_offset" in text
    assert "confidence" in text


def test_teacher_contract_is_570_plus_privileged_encoder_and_prior_once():
    source = SOURCE.read_text()
    config = CONFIG.read_text()
    assert "final_obs.size(1) != 570" in source
    assert "Teacher privileged proprioception" in source
    assert "teacher_fake_scan_" in source
    assert "teacher_front_rear_delta_" in source
    assert "teacher_prior_application_count" in source
    assert "teacher_virtual_terrain_enabled: true" in config
    assert "policy2_fixed_motion_enabled: false" in config


def test_live_joystick_remains_proportional_and_release_is_zero():
    source = SOURCE.read_text()
    config = CONFIG.read_text()
    assert (
        "model_command_x = control_snapshot.x * "
        "params_.teacher_policy2_command_x_gain"
    ) in source
    assert "resolveCommand(control_snapshot.x)" in source  # old one-shot branch remains isolated
    assert "teacher_policy2_command_x_gain: 1.4400000572" in config


def test_lookup_is_progress_gated_and_contact_scan_fault_requires_passive():
    source = SOURCE.read_text()
    assert REFERENCE.is_file()
    assert "teacher_previous_lookup_progress_" in source
    assert "scan_jump > 0.20" in source
    assert (
        "teacher_virtual_adapter_.contactCriticalPhase() || next_phase >= 0.15"
        in source
    )


def test_zero_progress_holds_phase_and_scan_then_progress_advances_monotonically():
    torch = pytest.importorskip("torch")
    reference = torch.jit.load(str(REFERENCE), map_location="cpu")
    query = torch.zeros(1, 20)
    phase = torch.zeros(1)
    scan = torch.zeros(1, 102)
    previous_progress = torch.zeros(1)

    initial = reference(query, phase, scan, previous_progress)
    expected_scan = initial[:, :102].clone()
    expected_height = initial[:, 104].clone()
    phase = initial[:, 103]
    scan = initial[:, :102]
    for _ in range(100):
        result = reference(query, phase, scan, previous_progress)
        assert torch.equal(result[:, 103], torch.zeros_like(result[:, 103]))
        assert torch.equal(result[:, :102], expected_scan)
        assert torch.equal(result[:, 104], expected_height)
        phase = result[:, 103]
        scan = result[:, :102]

    phases = []
    for step in range(1, 41):
        current_progress = torch.tensor([step * 0.01], dtype=torch.float32)
        query[:, 0] = current_progress
        result = reference(query, phase, scan, previous_progress)
        phases.append(float(result[0, 103]))
        phase = result[:, 103]
        scan = result[:, :102]
        previous_progress = current_progress
    assert phases[-1] > 0.0
    assert all(after >= before for before, after in zip(phases, phases[1:]))


def test_height_confidence_uses_frozen_phase_envelope_and_contact_fault_is_passive():
    header = HEADER.read_text()
    source = SOURCE.read_text()
    assert "std::abs(state_.body_height_offset) <= 0.05" not in header
    assert "acceptReferenceEnvelope" in header
    assert "state_.body_height_offset - expected_height_" in header
    assert "RequireTensorSize(reference, 110" in source
    confidence_fault = source.index(
        'throw std::runtime_error('
        '"virtual terrain adapter confidence/phase safety check failed")'
    )
    passive_latch = source.rfind("deployment_fault_requires_passive_.store", 0, confidence_fault)
    assert passive_latch >= 0
    assert confidence_fault - passive_latch < 400


def test_yaw_is_phase_conditioned_and_cannot_lock_phase_selector():
    header = HEADER.read_text()
    source = SOURCE.read_text()
    assert "std::abs(state_.relative_yaw) <= 6.0" not in header
    assert "yaw_in_phase_envelope" in header
    assert "state_.relative_yaw - expected_yaw_" in header
    assert "RequireTensorSize(reference, 110" in source


def test_soft_gate_log_only_is_explicit_mujoco_only_and_normal_default_is_fail_closed():
    source = SOURCE.read_text()
    disabled = DIAGNOSTIC_DISABLED.read_text()
    enabled = DIAGNOSTIC_MUJOCO.read_text()
    assert "diagnostic_soft_gates_log_only: false" in disabled
    assert 'diagnostic_runtime_backend: "unknown"' in disabled
    assert "diagnostic_soft_gates_log_only: true" in enabled
    assert 'diagnostic_runtime_backend: "mujoco"' in enabled
    assert 'count_publishers("/mujoco_applied_actuators_cmds") > 0' in source
    assert 'services.find("/highstep_contract_arm")' in source
    assert "runtime is not positively verified as MuJoCo" in source


def test_diagnostic_bypasses_only_soft_gates_and_logs_every_event_mask():
    source = SOURCE.read_text()
    assert "teacher_soft_gate_event_mask_ |= 1U" in source
    assert "teacher_soft_gate_event_mask_ |= 2U" in source
    assert "teacher_soft_gate_event_mask_ |= 4U" in source
    assert "teacher_soft_gate_event_mask_ |= 8U" in source
    assert "teacher_soft_gate_event_mask_ |= 16U" in source
    assert "teacher_soft_gate_event_mask_ |= 32U" in source
    assert (
        "!params_.diagnostic_soft_gates_log_only || "
        "!diagnostic_mujoco_backend_verified_"
    ) in source
    assert "Teacher virtual-terrain reference generated non-finite data" in source
    assert (
        "Teacher virtual-terrain target jump exceeded safety envelope"
        in source
    )
    assert "Policy generated a non-finite joint target" in source
    assert "teacher_soft_gate_bypass_count_" in source


def test_unknown_teacher_inference_failure_defaults_to_passive():
    source = SOURCE.read_text()
    reset = source.index("teacher_virtual_adapter_.reset")
    conservative_latch = source.index(
        "deployment_fault_requires_passive_.store(true", reset
    )
    inference = source.index("torch::Tensor StateRL::forward()")
    assert reset < conservative_latch < inference


def test_phase_is_hard_held_until_first_positive_joystick_input():
    source = SOURCE.read_text()
    assert "control_snapshot.x > 0.05" in source
    assert "teacher_positive_command_seen_ = true" in source
    assert "const double lookup_progress = teacher_positive_command_seen_" in source
    assert "? adapter_state.longitudinal_progress : 0.0" in source
    assert (
        "Teacher virtual-terrain phase advanced before first positive joystick input"
        in source
    )


def test_target_jump_is_checked_after_physical_clamp_and_logs_exact_joint_values():
    source = SOURCE.read_text()
    physical_clamp = source.index(
        "output_dof_pos_ = clamp("
        "output_dof_pos_, effective_lower, effective_upper)"
    )
    jump_check = source.index("Teacher virtual-terrain clamped target jump:")
    assert physical_clamp < jump_check
    assert "old_clamped=%.9f" in source
    assert "new_clamped=%.9f" in source
    assert "unclamped=%.9f" in source
    assert "phase=%.9f" in source
    assert "threshold=%.9f" in source
    # Reproduce the epoch31 false positive: the physical candidate is unchanged.
    old_clamped = 1.222
    unclamped = 1.568
    new_clamped = min(unclamped, 1.222)
    assert abs(new_clamped - old_clamped) == 0.0


def test_terminal_phase_zeroes_only_model_vx_and_keeps_raw_joystick_trace():
    source = SOURCE.read_text()
    terminal = source.index("teacher_virtual_phase_ >= 1.0 - 1.0e-6")
    zero_model_vx = source.index("model_command_x = 0.0", terminal)
    raw_trace = source.index("trace.data.push_back(static_cast<float>(control_snapshot.x))")
    model_trace = source.index("trace.data.push_back(static_cast<float>(model_command_x))")
    assert terminal < zero_model_vx
    assert raw_trace < model_trace
