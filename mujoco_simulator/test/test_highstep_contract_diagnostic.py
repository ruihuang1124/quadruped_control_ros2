from pathlib import Path
import re


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parent
SOURCE = PACKAGE / "src/adjustable_leg_mujoco_msg_handler.cpp"
HEADER = PACKAGE / "include/mujoco_node/adjustable_leg_mujoco_msg_handler.h"
LAUNCH = PACKAGE / "launch/mujoco.launch.py"
MODEL = (
    REPOSITORY
    / "robot_description/arcdog_adjustable_leg_description/xml"
    / "arcdog_adjustable_leg.xml"
)


def test_diagnostic_is_explicitly_disabled_by_default_and_armed_by_service():
    source = SOURCE.read_text()
    launch = LAUNCH.read_text()
    assert 'default_value="false"' in launch
    assert '"/highstep_contract_arm"' in source
    assert "contract_diagnostic_armed_ = true" in source
    assert "msg->axes[1] < 0.99F" in source


def test_exact_keyframe_dimensions_and_approved_values():
    text = MODEL.read_text()
    match = re.search(
        r'<key name="highstep_b300_contract_start"\s+qpos="([^"]+)"\s+qvel="([^"]+)"',
        text,
    )
    assert match
    qpos = [float(value) for value in match.group(1).split()]
    qvel = [float(value) for value in match.group(2).split()]
    assert len(qpos) == 23
    assert len(qvel) == 22
    assert qpos[:7] == [0.65, 0.0, 0.44, 1.0, 0.0, 0.0, 0.0]
    assert qpos[7] == 0.1975599229
    assert qpos[10] == 0.0229980294
    assert qvel[:6] == [0.0] * 6


def test_reset_uses_explicit_physical_height_parameter_and_state():
    source = SOURCE.read_text()
    assert 'mjOBJ_KEY, "highstep_b300_contract_start"' in source
    assert 'mjOBJ_GEOM, "highstep_box_L10_h0350"' in source
    assert 'declare_parameter<double>("highstep_contract_platform_height_m", 0.30)' in source
    assert "geom_pos[3 * platform_id + 2] = 0.5 * contract_platform_height_m_" in source
    assert "geom_size[3 * platform_id + 2] = 0.5 * contract_platform_height_m_" in source
    assert "mj_resetDataKeyframe" in source
    assert "mj_forward" in source


def test_box_response_tracks_measured_position_while_position_loop_is_disabled():
    source = SOURCE.read_text()
    passive_rebase = source.index("if (msg->kp[k] <= 0.0F)")
    measured_position = source.index("joint_position, box_target_lower_m_", passive_rebase)
    rate_limit = source.index("else if (!inserted && response_dt > 0.0)", passive_rebase)
    pd_output = source.index("msg->kp[k] * position_error", rate_limit)
    assert passive_rebase < measured_position < rate_limit < pd_output
