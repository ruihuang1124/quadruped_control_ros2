#!/usr/bin/env python3
"""Offline regression checks for the StateRL deployment safety contract.

This test does not instantiate ros2_control and never opens a hardware device.
"""

from pathlib import Path
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parent
CONFIG = (
    REPOSITORY
    / "robot_description/arcdog_adjustable_leg_description/config/rl_policy_e7700/config.yaml"
)
POLICY1_CONFIG = (
    REPOSITORY
    / "robot_description/arcdog_adjustable_leg_description/config/rl_policy/config.yaml"
)
CONTROLLER_CONFIG = (
    REPOSITORY
    / "robot_description/arcdog_adjustable_leg_description/config/robot_control_E7700.yaml"
)
SOURCE = PACKAGE / "src/FSM/StateRL.cpp"
CONTROLLER_SOURCE = PACKAGE / "src/RlQuadrupedControllerAdjustableLeg.cpp"
EXPECTED_JOINT_ORDER = [
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    "FL_box_joint", "FR_box_joint", "RL_box_joint", "RR_box_joint",
]


def transpose_leg_major(values, rows=4, cols=4):
    if rows <= 0 or cols <= 0:
        raise ValueError("matrix rows and cols must be positive")
    if len(values) != rows * cols:
        raise ValueError("expected a 4x4 deployment vector")
    return [values[row * cols + col] for col in range(cols) for row in range(rows)]


def validate_joint_order(actual, expected=EXPECTED_JOINT_ORDER):
    if list(actual) != list(expected):
        raise ValueError("controller joint order mismatch")


class StateRLDeploymentContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.policy1_config = yaml.safe_load(POLICY1_CONFIG.read_text(encoding="utf-8"))
        cls.controller_config = yaml.safe_load(
            CONTROLLER_CONFIG.read_text(encoding="utf-8")
        )
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.controller_source = CONTROLLER_SOURCE.read_text(encoding="utf-8")

    def test_policy_roles_are_explicit(self):
        self.assertNotIn("model_name", self.config)
        self.assertTrue(self.policy1_config["model_name"])
        self.assertTrue(self.config["model_name_policy2"])
        self.assertIn("Required YAML policy key", self.source)
        self.assertNotIn("Falling back to 'model_name'", self.source)

    def test_policy2_selects_plain_student_contract(self):
        params = self.controller_config["rl_quadruped_adjustable_leg_controller"][
            "ros__parameters"
        ]
        self.assertEqual(params["model_folder"], "rl_policy")
        self.assertEqual(params["model_folder_policy2"], "rl_policy_e7700")
        self.assertEqual(
            self.config["model_name_policy2"],
            "policy_student_b300_rl_preedge_E7700_model_181198.pt",
        )
        self.assertFalse(self.config["policy2_fixed_motion_enabled"])
        self.assertFalse(self.config["policy2_one_shot_enabled"])
        self.assertFalse(self.config["teacher_virtual_terrain_enabled"])

    def test_physical_target_envelope_is_complete(self):
        lower = transpose_leg_major(self.config["physical_dof_pos_lower"])
        upper = transpose_leg_major(self.config["physical_dof_pos_upper"])
        default = transpose_leg_major(self.config["default_dof_pos"])
        self.assertEqual(len(lower), 16)
        self.assertTrue(all(lo < hi for lo, hi in zip(lower, upper)))
        self.assertTrue(all(lo <= q <= hi for lo, q, hi in zip(lower, default, upper)))
        self.assertEqual(lower[12:16], [0.0] * 4)
        self.assertEqual(upper[12:16], [0.06] * 4)

    def test_yaml_matrix_rejects_short_long_and_nonpositive_shapes(self):
        for length in (15, 17):
            with self.assertRaises(ValueError):
                transpose_leg_major(list(range(length)))
        with self.assertRaises(ValueError):
            transpose_leg_major([], rows=0, cols=4)

        exact_size_check = self.source.index("values.size() != expected_size")
        first_transpose_index = self.source.index(
            "transposed_values[c * row_count + r] = values[r * col_count + c]"
        )
        self.assertLess(
            self.source.index("rows <= 0 || cols <= 0"), exact_size_check
        )
        self.assertIn("rows != 4 || cols != 4", self.source)
        self.assertLess(exact_size_check, first_transpose_index)

    def test_0707_hazardous_targets_are_clamped_by_reference_contract(self):
        lower = transpose_leg_major(self.config["physical_dof_pos_lower"])
        upper = transpose_leg_major(self.config["physical_dof_pos_upper"])
        raw = transpose_leg_major(self.config["default_dof_pos"])
        raw[0] = 2.5284  # First-bag FL hip q_des maximum.
        raw[11] = -0.40  # Representative RR calf upper-limit violation.
        clamped = [min(max(q, lo), hi) for q, lo, hi in zip(raw, lower, upper)]
        self.assertAlmostEqual(clamped[0], 1.2217304763960306)
        self.assertAlmostEqual(clamped[11], -0.6457718232379019)

    def test_enter_commits_hold_before_enabling_inference(self):
        enter = self.source[self.source.index("void StateRL::enter()"):]
        enter = enter[:enter.index("void StateRL::run(")]
        self.assertLess(
            enter.index("initializeHoldCommand(state_snapshot)"),
            enter.index("running_.store(true"),
        )
        self.assertIn("hold_command.motor_command.q[i] = position", self.source)
        self.assertIn("hold_command.motor_command.kp[i] = params_.rl_kp", self.source)
        self.assertIn("hold_command.motor_command.kd[i] = params_.rl_kd", self.source)
        self.assertIn("std::unique_lock<std::mutex> inference_lock(inference_mutex_)", enter)
        self.assertLess(
            enter.index("std::unique_lock<std::mutex> inference_lock(inference_mutex_)"),
            enter.index("inference_commit_count_.store(0"),
        )
        self.assertIn("inference_commit_count_.fetch_add", self.source)

    def test_bag_observability_topics_are_present(self):
        for topic in (
            "/rl_deployment_manifest",
            "/rl_command_safety_debug",
            "/rl_target_clamp_debug",
        ):
            self.assertIn(topic, self.source)

    def test_complete_frame_is_validated_before_any_interface_write(self):
        set_command = self.source[self.source.index("void StateRL::setCommand()"):]
        set_command = set_command[
            :set_command.index("std::string StateRL::buildDeploymentManifest")
        ]
        validation_marker = set_command.index(
            "Validate the complete 16-joint frame before writing any interface"
        )
        write_marker = set_command.index(
            "ctrl_interfaces_.joint_position_command_interface_[i]"
        )
        self.assertLess(validation_marker, write_marker)
        self.assertGreaterEqual(
            set_command.count("for (int i = 0; i < 16; ++i)"), 2
        )
        for required_check in (
            "!std::isfinite(q)",
            "!std::isfinite(dq)",
            "!std::isfinite(kp)",
            "!std::isfinite(kd)",
            "!std::isfinite(tau)",
            "q < lower || q > upper",
        ):
            self.assertIn(required_check, set_command[:write_marker])

        command_lock = set_command.index(
            "std::unique_lock<std::mutex> command_lock(command_mutex_)"
        )
        unlock_after_write = set_command.find("command_lock.unlock()", write_marker)
        self.assertLess(command_lock, validation_marker)
        self.assertGreater(unlock_after_write, write_marker)

    def test_controller_joint_order_is_exact_and_mismatch_fails_closed(self):
        actual = self.controller_config["rl_quadruped_adjustable_leg_controller"][
            "ros__parameters"
        ]["joints"]
        validate_joint_order(actual)
        swapped = list(actual)
        swapped[0], swapped[1] = swapped[1], swapped[0]
        with self.assertRaises(ValueError):
            validate_joint_order(swapped)

        self.assertIn(
            "verified_joint_names_[i] != kControllerJointOrder[i]", self.source
        )
        self.assertIn(
            "Refusing to map YAML gains/limits by position", self.source
        )
        self.assertGreaterEqual(self.controller_source.count("joint_names_"), 2)
        manifest = self.source[
            self.source.index("StateRL::buildDeploymentManifest"):
        ]
        self.assertIn("JsonEscape(verified_joint_names_[i])", manifest)

    def test_lifecycle_exit_quiesces_before_interfaces_are_released(self):
        deactivate = self.controller_source[
            self.controller_source.index("LeggedGymController::on_deactivate"):
        ]
        deactivate = deactivate[:deactivate.index("LeggedGymController::on_cleanup")]
        self.assertLess(
            deactivate.index("current_state_->exit()"),
            deactivate.index("release_interfaces()"),
        )
        exit_body = self.source[self.source.index("void StateRL::exit()"):]
        exit_body = exit_body[:exit_body.index("FSMStateName StateRL::checkChange")]
        self.assertIn(
            "std::lock_guard<std::mutex> inference_lock(inference_mutex_)",
            exit_body,
        )

    def test_startup_guards_and_instance_publishers_are_fail_closed(self):
        for guard in (
            "target_pos.size() != 16",
            "params_.decimation <= 0",
            "params_.torque_limits < 0",
            "manifest_qos.reliable().durability_volatile()",
        ):
            self.assertIn(guard, self.source)
        self.assertNotIn("static rclcpp::Publisher", self.source)


if __name__ == "__main__":
    unittest.main()
