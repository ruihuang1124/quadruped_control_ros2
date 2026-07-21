#!/usr/bin/env python3
"""
Offline checks for coherent callback-to-control-loop input transfer.

The test reads source and models the sequence rules. It never creates a ROS
node, loads a controller, or accesses hardware.
"""

from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / "src/RlQuadrupedControllerAdjustableLeg.cpp"
HEADER = PACKAGE / "src/RlQuadrupedControllerAdjustableLeg.h"


class InputTransferModel:
    """Small executable model of the C++ snapshot/version contract."""

    def __init__(self):
        self.pending = {"command": 0, "lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
        self.sequence = 0
        self.command_sequence = 0
        self.applied_sequence = 0
        self.applied_command_sequence = 0
        self.control = dict(self.pending)

    def full_message(self, **values):
        self.pending.update(values)
        self.sequence += 1
        self.command_sequence += 1

    def joy(self, *, lx, ly, rx, command=None):
        self.pending.update(lx=lx, ly=ly, rx=rx)
        self.sequence += 1
        if command is not None:
            self.pending["command"] = command
            self.command_sequence += 1

    def apply(self):
        if self.sequence == self.applied_sequence:
            return
        for field in ("lx", "ly", "rx", "ry"):
            self.control[field] = self.pending[field]
        if self.command_sequence != self.applied_command_sequence:
            self.control["command"] = self.pending["command"]
            self.applied_command_sequence = self.command_sequence
        self.applied_sequence = self.sequence


class ControlInputRealtimeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.header = HEADER.read_text(encoding="utf-8")

    def test_callbacks_publish_snapshots_without_touching_rt_owned_message(self):
        control_callback = self.source[
            self.source.index("control_input_subscription_"):
            self.source.index("sub_joy_ =")
        ]
        joy_callback = self.source[
            self.source.index("sub_joy_ ="):
            self.source.index("// // Handle message for xbox wireless controller")
        ]
        for callback in (control_callback, joy_callback):
            self.assertIn("control_input_writer_mutex_", callback)
            self.assertIn("control_input_buffer_.writeFromNonRT", callback)
            self.assertNotIn("ctrl_interfaces_.control_inputs_", callback)

    def test_update_reads_realtime_buffer_before_fsm_work(self):
        update = self.source[self.source.index("update(const rclcpp::Time&"):]
        update = update[: update.index("LeggedGymController::on_init")]
        self.assertLess(
            update.index("applyLatestControlInputSnapshot()"),
            update.index("current_state_->run(time, period)"),
        )
        apply = self.source[
            self.source.index("void LeggedGymController::applyLatestControlInputSnapshot"):
            self.source.index("LeggedGymController::on_init")
        ]
        self.assertIn("control_input_buffer_.readFromRT()", apply)
        self.assertNotIn("std::lock_guard", apply)
        self.assertNotIn("std::unique_lock", apply)
        for field in ("lx", "ly", "rx", "ry"):
            self.assertIn(f"control_inputs_.{field} = snapshot->inputs.{field}", apply)

    def test_snapshot_has_separate_command_version(self):
        self.assertIn("realtime_tools::RealtimeBuffer<ControlInputSnapshot>", self.header)
        self.assertIn("std::uint64_t command_sequence", self.header)
        self.assertIn(
            "command_sequence != last_applied_control_command_sequence_",
            self.source,
        )

    def test_axis_only_joy_does_not_resurrect_consumed_command(self):
        model = InputTransferModel()
        model.full_message(command=3, lx=0.4, ly=0.0, rx=0.0, ry=0.2)
        model.apply()
        self.assertEqual(model.control["command"], 3)

        # Mirrors an FSM enter() consuming the transition command.
        model.control["command"] = -1
        model.joy(lx=0.1, ly=-0.2, rx=0.3)
        model.apply()
        self.assertEqual(model.control["command"], -1)
        self.assertEqual(
            [model.control[key] for key in ("lx", "ly", "rx", "ry")],
            [0.1, -0.2, 0.3, 0.2],
        )

        model.joy(lx=0.1, ly=-0.2, rx=0.3, command=4)
        model.apply()
        self.assertEqual(model.control["command"], 4)


if __name__ == "__main__":
    unittest.main()
