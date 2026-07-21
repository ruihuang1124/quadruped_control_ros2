from pathlib import Path
import subprocess
import tempfile


PACKAGE = Path(__file__).resolve().parents[1]
HEADER = (
    PACKAGE
    / "include/rl_quadruped_adjustable_leg_controller/FSM"
    / "HighstepFixedMotionPolicy2Adapter.h"
)
STATE_RL = PACKAGE / "src/FSM/StateRL.cpp"


def test_header_only_phase_and_prior_golden_vectors():
    source = r'''
#include <cmath>
#include <iostream>
#include "rl_quadruped_adjustable_leg_controller/FSM/HighstepFixedMotionPolicy2Adapter.h"
int main() {
  highstep_fixed_motion::Policy2Adapter a;
  auto z=a.boxBiasMeters(); if(z[0]!=0 || z[3]!=0) return 1;
  a.advance(highstep_fixed_motion::Policy2Adapter::kFullPushVx);
  auto reach=a.boxBiasMeters();
  if(std::abs(reach[0]+0.020F)>1e-7F || std::abs(reach[2]-0.002F)>1e-7F) return 2;
  for(int i=1;i<9;i++) a.advance(highstep_fixed_motion::Policy2Adapter::kFullPushVx);
  auto push=a.boxBiasMeters();
  if(std::abs(push[0]-0.002F)>1e-7F || std::abs(push[2]+0.022F)>1e-7F) return 3;
  for(int i=9;i<59;i++) a.advance(highstep_fixed_motion::Policy2Adapter::kFullPushVx);
  if(a.phase()!=1.0) return 4;
  auto terminal=a.boxBiasMeters(); if(terminal[0]!=0 || terminal[3]!=0) return 5;
  a.advance(-1.0); if(a.phase()!=1.0) return 6;
  auto f=a.features(); if(f[0]!=1.0F || f[6]!=1.0F || f[7]!=1.0F) return 7;
  std::cout << "ok\n";
}
'''
    with tempfile.TemporaryDirectory() as temp:
        cpp = Path(temp) / "test.cpp"
        exe = Path(temp) / "test"
        cpp.write_text(source)
        subprocess.run(
            [
                "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                "-I", str(PACKAGE / "include"), str(cpp), "-o", str(exe),
            ],
            check=True,
        )
        result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert result.stdout.strip() == "ok"


def test_state_rl_uses_same_adapter_for_mujoco_and_real_robot():
    source = STATE_RL.read_text()
    assert "policy2_fixed_motion_adapter_.resolveCommand(" in source
    assert "policy2_fixed_motion_adapter_.features()" in source
    assert "policy2_fixed_motion_adapter_.boxBiasMeters()" in source
    assert "height_delta" not in HEADER.read_text()
    assert "raycast" not in HEADER.read_text()
    assert "camera" not in HEADER.read_text()


def test_policy2_mapping_and_strict_21_59_58_schedule():
    source = r'''
#include <cmath>
#include "rl_quadruped_adjustable_leg_controller/FSM/HighstepFixedMotionPolicy2Adapter.h"
int main() {
  using A=highstep_fixed_motion::Policy2Adapter;
  A diag;
  if(!diag.trigger()) return 1;
  if(diag.trigger()) return 2;
  for(int i=0;i<138;i++) {
    auto s=diag.resolveCommand(i % 2 == 0 ? -0.5 : 0.5);
    const double expected=(i>=21 && i<80) ? A::kFullPushVx : 0.0;
    if(s.diagnostic_step!=i || std::abs(s.model_vx-expected)>1e-12) return 3;
    if(i<79 && diag.phase()>=1.0) return 4;
  }
  if(diag.phase()!=1.0 || !diag.complete()) return 5;
  auto terminal=diag.resolveCommand(0.5);
  if(terminal.model_vx!=0.0 || !terminal.sequence_complete) return 6;
  if(diag.trigger()) return 7;
  diag.reset();
  if(!diag.trigger()) return 8;
  return 0;
}
'''
    with tempfile.TemporaryDirectory() as temp:
        cpp = Path(temp) / "schedule.cpp"
        exe = Path(temp) / "schedule"
        cpp.write_text(source)
        subprocess.run(
            [
                "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                "-I", str(PACKAGE / "include"), str(cpp), "-o", str(exe),
            ],
            check=True,
        )
        subprocess.run([str(exe)], check=True)


def test_fixed_motion_path_fails_closed_instead_of_shape_fallback():
    source = STATE_RL.read_text()
    fixed_branch = source[
        source.index(
            "if (params_.policy2_fixed_motion_enabled)\n"
            "    {\n"
            "        // The fixed-motion model"
        ):
    ]
    fixed_branch = fixed_branch[:fixed_branch.index("else try")]
    assert "dummy_latent" not in fixed_branch
    assert "model_.forward({final_obs})" in fixed_branch


def test_policy2_one_shot_has_fixed_stand_gate_and_rejects_repeat_trigger():
    source = STATE_RL.read_text()
    assert "validatePolicy2TriggerState(state_snapshot)" in source
    assert "Rejected repeated policy2 trigger" in source
    assert "21 zero + 59 vx=%.10f + 58 zero" in source
    assert "raw_joystick_vx *" not in HEADER.read_text()
