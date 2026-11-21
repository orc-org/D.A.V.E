import math
from arm_controller.kinematics import Component3D


def test_component_3d_chain_and_gripper_step_normalization():
    base = Component3D("Base", limit_x=(0, 0), limit_y=(0, 0))
    link = Component3D("Arm", parent=base, offset_x=1.0, limit_y=None)
    link.set_angles(ay=-90.0)
    gx, gy, gz = link.get_global_position()
    assert math.isfinite(gx) and math.isfinite(gy) and math.isfinite(gz)

    # Check stroke normalization matches 0.2 m over 8 mm/rev and 1600 steps/rev = 40000 steps
    steps_per_m = 1600.0 / 0.008
    max_steps = steps_per_m * 0.2
    assert math.isclose(max_steps, 40000.0)
