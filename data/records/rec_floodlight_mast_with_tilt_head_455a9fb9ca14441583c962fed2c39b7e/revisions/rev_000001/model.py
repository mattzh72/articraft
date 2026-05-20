from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_field_floodlight_mast")

    galvanized = model.material("galvanized_steel", color=(0.62, 0.65, 0.64, 1.0))
    dark_steel = model.material("dark_powder_coat", color=(0.04, 0.045, 0.05, 1.0))
    glass = model.material("slightly_blue_glass", color=(0.55, 0.76, 0.95, 0.62))
    concrete = model.material("weathered_concrete", color=(0.48, 0.48, 0.44, 1.0))

    mast = model.part("mast")

    # Ground anchor and tall tubular pole, sized like a small sports-field mast.
    mast.visual(Box((1.20, 1.20, 0.24)), origin=Origin(xyz=(0.0, 0.0, 0.12)), material=concrete, name="concrete_pedestal")
    mast.visual(Cylinder(radius=0.28, length=0.045), origin=Origin(xyz=(0.0, 0.0, 0.262)), material=galvanized, name="base_plate")
    mast.visual(Cylinder(radius=0.09, length=9.25), origin=Origin(xyz=(0.0, 0.0, 4.91)), material=galvanized, name="tubular_pole")
    mast.visual(Cylinder(radius=0.13, length=0.32), origin=Origin(xyz=(0.0, 0.0, 9.48)), material=galvanized, name="top_collar")

    # Base bolts and welded stiffener plates are part of the fixed mast assembly.
    for i, (x, y) in enumerate(((0.18, 0.18), (-0.18, 0.18), (-0.18, -0.18), (0.18, -0.18))):
        mast.visual(Cylinder(radius=0.025, length=0.07), origin=Origin(xyz=(x, y, 0.315)), material=dark_steel, name=f"anchor_bolt_{i}")
    for i, yaw in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        mast.visual(Box((0.035, 0.36, 0.34)), origin=Origin(xyz=(0.0, 0.14 * math.cos(yaw), 0.44), rpy=(0.0, 0.0, yaw)), material=galvanized, name=f"base_gusset_{i}")

    # Main crossarm and six short branching drop arms to the lamp yokes.
    mast.visual(Cylinder(radius=0.058, length=5.35), origin=Origin(xyz=(0.0, 0.0, 9.50), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="main_crossarm")
    mast.visual(Cylinder(radius=0.09, length=0.25), origin=Origin(xyz=(0.0, 0.0, 9.50), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="center_sleeve")

    lamp_xs = (-2.05, -1.23, -0.41, 0.41, 1.23, 2.05)
    joint_y = -0.55
    joint_z = 9.15
    branch_start_y = 0.0
    branch_start_z = 9.50
    branch_end_y = joint_y + 0.075
    branch_end_z = joint_z + 0.25
    branch_len = math.sqrt((branch_end_y - branch_start_y) ** 2 + (branch_end_z - branch_start_z) ** 2)
    # A cylinder's local Z is rolled into the negative Y / downward direction.
    branch_roll = math.atan2(abs(branch_end_y - branch_start_y), branch_end_z - branch_start_z)
    branch_center_y = 0.5 * (branch_start_y + branch_end_y)
    branch_center_z = 0.5 * (branch_start_z + branch_end_z)

    for i, x in enumerate(lamp_xs):
        mast.visual(Cylinder(radius=0.035, length=branch_len), origin=Origin(xyz=(x, branch_center_y, branch_center_z), rpy=(branch_roll, 0.0, 0.0)), material=galvanized, name=f"branch_tube_{i}")
        # Each yoke is a real fixed bracket with two side cheeks and a rear bridge.
        mast.visual(Box((0.040, 0.145, 0.42)), origin=Origin(xyz=(x - 0.31, joint_y, joint_z + 0.02)), material=galvanized, name=f"yoke_plate_neg_{i}")
        mast.visual(Box((0.040, 0.145, 0.42)), origin=Origin(xyz=(x + 0.31, joint_y, joint_z + 0.02)), material=galvanized, name=f"yoke_plate_pos_{i}")
        mast.visual(Box((0.66, 0.055, 0.055)), origin=Origin(xyz=(x, joint_y + 0.075, joint_z + 0.25)), material=galvanized, name=f"yoke_bridge_{i}")

    head_motion = MotionLimits(effort=35.0, velocity=0.8, lower=-0.35, upper=0.80)
    for i, x in enumerate(lamp_xs):
        head = model.part(f"head_{i}")
        # The head frame is the trunnion axis.  The fixture faces local -Y.
        head.visual(Cylinder(radius=0.034, length=0.58), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="trunnion_pin")
        head.visual(Box((0.50, 0.24, 0.40)), origin=Origin(xyz=(0.0, -0.12, -0.02)), material=dark_steel, name="housing")
        head.visual(Box((0.54, 0.040, 0.44)), origin=Origin(xyz=(0.0, -0.252, -0.02)), material=dark_steel, name="front_bezel")
        head.visual(Box((0.46, 0.014, 0.34)), origin=Origin(xyz=(0.0, -0.273, -0.02)), material=glass, name="lens")
        head.visual(Box((0.56, 0.16, 0.025)), origin=Origin(xyz=(0.0, -0.205, 0.192)), material=dark_steel, name="top_visor")
        for j, fin_x in enumerate((-0.19, -0.095, 0.0, 0.095, 0.19)):
            head.visual(Box((0.030, 0.11, 0.36)), origin=Origin(xyz=(fin_x, 0.045, -0.02)), material=dark_steel, name=f"rear_fin_{j}")

        model.articulation(
            f"tilt_{i}",
            ArticulationType.REVOLUTE,
            parent=mast,
            child=head,
            origin=Origin(xyz=(x, joint_y, joint_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=head_motion,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")

    tilt_joints = [object_model.get_articulation(f"tilt_{i}") for i in range(6)]
    ctx.check("six individual tilt joints", len(tilt_joints) == 6)

    for i, joint in enumerate(tilt_joints):
        head = object_model.get_part(f"head_{i}")
        ctx.expect_contact(head, mast, elem_a="trunnion_pin", elem_b=f"yoke_plate_neg_{i}", contact_tol=0.004, name=f"negative trunnion seats in yoke {i}")
        ctx.expect_contact(head, mast, elem_a="trunnion_pin", elem_b=f"yoke_plate_pos_{i}", contact_tol=0.004, name=f"positive trunnion seats in yoke {i}")
        ctx.expect_gap(mast, head, axis="x", positive_elem=f"yoke_plate_pos_{i}", negative_elem="housing", min_gap=0.025, max_gap=0.060, name=f"positive yoke clears housing {i}")
        ctx.expect_gap(head, mast, axis="x", positive_elem="housing", negative_elem=f"yoke_plate_neg_{i}", min_gap=0.025, max_gap=0.060, name=f"negative yoke clears housing {i}")

        rest_aabb = ctx.part_element_world_aabb(head, elem="lens")
        with ctx.pose({joint: 0.55}):
            down_aabb = ctx.part_element_world_aabb(head, elem="lens")
        rest_center_z = 0.5 * (rest_aabb[0][2] + rest_aabb[1][2]) if rest_aabb else None
        down_center_z = 0.5 * (down_aabb[0][2] + down_aabb[1][2]) if down_aabb else None
        ctx.check(
            f"positive tilt aims head downward {i}",
            rest_center_z is not None and down_center_z is not None and down_center_z < rest_center_z - 0.07,
            details=f"rest lens z={rest_center_z}, tilted lens z={down_center_z}",
        )

    return ctx.report()


object_model = build_object_model()
