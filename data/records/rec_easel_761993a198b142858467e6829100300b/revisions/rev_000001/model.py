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
    model = ArticulatedObject(name="wall_mount_fold_flat_display_easel")

    plate_w = 0.56
    plate_h = 0.74
    plate_t = 0.032
    plate_front_y = plate_t / 2.0
    hinge_y = plate_front_y + 0.020
    hinge_z = plate_h
    hinge_r = 0.012

    display_angle = 1.05
    brace_angle = 0.90
    brace_len = 0.46
    brace_x = 0.305
    brace_pivot_y = plate_front_y + 0.030
    brace_pivot_z = 0.235

    painted_steel = model.material("warm_white_powder_coat", rgba=(0.88, 0.86, 0.80, 1.0))
    dark_metal = model.material("dark_bronze_hardware", rgba=(0.08, 0.075, 0.065, 1.0))
    brushed_pin = model.material("brushed_steel_edges", rgba=(0.56, 0.57, 0.55, 1.0))
    cork_face = model.material("muted_cork_panel", rgba=(0.58, 0.42, 0.25, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((plate_w, plate_t, plate_h)),
        origin=Origin(xyz=(0.0, 0.0, plate_h / 2.0)),
        material=painted_steel,
        name="plate",
    )
    # Slight raised rim and screw heads make the wall-mounted back plate read as a real fixture.
    wall_plate.visual(
        Box((plate_w + 0.018, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, plate_front_y + 0.004, plate_h - 0.011)),
        material=painted_steel,
        name="top_rim",
    )
    wall_plate.visual(
        Box((plate_w + 0.018, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, plate_front_y + 0.004, 0.011)),
        material=painted_steel,
        name="bottom_rim",
    )
    wall_plate.visual(
        Box((0.022, 0.010, plate_h)),
        origin=Origin(xyz=(-plate_w / 2.0 - 0.004, plate_front_y + 0.004, plate_h / 2.0)),
        material=painted_steel,
        name="side_rim_0",
    )
    wall_plate.visual(
        Box((0.022, 0.010, plate_h)),
        origin=Origin(xyz=(plate_w / 2.0 + 0.004, plate_front_y + 0.004, plate_h / 2.0)),
        material=painted_steel,
        name="side_rim_1",
    )
    for i, (sx, sz) in enumerate(((-0.19, 0.61), (0.19, 0.61), (-0.19, 0.13), (0.19, 0.13))):
        wall_plate.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(sx, plate_front_y + 0.0025, sz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"screw_head_{i}",
        )

    # Wall-side hinge leaf and two outer knuckles.
    wall_plate.visual(
        Box((0.50, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, plate_front_y + 0.005, plate_h - 0.026)),
        material=dark_metal,
        name="hinge_leaf",
    )
    for i, x in enumerate((-0.152, 0.152)):
        wall_plate.visual(
            Cylinder(radius=hinge_r, length=0.145),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"hinge_knuckle_{i}",
        )

    # Side bracket for the locking brace pivot.
    wall_plate.visual(
        Box((0.058, 0.014, 0.060)),
        origin=Origin(xyz=(brace_x, plate_front_y + 0.007, brace_pivot_z)),
        material=dark_metal,
        name="brace_mount",
    )
    wall_plate.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(brace_x, plate_front_y + 0.024, brace_pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_pin,
        name="brace_pin",
    )

    panel_arm = model.part("panel_arm")
    panel_arm.visual(
        Box((0.45, 0.030, 0.660)),
        origin=Origin(xyz=(0.0, 0.020, -0.348)),
        material=cork_face,
        name="panel_board",
    )
    panel_arm.visual(
        Box((0.488, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.035, -0.668)),
        material=dark_metal,
        name="display_lip",
    )
    panel_arm.visual(
        Box((0.026, 0.014, 0.642)),
        origin=Origin(xyz=(-0.238, 0.032, -0.348)),
        material=dark_metal,
        name="panel_rail_0",
    )
    panel_arm.visual(
        Box((0.026, 0.014, 0.642)),
        origin=Origin(xyz=(0.238, 0.032, -0.348)),
        material=dark_metal,
        name="panel_rail_1",
    )
    panel_arm.visual(
        Box((0.488, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.032, -0.030)),
        material=dark_metal,
        name="top_rail",
    )
    panel_arm.visual(
        Box((0.150, 0.006, 0.050)),
        origin=Origin(xyz=(0.0, 0.008, -0.026)),
        material=dark_metal,
        name="hinge_leaf",
    )
    panel_arm.visual(
        Cylinder(radius=hinge_r, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_knuckle",
    )
    # A side catch is located where the brace nose meets the panel at the intended display angle.
    panel_arm.visual(
        Box((0.076, 0.018, 0.038)),
        origin=Origin(xyz=(0.254, 0.006, -0.430)),
        material=dark_metal,
        name="brace_socket",
    )

    leg_brace = model.part("leg_brace")
    leg_brace.visual(
        Box((0.016, 0.012, brace_len - 0.030)),
        origin=Origin(xyz=(0.0, 0.0, (brace_len + 0.030) / 2.0)),
        material=dark_metal,
        name="brace_bar",
    )
    leg_brace.visual(
        Box((0.014, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="eye_web",
    )
    leg_brace.visual(
        Box((0.040, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, brace_len)),
        material=brushed_pin,
        name="brace_tip",
    )
    leg_brace.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_pin,
        name="pivot_eye",
    )

    model.articulation(
        "plate_to_panel",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=panel_arm,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=display_angle),
    )
    model.articulation(
        "plate_to_brace",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=leg_brace,
        origin=Origin(xyz=(brace_x, brace_pivot_y, brace_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=brace_angle),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    panel_arm = object_model.get_part("panel_arm")
    leg_brace = object_model.get_part("leg_brace")
    panel_hinge = object_model.get_articulation("plate_to_panel")
    brace_hinge = object_model.get_articulation("plate_to_brace")

    ctx.allow_overlap(
        wall_plate,
        leg_brace,
        elem_a="brace_pin",
        elem_b="pivot_eye",
        reason="The brace hinge eye is intentionally captured around the wall-plate pivot pin.",
    )
    ctx.expect_contact(
        wall_plate,
        leg_brace,
        elem_a="brace_pin",
        elem_b="pivot_eye",
        contact_tol=0.004,
        name="brace pin is seated in pivot eye",
    )

    with ctx.pose({panel_hinge: 0.0, brace_hinge: 0.0}):
        ctx.expect_gap(
            panel_arm,
            wall_plate,
            axis="y",
            positive_elem="panel_board",
            negative_elem="plate",
            min_gap=0.010,
            max_gap=0.040,
            name="folded panel lies flat just proud of wall plate",
        )
        ctx.expect_overlap(
            panel_arm,
            wall_plate,
            axes="xz",
            elem_a="panel_board",
            elem_b="plate",
            min_overlap=0.35,
            name="folded panel covers the wall plate footprint",
        )

    folded_aabb = ctx.part_element_world_aabb(panel_arm, elem="display_lip")
    with ctx.pose({panel_hinge: 1.05, brace_hinge: 0.90}):
        display_aabb = ctx.part_element_world_aabb(panel_arm, elem="display_lip")
        ctx.expect_contact(
            leg_brace,
            panel_arm,
            elem_a="brace_tip",
            elem_b="brace_socket",
            contact_tol=0.018,
            name="brace tip reaches the panel socket at display angle",
        )
        ctx.expect_overlap(
            leg_brace,
            panel_arm,
            axes="x",
            elem_a="brace_tip",
            elem_b="brace_socket",
            min_overlap=0.006,
            name="brace tip is laterally captured by socket",
        )

    ctx.check(
        "panel lip swings outward for display",
        folded_aabb is not None
        and display_aabb is not None
        and display_aabb[0][1] > folded_aabb[0][1] + 0.20,
        details=f"folded={folded_aabb}, display={display_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
