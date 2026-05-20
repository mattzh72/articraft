from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_double_door_switchboard")

    steel = Material("powder_coated_steel", color=(0.58, 0.63, 0.64, 1.0))
    dark_steel = Material("dark_plinth_steel", color=(0.12, 0.14, 0.15, 1.0))
    gasket = Material("black_rubber_gasket", color=(0.015, 0.016, 0.014, 1.0))
    hinge_mat = Material("brushed_hinge_steel", color=(0.72, 0.72, 0.68, 1.0))
    latch_mat = Material("zinc_plated_latch", color=(0.80, 0.77, 0.68, 1.0))
    warning = Material("yellow_warning_label", color=(1.0, 0.78, 0.08, 1.0))
    red = Material("red_indicator", color=(0.85, 0.05, 0.03, 1.0))
    green = Material("green_indicator", color=(0.05, 0.55, 0.12, 1.0))

    width = 1.24
    depth = 0.36
    height = 2.02
    side_t = 0.030
    top_t = 0.040
    front_y = -depth / 2.0
    door_t = 0.035
    hinge_y = front_y - 0.026
    door_w = 0.612
    door_h = 1.74
    door_z = 1.05

    body = model.part("body")
    body.visual(
        Box((width, side_t, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - side_t / 2.0, height / 2.0)),
        material=steel,
        name="back_sheet",
    )
    for x, name in ((-width / 2.0 + side_t / 2.0, "side_sheet_0"), (width / 2.0 - side_t / 2.0, "side_sheet_1")):
        body.visual(
            Box((side_t, depth, height)),
            origin=Origin(xyz=(x, 0.0, height / 2.0)),
            material=steel,
            name=name,
        )
    body.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - top_t / 2.0)),
        material=steel,
        name="top_cap",
    )
    body.visual(
        Box((width, depth, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=steel,
        name="bottom_sill",
    )
    body.visual(
        Box((width + 0.08, depth + 0.05, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_steel,
        name="floor_plinth",
    )
    body.visual(
        Box((width, 0.038, 0.080)),
        origin=Origin(xyz=(0.0, front_y - 0.004, 1.960)),
        material=steel,
        name="front_header",
    )
    body.visual(
        Box((width, 0.038, 0.075)),
        origin=Origin(xyz=(0.0, front_y - 0.004, 0.135)),
        material=steel,
        name="front_threshold",
    )
    body.visual(
        Box((0.020, 0.026, door_h)),
        origin=Origin(xyz=(0.0, front_y + 0.015, door_z)),
        material=dark_steel,
        name="center_jamb",
    )

    # Fixed hinge-side leaves and exposed vertical pins mounted to the cabinet frame.
    for x, side_name in ((-width / 2.0 - 0.006, "hinge_0"), (width / 2.0 + 0.006, "hinge_1")):
        body.visual(
            Box((0.040, 0.014, 1.74)),
            origin=Origin(xyz=(x, front_y - 0.006, door_z)),
            material=hinge_mat,
            name=f"{side_name}_frame_leaf",
        )
        for zc, suffix in ((0.50, "lower"), (0.99, "middle"), (1.48, "upper")):
            body.visual(
                Cylinder(radius=0.016, length=0.28),
                origin=Origin(xyz=(x, hinge_y, zc), rpy=(0.0, 0.0, 0.0)),
                material=hinge_mat,
                name=f"{side_name}_{suffix}_knuckle",
            )

    door_0 = model.part("door_0")
    door_0.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_z)),
        material=steel,
        name="door_panel",
    )
    door_0.visual(
        Box((door_w - 0.070, 0.008, door_h - 0.160)),
        origin=Origin(xyz=(door_w / 2.0 + 0.010, -door_t / 2.0 - 0.004, door_z)),
        material=Material("slightly_recessed_door_0", color=(0.50, 0.56, 0.58, 1.0)),
        name="raised_field",
    )
    door_0.visual(
        Box((0.030, 0.010, door_h)),
        origin=Origin(xyz=(door_w - 0.004, -door_t / 2.0 - 0.005, door_z)),
        material=steel,
        name="center_overlap_lip",
    )
    door_0.visual(
        Box((0.020, 0.008, door_h - 0.080)),
        origin=Origin(xyz=(0.020, -door_t / 2.0 - 0.004, door_z)),
        material=hinge_mat,
        name="hinge_leaf",
    )
    door_0.visual(
        Box((0.080, 0.012, 0.120)),
        origin=Origin(xyz=(door_w - 0.125, -door_t / 2.0 - 0.006, 1.20)),
        material=latch_mat,
        name="latch_pivot_plate",
    )
    door_0.visual(
        Box((0.160, 0.004, 0.060)),
        origin=Origin(xyz=(0.220, -door_t / 2.0 - 0.002, 1.64)),
        material=warning,
        name="warning_placard",
    )
    for i in range(6):
        door_0.visual(
            Box((0.180, 0.004, 0.010)),
            origin=Origin(xyz=(0.300, -door_t / 2.0 - 0.002, 0.50 + i * 0.032)),
            material=gasket,
            name=f"vent_slot_0_{i}",
        )

    door_1 = model.part("door_1")
    door_1.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(-door_w / 2.0, 0.0, door_z)),
        material=steel,
        name="door_panel",
    )
    door_1.visual(
        Box((door_w - 0.070, 0.008, door_h - 0.160)),
        origin=Origin(xyz=(-door_w / 2.0 - 0.010, -door_t / 2.0 - 0.004, door_z)),
        material=Material("slightly_recessed_door_1", color=(0.50, 0.56, 0.58, 1.0)),
        name="raised_field",
    )
    door_1.visual(
        Box((0.025, 0.004, door_h - 0.030)),
        origin=Origin(xyz=(-door_w + 0.006, -door_t / 2.0 + 0.002, door_z)),
        material=gasket,
        name="seam_shadow",
    )
    door_1.visual(
        Box((0.020, 0.008, door_h - 0.080)),
        origin=Origin(xyz=(-0.020, -door_t / 2.0 - 0.004, door_z)),
        material=hinge_mat,
        name="hinge_leaf",
    )
    door_1.visual(
        Box((0.095, 0.012, 0.120)),
        origin=Origin(xyz=(-door_w + 0.135, -door_t / 2.0 - 0.006, 1.20)),
        material=latch_mat,
        name="latch_keeper_plate",
    )
    for zoff, suffix in ((0.040, "upper"), (-0.040, "lower")):
        door_1.visual(
            Box((0.085, 0.030, 0.018)),
            origin=Origin(xyz=(-door_w + 0.135, -door_t / 2.0 - 0.027, 1.20 + zoff)),
            material=latch_mat,
            name=f"keeper_{suffix}_jaw",
        )
    for i, (x, mat) in enumerate(((0.0, red), (-0.055, green), (-0.110, green))):
        door_1.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(-0.270 + x, -door_t / 2.0 - 0.003, 1.60), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name=f"indicator_{i}",
        )
    for i in range(6):
        door_1.visual(
            Box((0.180, 0.004, 0.010)),
            origin=Origin(xyz=(-0.300, -door_t / 2.0 - 0.002, 0.50 + i * 0.032)),
            material=gasket,
            name=f"vent_slot_1_{i}",
        )

    swing_bar = model.part("swing_bar")
    swing_bar.visual(
        Cylinder(radius=0.033, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_mat,
        name="pivot_hub",
    )
    swing_bar.visual(
        Box((0.365, 0.018, 0.035)),
        origin=Origin(xyz=(0.170, -0.010, 0.0)),
        material=latch_mat,
        name="draw_bar",
    )
    swing_bar.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.095, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pull_grip",
    )
    swing_bar.visual(
        Box((0.040, 0.026, 0.046)),
        origin=Origin(xyz=(0.330, -0.008, 0.0)),
        material=latch_mat,
        name="bolt_nose",
    )

    model.articulation(
        "body_to_door_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_0,
        origin=Origin(xyz=(-width / 2.0 + 0.006, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_door_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_1,
        origin=Origin(xyz=(width / 2.0 - 0.006, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_0_to_swing_bar",
        ArticulationType.REVOLUTE,
        parent=door_0,
        child=swing_bar,
        origin=Origin(xyz=(door_w - 0.125, -door_t / 2.0 - 0.021, 1.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    swing_bar = object_model.get_part("swing_bar")
    hinge_0 = object_model.get_articulation("body_to_door_0")
    hinge_1 = object_model.get_articulation("body_to_door_1")
    latch = object_model.get_articulation("door_0_to_swing_bar")

    ctx.expect_overlap(door_0, door_1, axes="z", min_overlap=1.70, name="doors span the same tall front opening")
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        max_gap=0.030,
        max_penetration=0.0,
        elem_a="door_panel",
        elem_b="door_panel",
        name="main door panels close with a narrow center gap",
    )
    ctx.expect_overlap(
        door_0,
        door_1,
        axes="x",
        min_overlap=0.010,
        elem_a="center_overlap_lip",
        elem_b="seam_shadow",
        name="overlap lip covers the center seam",
    )
    ctx.expect_overlap(swing_bar, door_1, axes="x", min_overlap=0.045, elem_a="draw_bar", elem_b="latch_keeper_plate", name="draw bar reaches the keeper")
    ctx.expect_gap(
        door_0,
        swing_bar,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        elem_a="latch_pivot_plate",
        elem_b="pivot_hub",
        name="swing bar hub seats on the pivot plate",
    )

    panel_0 = ctx.part_element_world_aabb(door_0, elem="door_panel")
    panel_1 = ctx.part_element_world_aabb(door_1, elem="door_panel")
    if panel_0 is not None and panel_1 is not None:
        width_0 = panel_0[1][0] - panel_0[0][0]
        width_1 = panel_1[1][0] - panel_1[0][0]
        ctx.check("front doors have equal panel widths", abs(width_0 - width_1) < 0.002, details=f"{width_0=}, {width_1=}")
    else:
        ctx.fail("front doors have equal panel widths", "could not measure door panel AABBs")

    closed_0 = ctx.part_world_aabb(door_0)
    closed_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({hinge_0: 1.10, hinge_1: 1.10, latch: 1.0}):
        opened_0 = ctx.part_world_aabb(door_0)
        opened_1 = ctx.part_world_aabb(door_1)
        raised_bar = ctx.part_world_aabb(swing_bar)
    ctx.check(
        "door_0 opens outward from the cabinet front",
        closed_0 is not None and opened_0 is not None and opened_0[0][1] < closed_0[0][1] - 0.20,
        details=f"closed={closed_0}, opened={opened_0}",
    )
    ctx.check(
        "door_1 opens outward from the cabinet front",
        closed_1 is not None and opened_1 is not None and opened_1[0][1] < closed_1[0][1] - 0.20,
        details=f"closed={closed_1}, opened={opened_1}",
    )
    ctx.check(
        "swing bar lifts away from keeper when unlatched",
        raised_bar is not None and closed_0 is not None and raised_bar[1][2] > 1.33,
        details=f"raised_bar={raised_bar}",
    )

    return ctx.report()


object_model = build_object_model()
