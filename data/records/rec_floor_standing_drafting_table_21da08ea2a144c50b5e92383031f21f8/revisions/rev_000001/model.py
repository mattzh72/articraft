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
    model = ArticulatedObject(name="floor_standing_drafting_table")

    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.015, 1.0))
    nylon = model.material("white_nylon_wear_pads", rgba=(0.86, 0.84, 0.78, 1.0))
    maple = model.material("varnished_maple_board", rgba=(0.74, 0.56, 0.32, 1.0))
    ivory = model.material("warm_drawing_sheet", rgba=(0.96, 0.93, 0.84, 1.0))
    knob_mat = model.material("black_bakelite_knob", rgba=(0.02, 0.018, 0.015, 1.0))

    base = model.part("trestle_base")
    base.visual(
        Box((0.12, 0.82, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="center_floor_rail",
    )
    for y, name in ((0.36, "front_foot"), (-0.36, "rear_foot")):
        base.visual(
            Box((1.10, 0.11, 0.06)),
            origin=Origin(xyz=(0.0, y, 0.03)),
            material=dark_steel,
            name=name,
        )
        base.visual(
            Box((0.16, 0.065, 0.018)),
            origin=Origin(xyz=(0.46, y, 0.009)),
            material=rubber,
            name=f"{name}_pad_0",
        )
        base.visual(
            Box((0.16, 0.065, 0.018)),
            origin=Origin(xyz=(-0.46, y, 0.009)),
            material=rubber,
            name=f"{name}_pad_1",
        )

    base.visual(
        Box((0.10, 0.08, 1.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.5825)),
        material=dark_steel,
        name="upright",
    )
    brace_len = math.sqrt(0.36**2 + 0.48**2)
    brace_angle = math.atan2(0.36, 0.48)
    for y, sign, name in ((0.18, -1.0, "front_brace"), (-0.18, 1.0, "rear_brace")):
        base.visual(
            Box((0.065, 0.045, brace_len)),
            origin=Origin(xyz=(0.0, y, 0.32), rpy=(sign * brace_angle, 0.0, 0.0)),
            material=dark_steel,
            name=name,
        )

    carriage = model.part("height_carriage")
    carriage.visual(
        Box((0.045, 0.20, 0.20)),
        origin=Origin(xyz=(0.0975, 0.0, 0.0)),
        material=satin_steel,
        name="side_plate_0",
    )
    carriage.visual(
        Box((0.045, 0.20, 0.20)),
        origin=Origin(xyz=(-0.0975, 0.0, 0.0)),
        material=satin_steel,
        name="side_plate_1",
    )
    carriage.visual(
        Box((0.24, 0.045, 0.20)),
        origin=Origin(xyz=(0.0, 0.0775, 0.0)),
        material=satin_steel,
        name="front_plate",
    )
    carriage.visual(
        Box((0.24, 0.045, 0.20)),
        origin=Origin(xyz=(0.0, -0.0775, 0.0)),
        material=satin_steel,
        name="rear_plate",
    )
    carriage.visual(
        Box((0.028, 0.070, 0.08)),
        origin=Origin(xyz=(0.064, 0.0, 0.0)),
        material=nylon,
        name="guide_pad_0",
    )
    carriage.visual(
        Box((0.028, 0.070, 0.08)),
        origin=Origin(xyz=(-0.064, 0.0, 0.0)),
        material=nylon,
        name="guide_pad_1",
    )
    for y, name in ((0.059, "guide_pad_2"), (-0.059, "guide_pad_3")):
        carriage.visual(
            Box((0.070, 0.038, 0.08)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=nylon,
            name=name,
        )
    carriage.visual(
        Box((0.070, 0.12, 0.065)),
        origin=Origin(xyz=(0.10, 0.0, 0.1325)),
        material=satin_steel,
        name="top_saddle_0",
    )
    carriage.visual(
        Box((0.070, 0.12, 0.065)),
        origin=Origin(xyz=(-0.10, 0.0, 0.1325)),
        material=satin_steel,
        name="top_saddle_1",
    )

    slide = model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.28),
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Box((1.44, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=dark_steel,
        name="pivot_crossbeam",
    )
    for y, name in ((0.31, "front_frame_rail"), (-0.31, "rear_frame_rail")):
        tilt_frame.visual(
            Box((1.15, 0.05, 0.045)),
            origin=Origin(xyz=(0.0, y, -0.11)),
            material=dark_steel,
            name=name,
        )
    for x, name in ((0.55, "side_frame_rail_0"), (-0.55, "side_frame_rail_1")):
        tilt_frame.visual(
            Box((0.05, 0.66, 0.045)),
            origin=Origin(xyz=(x, 0.0, -0.11)),
            material=dark_steel,
            name=name,
        )
    for x, name in ((0.72, "side_cheek_0"), (-0.72, "side_cheek_1")):
        tilt_frame.visual(
            Box((0.035, 0.20, 0.18)),
            origin=Origin(xyz=(x, 0.0, -0.04)),
            material=dark_steel,
            name=name,
        )
    tilt_frame.visual(
        Cylinder(radius=0.025, length=0.075),
        origin=Origin(xyz=(0.6875, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_pin_0",
    )
    tilt_frame.visual(
        Cylinder(radius=0.025, length=0.075),
        origin=Origin(xyz=(-0.6875, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_pin_1",
    )
    tilt_frame.visual(
        Box((0.060, 0.050, 0.260)),
        origin=Origin(xyz=(0.10, 0.0, -0.205)),
        material=dark_steel,
        name="support_post_0",
    )
    tilt_frame.visual(
        Box((0.060, 0.050, 0.260)),
        origin=Origin(xyz=(-0.10, 0.0, -0.205)),
        material=dark_steel,
        name="support_post_1",
    )
    tilt_frame.visual(
        Cylinder(radius=0.018, length=0.077),
        origin=Origin(xyz=(0.774, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="lock_shaft",
    )

    model.articulation(
        "carriage_to_frame",
        ArticulationType.FIXED,
        parent=carriage,
        child=tilt_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
    )

    surface = model.part("drawing_surface")
    surface.visual(
        Box((1.25, 0.85, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=maple,
        name="board_panel",
    )
    surface.visual(
        Box((1.16, 0.76, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0745)),
        material=ivory,
        name="drawing_sheet",
    )
    for y, name in ((0.425, "front_edge_lip"), (-0.425, "rear_edge_lip")):
        surface.visual(
            Box((1.30, 0.032, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.055)),
            material=maple,
            name=name,
        )
    for x, name in ((0.625, "side_edge_lip_0"), (-0.625, "side_edge_lip_1")):
        surface.visual(
            Box((0.035, 0.85, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=maple,
            name=name,
        )
    for x, name in ((0.655, "pivot_plate_0"), (-0.655, "pivot_plate_1")):
        surface.visual(
            Box((0.050, 0.14, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=satin_steel,
            name=name,
        )
    surface.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.680, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_boss_0",
    )
    surface.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(-0.680, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_boss_1",
    )

    tilt = model.articulation(
        "frame_to_surface",
        ArticulationType.REVOLUTE,
        parent=tilt_frame,
        child=surface,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.7, lower=-0.15, upper=1.10),
    )

    knob = model.part("locking_knob")
    knob.visual(
        Cylinder(radius=0.065, length=0.055),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_mat,
        name="knob_disk",
    )
    knob.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.0335, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_mat,
        name="front_cap",
    )
    for i in range(8):
        theta = i * math.tau / 8.0
        knob.visual(
            Box((0.060, 0.014, 0.026)),
            origin=Origin(
                xyz=(0.0, 0.064 * math.cos(theta), 0.064 * math.sin(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=knob_mat,
            name=f"grip_{i}",
        )

    model.articulation(
        "frame_to_knob",
        ArticulationType.REVOLUTE,
        parent=tilt_frame,
        child=knob,
        origin=Origin(xyz=(0.840, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    # Keep locals referenced so type checkers do not mark the central motions as dead.
    _ = (slide, tilt)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("trestle_base")
    carriage = object_model.get_part("height_carriage")
    frame = object_model.get_part("tilt_frame")
    surface = object_model.get_part("drawing_surface")
    knob = object_model.get_part("locking_knob")
    slide = object_model.get_articulation("base_to_carriage")
    tilt = object_model.get_articulation("frame_to_surface")
    knob_joint = object_model.get_articulation("frame_to_knob")

    ctx.allow_overlap(
        frame,
        surface,
        elem_a="pivot_pin_0",
        elem_b="pivot_boss_0",
        reason="The side pivot pin is intentionally captured inside the drawing board pivot boss.",
    )
    ctx.allow_overlap(
        frame,
        surface,
        elem_a="pivot_pin_1",
        elem_b="pivot_boss_1",
        reason="The side pivot pin is intentionally captured inside the drawing board pivot boss.",
    )

    ctx.expect_contact(
        carriage,
        base,
        elem_a="guide_pad_0",
        elem_b="upright",
        name="carriage side wear pad bears on upright",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a="support_post_0",
        elem_b="top_saddle_0",
        name="tilt frame post is seated on carriage saddle",
    )
    ctx.expect_contact(
        knob,
        frame,
        elem_a="knob_disk",
        elem_b="lock_shaft",
        name="locking knob is seated on its short shaft",
    )
    ctx.expect_overlap(
        frame,
        surface,
        axes="x",
        elem_a="pivot_pin_0",
        elem_b="pivot_boss_0",
        min_overlap=0.020,
        name="right pivot pin remains captured in boss",
    )
    ctx.expect_overlap(
        frame,
        surface,
        axes="x",
        elem_a="pivot_pin_1",
        elem_b="pivot_boss_1",
        min_overlap=0.020,
        name="left pivot pin remains captured in boss",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.28}):
        raised_carriage = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            base,
            elem_a="guide_pad_0",
            elem_b="upright",
            name="raised carriage remains guided by upright",
        )

    ctx.check(
        "height carriage slides upward on central upright",
        rest_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > rest_carriage[2] + 0.25,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )

    rest_board_aabb = ctx.part_element_world_aabb(surface, elem="board_panel")
    with ctx.pose({tilt: 0.85}):
        tilted_board_aabb = ctx.part_element_world_aabb(surface, elem="board_panel")
    ctx.check(
        "drawing surface tilts about horizontal side pivots",
        rest_board_aabb is not None
        and tilted_board_aabb is not None
        and tilted_board_aabb[1][2] > rest_board_aabb[1][2] + 0.20,
        details=f"rest={rest_board_aabb}, tilted={tilted_board_aabb}",
    )

    rest_grip = ctx.part_element_world_aabb(knob, elem="grip_1")
    with ctx.pose({knob_joint: 0.75}):
        turned_grip = ctx.part_element_world_aabb(knob, elem="grip_1")
    ctx.check(
        "locking knob rotates around side shaft",
        rest_grip is not None
        and turned_grip is not None
        and abs(turned_grip[0][2] - rest_grip[0][2]) > 0.010,
        details=f"rest={rest_grip}, turned={turned_grip}",
    )

    return ctx.report()


object_model = build_object_model()
