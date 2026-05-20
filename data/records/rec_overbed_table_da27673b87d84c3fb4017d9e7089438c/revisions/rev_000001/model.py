from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rect_tube_mesh(width: float, depth: float, height: float, wall: float, name: str):
    outer = cq.Workplane("XY").box(width, depth, height)
    inner = cq.Workplane("XY").box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.01)
    tube = outer.cut(inner)
    return mesh_from_cadquery(tube, name)


def _add_caster_mount(base, x: float, y: float, frame_mat, metal_mat, suffix: str) -> None:
    base.visual(
        Box((0.032, 0.032, 0.010)),
        origin=Origin(xyz=(x, y, 0.130)),
        material=frame_mat,
        name=f"tower_{suffix}",
    )
    base.visual(
        Box((0.050, 0.032, 0.006)),
        origin=Origin(xyz=(x, y, 0.122)),
        material=metal_mat,
        name=f"fork_cap_{suffix}",
    )
    for side_index, side_sign in enumerate((-1.0, 1.0)):
        base.visual(
            Box((0.008, 0.024, 0.054)),
            origin=Origin(xyz=(x + side_sign * 0.020, y, 0.092)),
            material=metal_mat,
            name=f"fork_{suffix}_{side_index}",
        )


def _add_caster_wheel(part, tire_mat, hub_mat) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.055, length=0.026),
        origin=spin_origin,
        material=tire_mat,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.038, length=0.032),
        origin=spin_origin,
        material=hub_mat,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=spin_origin,
        material=hub_mat,
        name="axle_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bariatric_overbed_table")

    frame_paint = model.material("frame_paint", rgba=(0.86, 0.87, 0.89, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.28, 0.30, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    tray_top = model.material("tray_top", rgba=(0.86, 0.83, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.110, 0.760, 0.050)),
        origin=Origin(xyz=(-0.330, 0.000, 0.160)),
        material=frame_paint,
        name="rail_0",
    )
    base.visual(
        Box((0.110, 0.760, 0.050)),
        origin=Origin(xyz=(0.330, 0.000, 0.160)),
        material=frame_paint,
        name="rail_1",
    )
    base.visual(
        Box((0.620, 0.110, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.160)),
        material=frame_paint,
        name="bridge",
    )
    base.visual(
        Box((0.050, 0.060, 0.090)),
        origin=Origin(xyz=(-0.330, 0.180, 0.090)),
        material=dark_trim,
        name="brake_boss_0",
    )
    base.visual(
        Box((0.050, 0.060, 0.090)),
        origin=Origin(xyz=(0.330, 0.180, 0.090)),
        material=dark_trim,
        name="brake_boss_1",
    )

    caster_positions = [
        (-0.330, -0.310),
        (0.330, -0.310),
        (-0.330, 0.310),
        (0.330, 0.310),
    ]
    for index, (x, y) in enumerate(caster_positions):
        _add_caster_mount(base, x, y, frame_paint, dark_trim, str(index))

    outer_post_0 = model.part("outer_post_0")
    outer_post_0.visual(
        Box((0.120, 0.120, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
        material=frame_paint,
        name="foot",
    )
    outer_post_0.visual(
        Box((0.100, 0.080, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.055)),
        material=frame_paint,
        name="shroud",
    )
    outer_post_0.visual(
        Box((0.005, 0.056, 0.520)),
        origin=Origin(xyz=(-0.0355, 0.000, 0.339)),
        material=steel,
        name="wall_0",
    )
    outer_post_0.visual(
        Box((0.005, 0.056, 0.520)),
        origin=Origin(xyz=(0.0355, 0.000, 0.339)),
        material=steel,
        name="wall_1",
    )
    outer_post_0.visual(
        Box((0.070, 0.005, 0.520)),
        origin=Origin(xyz=(0.000, -0.0255, 0.339)),
        material=steel,
        name="wall_2",
    )
    outer_post_0.visual(
        Box((0.070, 0.005, 0.520)),
        origin=Origin(xyz=(0.000, 0.0255, 0.339)),
        material=steel,
        name="wall_3",
    )

    outer_post_1 = model.part("outer_post_1")
    outer_post_1.visual(
        Box((0.120, 0.120, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
        material=frame_paint,
        name="foot",
    )
    outer_post_1.visual(
        Box((0.100, 0.080, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.055)),
        material=frame_paint,
        name="shroud",
    )
    outer_post_1.visual(
        Box((0.005, 0.056, 0.520)),
        origin=Origin(xyz=(-0.0355, 0.000, 0.339)),
        material=steel,
        name="wall_0",
    )
    outer_post_1.visual(
        Box((0.005, 0.056, 0.520)),
        origin=Origin(xyz=(0.0355, 0.000, 0.339)),
        material=steel,
        name="wall_1",
    )
    outer_post_1.visual(
        Box((0.070, 0.005, 0.520)),
        origin=Origin(xyz=(0.000, -0.0255, 0.339)),
        material=steel,
        name="wall_2",
    )
    outer_post_1.visual(
        Box((0.070, 0.005, 0.520)),
        origin=Origin(xyz=(0.000, 0.0255, 0.339)),
        material=steel,
        name="wall_3",
    )

    inner_post_0 = model.part("inner_post_0")
    inner_post_0.visual(
        Box((0.046, 0.032, 0.560)),
        origin=Origin(xyz=(0.000, 0.000, -0.090)),
        material=steel,
        name="mast",
    )
    inner_post_0.visual(
        Box((0.056, 0.040, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.183)),
        material=dark_trim,
        name="top_cap",
    )
    inner_post_0.visual(
        Box((0.020, 0.046, 0.120)),
        origin=Origin(xyz=(-0.023, 0.000, -0.060)),
        material=dark_trim,
        name="guide_0",
    )
    inner_post_0.visual(
        Box((0.020, 0.046, 0.120)),
        origin=Origin(xyz=(0.023, 0.000, -0.060)),
        material=dark_trim,
        name="guide_1",
    )

    inner_post_1 = model.part("inner_post_1")
    inner_post_1.visual(
        Box((0.046, 0.032, 0.560)),
        origin=Origin(xyz=(0.000, 0.000, -0.090)),
        material=steel,
        name="mast",
    )
    inner_post_1.visual(
        Box((0.056, 0.040, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.183)),
        material=dark_trim,
        name="top_cap",
    )
    inner_post_1.visual(
        Box((0.020, 0.046, 0.120)),
        origin=Origin(xyz=(-0.023, 0.000, -0.060)),
        material=dark_trim,
        name="guide_0",
    )
    inner_post_1.visual(
        Box((0.020, 0.046, 0.120)),
        origin=Origin(xyz=(0.023, 0.000, -0.060)),
        material=dark_trim,
        name="guide_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.460, 0.090, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
        material=frame_paint,
        name="crossbeam",
    )
    carriage.visual(
        Box((0.070, 0.050, 0.020)),
        origin=Origin(xyz=(-0.170, 0.000, 0.010)),
        material=dark_trim,
        name="left_saddle",
    )
    carriage.visual(
        Box((0.070, 0.050, 0.020)),
        origin=Origin(xyz=(0.170, 0.000, 0.010)),
        material=dark_trim,
        name="right_saddle",
    )
    carriage.visual(
        Box((0.200, 0.100, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 0.055)),
        material=frame_paint,
        name="hinge_block",
    )
    carriage.visual(
        Box((0.220, 0.200, 0.022)),
        origin=Origin(xyz=(0.000, 0.080, 0.055)),
        material=dark_trim,
        name="tray_mount",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.520),
        origin=Origin(xyz=(0.000, -0.040, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="hinge_pin",
    )

    tray = model.part("tray")
    tray.visual(
        Box((1.020, 0.500, 0.024)),
        origin=Origin(xyz=(0.000, 0.260, 0.000)),
        material=tray_top,
        name="deck",
    )
    tray.visual(
        Box((1.020, 0.018, 0.030)),
        origin=Origin(xyz=(0.000, 0.491, 0.015)),
        material=tray_top,
        name="front_lip",
    )
    tray.visual(
        Box((0.018, 0.500, 0.030)),
        origin=Origin(xyz=(-0.501, 0.260, 0.015)),
        material=tray_top,
        name="side_lip_0",
    )
    tray.visual(
        Box((0.018, 0.500, 0.030)),
        origin=Origin(xyz=(0.501, 0.260, 0.015)),
        material=tray_top,
        name="side_lip_1",
    )
    tray.visual(
        Box((0.300, 0.120, 0.036)),
        origin=Origin(xyz=(0.000, 0.280, -0.030)),
        material=dark_trim,
        name="undertray",
    )
    tray.visual(
        Box((0.220, 0.040, 0.014)),
        origin=Origin(xyz=(0.000, -0.085, -0.018)),
        material=dark_trim,
        name="hinge_leaf",
    )
    tray.visual(
        Box((0.140, 0.100, 0.014)),
        origin=Origin(xyz=(-0.180, -0.040, -0.018)),
        material=dark_trim,
        name="hinge_arm_0",
    )
    tray.visual(
        Box((0.140, 0.100, 0.014)),
        origin=Origin(xyz=(0.180, -0.040, -0.018)),
        material=dark_trim,
        name="hinge_arm_1",
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="pivot",
    )
    release_paddle.visual(
        Box((0.020, 0.012, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, -0.015)),
        material=charcoal,
        name="stem",
    )
    release_paddle.visual(
        Box((0.120, 0.012, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, -0.027)),
        material=charcoal,
        name="paddle",
    )
    release_paddle.visual(
        Cylinder(radius=0.009, length=0.120),
        origin=Origin(xyz=(0.000, 0.006, -0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="grip",
    )

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Cylinder(radius=0.012, length=0.530),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="bar",
    )
    brake_bar.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(-0.285, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="pivot_0",
    )
    brake_bar.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.285, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="pivot_1",
    )
    brake_bar.visual(
        Box((0.440, 0.080, 0.018)),
        origin=Origin(xyz=(0.000, 0.022, -0.015)),
        material=charcoal,
        name="tread",
    )

    for index in range(4):
        caster = model.part(f"caster_{index}")
        _add_caster_wheel(caster, rubber, steel)

    model.articulation(
        "outer_post_mount_0",
        ArticulationType.FIXED,
        parent=base,
        child=outer_post_0,
        origin=Origin(xyz=(-0.170, 0.000, 0.185)),
    )
    model.articulation(
        "outer_post_mount_1",
        ArticulationType.FIXED,
        parent=base,
        child=outer_post_1,
        origin=Origin(xyz=(0.170, 0.000, 0.185)),
    )
    model.articulation(
        "lift_0",
        ArticulationType.PRISMATIC,
        parent=outer_post_0,
        child=inner_post_0,
        origin=Origin(xyz=(0.000, 0.000, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.080, lower=0.000, upper=0.240),
    )
    model.articulation(
        "lift_1",
        ArticulationType.PRISMATIC,
        parent=outer_post_1,
        child=inner_post_1,
        origin=Origin(xyz=(0.000, 0.000, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.080, lower=0.000, upper=0.240),
        mimic=Mimic(joint="lift_0", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "carriage_mount",
        ArticulationType.FIXED,
        parent=inner_post_0,
        child=carriage,
        origin=Origin(xyz=(0.170, 0.000, 0.190)),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.000, -0.040, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.200, lower=0.000, upper=0.720),
    )
    model.articulation(
        "paddle_pivot",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release_paddle,
        origin=Origin(xyz=(0.300, 0.180, -0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.000, lower=-0.600, upper=0.250),
    )
    model.articulation(
        "brake_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=brake_bar,
        origin=Origin(xyz=(0.000, 0.180, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.000, lower=-0.280, upper=0.280),
    )

    for index, (x, y) in enumerate(caster_positions):
        model.articulation(
            f"caster_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=f"caster_{index}",
            origin=Origin(xyz=(x, y, 0.065)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray = object_model.get_part("tray")
    carriage = object_model.get_part("carriage")
    inner_post_0 = object_model.get_part("inner_post_0")
    inner_post_1 = object_model.get_part("inner_post_1")
    outer_post_0 = object_model.get_part("outer_post_0")
    outer_post_1 = object_model.get_part("outer_post_1")
    release_paddle = object_model.get_part("release_paddle")
    brake_bar = object_model.get_part("brake_bar")

    lift_0 = object_model.get_articulation("lift_0")
    tray_tilt = object_model.get_articulation("tray_tilt")
    brake_pivot = object_model.get_articulation("brake_pivot")

    lift_upper = lift_0.motion_limits.upper if lift_0.motion_limits is not None else 0.0
    tilt_upper = tray_tilt.motion_limits.upper if tray_tilt.motion_limits is not None else 0.0
    brake_upper = brake_pivot.motion_limits.upper if brake_pivot.motion_limits is not None else 0.0

    with ctx.pose({lift_0: 0.0, tray_tilt: 0.0}):
        ctx.expect_within(
            inner_post_0,
            outer_post_0,
            axes="xy",
            inner_elem="mast",
            margin=0.0,
            name="left lift post stays centered in its sleeve at rest",
        )
        ctx.expect_within(
            inner_post_1,
            outer_post_1,
            axes="xy",
            inner_elem="mast",
            margin=0.0,
            name="right lift post stays centered in its sleeve at rest",
        )
        ctx.expect_overlap(
            inner_post_0,
            outer_post_0,
            axes="z",
            elem_a="mast",
            min_overlap=0.360,
            name="left lift post remains deeply inserted at rest",
        )
        ctx.expect_overlap(
            inner_post_1,
            outer_post_1,
            axes="z",
            elem_a="mast",
            min_overlap=0.360,
            name="right lift post remains deeply inserted at rest",
        )
        ctx.expect_contact(
            inner_post_0,
            carriage,
            elem_a="top_cap",
            elem_b="left_saddle",
            name="left lift post supports the carriage",
        )
        ctx.expect_contact(
            inner_post_1,
            carriage,
            elem_a="top_cap",
            elem_b="right_saddle",
            name="right lift post supports the carriage",
        )
        ctx.expect_contact(
            tray,
            carriage,
            elem_a="deck",
            elem_b="tray_mount",
            name="tray rests on the carriage when level",
        )
        ctx.expect_gap(
            tray,
            release_paddle,
            axis="z",
            positive_elem="deck",
            negative_elem="paddle",
            min_gap=0.002,
            max_gap=0.022,
            name="release paddle hangs just below the tray",
        )
        tray_rest_pos = ctx.part_world_position(tray)

    with ctx.pose({lift_0: lift_upper, tray_tilt: 0.0}):
        ctx.expect_within(
            inner_post_0,
            outer_post_0,
            axes="xy",
            inner_elem="mast",
            margin=0.0,
            name="left lift post stays centered at full height",
        )
        ctx.expect_within(
            inner_post_1,
            outer_post_1,
            axes="xy",
            inner_elem="mast",
            margin=0.0,
            name="right lift post stays centered at full height",
        )
        ctx.expect_overlap(
            inner_post_0,
            outer_post_0,
            axes="z",
            elem_a="mast",
            min_overlap=0.120,
            name="left lift post retains insertion at full height",
        )
        ctx.expect_overlap(
            inner_post_1,
            outer_post_1,
            axes="z",
            elem_a="mast",
            min_overlap=0.120,
            name="right lift post retains insertion at full height",
        )
        ctx.expect_contact(
            inner_post_1,
            carriage,
            elem_a="top_cap",
            elem_b="right_saddle",
            name="right lift post still supports the carriage at full height",
        )
        tray_high_pos = ctx.part_world_position(tray)
        left_high_pos = ctx.part_world_position(inner_post_0)
        right_high_pos = ctx.part_world_position(inner_post_1)

    ctx.check(
        "tray lifts upward",
        tray_rest_pos is not None
        and tray_high_pos is not None
        and tray_high_pos[2] > tray_rest_pos[2] + 0.20,
        details=f"rest={tray_rest_pos}, raised={tray_high_pos}",
    )
    ctx.check(
        "twin lift posts stay matched",
        left_high_pos is not None
        and right_high_pos is not None
        and abs(left_high_pos[2] - right_high_pos[2]) < 1e-6,
        details=f"left={left_high_pos}, right={right_high_pos}",
    )

    with ctx.pose({lift_0: lift_upper, tray_tilt: 0.0}):
        level_deck_aabb = ctx.part_element_world_aabb(tray, elem="deck")
    with ctx.pose({lift_0: lift_upper, tray_tilt: tilt_upper}):
        tilted_deck_aabb = ctx.part_element_world_aabb(tray, elem="deck")

    ctx.check(
        "tray tilts upward at the front edge",
        level_deck_aabb is not None
        and tilted_deck_aabb is not None
        and tilted_deck_aabb[1][2] > level_deck_aabb[1][2] + 0.14,
        details=f"level={level_deck_aabb}, tilted={tilted_deck_aabb}",
    )

    with ctx.pose({brake_pivot: 0.0}):
        brake_rest_aabb = ctx.part_element_world_aabb(brake_bar, elem="tread")
    with ctx.pose({brake_pivot: brake_upper}):
        brake_pressed_aabb = ctx.part_element_world_aabb(brake_bar, elem="tread")

    ctx.check(
        "brake bar rocks on its pivots",
        brake_rest_aabb is not None
        and brake_pressed_aabb is not None
        and brake_pressed_aabb[1][2] > brake_rest_aabb[1][2] + 0.005,
        details=f"rest={brake_rest_aabb}, pressed={brake_pressed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
