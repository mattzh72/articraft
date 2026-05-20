from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bariatric_overbed_table")

    steel = model.material("steel", rgba=(0.72, 0.75, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.36, 0.39, 0.43, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    tray_top = model.material("tray_top", rgba=(0.87, 0.85, 0.79, 1.0))
    pedal = model.material("pedal", rgba=(0.83, 0.15, 0.13, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.10, 0.72, 0.05)),
        origin=Origin(xyz=(-0.34, 0.0, 0.15)),
        material=dark_steel,
        name="rail_0",
    )
    base_frame.visual(
        Box((0.10, 0.72, 0.05)),
        origin=Origin(xyz=(0.34, 0.0, 0.15)),
        material=dark_steel,
        name="rail_1",
    )
    base_frame.visual(
        Box((0.58, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_steel,
        name="cross_beam",
    )

    post_x_positions = (-0.23, 0.23)
    for index, post_x in enumerate(post_x_positions):
        outer_column = model.part(f"outer_column_{index}")
        outer_column.visual(
            Box((0.12, 0.12, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, 0.0175)),
            material=steel,
            name="base_collar",
        )
        outer_column.visual(
            Box((0.012, 0.082, 0.46)),
            origin=Origin(xyz=(-0.035, 0.0, 0.23)),
            material=steel,
            name="wall_0",
        )
        outer_column.visual(
            Box((0.012, 0.082, 0.46)),
            origin=Origin(xyz=(0.035, 0.0, 0.23)),
            material=steel,
            name="wall_1",
        )
        outer_column.visual(
            Box((0.058, 0.012, 0.46)),
            origin=Origin(xyz=(0.0, -0.035, 0.23)),
            material=steel,
            name="wall_2",
        )
        outer_column.visual(
            Box((0.058, 0.012, 0.46)),
            origin=Origin(xyz=(0.0, 0.035, 0.23)),
            material=steel,
            name="wall_3",
        )
        outer_column.visual(
            Box((0.01, 0.10, 0.04)),
            origin=Origin(xyz=(-0.045, 0.0, 0.46)),
            material=steel,
            name="top_ring_0",
        )
        outer_column.visual(
            Box((0.01, 0.10, 0.04)),
            origin=Origin(xyz=(0.045, 0.0, 0.46)),
            material=steel,
            name="top_ring_1",
        )
        outer_column.visual(
            Box((0.058, 0.012, 0.04)),
            origin=Origin(xyz=(0.0, -0.044, 0.46)),
            material=steel,
            name="top_ring_2",
        )
        outer_column.visual(
            Box((0.058, 0.012, 0.04)),
            origin=Origin(xyz=(0.0, 0.044, 0.46)),
            material=steel,
            name="top_ring_3",
        )
        model.articulation(
            f"column_mount_{index}",
            ArticulationType.FIXED,
            parent=base_frame,
            child=outer_column,
            origin=Origin(xyz=(post_x, 0.0, 0.175)),
        )

        inner_post = model.part(f"inner_post_{index}")
        inner_post.visual(
            Box((0.058, 0.058, 0.52)),
            origin=Origin(xyz=(0.0, 0.0, -0.04)),
            material=charcoal,
            name="mast",
        )
        inner_post.visual(
            Box((0.09, 0.12, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, 0.2175)),
            material=steel,
            name="top_head",
        )

        articulation_kwargs = {}
        if index == 1:
            articulation_kwargs["mimic"] = Mimic("lift_0")

        model.articulation(
            f"lift_{index}",
            ArticulationType.PRISMATIC,
            parent=outer_column,
            child=inner_post,
            origin=Origin(xyz=(0.0, 0.0, 0.48)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=180.0,
                velocity=0.20,
                lower=0.0,
                upper=0.18,
            ),
            **articulation_kwargs,
        )

    tray_support = model.part("tray_support")
    tray_support.visual(
        Box((0.10, 0.14, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=steel,
        name="left_saddle",
    )
    tray_support.visual(
        Box((0.10, 0.14, 0.045)),
        origin=Origin(xyz=(0.46, 0.0, 0.0225)),
        material=steel,
        name="right_saddle",
    )
    tray_support.visual(
        Box((0.52, 0.08, 0.04)),
        origin=Origin(xyz=(0.23, 0.0, 0.032)),
        material=steel,
        name="bridge",
    )
    tray_support.visual(
        Box((0.14, 0.18, 0.035)),
        origin=Origin(xyz=(0.23, -0.09, 0.047)),
        material=steel,
        name="hinge_rib",
    )
    tray_support.visual(
        Box((0.30, 0.10, 0.05)),
        origin=Origin(xyz=(0.23, -0.18, 0.075)),
        material=steel,
        name="hinge_block",
    )
    tray_support.visual(
        Box((0.08, 0.04, 0.018)),
        origin=Origin(xyz=(0.23, 0.05, 0.049)),
        material=steel,
        name="paddle_mount",
    )
    model.articulation(
        "left_post_to_support",
        ArticulationType.FIXED,
        parent="inner_post_0",
        child=tray_support,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((1.10, 0.50, 0.024)),
        origin=Origin(xyz=(0.0, 0.25, 0.017)),
        material=tray_top,
        name="panel",
    )
    tray.visual(
        Box((1.10, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.491, 0.029)),
        material=tray_top,
        name="front_rim",
    )
    tray.visual(
        Box((1.10, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.009, 0.026)),
        material=tray_top,
        name="rear_rim",
    )
    tray.visual(
        Box((0.018, 0.50, 0.024)),
        origin=Origin(xyz=(-0.541, 0.25, 0.029)),
        material=tray_top,
        name="side_rim_0",
    )
    tray.visual(
        Box((0.018, 0.50, 0.024)),
        origin=Origin(xyz=(0.541, 0.25, 0.029)),
        material=tray_top,
        name="side_rim_1",
    )
    tray.visual(
        Box((0.82, 0.10, 0.028)),
        origin=Origin(xyz=(0.0, 0.25, -0.003)),
        material=steel,
        name="stiffener",
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=tray_support,
        child=tray,
        origin=Origin(xyz=(0.23, -0.18, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=0.0,
            upper=0.72,
        ),
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Box((0.06, 0.035, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="paddle_arm",
    )
    release_paddle.visual(
        Box((0.32, 0.055, 0.012)),
        origin=Origin(xyz=(0.0, 0.042, -0.024)),
        material=pedal,
        name="paddle_pad",
    )
    model.articulation(
        "paddle_hinge",
        ArticulationType.REVOLUTE,
        parent=tray_support,
        child=release_paddle,
        origin=Origin(xyz=(0.23, 0.06, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=0.55,
        ),
    )

    caster_positions = (
        (-0.34, -0.31),
        (0.34, -0.31),
        (-0.34, 0.31),
        (0.34, 0.31),
    )
    for index, (caster_x, caster_y) in enumerate(caster_positions):
        caster_fork = model.part(f"caster_fork_{index}")
        caster_fork.visual(
            Box((0.034, 0.038, 0.02)),
            origin=Origin(xyz=(0.0, 0.0, -0.01)),
            material=steel,
            name="stem",
        )
        caster_fork.visual(
            Box((0.068, 0.11, 0.01)),
            origin=Origin(xyz=(0.0, 0.0, -0.01)),
            material=steel,
            name="yoke_bridge",
        )
        caster_fork.visual(
            Box((0.028, 0.045, 0.012)),
            origin=Origin(xyz=(0.0, -0.0725, -0.009)),
            material=steel,
            name="brake_lug",
        )
        caster_fork.visual(
            Box((0.008, 0.034, 0.112)),
            origin=Origin(xyz=(-0.023, 0.0, -0.071)),
            material=steel,
            name="yoke_leg_0",
        )
        caster_fork.visual(
            Box((0.008, 0.034, 0.112)),
            origin=Origin(xyz=(0.023, 0.0, -0.071)),
            material=steel,
            name="yoke_leg_1",
        )
        model.articulation(
            f"caster_mount_{index}",
            ArticulationType.FIXED,
            parent=base_frame,
            child=caster_fork,
            origin=Origin(xyz=(caster_x, caster_y, 0.125)),
        )

        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.055, length=0.034),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=charcoal,
            name="tire",
        )
        wheel.visual(
            Box((0.012, 0.012, 0.02)),
            origin=Origin(xyz=(0.0, 0.0, 0.033)),
            material=steel,
            name="hub_marker",
        )
        model.articulation(
            f"wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=caster_fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.07)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=25.0),
        )

        lock_tab = model.part(f"lock_tab_{index}")
        lock_tab.visual(
            Box((0.024, 0.018, 0.03)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_boss",
        )
        lock_tab.visual(
            Box((0.018, 0.028, 0.018)),
            origin=Origin(xyz=(0.0, -0.02, -0.012)),
            material=dark_steel,
            name="tab_arm",
        )
        lock_tab.visual(
            Box((0.05, 0.024, 0.008)),
            origin=Origin(xyz=(0.0, -0.046, -0.016)),
            material=pedal,
            name="tab_pad",
        )
        model.articulation(
            f"lock_hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=caster_fork,
            child=lock_tab,
            origin=Origin(xyz=(0.0, -0.08, -0.03)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=4.0,
                lower=0.0,
                upper=0.6,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_frame = object_model.get_part("base_frame")
    left_outer = object_model.get_part("outer_column_0")
    right_outer = object_model.get_part("outer_column_1")
    left_inner = object_model.get_part("inner_post_0")
    right_inner = object_model.get_part("inner_post_1")
    tray_support = object_model.get_part("tray_support")
    tray = object_model.get_part("tray")
    release_paddle = object_model.get_part("release_paddle")

    lift_0 = object_model.get_articulation("lift_0")
    tray_tilt = object_model.get_articulation("tray_tilt")
    paddle_hinge = object_model.get_articulation("paddle_hinge")
    wheel_spin_0 = object_model.get_articulation("wheel_spin_0")
    lock_hinge_0 = object_model.get_articulation("lock_hinge_0")

    lift_upper = 0.18
    tray_upper = 0.72
    paddle_upper = 0.55
    lock_upper = 0.6

    ctx.expect_origin_gap(
        tray,
        base_frame,
        axis="z",
        min_gap=0.85,
        name="tray sits at patient-room working height",
    )

    for outer_column, inner_post, side in (
        (left_outer, left_inner, "left"),
        (right_outer, right_inner, "right"),
    ):
        ctx.expect_within(
            inner_post,
            outer_column,
            axes="xy",
            margin=0.012,
            name=f"{side} mast stays centered in the sleeve at rest",
        )

    ctx.expect_contact(
        left_inner,
        tray_support,
        elem_a="top_head",
        elem_b="left_saddle",
        name="left post bears the tray support at rest",
    )
    ctx.expect_contact(
        right_inner,
        tray_support,
        elem_a="top_head",
        elem_b="right_saddle",
        name="right post bears the tray support at rest",
    )

    left_rest = ctx.part_world_position(left_inner)
    right_rest = ctx.part_world_position(right_inner)
    with ctx.pose({lift_0: lift_upper}):
        ctx.expect_within(
            left_inner,
            left_outer,
            axes="xy",
            margin=0.012,
            name="left mast stays centered in the sleeve at full lift",
        )
        ctx.expect_within(
            right_inner,
            right_outer,
            axes="xy",
            margin=0.012,
            name="right mast stays centered in the sleeve at full lift",
        )
        ctx.expect_overlap(
            left_inner,
            left_outer,
            axes="z",
            min_overlap=0.10,
            name="left mast remains inserted at full lift",
        )
        ctx.expect_overlap(
            right_inner,
            right_outer,
            axes="z",
            min_overlap=0.10,
            name="right mast remains inserted at full lift",
        )
        ctx.expect_contact(
            left_inner,
            tray_support,
            elem_a="top_head",
            elem_b="left_saddle",
            name="left post still bears the tray support at full lift",
        )
        ctx.expect_contact(
            right_inner,
            tray_support,
            elem_a="top_head",
            elem_b="right_saddle",
            name="right post still bears the tray support at full lift",
        )
        left_raised = ctx.part_world_position(left_inner)
        right_raised = ctx.part_world_position(right_inner)

    left_travel = None if left_rest is None or left_raised is None else left_raised[2] - left_rest[2]
    right_travel = None if right_rest is None or right_raised is None else right_raised[2] - right_rest[2]
    ctx.check(
        "twin lift posts rise together",
        left_travel is not None
        and right_travel is not None
        and left_travel > 0.15
        and abs(left_travel - right_travel) < 1e-5,
        details=f"left_travel={left_travel}, right_travel={right_travel}",
    )

    front_rest = ctx.part_element_world_aabb(tray, elem="front_rim")
    with ctx.pose({tray_tilt: tray_upper}):
        front_tilted = ctx.part_element_world_aabb(tray, elem="front_rim")
    ctx.check(
        "tray front edge rises when tilted",
        front_rest is not None
        and front_tilted is not None
        and front_tilted[0][2] > front_rest[0][2] + 0.12,
        details=f"front_rest={front_rest}, front_tilted={front_tilted}",
    )

    paddle_rest = ctx.part_element_world_aabb(release_paddle, elem="paddle_pad")
    with ctx.pose({paddle_hinge: paddle_upper}):
        paddle_open = ctx.part_element_world_aabb(release_paddle, elem="paddle_pad")
    ctx.check(
        "release paddle swings upward below the tray",
        paddle_rest is not None
        and paddle_open is not None
        and paddle_open[1][2] > paddle_rest[1][2] + 0.01,
        details=f"paddle_rest={paddle_rest}, paddle_open={paddle_open}",
    )

    wheel_rest = ctx.part_element_world_aabb("wheel_0", elem="hub_marker")
    with ctx.pose({wheel_spin_0: pi / 2.0}):
        wheel_spun = ctx.part_element_world_aabb("wheel_0", elem="hub_marker")
    ctx.check(
        "caster wheel rotates on its axle",
        wheel_rest is not None
        and wheel_spun is not None
        and abs(wheel_spun[0][2] - wheel_rest[0][2]) > 0.02,
        details=f"wheel_rest={wheel_rest}, wheel_spun={wheel_spun}",
    )

    tab_rest = ctx.part_element_world_aabb("lock_tab_0", elem="tab_pad")
    with ctx.pose({lock_hinge_0: lock_upper}):
        tab_locked = ctx.part_element_world_aabb("lock_tab_0", elem="tab_pad")
    ctx.check(
        "caster lock tab pivots on its local hinge",
        tab_rest is not None
        and tab_locked is not None
        and tab_locked[0][2] < tab_rest[0][2] - 0.005,
        details=f"tab_rest={tab_rest}, tab_locked={tab_locked}",
    )

    return ctx.report()


object_model = build_object_model()
