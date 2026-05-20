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
    model = ArticulatedObject(name="creator_monitor")

    shell_dark = model.material("shell_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.22, 0.24, 0.27, 1.0))
    glass = model.material("glass", rgba=(0.08, 0.11, 0.13, 0.72))
    stand_dark = model.material("stand_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    stand_mid = model.material("stand_mid", rgba=(0.28, 0.30, 0.33, 1.0))
    button_dark = model.material("button_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    label_grey = model.material("label_grey", rgba=(0.36, 0.38, 0.40, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.310, 0.225, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_dark,
        name="base_plate",
    )
    stand.visual(
        Box((0.102, 0.010, 0.220)),
        origin=Origin(xyz=(0.0, 0.021, 0.128)),
        material=stand_mid,
        name="column_back",
    )
    stand.visual(
        Box((0.086, 0.010, 0.220)),
        origin=Origin(xyz=(0.0, -0.021, 0.128)),
        material=stand_mid,
        name="column_front",
    )
    stand.visual(
        Box((0.010, 0.052, 0.220)),
        origin=Origin(xyz=(-0.046, 0.0, 0.128)),
        material=stand_mid,
        name="column_side_0",
    )
    stand.visual(
        Box((0.010, 0.052, 0.220)),
        origin=Origin(xyz=(0.046, 0.0, 0.128)),
        material=stand_mid,
        name="column_side_1",
    )
    stand.visual(
        Box((0.118, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.022, 0.245)),
        material=stand_dark,
        name="column_crown",
    )
    stand.visual(
        Box((0.076, 0.018, 0.072)),
        origin=Origin(xyz=(0.0, -0.026, 0.103)),
        material=stand_dark,
        name="front_rib",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.060, 0.028, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=stand_mid,
        name="inner_mast",
    )
    mast.visual(
        Box((0.110, 0.048, 0.038)),
        origin=Origin(xyz=(0.0, 0.038, 0.259)),
        material=stand_dark,
        name="head_block",
    )
    mast.visual(
        Box((0.016, 0.040, 0.050)),
        origin=Origin(xyz=(-0.032, 0.016, 0.225)),
        material=stand_dark,
        name="head_brace_0",
    )
    mast.visual(
        Box((0.016, 0.040, 0.050)),
        origin=Origin(xyz=(0.032, 0.016, 0.225)),
        material=stand_dark,
        name="head_brace_1",
    )

    model.articulation(
        "stand_to_mast",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.16, lower=0.0, upper=0.090),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_dark,
        name="tilt_barrel",
    )
    carriage.visual(
        Box((0.132, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=stand_dark,
        name="hinge_bridge",
    )
    carriage.visual(
        Box((0.022, 0.060, 0.080)),
        origin=Origin(xyz=(-0.066, -0.014, 0.0)),
        material=stand_mid,
        name="arm_0",
    )
    carriage.visual(
        Box((0.022, 0.060, 0.080)),
        origin=Origin(xyz=(0.066, -0.014, 0.0)),
        material=stand_mid,
        name="arm_1",
    )
    carriage.visual(
        Box((0.110, 0.010, 0.080)),
        origin=Origin(xyz=(0.0, -0.044, 0.0)),
        material=stand_dark,
        name="yoke_bridge",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.038, 0.300)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=math.radians(-5.0),
            upper=math.radians(23.0),
        ),
    )

    pivot = model.part("pivot")
    pivot.visual(
        Box((0.096, 0.006, 0.104)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=stand_mid,
        name="pivot_plate",
    )
    pivot.visual(
        Box((0.132, 0.010, 0.088)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material=stand_dark,
        name="pivot_carrier",
    )
    pivot.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_mid,
        name="pivot_hub",
    )

    model.articulation(
        "carriage_to_pivot",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=pivot,
        origin=Origin(xyz=(0.0, -0.090, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.5),
    )

    display = model.part("display")
    display.visual(
        Box((0.710, 0.032, 0.430)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=shell_mid,
        name="rear_shell",
    )
    display.visual(
        Box((0.300, 0.018, 0.190)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=shell_dark,
        name="back_bulge",
    )
    display.visual(
        Box((0.126, 0.012, 0.126)),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=shell_mid,
        name="mount_pad",
    )
    display.visual(
        Box((0.710, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.040, 0.206)),
        material=shell_dark,
        name="top_bezel",
    )
    display.visual(
        Box((0.018, 0.014, 0.362)),
        origin=Origin(xyz=(-0.346, -0.040, 0.016)),
        material=shell_dark,
        name="side_bezel_0",
    )
    display.visual(
        Box((0.018, 0.014, 0.362)),
        origin=Origin(xyz=(0.346, -0.040, 0.016)),
        material=shell_dark,
        name="side_bezel_1",
    )
    display.visual(
        Box((0.710, 0.016, 0.050)),
        origin=Origin(xyz=(0.0, -0.039, -0.190)),
        material=shell_dark,
        name="bottom_bezel",
    )
    display.visual(
        Box((0.676, 0.0025, 0.364)),
        origin=Origin(xyz=(0.0, -0.046, 0.016)),
        material=glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.170, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, -0.0475, -0.206)),
        material=label_grey,
        name="logo_strip",
    )

    model.articulation(
        "pivot_to_display",
        ArticulationType.FIXED,
        parent=pivot,
        child=display,
        origin=Origin(),
    )

    button_x_positions = (0.172, 0.200, 0.228, 0.256)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.016, 0.003, 0.006)),
            origin=Origin(xyz=(0.0, -0.0015, 0.0)),
            material=button_dark,
            name="button_cap",
        )
        button.visual(
            Box((0.009, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=button_dark,
            name="button_stem",
        )
        model.articulation(
            f"display_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x_pos, -0.049, -0.206)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0018,
            ),
        )

    return model


def _aabb_dims(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        upper[0] - lower[0],
        upper[1] - lower[1],
        upper[2] - lower[2],
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) * 0.5,
        (lower[1] + upper[1]) * 0.5,
        (lower[2] + upper[2]) * 0.5,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    mast = object_model.get_part("mast")
    display = object_model.get_part("display")
    column_joint = object_model.get_articulation("stand_to_mast")
    tilt_joint = object_model.get_articulation("mast_to_carriage")
    swivel_joint = object_model.get_articulation("carriage_to_pivot")
    button_joint = object_model.get_articulation("display_to_button_0")
    button = object_model.get_part("button_0")

    column_limits = column_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits

    if column_limits is not None and column_limits.upper is not None:
        with ctx.pose({column_joint: column_limits.upper}):
            ctx.expect_origin_distance(
                mast,
                stand,
                axes="xy",
                max_dist=0.002,
                name="mast stays centered inside shroud",
            )
            ctx.expect_overlap(
                mast,
                stand,
                axes="z",
                elem_a="inner_mast",
                elem_b="column_back",
                min_overlap=0.080,
                name="mast retains insertion at max height",
            )

        rest_display_pos = ctx.part_world_position(display)
        with ctx.pose({column_joint: column_limits.upper}):
            raised_display_pos = ctx.part_world_position(display)
        ctx.check(
            "display rises with height adjustment",
            rest_display_pos is not None
            and raised_display_pos is not None
            and raised_display_pos[2] > rest_display_pos[2] + 0.05,
            details=f"rest={rest_display_pos}, raised={raised_display_pos}",
        )

    landscape_aabb = ctx.part_world_aabb(display)
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        portrait_aabb = ctx.part_world_aabb(display)
    landscape_dims = _aabb_dims(landscape_aabb)
    portrait_dims = _aabb_dims(portrait_aabb)
    ctx.check(
        "portrait rotation swaps width and height",
        landscape_dims is not None
        and portrait_dims is not None
        and landscape_dims[0] > landscape_dims[2]
        and portrait_dims[2] > portrait_dims[0],
        details=f"landscape_dims={landscape_dims}, portrait_dims={portrait_dims}",
    )

    if tilt_limits is not None and tilt_limits.upper is not None:
        top_bezel_rest = ctx.part_element_world_aabb(display, elem="top_bezel")
        with ctx.pose({tilt_joint: tilt_limits.upper}):
            top_bezel_tilted = ctx.part_element_world_aabb(display, elem="top_bezel")
        rest_center = _aabb_center(top_bezel_rest)
        tilted_center = _aabb_center(top_bezel_tilted)
        ctx.check(
            "positive tilt moves top edge rearward",
            rest_center is not None
            and tilted_center is not None
            and tilted_center[1] > rest_center[1] + 0.04,
            details=f"rest_center={rest_center}, tilted_center={tilted_center}",
        )

    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.0018}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "menu button presses inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] > button_rest[1] + 0.001,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
