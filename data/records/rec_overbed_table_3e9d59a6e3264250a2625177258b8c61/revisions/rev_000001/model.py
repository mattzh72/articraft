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


def _add_square_tube(
    part,
    *,
    center: tuple[float, float],
    bottom_z: float,
    height: float,
    outer: float,
    wall: float,
    material,
    name_prefix: str,
) -> None:
    cx, cy = center
    z = bottom_z + height * 0.5
    half_outer = outer * 0.5
    side_depth = max(outer - 2.0 * wall, wall)

    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(cx, cy + half_outer - wall * 0.5, z)),
        material=material,
        name=f"{name_prefix}_front",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(cx, cy - half_outer + wall * 0.5, z)),
        material=material,
        name=f"{name_prefix}_rear",
    )
    part.visual(
        Box((wall, side_depth, height)),
        origin=Origin(xyz=(cx - half_outer + wall * 0.5, cy, z)),
        material=material,
        name=f"{name_prefix}_inner",
    )
    part.visual(
        Box((wall, side_depth, height)),
        origin=Origin(xyz=(cx + half_outer - wall * 0.5, cy, z)),
        material=material,
        name=f"{name_prefix}_outer",
    )


def _add_front_fork(part, *, x: float, y: float, material, prefix: str) -> None:
    part.visual(
        Box((0.035, 0.055, 0.014)),
        origin=Origin(xyz=(x, y, 0.0645)),
        material=material,
        name=f"{prefix}_crown",
    )
    for side in (-1.0, 1.0):
        part.visual(
            Box((0.006, 0.024, 0.027)),
            origin=Origin(xyz=(x + side * 0.015, y, 0.044)),
            material=material,
            name=f"{prefix}_leg_{int((side + 1.0) * 0.5)}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overbed_table")

    frame_paint = model.material("frame_paint", rgba=(0.78, 0.78, 0.76, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.27, 0.28, 0.30, 1.0))
    laminate = model.material("laminate", rgba=(0.88, 0.83, 0.71, 1.0))
    laminate_edge = model.material("laminate_edge", rgba=(0.48, 0.35, 0.23, 1.0))
    caster_hub = model.material("caster_hub", rgba=(0.63, 0.65, 0.68, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))

    rail_x = 0.165
    rail_y = 0.0
    rail_z = 0.075
    rail_size = (0.055, 0.74, 0.035)
    post_xy = (rail_x, -0.18)
    sleeve_bottom_z = 0.1225
    sleeve_height = 0.35
    sleeve_top_z = sleeve_bottom_z + sleeve_height
    wheel_y = 0.315
    wheel_z = 0.027

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box(rail_size),
        origin=Origin(xyz=(-rail_x, rail_y, rail_z)),
        material=frame_paint,
        name="rail_0",
    )
    base_frame.visual(
        Box(rail_size),
        origin=Origin(xyz=(rail_x, rail_y, rail_z)),
        material=frame_paint,
        name="rail_1",
    )
    base_frame.visual(
        Box((0.385, 0.070, 0.035)),
        origin=Origin(xyz=(0.0, -0.320, rail_z)),
        material=frame_paint,
        name="rear_crossmember",
    )
    for index, x in enumerate((-rail_x, rail_x)):
        base_frame.visual(
            Box((0.035, 0.030, 0.0475)),
            origin=Origin(xyz=(x, -0.355, 0.03375)),
            material=frame_paint,
            name=f"rear_leg_{index}",
        )
        base_frame.visual(
            Box((0.050, 0.040, 0.010)),
            origin=Origin(xyz=(x, -0.355, 0.005)),
            material=trim_dark,
            name=f"rear_glide_{index}",
        )
    base_frame.visual(
        Box((0.100, 0.100, 0.030)),
        origin=Origin(xyz=(post_xy[0], post_xy[1], 0.1075)),
        material=frame_paint,
        name="post_shoe",
    )
    _add_square_tube(
        base_frame,
        center=post_xy,
        bottom_z=sleeve_bottom_z,
        height=sleeve_height,
        outer=0.065,
        wall=0.008,
        material=frame_paint,
        name_prefix="sleeve",
    )
    _add_square_tube(
        base_frame,
        center=post_xy,
        bottom_z=0.410,
        height=0.070,
        outer=0.080,
        wall=0.008,
        material=trim_dark,
        name_prefix="collar",
    )
    base_frame.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.216, post_xy[1], 0.445), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="collar_hub",
    )

    _add_front_fork(base_frame, x=-rail_x, y=wheel_y, material=frame_paint, prefix="fork_0")
    _add_front_fork(base_frame, x=rail_x, y=wheel_y, material=frame_paint, prefix="fork_1")

    for x_sign, name in ((-1.0, "bar_bracket_0"), (1.0, "bar_bracket_1")):
        x = x_sign * 0.1335
        base_frame.visual(
            Box((0.008, 0.028, 0.018)),
            origin=Origin(xyz=(x, 0.080, 0.067)),
            material=trim_dark,
            name=f"{name}_upper",
        )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Box((0.041, 0.041, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=trim_dark,
        name="mast",
    )
    inner_post.visual(
        Box((0.049, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        material=frame_paint,
        name="guide_x",
    )
    inner_post.visual(
        Box((0.010, 0.049, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        material=frame_paint,
        name="guide_y",
    )

    top = model.part("top")
    top.visual(
        Box((0.072, 0.072, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=trim_dark,
        name="top_mount",
    )
    top.visual(
        Box((0.050, 0.580, 0.040)),
        origin=Origin(xyz=(0.0, 0.290, 0.040)),
        material=frame_paint,
        name="forward_beam",
    )
    top.visual(
        Box((0.320, 0.050, 0.040)),
        origin=Origin(xyz=(-0.160, 0.330, 0.040)),
        material=frame_paint,
        name="cross_beam",
    )
    top.visual(
        Box((0.240, 0.180, 0.028)),
        origin=Origin(xyz=(-0.120, 0.330, 0.069)),
        material=frame_paint,
        name="undersupport",
    )
    top.visual(
        Box((0.782, 0.422, 0.018)),
        origin=Origin(xyz=(-0.240, 0.330, 0.088)),
        material=laminate_edge,
        name="top_edge",
    )
    top.visual(
        Box((0.780, 0.420, 0.028)),
        origin=Origin(xyz=(-0.240, 0.330, 0.097)),
        material=laminate,
        name="top_panel",
    )

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="shaft",
    )
    collar_knob.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    collar_knob.visual(
        Box((0.010, 0.032, 0.008)),
        origin=Origin(xyz=(0.023, 0.0, 0.0)),
        material=knob_black,
        name="knob_wing_y",
    )
    collar_knob.visual(
        Box((0.010, 0.008, 0.032)),
        origin=Origin(xyz=(0.023, 0.0, 0.0)),
        material=knob_black,
        name="knob_wing_z",
    )

    for index, x in enumerate((-rail_x, rail_x)):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.027, length=0.022),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="wheel",
        )
        caster.visual(
            Cylinder(radius=0.014, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=caster_hub,
            name="hub",
        )
        model.articulation(
            f"base_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=base_frame,
            child=caster,
            origin=Origin(xyz=(x, wheel_y, wheel_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Cylinder(radius=0.010, length=0.250),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="brake_tube",
    )
    brake_bar.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(-0.128, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="pivot_0",
    )
    brake_bar.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.128, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="pivot_1",
    )
    brake_bar.visual(
        Box((0.120, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.023, 0.012)),
        material=trim_dark,
        name="pedal",
    )

    model.articulation(
        "base_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=inner_post,
        origin=Origin(xyz=(post_xy[0], post_xy[1], sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.28),
    )
    model.articulation(
        "inner_post_to_top",
        ArticulationType.FIXED,
        parent=inner_post,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )
    model.articulation(
        "base_to_collar_knob",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=collar_knob,
        origin=Origin(xyz=(0.227, post_xy[1], 0.445)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    model.articulation(
        "base_to_brake_bar",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=brake_bar,
        origin=Origin(xyz=(0.0, 0.080, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.35, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_frame = object_model.get_part("base_frame")
    inner_post = object_model.get_part("inner_post")
    top = object_model.get_part("top")
    brake_bar = object_model.get_part("brake_bar")
    slide = object_model.get_articulation("base_to_inner_post")

    limits = slide.motion_limits
    lower = 0.0
    upper = 0.0
    if limits is not None:
        lower = 0.0 if limits.lower is None else limits.lower
        upper = 0.0 if limits.upper is None else limits.upper

    with ctx.pose({slide: lower}):
        low_top = ctx.part_element_world_aabb(top, elem="top_panel")
    with ctx.pose({slide: upper}):
        high_top = ctx.part_element_world_aabb(top, elem="top_panel")
        high_post = ctx.part_world_aabb(inner_post)
        sleeve = ctx.part_element_world_aabb(base_frame, elem="sleeve_front")

    top_rises = (
        low_top is not None
        and high_top is not None
        and high_top[1][2] > low_top[1][2] + 0.24
    )
    ctx.check(
        "top lifts through realistic height range",
        top_rises,
        details=f"low_top={low_top}, high_top={high_top}",
    )

    low_height_ok = low_top is not None and 0.78 <= low_top[1][2] <= 0.86
    ctx.check(
        "low top height reads as home-care furniture scale",
        low_height_ok,
        details=f"low_top={low_top}",
    )

    retained_insertion = (
        high_post is not None
        and sleeve is not None
        and high_post[0][2] < sleeve[1][2] - 0.02
    )
    ctx.check(
        "inner post stays captured in the sleeve at max height",
        retained_insertion,
        details=f"inner_post={high_post}, sleeve={sleeve}",
    )

    bar_aabb = ctx.part_world_aabb(brake_bar)
    brake_bar_ok = (
        bar_aabb is not None
        and bar_aabb[0][0] < -0.12
        and bar_aabb[1][0] > 0.12
        and bar_aabb[1][2] < 0.09
    )
    ctx.check(
        "brake bar spans low across the base rails",
        brake_bar_ok,
        details=f"brake_bar={bar_aabb}",
    )

    for index in (0, 1):
        caster = object_model.get_part(f"caster_{index}")
        caster_pos = ctx.part_world_position(caster)
        ctx.check(
            f"caster_{index} sits at the patient side front",
            caster_pos is not None and caster_pos[1] > 0.28 and caster_pos[2] < 0.05,
            details=f"caster_{index}_position={caster_pos}",
        )

    return ctx.report()


object_model = build_object_model()
