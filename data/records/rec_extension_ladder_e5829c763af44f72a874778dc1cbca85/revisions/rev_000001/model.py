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


BASE_LENGTH = 3.60
FLY_LENGTH = 3.30
SLIDE_TRAVEL = 1.35


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(horizontal, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_rpy_for_segment(start, end)),
        material=material,
        name=name,
    )


def _add_ladder_section(
    part,
    *,
    rail_center_x: float,
    rail_width: float,
    rail_depth: float,
    rail_center_y: float,
    bottom_z: float,
    length: float,
    rung_zs: list[float],
    rung_length: float,
    rung_radius: float,
    rail_material,
    rung_material,
    prefix: str,
) -> None:
    for side, x_pos in enumerate((-rail_center_x, rail_center_x)):
        part.visual(
            Box((rail_width, rail_depth, length)),
            origin=Origin(xyz=(x_pos, rail_center_y, bottom_z + length * 0.5)),
            material=rail_material,
            name=f"{prefix}_rail_{side}",
        )

    for index, z_pos in enumerate(rung_zs):
        part.visual(
            Cylinder(radius=rung_radius, length=rung_length),
            origin=Origin(
                xyz=(0.0, rail_center_y, z_pos),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=rung_material,
            name=f"{prefix}_rung_{index}",
        )


def _tip_extents(aabb) -> tuple[float | None, float | None]:
    if aabb is None:
        return (None, None)
    return (aabb[1][1], aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_extension_ladder")

    rail_orange = model.material("rail_orange", rgba=(0.92, 0.45, 0.12, 1.0))
    fly_orange = model.material("fly_orange", rgba=(0.97, 0.54, 0.18, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.69, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base_rungs = [0.34 + 0.282 * i for i in range(11)]
    _add_ladder_section(
        base,
        rail_center_x=0.185,
        rail_width=0.052,
        rail_depth=0.030,
        rail_center_y=0.0,
        bottom_z=0.0,
        length=BASE_LENGTH,
        rung_zs=base_rungs,
        rung_length=0.354,
        rung_radius=0.011,
        rail_material=rail_orange,
        rung_material=aluminum,
        prefix="base",
    )
    for side, x_pos in enumerate((-0.195, 0.195)):
        base.visual(
            Box((0.084, 0.042, 0.080)),
            origin=Origin(xyz=(x_pos, 0.002, 0.040)),
            material=rubber,
            name=f"foot_{side}",
        )

    fly = model.part("fly")
    fly_bottom = -1.23
    fly_rungs = [fly_bottom + 0.36 + 0.292 * i for i in range(10)]
    _add_ladder_section(
        fly,
        rail_center_x=0.165,
        rail_width=0.044,
        rail_depth=0.022,
        rail_center_y=0.026,
        bottom_z=fly_bottom,
        length=FLY_LENGTH,
        rung_zs=fly_rungs,
        rung_length=0.320,
        rung_radius=0.0095,
        rail_material=fly_orange,
        rung_material=aluminum,
        prefix="fly",
    )
    fly.visual(
        Box((0.320, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, 0.028, 1.985)),
        material=galvanized,
        name="top_head",
    )
    for side, x_pos in enumerate((-0.090, 0.090)):
        fly.visual(
            Box((0.026, 0.032, 0.070)),
            origin=Origin(xyz=(x_pos, 0.030, 2.035)),
            material=galvanized,
            name=f"hinge_ear_{side}",
        )
    v_bracket = model.part("v_bracket")
    v_bracket.visual(
        Cylinder(radius=0.012, length=0.154),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    v_bracket.visual(
        Box((0.050, 0.016, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=galvanized,
        name="center_gusset",
    )
    _add_tube(
        v_bracket,
        start=(-0.018, 0.0, 0.012),
        end=(-0.168, 0.0, 0.292),
        radius=0.011,
        material=galvanized,
        name="arm_0",
    )
    _add_tube(
        v_bracket,
        start=(0.018, 0.0, 0.012),
        end=(0.168, 0.0, 0.292),
        radius=0.011,
        material=galvanized,
        name="arm_1",
    )
    v_bracket.visual(
        Cylinder(radius=0.016, length=0.075),
        origin=Origin(xyz=(-0.172, 0.0, 0.302), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=rubber,
        name="left_pad",
    )
    v_bracket.visual(
        Cylinder(radius=0.016, length=0.075),
        origin=Origin(xyz=(0.172, 0.0, 0.302), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=rubber,
        name="right_pad",
    )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=250.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "bracket_hinge",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=v_bracket,
        origin=Origin(xyz=(0.0, 0.030, 2.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.00,
            upper=0.0,
            effort=18.0,
            velocity=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    bracket = object_model.get_part("v_bracket")
    fly_slide = object_model.get_articulation("fly_slide")
    bracket_hinge = object_model.get_articulation("bracket_hinge")

    slide_upper = 0.0
    if fly_slide.motion_limits is not None and fly_slide.motion_limits.upper is not None:
        slide_upper = fly_slide.motion_limits.upper

    hinge_lower = 0.0
    if bracket_hinge.motion_limits is not None and bracket_hinge.motion_limits.lower is not None:
        hinge_lower = bracket_hinge.motion_limits.lower

    with ctx.pose({fly_slide: 0.0}):
        ctx.expect_within(
            fly,
            base,
            axes="x",
            margin=0.03,
            name="collapsed fly stays between base rails",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=2.00,
            name="collapsed fly remains deeply nested in the base section",
        )
        collapsed_pos = ctx.part_world_position(fly)

    with ctx.pose({fly_slide: slide_upper}):
        ctx.expect_within(
            fly,
            base,
            axes="x",
            margin=0.03,
            name="extended fly remains centered between base rails",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.80,
            name="extended fly retains insertion in the base section",
        )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        collapsed_pos is not None
        and extended_pos is not None
        and extended_pos[2] > collapsed_pos[2] + 1.2,
        details=f"collapsed={collapsed_pos}, extended={extended_pos}",
    )

    with ctx.pose({bracket_hinge: hinge_lower}):
        stowed_left = ctx.part_element_world_aabb(bracket, elem="left_pad")
        stowed_right = ctx.part_element_world_aabb(bracket, elem="right_pad")

    with ctx.pose({bracket_hinge: 0.0}):
        raised_left = ctx.part_element_world_aabb(bracket, elem="left_pad")
        raised_right = ctx.part_element_world_aabb(bracket, elem="right_pad")

    stowed_y = max(v for v in (_tip_extents(stowed_left)[0], _tip_extents(stowed_right)[0]) if v is not None)
    raised_y = max(v for v in (_tip_extents(raised_left)[0], _tip_extents(raised_right)[0]) if v is not None)
    stowed_z = max(v for v in (_tip_extents(stowed_left)[1], _tip_extents(stowed_right)[1]) if v is not None)
    raised_z = max(v for v in (_tip_extents(raised_left)[1], _tip_extents(raised_right)[1]) if v is not None)

    ctx.check(
        "v bracket raises above its stowed pose",
        raised_z > stowed_z + 0.10,
        details=f"stowed_z={stowed_z}, raised_z={raised_z}",
    )
    ctx.check(
        "v bracket folds back toward the ladder when raised",
        raised_y < stowed_y - 0.15,
        details=f"stowed_y={stowed_y}, raised_y={raised_y}",
    )

    return ctx.report()


object_model = build_object_model()
