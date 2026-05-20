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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_aligned_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_round_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_aligned_member(a, b)),
        material=material,
        name=name,
    )


def _add_ladder_section(
    part,
    *,
    rail_x: float,
    rail_y: float,
    rail_width: float,
    rail_depth: float,
    rail_length: float,
    rung_radius: float,
    rung_count: int,
    rung_bottom: float,
    rung_pitch: float,
    rail_material,
    rung_material,
    name_prefix: str,
) -> None:
    for side_name, x_pos in (("left", -rail_x), ("right", rail_x)):
        part.visual(
            Box((rail_width, rail_depth, rail_length)),
            origin=Origin(xyz=(x_pos, rail_y, rail_length * 0.5)),
            material=rail_material,
            name=f"{name_prefix}_{side_name}_rail",
        )

    rung_clear = rail_x * 2.0 - rail_width * 0.45
    rung_depth = rail_depth * 0.52
    rung_height = rung_radius * 1.70
    for index in range(rung_count):
        z_pos = rung_bottom + rung_pitch * index
        part.visual(
            Box((rung_clear, rung_depth, rung_height)),
            origin=Origin(xyz=(0.0, rail_y, z_pos)),
            material=rung_material,
            name=f"{name_prefix}_rung_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.74, 0.77, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.42, 0.45, 0.48, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    rope = model.material("rope", rgba=(0.67, 0.58, 0.39, 1.0))
    warning = model.material("warning", rgba=(0.84, 0.71, 0.11, 1.0))

    base_section = model.part("base_section")
    _add_ladder_section(
        base_section,
        rail_x=0.205,
        rail_y=-0.008,
        rail_width=0.032,
        rail_depth=0.066,
        rail_length=3.62,
        rung_radius=0.016,
        rung_count=11,
        rung_bottom=0.42,
        rung_pitch=0.295,
        rail_material=aluminum,
        rung_material=aluminum,
        name_prefix="base",
    )
    for x_pos, side_name in ((-0.205, "left"), (0.205, "right")):
        base_section.visual(
            Box((0.050, 0.088, 0.035)),
            origin=Origin(xyz=(x_pos, -0.008, 0.0175)),
            material=rubber,
            name=f"{side_name}_foot",
        )
        base_section.visual(
            Box((0.060, 0.040, 0.140)),
            origin=Origin(xyz=(x_pos, 0.028, 0.31)),
            material=steel,
            name=f"{side_name}_bracket",
        )
        base_section.visual(
            Cylinder(radius=0.018, length=0.062),
            origin=Origin(
                xyz=(x_pos, 0.032, 0.31),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=steel,
            name=f"{side_name}_hinge_barrel",
        )

    base_section.visual(
        Box((0.46, 0.048, 0.050)),
        origin=Origin(xyz=(0.0, -0.005, 3.47)),
        material=steel,
        name="top_cap",
    )
    base_section.visual(
        Box((0.40, 0.050, 0.075)),
        origin=Origin(xyz=(0.0, 0.000, 2.92)),
        material=warning,
        name="lock_housing",
    )
    base_section.visual(
        Cylinder(radius=0.036, length=0.435),
        origin=Origin(
            xyz=(0.0, -0.052, 3.34),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="pulley",
    )
    _add_round_member(
        base_section,
        (0.0, -0.082, 1.33),
        (0.0, -0.082, 3.23),
        radius=0.007,
        material=rope,
        name="haul_rope",
    )
    base_section.visual(
        Box((0.090, 0.080, 0.070)),
        origin=Origin(xyz=(0.0, -0.050, 1.34)),
        material=steel,
        name="rope_cleat",
    )

    fly_section = model.part("fly_section")
    _add_ladder_section(
        fly_section,
        rail_x=0.170,
        rail_y=0.070,
        rail_width=0.028,
        rail_depth=0.058,
        rail_length=3.34,
        rung_radius=0.014,
        rung_count=10,
        rung_bottom=0.25,
        rung_pitch=0.307,
        rail_material=aluminum,
        rung_material=aluminum,
        name_prefix="fly",
    )
    for x_pos, side_name in ((-0.170, "left"), (0.170, "right")):
        fly_section.visual(
            Box((0.040, 0.040, 0.100)),
            origin=Origin(xyz=(x_pos, 0.045, 0.33)),
            material=warning,
            name=f"{side_name}_guide_shoe_low",
        )
        fly_section.visual(
            Box((0.040, 0.040, 0.100)),
            origin=Origin(xyz=(x_pos, 0.045, 2.56)),
            material=warning,
            name=f"{side_name}_guide_shoe_high",
        )
    fly_section.visual(
        Box((0.37, 0.042, 0.045)),
        origin=Origin(xyz=(0.0, 0.070, 3.31)),
        material=steel,
        name="top_cap",
    )
    fly_section.visual(
        Box((0.340, 0.024, 0.110)),
        origin=Origin(xyz=(0.0, 0.030, 0.22)),
        material=steel,
        name="rope_anchor",
    )

    model.articulation(
        "fly_extension",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=fly_section,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=0.0,
            upper=1.20,
        ),
    )

    left_arm = model.part("left_arm")
    left_arm.visual(
        Box((0.050, 0.032, 0.84)),
        origin=Origin(xyz=(-0.030, 0.0, 0.42)),
        material=aluminum,
        name="arm_beam",
    )
    left_arm.visual(
        Box((0.090, 0.095, 0.040)),
        origin=Origin(xyz=(-0.065, 0.0, 0.805)),
        material=rubber,
        name="foot_pad",
    )
    left_arm.visual(
        Box((0.052, 0.060, 0.090)),
        origin=Origin(xyz=(-0.038, 0.0, 0.085)),
        material=steel,
        name="hinge_shoe",
    )

    right_arm = model.part("right_arm")
    right_arm.visual(
        Box((0.050, 0.032, 0.84)),
        origin=Origin(xyz=(0.030, 0.0, 0.42)),
        material=aluminum,
        name="arm_beam",
    )
    right_arm.visual(
        Box((0.090, 0.095, 0.040)),
        origin=Origin(xyz=(0.065, 0.0, 0.805)),
        material=rubber,
        name="foot_pad",
    )
    right_arm.visual(
        Box((0.052, 0.060, 0.090)),
        origin=Origin(xyz=(0.038, 0.0, 0.085)),
        material=steel,
        name="hinge_shoe",
    )

    model.articulation(
        "left_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=left_arm,
        origin=Origin(xyz=(-0.230, 0.032, 0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.0,
            lower=0.0,
            upper=1.08,
        ),
    )
    model.articulation(
        "right_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=right_arm,
        origin=Origin(xyz=(0.230, 0.032, 0.31)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.0,
            lower=0.0,
            upper=1.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    fly_section = object_model.get_part("fly_section")
    fly_extension = object_model.get_articulation("fly_extension")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    left_arm_hinge = object_model.get_articulation("left_arm_hinge")
    right_arm_hinge = object_model.get_articulation("right_arm_hinge")

    def _elem_center_x(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    ctx.expect_overlap(
        fly_section,
        base_section,
        axes="x",
        min_overlap=0.28,
        name="fly section stays centered between the base rails",
    )
    ctx.expect_gap(
        fly_section,
        base_section,
        axis="y",
        min_gap=0.016,
        max_gap=0.120,
        positive_elem="fly_left_rail",
        negative_elem="base_left_rail",
        name="fly section rides in front of the base section",
    )
    ctx.expect_overlap(
        fly_section,
        base_section,
        axes="z",
        min_overlap=1.60,
        name="collapsed fly section retains deep insertion",
    )

    rest_pos = ctx.part_world_position(fly_section)
    with ctx.pose({fly_extension: 1.20}):
        ctx.expect_overlap(
            fly_section,
            base_section,
            axes="z",
            min_overlap=0.90,
            name="extended fly section still retains insertion",
        )
        extended_pos = ctx.part_world_position(fly_section)

    ctx.check(
        "fly section extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 1.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    left_rest = _elem_center_x(left_arm, "foot_pad")
    right_rest = _elem_center_x(right_arm, "foot_pad")
    with ctx.pose({left_arm_hinge: 1.08, right_arm_hinge: 1.08}):
        left_open = _elem_center_x(left_arm, "foot_pad")
        right_open = _elem_center_x(right_arm, "foot_pad")

    ctx.check(
        "left arm swings outward",
        left_rest is not None
        and left_open is not None
        and left_open < left_rest - 0.18,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right arm swings outward",
        right_rest is not None
        and right_open is not None
        and right_open > right_rest + 0.18,
        details=f"rest={right_rest}, open={right_open}",
    )

    return ctx.report()


object_model = build_object_model()
