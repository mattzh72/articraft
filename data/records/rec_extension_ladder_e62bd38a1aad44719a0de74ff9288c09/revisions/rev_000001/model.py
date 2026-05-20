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


BASE_SECTION_LENGTH = 4.20
UPPER_SECTION_LENGTH = 3.82
SECTION_TRAVEL = 1.55

BASE_OUTER_WIDTH = 0.50
UPPER_OUTER_WIDTH = 0.442

BASE_RAIL_WIDTH = 0.082
UPPER_RAIL_WIDTH = 0.052

BASE_RAIL_DEPTH = 0.038
UPPER_RAIL_DEPTH = 0.026

BASE_WALL = 0.0045
UPPER_WALL = 0.0035

BASE_RAIL_BOTTOM_Z = 0.11
UPPER_HOME_Z = 0.58

RUNG_DEPTH = 0.030
RUNG_HEIGHT = 0.028
RUNG_SPACING = 0.305

FOOT_PIVOT_Z = 0.083
FOOT_PAD_LENGTH = 0.112
FOOT_PAD_DEPTH = 0.072
FOOT_PAD_THICKNESS = 0.020
FOOT_SWING = 0.45


def _add_channel_rail(
    part,
    *,
    prefix: str,
    outer_face_x: float,
    side_sign: float,
    rail_width: float,
    rail_depth: float,
    wall: float,
    length: float,
    bottom_z: float,
    material,
) -> None:
    z_center = bottom_z + (length * 0.5)
    flange_center_x = outer_face_x + (side_sign * rail_width * 0.5)

    part.visual(
        Box((wall, rail_depth, length)),
        origin=Origin(xyz=(outer_face_x + (side_sign * wall * 0.5), 0.0, z_center)),
        material=material,
        name=f"{prefix}_web",
    )
    part.visual(
        Box((rail_width, wall, length)),
        origin=Origin(
            xyz=(flange_center_x, rail_depth * 0.5 - wall * 0.5, z_center)
        ),
        material=material,
        name=f"{prefix}_front_flange",
    )
    part.visual(
        Box((rail_width, wall, length)),
        origin=Origin(
            xyz=(flange_center_x, -rail_depth * 0.5 + wall * 0.5, z_center)
        ),
        material=material,
        name=f"{prefix}_rear_flange",
    )


def _rung_positions(length: float, *, bottom_margin: float, top_margin: float) -> list[float]:
    positions: list[float] = []
    z = bottom_margin
    while z <= length - top_margin:
        positions.append(z)
        z += RUNG_SPACING
    return positions


def _add_rungs(
    part,
    *,
    prefix: str,
    width: float,
    bottom_z: float,
    positions: list[float],
    material,
    y_offset: float,
) -> None:
    for index, z in enumerate(positions):
        part.visual(
            Box((width, RUNG_DEPTH, RUNG_HEIGHT)),
            origin=Origin(xyz=(0.0, y_offset, bottom_z + z)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    guide_nylon = model.material("guide_nylon", rgba=(0.17, 0.18, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base_section = model.part("base_section")
    _add_channel_rail(
        base_section,
        prefix="left_rail",
        outer_face_x=-BASE_OUTER_WIDTH * 0.5,
        side_sign=1.0,
        rail_width=BASE_RAIL_WIDTH,
        rail_depth=BASE_RAIL_DEPTH,
        wall=BASE_WALL,
        length=BASE_SECTION_LENGTH,
        bottom_z=BASE_RAIL_BOTTOM_Z,
        material=aluminum,
    )
    _add_channel_rail(
        base_section,
        prefix="right_rail",
        outer_face_x=BASE_OUTER_WIDTH * 0.5,
        side_sign=-1.0,
        rail_width=BASE_RAIL_WIDTH,
        rail_depth=BASE_RAIL_DEPTH,
        wall=BASE_WALL,
        length=BASE_SECTION_LENGTH,
        bottom_z=BASE_RAIL_BOTTOM_Z,
        material=aluminum,
    )
    _add_rungs(
        base_section,
        prefix="base_rung",
        width=BASE_OUTER_WIDTH - (2.0 * BASE_RAIL_WIDTH),
        bottom_z=BASE_RAIL_BOTTOM_Z,
        positions=_rung_positions(BASE_SECTION_LENGTH, bottom_margin=0.27, top_margin=0.25),
        material=aluminum,
        y_offset=-0.006,
    )
    for side_sign, label in ((1.0, "left"), (-1.0, "right")):
        guide_x = side_sign * (BASE_OUTER_WIDTH * 0.5 - BASE_WALL - 0.009)
        for guide_index, guide_z in enumerate((3.82, 4.10)):
            base_section.visual(
                Box((0.022, 0.018, 0.060)),
                origin=Origin(xyz=(guide_x, 0.0, guide_z)),
                material=guide_nylon,
                name=f"{label}_guide_{guide_index}",
            )
    rail_center_offset = BASE_OUTER_WIDTH * 0.5 - BASE_RAIL_WIDTH * 0.5
    for side_sign, label in ((1.0, "left"), (-1.0, "right")):
        rail_center_x = side_sign * rail_center_offset
        for brace_name, brace_y in (("front", 0.023), ("rear", -0.023)):
            base_section.visual(
                Box((0.070, 0.010, 0.032)),
                origin=Origin(xyz=(rail_center_x, brace_y, 0.098)),
                material=guide_nylon,
                name=f"{label}_foot_brace_{brace_name}",
            )
        for lug_index, lug_offset in enumerate((-0.026, 0.026)):
            base_section.visual(
                Box((0.012, 0.040, 0.026)),
                origin=Origin(xyz=(rail_center_x + lug_offset, 0.0, 0.079)),
                material=guide_nylon,
                name=f"{label}_foot_lug_{lug_index}",
            )

    upper_section = model.part("upper_section")
    _add_channel_rail(
        upper_section,
        prefix="left_rail",
        outer_face_x=-UPPER_OUTER_WIDTH * 0.5,
        side_sign=1.0,
        rail_width=UPPER_RAIL_WIDTH,
        rail_depth=UPPER_RAIL_DEPTH,
        wall=UPPER_WALL,
        length=UPPER_SECTION_LENGTH,
        bottom_z=0.0,
        material=aluminum,
    )
    _add_channel_rail(
        upper_section,
        prefix="right_rail",
        outer_face_x=UPPER_OUTER_WIDTH * 0.5,
        side_sign=-1.0,
        rail_width=UPPER_RAIL_WIDTH,
        rail_depth=UPPER_RAIL_DEPTH,
        wall=UPPER_WALL,
        length=UPPER_SECTION_LENGTH,
        bottom_z=0.0,
        material=aluminum,
    )
    _add_rungs(
        upper_section,
        prefix="upper_rung",
        width=UPPER_OUTER_WIDTH - (2.0 * UPPER_RAIL_WIDTH),
        bottom_z=0.0,
        positions=_rung_positions(UPPER_SECTION_LENGTH, bottom_margin=0.22, top_margin=0.22),
        material=aluminum,
        y_offset=0.001,
    )
    upper_section.visual(
        Cylinder(radius=0.012, length=UPPER_OUTER_WIDTH - 0.08),
        origin=Origin(
            xyz=(0.0, 0.0, UPPER_SECTION_LENGTH - 0.07),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=guide_nylon,
        name="top_cap",
    )

    for side_sign, label in ((1.0, "left"), (-1.0, "right")):
        foot = model.part(f"{label}_foot")
        foot.visual(
            Cylinder(radius=0.012, length=0.040),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=guide_nylon,
            name="barrel",
        )
        foot.visual(
            Box((0.034, 0.024, 0.076)),
            origin=Origin(xyz=(0.0, -0.010, -0.048)),
            material=guide_nylon,
            name="arm",
        )
        foot.visual(
            Box((FOOT_PAD_LENGTH, FOOT_PAD_DEPTH, FOOT_PAD_THICKNESS)),
            origin=Origin(xyz=(0.0, -0.020, -0.082)),
            material=rubber,
            name="pad",
        )
        foot.visual(
            Box((FOOT_PAD_LENGTH, 0.016, 0.012)),
            origin=Origin(xyz=(0.0, 0.010, -0.078)),
            material=rubber,
            name="toe",
        )
        model.articulation(
            f"{label}_foot_hinge",
            ArticulationType.REVOLUTE,
            parent=base_section,
            child=foot,
            origin=Origin(xyz=(side_sign * rail_center_offset, 0.0, FOOT_PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=2.2,
                lower=-0.55,
                upper=0.55,
            ),
        )

    model.articulation(
        "base_to_upper",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=upper_section,
        origin=Origin(xyz=(0.0, 0.0, UPPER_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.30,
            lower=0.0,
            upper=SECTION_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    upper_section = object_model.get_part("upper_section")
    base_to_upper = object_model.get_articulation("base_to_upper")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    left_foot_hinge = object_model.get_articulation("left_foot_hinge")
    right_foot_hinge = object_model.get_articulation("right_foot_hinge")

    ctx.expect_within(
        upper_section,
        base_section,
        axes="xy",
        margin=0.0,
        name="upper section stays nested within base section footprint",
    )
    ctx.expect_overlap(
        upper_section,
        base_section,
        axes="z",
        min_overlap=2.0,
        name="collapsed ladder keeps long retained insertion",
    )

    rest_pos = ctx.part_world_position(upper_section)
    with ctx.pose({base_to_upper: SECTION_TRAVEL}):
        ctx.expect_within(
            upper_section,
            base_section,
            axes="xy",
            margin=0.0,
            name="extended upper section stays laterally guided by base rails",
        )
        ctx.expect_overlap(
            upper_section,
            base_section,
            axes="z",
            min_overlap=1.8,
            name="extended upper section still remains inserted in base rails",
        )
        extended_pos = ctx.part_world_position(upper_section)

    ctx.check(
        "upper section extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 1.4,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    left_pad_rest = _aabb_center(ctx.part_element_world_aabb(left_foot, elem="pad"))
    right_pad_rest = _aabb_center(ctx.part_element_world_aabb(right_foot, elem="pad"))
    with ctx.pose({left_foot_hinge: FOOT_SWING, right_foot_hinge: FOOT_SWING}):
        left_pad_swung = _aabb_center(ctx.part_element_world_aabb(left_foot, elem="pad"))
        right_pad_swung = _aabb_center(ctx.part_element_world_aabb(right_foot, elem="pad"))

    ctx.check(
        "left foot rotates on its pivot",
        left_pad_rest is not None
        and left_pad_swung is not None
        and left_pad_swung[1] > left_pad_rest[1] + 0.02,
        details=f"rest={left_pad_rest}, swung={left_pad_swung}",
    )
    ctx.check(
        "right foot rotates on its pivot",
        right_pad_rest is not None
        and right_pad_swung is not None
        and right_pad_swung[1] > right_pad_rest[1] + 0.02,
        details=f"rest={right_pad_rest}, swung={right_pad_swung}",
    )

    return ctx.report()


object_model = build_object_model()
