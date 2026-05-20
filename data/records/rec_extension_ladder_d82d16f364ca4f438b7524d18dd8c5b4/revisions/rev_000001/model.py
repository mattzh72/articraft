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


LOWER_LENGTH = 3.60
UPPER_LENGTH = 3.20
LADDER_WIDTH = 0.38
LOWER_RAIL_X = 0.0
LOWER_RAIL_DEPTH = 0.026
LOWER_RAIL_WIDTH = 0.042
LOWER_RAIL_CENTERS = (-LADDER_WIDTH / 2.0, LADDER_WIDTH / 2.0)
UPPER_RAIL_X = 0.040
UPPER_RAIL_DEPTH = 0.022
UPPER_RAIL_WIDTH = 0.034
UPPER_RAIL_CENTERS = (-0.155, 0.155)
LOWER_INSERT_HEIGHT = 0.55
UPPER_TRAVEL = 2.25
LOWER_RUNG_RADIUS = 0.016
UPPER_RUNG_RADIUS = 0.014
LOWER_RUNG_ZS = tuple(0.32 + 0.28 * i for i in range(12))
UPPER_RUNG_ZS = tuple(0.26 + 0.28 * i for i in range(10))
COVER_OPEN_ANGLE = 1.35


def _add_ladder_section(
    part,
    *,
    prefix: str,
    rail_x: float,
    rail_depth: float,
    rail_width: float,
    rail_centers: tuple[float, float],
    rail_length: float,
    rung_radius: float,
    rung_positions: tuple[float, ...],
    rail_material,
    rung_material,
) -> None:
    for index, y_pos in enumerate(rail_centers):
        part.visual(
            Box((rail_depth, rail_width, rail_length)),
            origin=Origin(xyz=(rail_x, y_pos, rail_length / 2.0)),
            material=rail_material,
            name=f"{prefix}_rail_{index}",
        )

    rung_length = abs(rail_centers[1] - rail_centers[0]) + rail_width
    for index, z_pos in enumerate(rung_positions):
        part.visual(
            Cylinder(radius=rung_radius, length=rung_length),
            origin=Origin(
                xyz=(rail_x, 0.0, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rung_material,
            name=f"{prefix}_rung_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    galvanized = model.material("galvanized", rgba=(0.63, 0.66, 0.69, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    amber = model.material("amber", rgba=(0.85, 0.63, 0.18, 1.0))

    lower_section = model.part("lower_section")
    _add_ladder_section(
        lower_section,
        prefix="lower",
        rail_x=LOWER_RAIL_X,
        rail_depth=LOWER_RAIL_DEPTH,
        rail_width=LOWER_RAIL_WIDTH,
        rail_centers=LOWER_RAIL_CENTERS,
        rail_length=LOWER_LENGTH,
        rung_radius=LOWER_RUNG_RADIUS,
        rung_positions=LOWER_RUNG_ZS,
        rail_material=aluminum,
        rung_material=galvanized,
    )
    for index, y_pos in enumerate(LOWER_RAIL_CENTERS):
        lower_section.visual(
            Box((0.040, 0.052, 0.050)),
            origin=Origin(xyz=(LOWER_RAIL_X, y_pos, 0.025)),
            material=rubber,
            name=f"foot_{index}",
        )

    upper_section = model.part("upper_section")
    _add_ladder_section(
        upper_section,
        prefix="upper",
        rail_x=UPPER_RAIL_X,
        rail_depth=UPPER_RAIL_DEPTH,
        rail_width=UPPER_RAIL_WIDTH,
        rail_centers=UPPER_RAIL_CENTERS,
        rail_length=UPPER_LENGTH,
        rung_radius=UPPER_RUNG_RADIUS,
        rung_positions=UPPER_RUNG_ZS,
        rail_material=aluminum,
        rung_material=galvanized,
    )
    for rail_index, y_pos in enumerate(UPPER_RAIL_CENTERS):
        for shoe_index, z_pos in enumerate((0.34, 0.74)):
            upper_section.visual(
                Box((0.018, 0.020, 0.120)),
                origin=Origin(xyz=(0.022, y_pos, z_pos)),
                material=graphite,
                name=f"guide_shoe_{rail_index}_{shoe_index}",
            )

    upper_section.visual(
        Box((0.034, 0.130, 0.080)),
        origin=Origin(xyz=(0.043, 0.0, 0.580)),
        material=graphite,
        name="lock_housing",
    )
    upper_section.visual(
        Box((0.018, 0.086, 0.020)),
        origin=Origin(xyz=(0.052, 0.0, 0.536)),
        material=amber,
        name="lock_trigger",
    )
    for index, y_pos in enumerate((-0.054, 0.054)):
        upper_section.visual(
            Box((0.014, 0.016, 0.040)),
            origin=Origin(xyz=(0.055, y_pos, 0.640)),
            material=graphite,
            name=f"cover_mount_{index}",
        )
        upper_section.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(0.062, y_pos, 0.660),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=graphite,
            name=f"cover_knuckle_{index}",
        )

    lock_cover = model.part("lock_cover")
    lock_cover.visual(
        Cylinder(radius=0.006, length=0.094),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="cover_barrel",
    )
    lock_cover.visual(
        Box((0.008, 0.090, 0.062)),
        origin=Origin(xyz=(0.005, 0.0, -0.034)),
        material=graphite,
        name="cover_panel",
    )
    lock_cover.visual(
        Box((0.008, 0.118, 0.024)),
        origin=Origin(xyz=(0.006, 0.0, -0.071)),
        material=graphite,
        name="cover_skirt",
    )
    lock_cover.visual(
        Box((0.014, 0.118, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, -0.084)),
        material=amber,
        name="cover_lip",
    )

    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_section,
        child=upper_section,
        origin=Origin(xyz=(0.0, 0.0, LOWER_INSERT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.45,
            lower=0.0,
            upper=UPPER_TRAVEL,
        ),
    )
    model.articulation(
        "upper_to_lock_cover",
        ArticulationType.REVOLUTE,
        parent=upper_section,
        child=lock_cover,
        origin=Origin(xyz=(0.062, 0.0, 0.660)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=COVER_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_section = object_model.get_part("lower_section")
    upper_section = object_model.get_part("upper_section")
    lock_cover = object_model.get_part("lock_cover")
    slide = object_model.get_articulation("lower_to_upper")
    cover_hinge = object_model.get_articulation("upper_to_lock_cover")

    ctx.expect_gap(
        upper_section,
        lower_section,
        axis="x",
        positive_elem="guide_shoe_0_0",
        negative_elem="lower_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper guide shoe rides on one lower rail",
    )
    ctx.expect_gap(
        upper_section,
        lower_section,
        axis="x",
        positive_elem="guide_shoe_1_0",
        negative_elem="lower_rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper guide shoe rides on the opposite lower rail",
    )
    ctx.expect_overlap(
        upper_section,
        lower_section,
        axes="z",
        elem_a="upper_rail_0",
        elem_b="lower_rail_0",
        min_overlap=2.95,
        name="collapsed fly section remains deeply nested in the base section",
    )

    rest_pos = ctx.part_world_position(upper_section)
    with ctx.pose({slide: UPPER_TRAVEL}):
        ctx.expect_gap(
            upper_section,
            lower_section,
            axis="x",
            positive_elem="guide_shoe_0_0",
            negative_elem="lower_rail_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="guide shoe stays seated on the rail at full extension",
        )
        ctx.expect_overlap(
            upper_section,
            lower_section,
            axes="z",
            elem_a="upper_rail_0",
            elem_b="lower_rail_0",
            min_overlap=0.78,
            name="extended fly section retains insertion in the base section",
        )
        extended_pos = ctx.part_world_position(upper_section)

    ctx.check(
        "upper section extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 2.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        lock_cover,
        upper_section,
        axis="x",
        positive_elem="cover_panel",
        negative_elem="lock_housing",
        min_gap=0.002,
        max_gap=0.006,
        name="closed rung lock cover sits just proud of the lock housing",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(lock_cover, elem="cover_panel")
    with ctx.pose({cover_hinge: COVER_OPEN_ANGLE}):
        ctx.expect_gap(
            lock_cover,
            upper_section,
            axis="z",
            positive_elem="cover_panel",
            negative_elem="lock_housing",
            min_gap=0.018,
            name="opened cover lifts clear above the lock housing",
        )
        open_panel_aabb = ctx.part_element_world_aabb(lock_cover, elem="cover_panel")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(min_corner, max_corner))

    closed_center = _aabb_center(closed_panel_aabb)
    open_center = _aabb_center(open_panel_aabb)
    ctx.check(
        "cover folds forward from its hinge",
        closed_center is not None
        and open_center is not None
        and open_center[0] > closed_center[0] + 0.025
        and open_center[2] > closed_center[2] + 0.020,
        details=f"closed={closed_center}, open={open_center}",
    )

    return ctx.report()


object_model = build_object_model()
