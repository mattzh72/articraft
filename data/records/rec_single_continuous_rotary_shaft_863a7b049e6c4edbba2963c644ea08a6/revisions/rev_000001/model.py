from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_LENGTH = 0.320
RAIL_CORE_WIDTH = 0.044
RAIL_FOOT_WIDTH = 0.068
RAIL_HEIGHT = 0.018
RAIL_FOOT_LENGTH = 0.062
RAIL_FOOT_OFFSET = 0.105

BEARING_SPACING = 0.170
BEARING_LENGTH = 0.056
BEARING_WIDTH = 0.050
BEARING_BASE_THICKNESS = 0.012
BEARING_PEDESTAL_THICKNESS = 0.012
BEARING_AXIS_HEIGHT = 0.038
BEARING_HOUSING_RADIUS = 0.024
BEARING_BORE_RADIUS = 0.0090

SHAFT_AXIS_Z = RAIL_HEIGHT + BEARING_AXIS_HEIGHT
SHAFT_MAIN_RADIUS = 0.0090
SHAFT_JOURNAL_RADIUS = 0.0080
DRIVE_COLLAR_RADIUS = 0.015
CENTER_HUB_RADIUS = 0.018
END_FLANGE_RADIUS = 0.014

LEFT_JOURNAL_LENGTH = 0.040
RIGHT_JOURNAL_LENGTH = 0.058
MAIN_SHAFT_LENGTH = 0.252
DRIVE_COLLAR_LENGTH = 0.014
CENTER_HUB_LENGTH = 0.044
END_FLANGE_LENGTH = 0.010

LEFT_JOURNAL_CENTER_X = -0.133
DRIVE_COLLAR_CENTER_X = -0.122
MAIN_SHAFT_CENTER_X = 0.0
CENTER_HUB_CENTER_X = 0.0
RIGHT_JOURNAL_CENTER_X = 0.096
END_FLANGE_CENTER_X = 0.130


def _rail_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_CORE_WIDTH,
        RAIL_HEIGHT,
        centered=(True, True, False),
    )
    left_foot = cq.Workplane("XY").box(
        RAIL_FOOT_LENGTH,
        RAIL_FOOT_WIDTH,
        RAIL_HEIGHT * 0.55,
        centered=(True, True, False),
    ).translate((-RAIL_FOOT_OFFSET, 0.0, 0.0))
    right_foot = cq.Workplane("XY").box(
        RAIL_FOOT_LENGTH,
        RAIL_FOOT_WIDTH,
        RAIL_HEIGHT * 0.55,
        centered=(True, True, False),
    ).translate((RAIL_FOOT_OFFSET, 0.0, 0.0))
    rail = beam.union(left_foot).union(right_foot)
    return rail.edges("|Z").chamfer(0.002)


def _bearing_block_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BEARING_LENGTH,
        BEARING_WIDTH,
        BEARING_BASE_THICKNESS,
        centered=(True, True, False),
    )
    shoulder = cq.Workplane("XY").box(
        BEARING_LENGTH * 0.78,
        BEARING_WIDTH * 0.34,
        0.019,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BEARING_BASE_THICKNESS - 0.001))
    cheek_left = cq.Workplane("XY").box(
        BEARING_LENGTH * 0.74,
        0.010,
        0.031,
        centered=(True, True, False),
    ).translate((0.0, 0.018, BEARING_BASE_THICKNESS - 0.001))
    cheek_right = cq.Workplane("XY").box(
        BEARING_LENGTH * 0.74,
        0.010,
        0.031,
        centered=(True, True, False),
    ).translate((0.0, -0.018, BEARING_BASE_THICKNESS - 0.001))
    block = base.union(shoulder).union(cheek_left).union(cheek_right)
    return block.translate((0.0, 0.0, -BEARING_AXIS_HEIGHT))


def _add_bearing_block_visuals(part) -> None:
    part.visual(
        Box((BEARING_LENGTH, BEARING_WIDTH, BEARING_BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material="bearing_housing",
        name="base_foot",
    )
    part.visual(
        Box((BEARING_LENGTH * 0.78, BEARING_WIDTH * 0.34, 0.019)),
        origin=Origin(xyz=(0.0, 0.0, -0.0185)),
        material="bearing_housing",
        name="lower_saddle",
    )
    part.visual(
        Box((BEARING_LENGTH * 0.74, 0.010, 0.031)),
        origin=Origin(xyz=(0.0, 0.018, -0.0115)),
        material="bearing_housing",
        name="left_cheek",
    )
    part.visual(
        Box((BEARING_LENGTH * 0.74, 0.010, 0.031)),
        origin=Origin(xyz=(0.0, -0.018, -0.0115)),
        material="bearing_housing",
        name="right_cheek",
    )


def _shaft_visual_origin(x_center: float) -> Origin:
    return Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stepped_rotary_shaft_assembly")

    model.material("rail_paint", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("bearing_housing", rgba=(0.56, 0.60, 0.64, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("oxide_steel", rgba=(0.22, 0.24, 0.27, 1.0))

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(_rail_shape(), "rail_body"),
        material="rail_paint",
        name="rail_body",
    )

    left_bearing = model.part("left_bearing")
    _add_bearing_block_visuals(left_bearing)

    right_bearing = model.part("right_bearing")
    _add_bearing_block_visuals(right_bearing)

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_MAIN_RADIUS, length=MAIN_SHAFT_LENGTH),
        origin=_shaft_visual_origin(MAIN_SHAFT_CENTER_X),
        material="machined_steel",
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=SHAFT_JOURNAL_RADIUS, length=LEFT_JOURNAL_LENGTH),
        origin=_shaft_visual_origin(LEFT_JOURNAL_CENTER_X),
        material="machined_steel",
        name="left_journal",
    )
    shaft.visual(
        Cylinder(radius=SHAFT_JOURNAL_RADIUS, length=RIGHT_JOURNAL_LENGTH),
        origin=_shaft_visual_origin(RIGHT_JOURNAL_CENTER_X),
        material="machined_steel",
        name="right_journal",
    )
    shaft.visual(
        Cylinder(radius=DRIVE_COLLAR_RADIUS, length=DRIVE_COLLAR_LENGTH),
        origin=_shaft_visual_origin(DRIVE_COLLAR_CENTER_X),
        material="oxide_steel",
        name="drive_collar",
    )
    shaft.visual(
        Cylinder(radius=CENTER_HUB_RADIUS, length=CENTER_HUB_LENGTH),
        origin=_shaft_visual_origin(CENTER_HUB_CENTER_X),
        material="oxide_steel",
        name="center_hub",
    )
    shaft.visual(
        Cylinder(radius=END_FLANGE_RADIUS, length=END_FLANGE_LENGTH),
        origin=_shaft_visual_origin(END_FLANGE_CENTER_X),
        material="machined_steel",
        name="end_flange",
    )

    model.articulation(
        "rail_to_left_bearing",
        ArticulationType.FIXED,
        parent=rail,
        child=left_bearing,
        origin=Origin(xyz=(-BEARING_SPACING / 2.0, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "rail_to_right_bearing",
        ArticulationType.FIXED,
        parent=rail,
        child=right_bearing,
        origin=Origin(xyz=(BEARING_SPACING / 2.0, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "rail_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=rail,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    left_bearing = object_model.get_part("left_bearing")
    right_bearing = object_model.get_part("right_bearing")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("rail_to_shaft")

    ctx.check(
        "shaft articulation is continuous about the supported centerline",
        shaft_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in shaft_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={shaft_spin.articulation_type}, axis={shaft_spin.axis}",
    )

    ctx.expect_gap(
        left_bearing,
        rail,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.00001,
        name="left bearing foot seats on the rail",
    )
    ctx.expect_gap(
        right_bearing,
        rail,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.00001,
        name="right bearing foot seats on the rail",
    )

    ctx.expect_origin_distance(
        shaft,
        left_bearing,
        axes="yz",
        max_dist=0.0005,
        name="shaft centerline aligns with the left bearing bore",
    )
    ctx.expect_origin_distance(
        shaft,
        right_bearing,
        axes="yz",
        max_dist=0.0005,
        name="shaft centerline aligns with the right bearing bore",
    )

    ctx.expect_gap(
        left_bearing,
        shaft,
        axis="x",
        negative_elem="drive_collar",
        min_gap=0.001,
        max_gap=0.006,
        name="drive collar clears the left bearing block",
    )
    ctx.expect_gap(
        shaft,
        right_bearing,
        axis="x",
        positive_elem="end_flange",
        min_gap=0.010,
        max_gap=0.030,
        name="end flange sits beyond the right bearing block",
    )

    rest_position = ctx.part_world_position(shaft)
    with ctx.pose({shaft_spin: 1.2}):
        turned_position = ctx.part_world_position(shaft)
        ctx.expect_origin_distance(
            shaft,
            left_bearing,
            axes="yz",
            max_dist=0.0005,
            name="turned shaft stays concentric with the left bearing",
        )
        ctx.expect_origin_distance(
            shaft,
            right_bearing,
            axes="yz",
            max_dist=0.0005,
            name="turned shaft stays concentric with the right bearing",
        )
        ctx.check(
            "shaft rotation does not translate the shaft axis",
            rest_position is not None
            and turned_position is not None
            and max(abs(a - b) for a, b in zip(rest_position, turned_position)) <= 1e-6,
            details=f"rest={rest_position}, turned={turned_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
