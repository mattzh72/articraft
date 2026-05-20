from __future__ import annotations

import math

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


BED_X = 0.62
BED_Y = 0.44
BED_Z = 0.035
BED_TOP = BED_Z

PIVOT_X = 0.270
PIVOT_Y = 0.185
PIVOT_Z = 0.086

HOLD_X = 0.220
HOLD_Y = -0.005
HOLD_Z = 0.061
HOLD_LOWER = -0.018
HOLD_UPPER = 0.018


def _rounded_board() -> cq.Workplane:
    """Slightly softened rectangular cutting bed, authored in world scale."""
    return (
        cq.Workplane("XY")
        .box(BED_X, BED_Y, BED_Z)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, BED_Z / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_guillotine_paper_cutter")

    wood = model.material("varnished_wood", color=(0.72, 0.52, 0.30, 1.0))
    dark_wood = model.material("dark_fence", color=(0.24, 0.17, 0.10, 1.0))
    steel = model.material("brushed_steel", color=(0.72, 0.74, 0.73, 1.0))
    dark_steel = model.material("dark_steel", color=(0.22, 0.23, 0.24, 1.0))
    black = model.material("black_plastic", color=(0.015, 0.015, 0.018, 1.0))
    rubber = model.material("rubber", color=(0.04, 0.045, 0.045, 1.0))
    red = model.material("red_cut_line", color=(0.72, 0.06, 0.035, 1.0))
    ink = model.material("printed_marks", color=(0.055, 0.045, 0.035, 1.0))

    bed = model.part("bed")
    bed.visual(
        mesh_from_cadquery(_rounded_board(), "bed_board", tolerance=0.001),
        material=wood,
        name="bed_board",
    )
    bed.visual(
        Box((0.018, 0.390, 0.003)),
        origin=Origin(xyz=(0.265, -0.005, BED_TOP + 0.0015)),
        material=red,
        name="cut_strip",
    )
    bed.visual(
        Box((0.560, 0.024, 0.028)),
        origin=Origin(xyz=(-0.010, 0.205, BED_TOP + 0.014)),
        material=dark_wood,
        name="rear_fence",
    )
    bed.visual(
        Box((0.030, 0.355, 0.026)),
        origin=Origin(xyz=(-0.292, -0.010, BED_TOP + 0.013)),
        material=dark_wood,
        name="side_fence",
    )

    # Printed ruler ticks and paper alignment marks on the bed surface.
    for i, y in enumerate((-0.160, -0.100, -0.040, 0.020, 0.080, 0.140)):
        bed.visual(
            Box((0.160 if i % 2 == 0 else 0.105, 0.0025, 0.0012)),
            origin=Origin(xyz=(-0.150, y, BED_TOP + 0.0006)),
            material=ink,
            name=f"grid_tick_{i}",
        )
    for i, x in enumerate((-0.230, -0.165, -0.100, -0.035, 0.030, 0.095, 0.160)):
        bed.visual(
            Box((0.0022, 0.085, 0.0012)),
            origin=Origin(xyz=(x, -0.150, BED_TOP + 0.0006)),
            material=ink,
            name=f"ruler_tick_{i}",
        )

    # Fixed rear-corner pivot yoke.  The moving blade barrel sits between the
    # cheeks with a small visible clearance.
    for suffix, x in (("inner", PIVOT_X - 0.035), ("outer", PIVOT_X + 0.035)):
        bed.visual(
            Box((0.018, 0.050, 0.008)),
            origin=Origin(xyz=(x, PIVOT_Y, BED_TOP + 0.004)),
            material=dark_steel,
            name=f"pivot_foot_{suffix}",
        )
        bed.visual(
            Box((0.012, 0.060, 0.074)),
            origin=Origin(xyz=(x, PIVOT_Y, BED_TOP + 0.037)),
            material=dark_steel,
            name=f"pivot_cheek_{suffix}",
        )
    bed.visual(
        Cylinder(radius=0.009, length=0.095),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    # Two short guide stations for the vertically sliding hold-down bar.
    for station, y in enumerate((-0.145, 0.135)):
        for side, x in (("inner", HOLD_X - 0.021), ("outer", HOLD_X + 0.021)):
            bed.visual(
                Box((0.010, 0.030, 0.006)),
                origin=Origin(xyz=(x, y, BED_TOP + 0.003)),
                material=dark_steel,
                name=f"guide_foot_{station}_{side}",
            )
            bed.visual(
                Box((0.010, 0.020, 0.066)),
                origin=Origin(xyz=(x, y, BED_TOP + 0.033)),
                material=steel,
                name=f"guide_post_{station}_{side}",
            )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    blade_arm.visual(
        Box((0.010, 0.450, 0.050)),
        origin=Origin(xyz=(0.0, -0.245, -0.020)),
        material=steel,
        name="blade_plate",
    )
    blade_arm.visual(
        Box((0.013, 0.435, 0.006)),
        origin=Origin(xyz=(0.0, -0.240, -0.042)),
        material=dark_steel,
        name="cutting_edge",
    )
    blade_arm.visual(
        Box((0.028, 0.460, 0.018)),
        origin=Origin(xyz=(0.0, -0.242, 0.012)),
        material=dark_steel,
        name="top_spine",
    )
    for i, y in enumerate((-0.335, -0.430)):
        blade_arm.visual(
            Box((0.016, 0.020, 0.052)),
            origin=Origin(xyz=(0.0, y, 0.030)),
            material=black,
            name=f"grip_stem_{i}",
        )
    blade_arm.visual(
        Cylinder(radius=0.017, length=0.150),
        origin=Origin(xyz=(0.0, -0.385, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip",
    )

    hold_down = model.part("hold_down")
    hold_down.visual(
        Box((0.030, 0.300, 0.014)),
        origin=Origin(),
        material=rubber,
        name="clamp_bar",
    )
    for i, y in enumerate((-0.145, 0.135)):
        hold_down.visual(
            Box((0.032, 0.034, 0.024)),
            origin=Origin(xyz=(0.0, y - HOLD_Y, 0.008)),
            material=dark_steel,
            name=f"slider_block_{i}",
        )
        hold_down.visual(
            Box((0.024, 0.012, 0.026)),
            origin=Origin(xyz=(0.0, y - HOLD_Y, 0.023)),
            material=black,
            name=f"slider_cap_{i}",
        )

    model.articulation(
        "blade_hinge",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=blade_arm,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "hold_down_slide",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=hold_down,
        origin=Origin(xyz=(HOLD_X, HOLD_Y, HOLD_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.20, lower=HOLD_LOWER, upper=HOLD_UPPER),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bed = object_model.get_part("bed")
    blade = object_model.get_part("blade_arm")
    hold_down = object_model.get_part("hold_down")
    blade_hinge = object_model.get_articulation("blade_hinge")
    hold_slide = object_model.get_articulation("hold_down_slide")

    ctx.allow_overlap(
        bed,
        blade,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The fixed hinge pin is intentionally captured inside the blade arm barrel.",
    )
    ctx.expect_overlap(
        bed,
        blade,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.040,
        name="hinge pin passes through blade barrel",
    )
    ctx.expect_overlap(
        blade,
        bed,
        axes="xy",
        elem_a="blade_plate",
        elem_b="cut_strip",
        min_overlap=0.006,
        name="closed blade aligns over cut strip",
    )
    ctx.expect_gap(
        blade,
        bed,
        axis="z",
        positive_elem="cutting_edge",
        negative_elem="cut_strip",
        min_gap=0.001,
        max_gap=0.008,
        name="closed cutting edge sits just above strip",
    )

    grip_aabb = ctx.part_element_world_aabb(blade, elem="grip")
    rest_grip_z = None if grip_aabb is None else (grip_aabb[0][2] + grip_aabb[1][2]) / 2.0
    with ctx.pose({blade_hinge: 1.25}):
        raised_aabb = ctx.part_element_world_aabb(blade, elem="grip")
        raised_grip_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0
    ctx.check(
        "blade arm lifts upward about rear corner",
        rest_grip_z is not None and raised_grip_z is not None and raised_grip_z > rest_grip_z + 0.22,
        details=f"rest_grip_z={rest_grip_z}, raised_grip_z={raised_grip_z}",
    )

    with ctx.pose({hold_slide: HOLD_LOWER}):
        ctx.expect_gap(
            hold_down,
            bed,
            axis="z",
            positive_elem="clamp_bar",
            negative_elem="bed_board",
            min_gap=0.0005,
            max_gap=0.003,
            name="lowered hold-down nearly clamps bed surface",
        )
        lower_pos = ctx.part_world_position(hold_down)
    with ctx.pose({hold_slide: HOLD_UPPER}):
        upper_pos = ctx.part_world_position(hold_down)
    ctx.check(
        "hold-down slides vertically on guides",
        lower_pos is not None and upper_pos is not None and upper_pos[2] > lower_pos[2] + 0.030,
        details=f"lower={lower_pos}, upper={upper_pos}",
    )

    return ctx.report()


object_model = build_object_model()
