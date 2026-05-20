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


def _cylinder_along_y(*, radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _bearing_block_mesh():
    """Boxy bearing housing with a real through-bore along the spindle axis."""
    block = cq.Workplane("XY").box(0.20, 0.14, 0.12)
    bore = cq.Workplane("XZ").circle(0.027).extrude(0.24, both=True)
    return block.cut(bore)


def _bearing_barrel_mesh():
    """Short hollow boss around the through-bore, coaxial with the spindle."""
    return cq.Workplane("XZ").circle(0.055).circle(0.027).extrude(0.095, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_mounted_tool_slide")

    painted_cast = model.material("painted_cast", rgba=(0.18, 0.28, 0.34, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.10, 0.12, 0.14, 1.0))
    linear_steel = model.material("linear_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    oiled_steel = model.material("oiled_steel", rgba=(0.38, 0.40, 0.42, 1.0))
    brass = model.material("brass", rgba=(0.80, 0.62, 0.26, 1.0))
    motor_black = model.material("motor_black", rgba=(0.03, 0.035, 0.04, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.34, 0.08, 1.0))

    bearing_block = mesh_from_cadquery(
        _bearing_block_mesh(), "spindle_bearing_block", tolerance=0.0008, angular_tolerance=0.08
    )
    bearing_barrel = mesh_from_cadquery(
        _bearing_barrel_mesh(), "spindle_bearing_barrel", tolerance=0.0008, angular_tolerance=0.08
    )

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((1.25, 0.32, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=painted_cast,
        name="base_plate",
    )
    base_rail.visual(
        Box((1.12, 0.055, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=oiled_steel,
        name="rail_web",
    )
    base_rail.visual(
        Box((1.12, 0.12, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=linear_steel,
        name="rail_cap",
    )
    base_rail.visual(
        Box((0.050, 0.24, 0.105)),
        origin=Origin(xyz=(-0.585, 0.0, 0.092)),
        material=dark_cast,
        name="end_stop_0",
    )
    base_rail.visual(
        Box((0.050, 0.24, 0.105)),
        origin=Origin(xyz=(0.585, 0.0, 0.092)),
        material=dark_cast,
        name="end_stop_1",
    )
    for x in (-0.46, -0.23, 0.23, 0.46):
        for y in (-0.115, 0.115):
            base_rail.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(x, y, 0.044)),
                material=linear_steel,
                name=f"bed_bolt_{x}_{y}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.285, 0.225, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_cast,
        name="saddle_top",
    )
    for y, name in ((-0.096, "side_shoe_0"), (0.096, "side_shoe_1")):
        carriage.visual(
            Box((0.285, 0.032, 0.106)),
            origin=Origin(xyz=(0.0, y, -0.050)),
            material=painted_cast,
            name=name,
        )
        carriage.visual(
            Box((0.245, 0.010, 0.060)),
            origin=Origin(xyz=(0.0, y * 0.79, -0.045)),
            material=brass,
            name=f"gib_strip_{name[-1]}",
        )
    carriage.visual(
        Box((0.170, 0.150, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=dark_cast,
        name="riser_pad",
    )
    carriage.visual(
        Box((0.030, 0.110, 0.035)),
        origin=Origin(xyz=(-0.125, 0.0, 0.015)),
        material=dark_cast,
        name="front_clamp",
    )
    carriage.visual(
        Box((0.030, 0.110, 0.035)),
        origin=Origin(xyz=(0.125, 0.0, 0.015)),
        material=dark_cast,
        name="rear_clamp",
    )

    bearing_housing = model.part("bearing_housing")
    bearing_housing.visual(
        bearing_block,
        origin=Origin(),
        material=painted_cast,
        name="housing_block",
    )
    bearing_housing.visual(
        bearing_barrel,
        origin=Origin(),
        material=oiled_steel,
        name="bearing_barrel",
    )
    bearing_housing.visual(
        _cylinder_along_y(radius=0.050, length=0.165),
        origin=Origin(xyz=(0.0, -0.118, 0.068), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=motor_black,
        name="motor_can",
    )
    bearing_housing.visual(
        Box((0.105, 0.045, 0.034)),
        origin=Origin(xyz=(0.0, -0.058, 0.095)),
        material=motor_black,
        name="terminal_box",
    )
    bearing_housing.visual(
        Box((0.030, 0.130, 0.045)),
        origin=Origin(xyz=(-0.100, 0.0, -0.037)),
        material=dark_cast,
        name="mount_ear_0",
    )
    bearing_housing.visual(
        Box((0.030, 0.130, 0.045)),
        origin=Origin(xyz=(0.100, 0.0, -0.037)),
        material=dark_cast,
        name="mount_ear_1",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.018, length=0.325),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=linear_steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.035, length=0.070),
        origin=Origin(xyz=(0.0, 0.182, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oiled_steel,
        name="front_chuck",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.115),
        origin=Origin(xyz=(0.0, 0.272, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=linear_steel,
        name="tool_bit",
    )
    spindle.visual(
        Box((0.010, 0.045, 0.012)),
        origin=Origin(xyz=(0.034, 0.182, 0.0)),
        material=safety_orange,
        name="spin_mark",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=-0.34, upper=0.34),
    )
    model.articulation(
        "housing_mount",
        ArticulationType.FIXED,
        parent=carriage,
        child=bearing_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
    )
    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=bearing_housing,
        child=spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=420.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    housing = object_model.get_part("bearing_housing")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("carriage_slide")
    spin = object_model.get_articulation("spindle_spin")

    ctx.check(
        "carriage joint is prismatic along rail",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "spindle joint rotates about tool axis",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="saddle_top",
        negative_elem="rail_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="saddle bears on rail cap",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="y",
        inner_elem="saddle_top",
        outer_elem="base_plate",
        margin=0.0,
        name="carriage stays within rail bed width",
    )
    ctx.expect_gap(
        housing,
        carriage,
        axis="z",
        positive_elem="housing_block",
        negative_elem="riser_pad",
        max_gap=0.001,
        max_penetration=0.0,
        name="bearing housing is seated on riser pad",
    )
    ctx.expect_within(
        spindle,
        housing,
        axes="xz",
        inner_elem="spindle_shaft",
        outer_elem="housing_block",
        margin=0.0,
        name="spindle shaft is centered inside bearing block",
    )
    ctx.expect_overlap(
        spindle,
        housing,
        axes="y",
        elem_a="spindle_shaft",
        elem_b="bearing_barrel",
        min_overlap=0.16,
        name="spindle passes through bearing barrel",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.34}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="saddle_top",
            elem_b="rail_cap",
            min_overlap=0.20,
            name="extended carriage remains on rail",
        )
        extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage translates in positive rail direction",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({spin: 0.0}):
        mark_rest = ctx.part_element_world_aabb(spindle, elem="spin_mark")
    with ctx.pose({spin: math.pi / 2.0}):
        mark_quarter = ctx.part_element_world_aabb(spindle, elem="spin_mark")
    if mark_rest is not None and mark_quarter is not None:
        rest_center = tuple((mark_rest[0][i] + mark_rest[1][i]) * 0.5 for i in range(3))
        quarter_center = tuple((mark_quarter[0][i] + mark_quarter[1][i]) * 0.5 for i in range(3))
    else:
        rest_center = None
        quarter_center = None
    ctx.check(
        "visible spindle mark orbits around tool axis",
        rest_center is not None
        and quarter_center is not None
        and abs(rest_center[0] - quarter_center[0]) > 0.020
        and abs(rest_center[2] - quarter_center[2]) > 0.020,
        details=f"rest={rest_center}, quarter={quarter_center}",
    )

    return ctx.report()


object_model = build_object_model()
