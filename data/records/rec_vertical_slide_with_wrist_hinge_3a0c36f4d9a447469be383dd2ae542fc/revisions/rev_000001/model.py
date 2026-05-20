from __future__ import annotations

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
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


SLIDE_UPPER = 0.22
WRIST_UPPER = 0.80


def _carriage_block_mesh():
    """Compact linear-bearing carriage with rail and screw clearance bores."""
    block = cq.Workplane("XY").box(0.130, 0.065, 0.120)
    block = (
        block.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.040, -0.005), (0.040, -0.005)])
        .hole(0.0155)
    )
    block = (
        block.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, 0.005)])
        .hole(0.0098)
    )
    return block.edges("|Z").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lift_axis_wrist")

    painted = model.material("warm_light_gray_paint", color=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("blackened_steel", color=(0.03, 0.035, 0.04, 1.0))
    rail_mat = model.material("polished_linear_rail", color=(0.80, 0.82, 0.83, 1.0))
    carriage_mat = model.material("blue_anodized_carriage", color=(0.08, 0.20, 0.42, 1.0))
    screw_mat = model.material("brushed_lead_screw", color=(0.62, 0.58, 0.50, 1.0))
    face_mat = model.material("dark_tool_face", color=(0.015, 0.017, 0.018, 1.0))

    fixed_column = model.part("fixed_column")
    fixed_column.visual(
        Box((0.240, 0.180, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=painted,
        name="base_plate",
    )
    fixed_column.visual(
        Box((0.072, 0.060, 0.542)),
        origin=Origin(xyz=(0.0, -0.055, 0.294)),
        material=painted,
        name="column_spine",
    )
    for z, name in ((0.065, "lower_rail_clamp"), (0.515, "upper_rail_clamp")):
        fixed_column.visual(
            Box((0.135, 0.055, 0.025)),
            origin=Origin(xyz=(0.0, -0.005, z)),
            material=dark,
            name=name,
        )
    for x, name in ((-0.040, "guide_rail_0"), (0.040, "guide_rail_1")):
        fixed_column.visual(
            Cylinder(radius=0.008, length=0.460),
            origin=Origin(xyz=(x, 0.005, 0.290)),
            material=rail_mat,
            name=name,
        )
    fixed_column.visual(
        Cylinder(radius=0.005, length=0.462),
        origin=Origin(xyz=(0.0, 0.015, 0.291)),
        material=screw_mat,
        name="lead_screw",
    )
    fixed_column.visual(
        Box((0.080, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, -0.055, 0.595)),
        material=dark,
        name="top_motor",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_block_mesh(), "carriage_block", tolerance=0.0008),
        material=carriage_mat,
        name="bearing_block",
    )
    yoke_geometry = TrunnionYokeGeometry(
        (0.140, 0.066, 0.094),
        span_width=0.072,
        trunnion_diameter=0.024,
        trunnion_center_z=0.058,
        base_thickness=0.014,
        corner_radius=0.004,
        center=False,
    )
    carriage.visual(
        mesh_from_geometry(yoke_geometry, "carriage_yoke"),
        origin=Origin(xyz=(0.0, 0.058, -0.058)),
        material=carriage_mat,
        name="yoke",
    )
    carriage.visual(
        Box((0.090, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.042, -0.030)),
        material=carriage_mat,
        name="front_shelf",
    )

    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        Cylinder(radius=0.0075, length=0.134),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_mat,
        name="pivot_pin",
    )
    wrist_plate.visual(
        Cylinder(radius=0.014, length=0.058),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="center_barrel",
    )
    wrist_plate.visual(
        Box((0.030, 0.124, 0.020)),
        origin=Origin(xyz=(0.0, 0.074, -0.010)),
        material=dark,
        name="tilt_arm",
    )
    wrist_plate.visual(
        Box((0.092, 0.012, 0.095)),
        origin=Origin(xyz=(0.0, 0.138, -0.036)),
        material=painted,
        name="plate_body",
    )
    wrist_plate.visual(
        Box((0.072, 0.005, 0.070)),
        origin=Origin(xyz=(0.0, 0.1465, -0.036)),
        material=face_mat,
        name="tool_face",
    )
    for x in (-0.024, 0.024):
        for z in (-0.058, -0.014):
            wrist_plate.visual(
                Cylinder(radius=0.004, length=0.006),
                origin=Origin(xyz=(x, 0.151, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=rail_mat,
                name=f"face_screw_{x}_{z}",
            )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=fixed_column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.010, 0.175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=SLIDE_UPPER),
    )
    model.articulation(
        "wrist_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_plate,
        origin=Origin(xyz=(0.0, 0.058, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.20, upper=WRIST_UPPER),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_column = object_model.get_part("fixed_column")
    carriage = object_model.get_part("carriage")
    wrist_plate = object_model.get_part("wrist_plate")
    slide = object_model.get_articulation("column_slide")
    pivot = object_model.get_articulation("wrist_pivot")

    for guide in ("guide_rail_0", "guide_rail_1", "lead_screw"):
        ctx.allow_overlap(
            carriage,
            fixed_column,
            elem_a="bearing_block",
            elem_b=guide,
            reason=(
                "The rail/screw is intentionally represented as passing through "
                "the carriage bearing bore with slight proxy interference so the "
                "sliding support path is physically captured."
            ),
        )
        ctx.expect_within(
            fixed_column,
            carriage,
            axes="xy",
            inner_elem=guide,
            outer_elem="bearing_block",
            margin=0.004,
            name=f"{guide} runs through the bearing block footprint",
        )
        ctx.expect_overlap(
            carriage,
            fixed_column,
            axes="z",
            elem_a="bearing_block",
            elem_b=guide,
            min_overlap=0.10,
            name=f"{guide} has retained insertion through the bearing block at rest",
        )

    ctx.expect_overlap(
        carriage,
        fixed_column,
        axes="z",
        min_overlap=0.10,
        name="carriage remains engaged on the column at rest",
    )
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_UPPER}):
        for guide in ("guide_rail_0", "guide_rail_1", "lead_screw"):
            ctx.expect_within(
                fixed_column,
                carriage,
                axes="xy",
                inner_elem=guide,
                outer_elem="bearing_block",
                margin=0.004,
                name=f"{guide} stays centered in the lifted bearing block",
            )
            ctx.expect_overlap(
                carriage,
                fixed_column,
                axes="z",
                elem_a="bearing_block",
                elem_b=guide,
                min_overlap=0.10,
                name=f"{guide} retains insertion at full lift",
            )
        ctx.expect_overlap(
            carriage,
            fixed_column,
            axes="z",
            min_overlap=0.10,
            name="carriage remains engaged on the column at full lift",
        )
        lifted_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage translates upward along the fixed column",
        rest_pos is not None and lifted_pos is not None and lifted_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    ctx.expect_within(
        wrist_plate,
        carriage,
        axes="yz",
        inner_elem="pivot_pin",
        outer_elem="yoke",
        margin=0.003,
        name="wrist pin is captured inside the yoke envelope",
    )
    ctx.expect_overlap(
        wrist_plate,
        carriage,
        axes="x",
        elem_a="pivot_pin",
        elem_b="yoke",
        min_overlap=0.12,
        name="wrist pin spans the horizontal clevis axis",
    )

    rest_face = ctx.part_element_world_aabb(wrist_plate, elem="tool_face")
    with ctx.pose({pivot: WRIST_UPPER}):
        tilted_face = ctx.part_element_world_aabb(wrist_plate, elem="tool_face")
    rest_center_z = None if rest_face is None else (rest_face[0][2] + rest_face[1][2]) * 0.5
    tilted_center_z = None if tilted_face is None else (tilted_face[0][2] + tilted_face[1][2]) * 0.5
    ctx.check(
        "positive wrist rotation tilts the tool face upward",
        rest_center_z is not None
        and tilted_center_z is not None
        and tilted_center_z > rest_center_z + 0.07,
        details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
