from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _c_channel_shape(width: float, depth: float, wall: float, height: float) -> cq.Workplane:
    """Return a vertical C-channel extrusion, open toward local -Y."""
    profile = [
        (-width / 2.0, -depth / 2.0),
        (-width / 2.0, depth / 2.0),
        (width / 2.0, depth / 2.0),
        (width / 2.0, -depth / 2.0),
        (width / 2.0 - wall, -depth / 2.0),
        (width / 2.0 - wall, depth / 2.0 - wall),
        (-width / 2.0 + wall, depth / 2.0 - wall),
        (-width / 2.0 + wall, -depth / 2.0),
    ]
    return cq.Workplane("XY").polyline(profile).close().extrude(height)


def _fork_tine_geometry(
    *,
    width: float = 0.075,
    length: float = 0.56,
    thickness: float = 0.045,
    tip_thickness: float = 0.016,
    taper_length: float = 0.13,
) -> MeshGeometry:
    """Closed prism for a short pallet fork tine with a tapered front nose."""
    rear_y = length / 2.0
    front_y = -length / 2.0
    taper_y = front_y + taper_length
    bottom_z = -thickness / 2.0
    top_z = thickness / 2.0
    tip_top_z = bottom_z + tip_thickness

    side_profile = [
        (rear_y, bottom_z),
        (rear_y, top_z),
        (taper_y, top_z),
        (front_y, tip_top_z),
        (front_y, bottom_z),
    ]

    geom = MeshGeometry()
    for x in (-width / 2.0, width / 2.0):
        for y, z in side_profile:
            geom.add_vertex(x, y, z)

    n = len(side_profile)
    # End caps.
    geom.add_face(0, 1, 2)
    geom.add_face(0, 2, 3)
    geom.add_face(0, 3, 4)
    geom.add_face(n, n + 2, n + 1)
    geom.add_face(n, n + 3, n + 2)
    geom.add_face(n, n + 4, n + 3)

    # Side walls.
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(i, j, j + n)
        geom.add_face(i, j + n, i + n)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_fork_carriage")

    mast_steel = model.material("dark_mast_steel", rgba=(0.12, 0.15, 0.18, 1.0))
    carriage_yellow = model.material("safety_yellow", rgba=(1.0, 0.66, 0.05, 1.0))
    fork_steel = model.material("worn_fork_steel", rgba=(0.08, 0.08, 0.075, 1.0))
    roller_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    pin_steel = model.material("bright_pin_steel", rgba=(0.55, 0.58, 0.56, 1.0))

    mast = model.part("mast")
    rail_mesh = mesh_from_cadquery(
        _c_channel_shape(width=0.085, depth=0.085, wall=0.014, height=2.00),
        "mast_c_channel",
        tolerance=0.0008,
    )
    mast.visual(
        rail_mesh,
        origin=Origin(xyz=(-0.23, 0.0, 0.04)),
        material=mast_steel,
        name="rail_0",
    )
    mast.visual(
        rail_mesh,
        origin=Origin(xyz=(0.23, 0.0, 0.04)),
        material=mast_steel,
        name="rail_1",
    )

    mast.visual(
        Box((0.61, 0.11, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 2.03)),
        material=mast_steel,
        name="top_crosshead",
    )
    mast.visual(
        Box((0.55, 0.085, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=mast_steel,
        name="bottom_tie",
    )
    mast.visual(
        Box((0.50, 0.035, 0.06)),
        origin=Origin(xyz=(0.0, 0.055, 1.18)),
        material=mast_steel,
        name="rear_bridge",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.50, 0.055, 0.32)),
        origin=Origin(xyz=(0.0, -0.10, -0.065)),
        material=carriage_yellow,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.48, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, -0.098, 0.120)),
        material=carriage_yellow,
        name="upper_carriage_bar",
    )
    carriage.visual(
        Box((0.48, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, -0.098, -0.255)),
        material=carriage_yellow,
        name="lower_carriage_bar",
    )

    # Load backrest frame: vertical guard bars welded to the compact carriage.
    carriage.visual(
        Box((0.46, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, -0.130, 0.175)),
        material=carriage_yellow,
        name="backrest_lower_rail",
    )
    carriage.visual(
        Box((0.46, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, -0.130, 0.715)),
        material=carriage_yellow,
        name="backrest_top_rail",
    )
    for idx, x in enumerate((-0.21, -0.07, 0.07, 0.21)):
        carriage.visual(
            Box((0.035, 0.038, 0.56)),
            origin=Origin(xyz=(x, -0.130, 0.445)),
            material=carriage_yellow,
            name=f"backrest_bar_{idx}",
        )

    # Forks are short pallet tines with upright heels latched to the carriage.
    fork_tine_mesh = mesh_from_geometry(_fork_tine_geometry(), "tapered_fork_tine")
    carriage.visual(
        Box((0.085, 0.060, 0.31)),
        origin=Origin(xyz=(-0.13, -0.112, -0.175)),
        material=fork_steel,
        name="fork_heel_0",
    )
    carriage.visual(
        fork_tine_mesh,
        origin=Origin(xyz=(-0.13, -0.385, -0.305)),
        material=fork_steel,
        name="fork_0",
    )
    carriage.visual(
        Box((0.085, 0.060, 0.31)),
        origin=Origin(xyz=(0.13, -0.112, -0.175)),
        material=fork_steel,
        name="fork_heel_1",
    )
    carriage.visual(
        fork_tine_mesh,
        origin=Origin(xyz=(0.13, -0.385, -0.305)),
        material=fork_steel,
        name="fork_1",
    )

    # Four guide rollers sit inside the two C-channel mouths and visibly capture
    # the moving carriage on the mast rails.
    carriage.visual(
        Box((0.055, 0.045, 0.41)),
        origin=Origin(xyz=(-0.23, -0.107, -0.030)),
        material=carriage_yellow,
        name="guide_bracket_0",
    )
    carriage.visual(
        Box((0.055, 0.045, 0.41)),
        origin=Origin(xyz=(0.23, -0.107, -0.030)),
        material=carriage_yellow,
        name="guide_bracket_1",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(-0.23, -0.084, -0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="roller_axle_0_0",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(-0.23, -0.0515, -0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_rubber,
        name="guide_roller_0_0",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(-0.23, -0.084, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="roller_axle_0_1",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(-0.23, -0.0515, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_rubber,
        name="guide_roller_0_1",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.23, -0.084, -0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="roller_axle_1_0",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.23, -0.0515, -0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_rubber,
        name="guide_roller_1_0",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.23, -0.084, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="roller_axle_1_1",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.23, -0.0515, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_rubber,
        name="guide_roller_1_1",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=0.45, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single vertical prismatic carriage joint",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={object_model.articulations}, axis={slide.axis}",
    )

    ctx.expect_overlap(
        carriage,
        mast,
        axes="x",
        elem_a="guide_roller_0_0",
        elem_b="rail_0",
        min_overlap=0.050,
        name="left lower roller is seated in rail width",
    )
    ctx.expect_gap(
        mast,
        carriage,
        axis="y",
        positive_elem="rail_0",
        negative_elem="guide_roller_0_0",
        max_penetration=0.0002,
        max_gap=0.001,
        name="roller kisses rail mouth without penetration",
    )

    fork_bb = ctx.part_element_world_aabb(carriage, elem="fork_0")
    plate_bb = ctx.part_element_world_aabb(carriage, elem="carriage_plate")
    ctx.check(
        "forks project forward from compact carriage",
        fork_bb is not None
        and plate_bb is not None
        and fork_bb[0][1] < plate_bb[0][1] - 0.45,
        details=f"fork_bb={fork_bb}, plate_bb={plate_bb}",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.75}):
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="guide_roller_1_1",
            elem_b="rail_1",
            min_overlap=0.060,
            name="raised carriage remains captured on rail",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage raises on the mast",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
