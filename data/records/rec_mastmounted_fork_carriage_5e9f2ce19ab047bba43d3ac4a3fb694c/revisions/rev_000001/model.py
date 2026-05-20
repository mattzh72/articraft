from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    return (0.0, math.atan2(length_xy, dz), math.atan2(dy, dx))


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _fork_blade_geometry(
    *,
    x0: float,
    x1: float,
    y_center: float,
    width: float,
    bottom_z: float,
    root_thickness: float,
    tip_thickness: float,
) -> MeshGeometry:
    half_w = width * 0.5
    y0 = y_center - half_w
    y1 = y_center + half_w
    z0 = bottom_z
    z_root = bottom_z + root_thickness
    z_tip = bottom_z + tip_thickness

    geom = MeshGeometry()
    vertices = [
        (x0, y0, z0),
        (x0, y1, z0),
        (x1, y1, z0),
        (x1, y0, z0),
        (x0, y0, z_root),
        (x0, y1, z_root),
        (x1, y1, z_tip),
        (x1, y0, z_tip),
    ]
    for vertex in vertices:
        geom.add_vertex(*vertex)
    for face in (
        (0, 2, 1),
        (0, 3, 2),
        (0, 1, 5),
        (0, 5, 4),
        (3, 7, 6),
        (3, 6, 2),
        (0, 4, 7),
        (0, 7, 3),
        (1, 2, 6),
        (1, 6, 5),
        (4, 5, 6),
        (4, 6, 7),
    ):
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_service_fork_carriage")

    mast_yellow = model.material("mast_yellow", rgba=(0.88, 0.62, 0.10, 1.0))
    carriage_orange = model.material("carriage_orange", rgba=(0.92, 0.45, 0.10, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.46, 0.47, 0.46, 1.0))
    polished_rail = model.material("polished_rail", rgba=(0.72, 0.73, 0.70, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.44, 0.58, 0.050)),
        origin=Origin(xyz=(-0.09, 0.0, 0.025)),
        material=dark_steel,
        name="floor_plate",
    )
    mast.visual(
        Box((0.15, 0.54, 0.060)),
        origin=Origin(xyz=(-0.105, 0.0, 0.080)),
        material=mast_yellow,
        name="base_crossmember",
    )
    mast.visual(
        Box((0.085, 0.54, 0.065)),
        origin=Origin(xyz=(-0.080, 0.0, 1.435)),
        material=mast_yellow,
        name="top_crossmember",
    )
    mast.visual(
        Box((0.075, 0.48, 0.050)),
        origin=Origin(xyz=(-0.080, 0.0, 0.745)),
        material=mast_yellow,
        name="rear_tiebar",
    )

    mast.visual(
        Box((0.055, 0.042, 1.300)),
        origin=Origin(xyz=(-0.060, -0.215, 0.760)),
        material=dark_steel,
        name="upright_0",
    )
    mast.visual(
        Box((0.012, 0.026, 1.180)),
        origin=Origin(xyz=(-0.029, -0.215, 0.800)),
        material=polished_rail,
        name="guide_face_0",
    )
    mast.visual(
        Box((0.060, 0.014, 1.260)),
        origin=Origin(xyz=(-0.095, -0.187, 0.780)),
        material=mast_yellow,
        name="channel_lip_0",
    )
    _add_member(
        mast,
        (-0.245, -0.215, 0.060),
        (-0.080, -0.215, 1.365),
        0.012,
        mast_yellow,
        name="rear_brace_0",
    )
    mast.visual(
        Box((0.055, 0.042, 1.300)),
        origin=Origin(xyz=(-0.060, 0.215, 0.760)),
        material=dark_steel,
        name="upright_1",
    )
    mast.visual(
        Box((0.012, 0.026, 1.180)),
        origin=Origin(xyz=(-0.029, 0.215, 0.800)),
        material=polished_rail,
        name="guide_face_1",
    )
    mast.visual(
        Box((0.060, 0.014, 1.260)),
        origin=Origin(xyz=(-0.095, 0.187, 0.780)),
        material=mast_yellow,
        name="channel_lip_1",
    )
    _add_member(
        mast,
        (-0.245, 0.215, 0.060),
        (-0.080, 0.215, 1.365),
        0.012,
        mast_yellow,
        name="rear_brace_1",
    )

    for index, y in enumerate((-0.110, 0.110)):
        mast.visual(
            Cylinder(radius=0.0045, length=1.205),
            origin=Origin(xyz=(-0.012, y, 0.788)),
            material=dark_steel,
            name=f"lift_chain_{index}",
        )
        mast.visual(
            Cylinder(radius=0.032, length=0.032),
            origin=Origin(xyz=(-0.013, y, 1.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"chain_sheave_{index}",
        )

    mast.visual(
        Cylinder(radius=0.030, length=0.430),
        origin=Origin(xyz=(-0.135, 0.0, 0.360)),
        material=dark_steel,
        name="lift_cylinder_body",
    )
    mast.visual(
        Box((0.080, 0.085, 0.055)),
        origin=Origin(xyz=(-0.135, 0.0, 0.135)),
        material=dark_steel,
        name="cylinder_foot_bracket",
    )
    mast.visual(
        Cylinder(radius=0.016, length=0.720),
        origin=Origin(xyz=(-0.135, 0.0, 0.820)),
        material=polished_rail,
        name="lift_cylinder_rod",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.48, 0.60, 1.48)),
        mass=42.0,
        origin=Origin(xyz=(-0.08, 0.0, 0.74)),
    )

    platen = model.part("platen")
    platen.visual(
        Box((0.035, 0.400, 0.320)),
        origin=Origin(xyz=(0.025, 0.0, 0.160)),
        material=carriage_orange,
        name="back_plate",
    )
    platen.visual(
        Box((0.075, 0.470, 0.065)),
        origin=Origin(xyz=(0.035, 0.0, -0.075)),
        material=carriage_orange,
        name="lower_bar",
    )
    platen.visual(
        Box((0.075, 0.470, 0.055)),
        origin=Origin(xyz=(0.035, 0.0, 0.315)),
        material=carriage_orange,
        name="upper_bar",
    )

    platen.visual(
        mesh_from_geometry(
            _fork_blade_geometry(
                x0=0.075,
                x1=0.760,
                y_center=-0.110,
                width=0.055,
                bottom_z=-0.145,
                root_thickness=0.044,
                tip_thickness=0.018,
            ),
            "fork_blade_mesh_0",
        ),
        material=worn_steel,
        name="fork_blade_0",
    )
    platen.visual(
        Box((0.070, 0.055, 0.350)),
        origin=Origin(xyz=(0.072, -0.110, 0.035)),
        material=dark_steel,
        name="fork_shank_0",
    )
    platen.visual(
        Box((0.086, 0.065, 0.036)),
        origin=Origin(xyz=(0.056, -0.110, 0.238)),
        material=worn_steel,
        name="fork_hook_0",
    )
    platen.visual(
        mesh_from_geometry(
            _fork_blade_geometry(
                x0=0.075,
                x1=0.760,
                y_center=0.110,
                width=0.055,
                bottom_z=-0.145,
                root_thickness=0.044,
                tip_thickness=0.018,
            ),
            "fork_blade_mesh_1",
        ),
        material=worn_steel,
        name="fork_blade_1",
    )
    platen.visual(
        Box((0.070, 0.055, 0.350)),
        origin=Origin(xyz=(0.072, 0.110, 0.035)),
        material=dark_steel,
        name="fork_shank_1",
    )
    platen.visual(
        Box((0.086, 0.065, 0.036)),
        origin=Origin(xyz=(0.056, 0.110, 0.238)),
        material=worn_steel,
        name="fork_hook_1",
    )

    platen.visual(
        Box((0.055, 0.040, 0.110)),
        origin=Origin(xyz=(-0.045, -0.160, 0.055)),
        material=dark_steel,
        name="lower_guide_shoe_0",
    )
    platen.visual(
        Box((0.065, 0.038, 0.070)),
        origin=Origin(xyz=(-0.005, -0.160, 0.055)),
        material=dark_steel,
        name="lower_guide_mount_0",
    )
    platen.visual(
        Box((0.055, 0.040, 0.110)),
        origin=Origin(xyz=(-0.045, -0.160, 0.245)),
        material=dark_steel,
        name="upper_guide_shoe_0",
    )
    platen.visual(
        Box((0.065, 0.038, 0.070)),
        origin=Origin(xyz=(-0.005, -0.160, 0.245)),
        material=dark_steel,
        name="upper_guide_mount_0",
    )
    platen.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.038, -0.193, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="lower_roller_0",
    )
    platen.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(-0.038, -0.181, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="lower_roller_axle_0",
    )
    platen.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.038, -0.193, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="upper_roller_0",
    )
    platen.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(-0.038, -0.181, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="upper_roller_axle_0",
    )
    platen.visual(
        Box((0.055, 0.040, 0.110)),
        origin=Origin(xyz=(-0.045, 0.160, 0.055)),
        material=dark_steel,
        name="lower_guide_shoe_1",
    )
    platen.visual(
        Box((0.065, 0.038, 0.070)),
        origin=Origin(xyz=(-0.005, 0.160, 0.055)),
        material=dark_steel,
        name="lower_guide_mount_1",
    )
    platen.visual(
        Box((0.055, 0.040, 0.110)),
        origin=Origin(xyz=(-0.045, 0.160, 0.245)),
        material=dark_steel,
        name="upper_guide_shoe_1",
    )
    platen.visual(
        Box((0.065, 0.038, 0.070)),
        origin=Origin(xyz=(-0.005, 0.160, 0.245)),
        material=dark_steel,
        name="upper_guide_mount_1",
    )
    platen.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.038, 0.193, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="lower_roller_1",
    )
    platen.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(-0.038, 0.181, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="lower_roller_axle_1",
    )
    platen.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.038, 0.193, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="upper_roller_1",
    )
    platen.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(-0.038, 0.181, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="upper_roller_axle_1",
    )

    platen.visual(
        Box((0.040, 0.480, 0.035)),
        origin=Origin(xyz=(0.070, 0.0, 0.300)),
        material=carriage_orange,
        name="guard_bottom_rail",
    )
    platen.visual(
        Box((0.035, 0.480, 0.032)),
        origin=Origin(xyz=(0.070, 0.0, 0.585)),
        material=carriage_orange,
        name="guard_middle_rail",
    )
    platen.visual(
        Box((0.035, 0.480, 0.035)),
        origin=Origin(xyz=(0.070, 0.0, 0.885)),
        material=carriage_orange,
        name="guard_top_rail",
    )
    for index, y in enumerate((-0.225, 0.225)):
        platen.visual(
            Box((0.035, 0.026, 0.620)),
            origin=Origin(xyz=(0.070, y, 0.600)),
            material=carriage_orange,
            name=f"guard_side_post_{index}",
        )
    for index, y in enumerate((-0.075, 0.075, 0.0)):
        platen.visual(
            Box((0.026, 0.018, 0.570)),
            origin=Origin(xyz=(0.073, y, 0.595)),
            material=dark_steel,
            name=f"guard_upright_{index}",
        )
    platen.visual(
        Box((0.024, 0.420, 0.024)),
        origin=Origin(xyz=(0.074, 0.0, 0.445)),
        material=dark_steel,
        name="guard_lower_mesh_rail",
    )
    platen.visual(
        Box((0.024, 0.420, 0.024)),
        origin=Origin(xyz=(0.074, 0.0, 0.735)),
        material=dark_steel,
        name="guard_upper_mesh_rail",
    )
    platen.inertial = Inertial.from_geometry(
        Box((0.84, 0.50, 1.05)),
        mass=28.0,
        origin=Origin(xyz=(0.20, 0.0, 0.32)),
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=platen,
        origin=Origin(xyz=(0.025, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.25, lower=0.0, upper=0.450),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    platen = object_model.get_part("platen")
    lift = object_model.get_articulation("mast_lift")

    limits = lift.motion_limits
    ctx.check(
        "single vertical prismatic lift",
        lift.articulation_type == ArticulationType.PRISMATIC
        and lift.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper == 0.450,
        details=f"type={lift.articulation_type}, axis={lift.axis}, limits={limits}",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_gap(
            platen,
            mast,
            axis="z",
            min_gap=0.015,
            positive_elem="fork_blade_0",
            negative_elem="floor_plate",
            name="fork blades clear the floor plate",
        )
        ctx.expect_overlap(
            platen,
            mast,
            axes="xz",
            min_overlap=0.010,
            elem_a="lower_roller_0",
            elem_b="guide_face_0",
            name="platen is guided on the mast",
        )
        ctx.expect_contact(
            platen,
            mast,
            elem_a="lower_roller_0",
            elem_b="guide_face_0",
            contact_tol=0.001,
            name="guide roller bears on mast rail",
        )
        rest_position = ctx.part_world_position(platen)

    with ctx.pose({lift: 0.450}):
        ctx.expect_within(
            platen,
            mast,
            axes="y",
            margin=0.010,
            inner_elem="guard_top_rail",
            outer_elem="top_crossmember",
            name="raised guard stays within mast width",
        )
        ctx.expect_overlap(
            platen,
            mast,
            axes="z",
            min_overlap=0.25,
            elem_a="back_plate",
            elem_b="upright_0",
            name="raised platen remains engaged with rails",
        )
        raised_position = ctx.part_world_position(platen)

    ctx.check(
        "lift raises the platen",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.40,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
