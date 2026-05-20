from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
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
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_swivel_bar_stool")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.30, 0.32, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.66, 0.08, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.02, 0.022, 0.025, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    bronze = model.material("bronze_wear", rgba=(0.72, 0.48, 0.20, 1.0))
    nylon = model.material("ivory_nylon", rgba=(0.88, 0.82, 0.66, 1.0))
    bolt_metal = model.material("zinc_bolts", rgba=(0.70, 0.72, 0.74, 1.0))

    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.305, tube=0.019, radial_segments=20, tubular_segments=80),
        "foot_ring_tube",
    )
    lower_race_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.105, tube=0.018, radial_segments=18, tubular_segments=72),
        "lower_bearing_race",
    )
    upper_race_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.105, tube=0.018, radial_segments=18, tubular_segments=72),
        "upper_bearing_race",
    )
    nylon_wear_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.165, tube=0.007, radial_segments=12, tubular_segments=72),
        "replaceable_wear_ring",
    )
    cushion_profile = [
        (0.000, 0.143),
        (0.170, 0.143),
        (0.214, 0.150),
        (0.235, 0.178),
        (0.226, 0.210),
        (0.185, 0.228),
        (0.000, 0.230),
    ]
    cushion_mesh = mesh_from_geometry(
        LatheGeometry(cushion_profile, segments=80),
        "rounded_vinyl_seat",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.345, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=gunmetal,
        name="floor_plate",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pedestal.visual(
            Cylinder(radius=0.040, length=0.016),
            origin=Origin(xyz=(0.255 * math.cos(angle), 0.255 * math.sin(angle), 0.004)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )
    pedestal.visual(
        Cylinder(radius=0.076, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=dark_steel,
        name="center_column",
    )
    pedestal.visual(
        Cylinder(radius=0.140, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=gunmetal,
        name="base_weld_collar",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pedestal.visual(
            Box((0.250, 0.030, 0.165)),
            origin=Origin(
                xyz=(0.115 * math.cos(angle), 0.115 * math.sin(angle), 0.130),
                rpy=(0.0, 0.0, angle),
            ),
            material=safety_yellow,
            name=f"weld_gusset_{index}",
        )
    pedestal.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=dark_steel,
        name="foot_ring",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        a = (0.070 * math.cos(angle), 0.070 * math.sin(angle), 0.300)
        b = (0.305 * math.cos(angle), 0.305 * math.sin(angle), 0.300)
        _add_member(pedestal, a, b, 0.015, gunmetal, name=f"foot_ring_spoke_{index}")

    pedestal.visual(
        Box((0.018, 0.125, 0.185)),
        origin=Origin(xyz=(0.083, 0.0, 0.405)),
        material=safety_yellow,
        name="service_cover",
    )
    pedestal.visual(
        Box((0.012, 0.018, 0.168)),
        origin=Origin(xyz=(0.097, -0.064, 0.405)),
        material=gunmetal,
        name="cover_hinge_leaf",
    )
    for index, (y, z) in enumerate(((-0.046, 0.334), (0.046, 0.334), (-0.046, 0.476), (0.046, 0.476))):
        pedestal.visual(
            Cylinder(radius=0.009, length=0.014),
            origin=Origin(xyz=(0.0985, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_metal,
            name=f"cover_bolt_{index}",
        )

    pedestal.visual(
        Cylinder(radius=0.092, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=gunmetal,
        name="top_clamp_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.136, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=dark_steel,
        name="bearing_shelf",
    )
    pedestal.visual(
        lower_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        material=bolt_metal,
        name="lower_race",
    )
    pedestal.visual(
        Cylinder(radius=0.011, length=0.085),
        origin=Origin(xyz=(0.133, 0.0, 0.663), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="grease_fitting_body",
    )
    pedestal.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.182, 0.0, 0.663)),
        material=bolt_metal,
        name="grease_fitting_cap",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.35, length=0.70),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        upper_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.035454)),
        material=bolt_metal,
        name="upper_race",
    )
    seat_stage.visual(
        Cylinder(radius=0.174, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=gunmetal,
        name="rotating_flange",
    )
    seat_stage.visual(
        nylon_wear_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=nylon,
        name="wear_ring",
    )
    seat_stage.visual(
        Cylinder(radius=0.083, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=dark_steel,
        name="swivel_hub",
    )
    seat_stage.visual(
        Cylinder(radius=0.205, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=gunmetal,
        name="seat_mount_plate",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        seat_stage.visual(
            Box((0.280, 0.035, 0.030)),
            origin=Origin(
                xyz=(0.090 * math.cos(angle), 0.090 * math.sin(angle), 0.123),
                rpy=(0.0, 0.0, angle),
            ),
            material=safety_yellow,
            name=f"seat_spider_arm_{index}",
        )
    seat_stage.visual(
        Cylinder(radius=0.238, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=dark_steel,
        name="steel_seat_pan",
    )
    for index in range(8):
        angle = index * 2.0 * math.pi / 8.0
        seat_stage.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(0.172 * math.cos(angle), 0.172 * math.sin(angle), 0.121)),
            material=bolt_metal,
            name=f"flange_bolt_{index}",
        )
    seat_stage.visual(
        cushion_mesh,
        material=black_vinyl,
        name="vinyl_cushion",
    )
    seat_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.23),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat_stage = object_model.get_part("seat_stage")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_origin_distance(
        pedestal,
        seat_stage,
        axes="xy",
        max_dist=0.001,
        name="swivel axis centered on pedestal",
    )
    ctx.expect_gap(
        seat_stage,
        pedestal,
        axis="z",
        positive_elem="upper_race",
        negative_elem="lower_race",
        max_gap=0.0006,
        max_penetration=0.0001,
        name="bearing races are seated on rolling contact",
    )
    ctx.expect_overlap(
        seat_stage,
        pedestal,
        axes="xy",
        elem_a="upper_race",
        elem_b="lower_race",
        min_overlap=0.18,
        name="upper and lower bearing races remain coaxial",
    )

    rest_pos = ctx.part_world_position(seat_stage)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(seat_stage)
        ctx.expect_origin_distance(
            pedestal,
            seat_stage,
            axes="xy",
            max_dist=0.001,
            name="swivel keeps seat centered after rotation",
        )
    ctx.check(
        "seat swivel has no translational drift",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 0.001
        and abs(rest_pos[1] - turned_pos[1]) < 0.001,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
