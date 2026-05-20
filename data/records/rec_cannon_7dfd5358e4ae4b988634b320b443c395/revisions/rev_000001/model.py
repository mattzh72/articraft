from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _barrel_mesh():
    # Radius-vs-axis profile for a mid-19th-century cast iron field gun.
    # The polygon turns inward at the muzzle and follows the bore back toward
    # the breech, so the visible muzzle is a real hollow opening rather than a
    # painted-on circle.
    profile = [
        (0.000, -0.600),
        (0.050, -0.600),
        (0.075, -0.585),
        (0.088, -0.555),
        (0.078, -0.520),
        (0.055, -0.505),
        (0.058, -0.470),
        (0.132, -0.465),
        (0.150, -0.440),
        (0.185, -0.425),
        (0.205, -0.370),
        (0.190, -0.310),
        (0.174, -0.275),
        (0.170, -0.160),
        (0.192, -0.145),
        (0.194, -0.095),
        (0.171, -0.075),
        (0.160, 0.120),
        (0.178, 0.145),
        (0.178, 0.205),
        (0.156, 0.230),
        (0.142, 0.540),
        (0.134, 0.830),
        (0.128, 1.020),
        (0.150, 1.055),
        (0.154, 1.132),
        (0.129, 1.168),
        (0.116, 1.260),
        (0.123, 1.300),
        (0.046, 1.300),
        (0.046, -0.265),
        (0.000, -0.265),
    ]
    return LatheGeometry(profile, segments=80, closed=True).rotate_x(-math.pi / 2.0)


def _trail_beam_mesh(side_sign: float):
    profile = rounded_rect_profile(0.090, 0.125, 0.014, corner_segments=5)
    points = [
        (0.245 * side_sign, 0.230, 0.665),
        (0.240 * side_sign, -0.180, 0.555),
        (0.155 * side_sign, -0.760, 0.350),
        (0.060 * side_sign, -1.300, 0.230),
    ]
    return sweep_profile_along_spline(
        points,
        profile=profile,
        samples_per_segment=10,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _add_carriage_box(part, size, xyz, material, name, *, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civil_war_field_cannon")

    weathered_oak = model.material("weathered_oak", rgba=(0.48, 0.30, 0.16, 1.0))
    end_grain = model.material("end_grain", rgba=(0.38, 0.22, 0.12, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.05, 0.052, 0.048, 1.0))
    worn_iron = model.material("worn_iron", rgba=(0.12, 0.12, 0.115, 1.0))
    brass = model.material("brass_fittings", rgba=(0.72, 0.54, 0.24, 1.0))

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((1.55, 1.95, 0.95)),
        mass=430.0,
        origin=Origin(xyz=(0.0, -0.30, 0.45)),
    )

    # Axle and main wooden cheeks.
    carriage.visual(
        Cylinder(radius=0.055, length=1.28),
        origin=Origin(xyz=(0.0, -0.150, 0.460), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_oak,
        name="axle",
    )
    carriage.visual(
        Cylinder(radius=0.065, length=1.12),
        origin=Origin(xyz=(0.0, -0.150, 0.460), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_iron,
        name="axle_iron_skin",
    )
    for index, side_sign in enumerate((1.0, -1.0)):
        x = 0.285 * side_sign
        bearing_name = "bearing_block_0" if index == 0 else "bearing_block_1"
        _add_carriage_box(
            carriage,
            (0.105, 0.860, 0.185),
            (x, 0.015, 0.650),
            weathered_oak,
            f"cheek_{index}",
            rpy=(0.0, 0.0, 0.025 * -side_sign),
        )
        _add_carriage_box(
            carriage,
            (0.115, 0.270, 0.150),
            (x, 0.180, 0.690),
            end_grain,
            bearing_name,
        )
        _add_carriage_box(
            carriage,
            (0.120, 0.300, 0.030),
            (x, 0.180, 0.930),
            worn_iron,
            f"trunnion_cap_{index}",
        )
        _add_carriage_box(
            carriage,
            (0.122, 0.030, 0.250),
            (x, 0.064, 0.820),
            worn_iron,
            f"front_iron_strap_{index}",
        )
        _add_carriage_box(
            carriage,
            (0.122, 0.030, 0.250),
            (x, 0.296, 0.820),
            worn_iron,
            f"rear_iron_strap_{index}",
        )

    # Trail beams and cross members form the long rear wooden carriage.
    for index, side_sign in enumerate((1.0, -1.0)):
        carriage.visual(
            mesh_from_geometry(_trail_beam_mesh(side_sign), f"trail_beam_{index}"),
            material=weathered_oak,
            name=f"trail_beam_{index}",
        )
    _add_carriage_box(
        carriage,
        (0.650, 0.110, 0.110),
        (0.0, 0.030, 0.575),
        weathered_oak,
        "front_transom",
    )
    _add_carriage_box(
        carriage,
        (0.580, 0.100, 0.100),
        (0.0, -0.430, 0.445),
        weathered_oak,
        "middle_transom",
    )
    _add_carriage_box(
        carriage,
        (0.240, 0.130, 0.100),
        (0.0, -1.190, 0.245),
        weathered_oak,
        "trail_tail",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.610),
        origin=Origin(xyz=(0.0, 0.420, 0.540), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_iron,
        name="front_tie_rod",
    )
    carriage.visual(
        Cylinder(radius=0.020, length=0.410),
        origin=Origin(xyz=(0.0, -0.820, 0.315), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_iron,
        name="rear_tie_rod",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.180),
        origin=Origin(xyz=(0.0, -1.345, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="lunette_ring",
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_barrel_mesh(), "cast_iron_barrel"),
        material=cast_iron,
        name="barrel_casting",
    )
    barrel.visual(
        Cylinder(radius=0.065, length=0.780),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="trunnion_pin",
    )
    barrel.visual(
        Cylinder(radius=0.035, length=0.160),
        origin=Origin(xyz=(0.0, -0.575, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="cascabel_ring",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=1.90),
        mass=360.0,
        origin=Origin(xyz=(0.0, 0.335, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.405,
            0.120,
            rim=WheelRim(
                inner_radius=0.292,
                flange_height=0.018,
                flange_thickness=0.012,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.102,
                width=0.190,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=8, circle_diameter=0.135, hole_diameter=0.012),
            ),
            spokes=WheelSpokes(style="straight", count=12, thickness=0.018, window_radius=0.055),
            bore=WheelBore(style="round", diameter=0.130),
        ),
        "wooden_spoked_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.455,
            0.112,
            inner_radius=0.398,
            carcass=TireCarcass(belt_width_ratio=0.94, sidewall_bulge=0.01),
            sidewall=TireSidewall(style="square", bulge=0.01),
            shoulder=TireShoulder(width=0.010, radius=0.002),
        ),
        "iron_wheel_tire",
    )
    for index, side_sign in enumerate((1.0, -1.0)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(wheel_mesh, material=weathered_oak, name="wooden_wheel")
        wheel.visual(tire_mesh, material=worn_iron, name="iron_tire")
        wheel.visual(
            Cylinder(radius=0.075, length=0.038),
            origin=Origin(xyz=(0.074 * side_sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_iron,
            name="hub_band",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.455, length=0.120),
            mass=62.0,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

        model.articulation(
            f"wheel_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(0.652 * side_sign, -0.150, 0.460)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=150.0, velocity=20.0),
        )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.180, 0.830)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=-0.08, upper=0.48),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    elevation = object_model.get_articulation("barrel_elevation")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    ctx.check(
        "barrel elevates on trunnion axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in elevation.axis) == (1.0, 0.0, 0.0),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    ctx.check(
        "wheels spin on axle axes",
        wheel_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_1_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_0_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 6) for v in wheel_1_spin.axis) == (1.0, 0.0, 0.0),
        details=f"wheel_0={wheel_0_spin.axis}, wheel_1={wheel_1_spin.axis}",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a="trunnion_pin",
        elem_b="bearing_block_0",
        contact_tol=0.004,
        name="trunnion pin sits in first bearing",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a="trunnion_pin",
        elem_b="bearing_block_1",
        contact_tol=0.004,
        name="trunnion pin sits in second bearing",
    )
    ctx.expect_origin_gap(
        wheel_0,
        wheel_1,
        axis="x",
        min_gap=1.20,
        max_gap=1.40,
        name="wheels are on opposite sides of the carriage",
    )

    rest_aabb = ctx.part_world_aabb(barrel)
    with ctx.pose({elevation: 0.42}):
        raised_aabb = ctx.part_world_aabb(barrel)
    ctx.check(
        "positive elevation raises the muzzle",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.18,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
