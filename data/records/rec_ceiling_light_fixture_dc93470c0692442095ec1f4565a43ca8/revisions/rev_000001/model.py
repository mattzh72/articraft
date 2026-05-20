from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _lathe(profile: list[tuple[float, float]], name: str):
    return mesh_from_geometry(LatheGeometry(profile, segments=72, closed=True), name)


def _torus(major: float, tube: float, name: str):
    return mesh_from_geometry(
        TorusGeometry(major, tube, radial_segments=18, tubular_segments=72),
        name,
    )


def _tube(points, radius: float, name: str, *, closed: bool = False, samples: int = 16):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=12,
            closed_spline=closed,
            cap_ends=not closed,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_edison_pull_chain_pendant")

    aged_brass = _mat(model, "aged_brass", (0.67, 0.47, 0.21, 1.0))
    dark_bronze = _mat(model, "dark_bronze", (0.23, 0.15, 0.08, 1.0))
    black_rubber = _mat(model, "black_rubber", (0.01, 0.009, 0.008, 1.0))
    warm_glass = _mat(model, "warm_smoked_glass", (1.0, 0.72, 0.36, 0.32))
    amber_glow = _mat(model, "warm_tungsten", (1.0, 0.47, 0.08, 1.0))
    nickel = _mat(model, "dull_nickel", (0.62, 0.57, 0.49, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        _lathe(
            [
                (0.014, 0.018),
                (0.095, 0.018),
                (0.108, 0.008),
                (0.102, -0.010),
                (0.070, -0.038),
                (0.014, -0.040),
            ],
            "canopy_shell",
        ),
        material=aged_brass,
        name="canopy_shell",
    )
    canopy.visual(
        _torus(0.073, 0.0035, "canopy_bead"),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=dark_bronze,
        name="canopy_bead",
    )
    for i, x in enumerate((-0.058, 0.058)):
        canopy.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.0195)),
            material=dark_bronze,
            name=f"screw_head_{i}",
        )

    cord_grip = model.part("cord_grip")
    cord_grip.visual(
        Cylinder(radius=0.004, length=0.90),
        # The cord intentionally has a hidden upward tail so it remains threaded
        # through the canopy throughout the short adjustment stroke.
        origin=Origin(xyz=(0.0, 0.0, -0.330)),
        material=black_rubber,
        name="cord",
    )
    cord_grip.visual(
        Cylinder(radius=0.019, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=dark_bronze,
        name="grip_collar",
    )
    cord_grip.visual(
        _torus(0.0195, 0.0015, "grip_knurl_0"),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=aged_brass,
        name="grip_knurl_0",
    )
    cord_grip.visual(
        _torus(0.0195, 0.0015, "grip_knurl_1"),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=aged_brass,
        name="grip_knurl_1",
    )
    cord_grip.visual(
        _torus(0.0195, 0.0015, "grip_knurl_2"),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=aged_brass,
        name="grip_knurl_2",
    )

    model.articulation(
        "canopy_to_cord_grip",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=cord_grip,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.12, lower=0.0, upper=0.12),
    )

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.014, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_bronze,
        name="cord_ferrule",
    )
    housing.visual(
        Cylinder(radius=0.036, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=aged_brass,
        name="socket_shell",
    )
    housing.visual(
        _torus(0.035, 0.004, "socket_top_bead"),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark_bronze,
        name="socket_top_bead",
    )
    housing.visual(
        _torus(0.036, 0.0045, "socket_lower_bead"),
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        material=dark_bronze,
        name="socket_lower_bead",
    )
    housing.visual(
        _torus(0.074, 0.0045, "top_guard_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=aged_brass,
        name="top_guard_ring",
    )
    housing.visual(
        _torus(0.084, 0.0045, "bottom_guard_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        material=aged_brass,
        name="bottom_guard_ring",
    )
    for i, angle in enumerate(math.radians(a) for a in (35, 105, 175, 245, 315)):
        x = 0.078 * math.cos(angle)
        y = 0.078 * math.sin(angle)
        housing.visual(
            Cylinder(radius=0.0033, length=0.198),
            origin=Origin(xyz=(x, y, -0.190)),
            material=aged_brass,
            name=f"guard_rod_{i}",
        )
    housing.visual(
        Box((0.034, 0.008, 0.040)),
        origin=Origin(xyz=(0.0, -0.026, -0.055)),
        material=aged_brass,
        name="pulley_bracket",
    )
    for i, x in enumerate((-0.018, 0.018)):
        housing.visual(
            Box((0.006, 0.026, 0.006)),
            origin=Origin(xyz=(x, -0.042, -0.055)),
            material=aged_brass,
            name=f"pulley_yoke_{i}",
        )
    housing.visual(
        Cylinder(radius=0.004, length=0.044),
        origin=Origin(xyz=(0.0, -0.054, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="pulley_axle",
    )
    for i, angle in enumerate(math.radians(a) for a in (20, 140, 260)):
        housing.visual(
            _tube(
                [
                    (0.039 * math.cos(angle), 0.039 * math.sin(angle), -0.080),
                    (0.056 * math.cos(angle), 0.056 * math.sin(angle), -0.088),
                    (0.074 * math.cos(angle), 0.074 * math.sin(angle), -0.096),
                ],
                0.0028,
                f"guard_neck_strut_{i}",
                samples=6,
            ),
            material=aged_brass,
            name=f"guard_neck_strut_{i}",
        )

    model.articulation(
        "cord_grip_to_housing",
        ArticulationType.FIXED,
        parent=cord_grip,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, -0.820)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.020, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=nickel,
        name="screw_cap",
    )
    bulb.visual(
        _torus(0.020, 0.002, "cap_thread_0"),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=aged_brass,
        name="cap_thread_0",
    )
    bulb.visual(
        _torus(0.020, 0.002, "cap_thread_1"),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=aged_brass,
        name="cap_thread_1",
    )
    bulb.visual(
        _lathe(
            [
                (0.012, -0.050),
                (0.021, -0.066),
                (0.032, -0.103),
                (0.034, -0.150),
                (0.025, -0.186),
                (0.010, -0.206),
                (0.000, -0.212),
            ],
            "glass_envelope",
        ),
        material=warm_glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0045, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=warm_glass,
        name="glass_stem",
    )
    bulb.visual(
        _tube(
            [
                (-0.013, 0.0, -0.145),
                (-0.010, 0.0, -0.124),
                (-0.008, 0.0, -0.106),
            ],
            0.0009,
            "filament_support_0",
            samples=10,
        ),
        material=nickel,
        name="filament_support_0",
    )
    bulb.visual(
        _tube(
            [
                (0.013, 0.0, -0.145),
                (0.010, 0.0, -0.124),
                (0.008, 0.0, -0.106),
            ],
            0.0009,
            "filament_support_1",
            samples=10,
        ),
        material=nickel,
        name="filament_support_1",
    )
    filament_points = []
    for i in range(17):
        t = i / 16.0
        x = -0.008 + 0.016 * t
        z = -0.106 + 0.004 * math.sin(t * math.tau * 4.0)
        filament_points.append((x, 0.0, z))
    bulb.visual(
        _tube(filament_points, 0.0011, "glowing_filament", samples=8),
        material=amber_glow,
        name="glowing_filament",
    )

    model.articulation(
        "housing_to_bulb",
        ArticulationType.FIXED,
        parent=housing,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    chain_pulley = model.part("chain_pulley")
    chain_pulley.visual(
        _torus(0.016, 0.0035, "pulley_wheel"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_bronze,
        name="pulley_wheel",
    )
    chain_path = []
    for i in range(11):
        theta = math.radians(120 - 240 * (i / 10.0))
        chain_path.append((0.0, 0.018 * math.cos(theta), 0.018 * math.sin(theta)))
    chain_path.extend(
        [
            (0.0, -0.017, -0.030),
            (0.0, -0.022, -0.085),
            (0.0, -0.024, -0.150),
            (0.0, -0.022, -0.225),
            (0.0, -0.018, -0.300),
        ]
    )
    chain_pulley.visual(
        _tube(chain_path, 0.0012, "pull_chain_strand", samples=12),
        material=nickel,
        name="pull_chain_strand",
    )
    bead_positions = []
    for i in range(18):
        theta = math.radians(130 - 260 * (i / 17.0))
        bead_positions.append((0.0, 0.018 * math.cos(theta), 0.018 * math.sin(theta)))
    for i in range(13):
        bead_positions.append((0.0, -0.022, -0.035 - i * 0.020))
    for i, pos in enumerate(bead_positions):
        chain_pulley.visual(
            Sphere(radius=0.0032),
            origin=Origin(xyz=pos),
            material=nickel,
            name=f"chain_bead_{i}",
        )
    chain_pulley.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, -0.018, -0.305)),
        material=aged_brass,
        name="pull_weight",
    )

    model.articulation(
        "housing_to_chain_pulley",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=chain_pulley,
        origin=Origin(xyz=(0.0, -0.054, -0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    cord_grip = object_model.get_part("cord_grip")
    housing = object_model.get_part("housing")
    bulb = object_model.get_part("bulb")
    chain_pulley = object_model.get_part("chain_pulley")
    grip_slide = object_model.get_articulation("canopy_to_cord_grip")
    chain_spin = object_model.get_articulation("housing_to_chain_pulley")

    ctx.check(
        "cord grip uses a bounded prismatic adjustment",
        grip_slide.articulation_type == ArticulationType.PRISMATIC
        and grip_slide.motion_limits is not None
        and grip_slide.motion_limits.upper is not None
        and grip_slide.motion_limits.upper >= 0.10,
        details=f"type={grip_slide.articulation_type}, limits={grip_slide.motion_limits}",
    )
    ctx.check(
        "pull chain uses a continuous pulley rotation",
        chain_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={chain_spin.articulation_type}",
    )

    ctx.expect_gap(
        canopy,
        cord_grip,
        axis="z",
        positive_elem="canopy_shell",
        negative_elem="grip_collar",
        min_gap=0.0,
        max_gap=0.006,
        name="grip collar sits just below the canopy",
    )
    ctx.expect_within(
        bulb,
        housing,
        axes="xy",
        inner_elem="glass_envelope",
        outer_elem="bottom_guard_ring",
        margin=0.002,
        name="filament bulb is centered inside the guard ring",
    )
    ctx.expect_contact(
        bulb,
        housing,
        elem_a="screw_cap",
        elem_b="socket_shell",
        contact_tol=0.001,
        name="bulb cap seats against the socket shell",
    )

    rest_housing_pos = ctx.part_world_position(housing)
    with ctx.pose({grip_slide: grip_slide.motion_limits.upper}):
        extended_housing_pos = ctx.part_world_position(housing)
        ctx.expect_within(
            cord_grip,
            canopy,
            axes="xy",
            inner_elem="cord",
            outer_elem="canopy_shell",
            margin=0.002,
            name="adjusted cord remains threaded through canopy center",
        )

    ctx.check(
        "prismatic grip lengthens the pendant downward",
        rest_housing_pos is not None
        and extended_housing_pos is not None
        and extended_housing_pos[2] < rest_housing_pos[2] - 0.09,
        details=f"rest={rest_housing_pos}, extended={extended_housing_pos}",
    )

    pulley_pos = ctx.part_world_position(chain_pulley)
    with ctx.pose({chain_spin: math.pi * 0.75}):
        rotated_pulley_pos = ctx.part_world_position(chain_pulley)
    ctx.check(
        "pulley spin rotates in place on its axle",
        pulley_pos is not None
        and rotated_pulley_pos is not None
        and sum((a - b) ** 2 for a, b in zip(pulley_pos, rotated_pulley_pos)) < 1e-8,
        details=f"rest={pulley_pos}, rotated={rotated_pulley_pos}",
    )

    return ctx.report()


object_model = build_object_model()
