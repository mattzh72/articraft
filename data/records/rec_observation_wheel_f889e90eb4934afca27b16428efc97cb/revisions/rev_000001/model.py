from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


HUB_Z = 3.90
RIM_RADIUS = 2.55
PIVOT_RADIUS = 2.98
CABIN_Y_OFFSET = 0.56
CAPSULE_COUNT = 10


def _radial_origin(radius: float, theta: float, *, z_offset: float = 0.0) -> Origin:
    """Origin whose local +X points radially outward in the wheel XZ plane."""

    return Origin(
        xyz=(radius * math.cos(theta), 0.0, z_offset + radius * math.sin(theta)),
        rpy=(0.0, -theta, 0.0),
    )


def _add_xz_beam(part, name: str, start, end, thickness, material) -> None:
    """Add a rectangular tube-like beam between two points in a constant-Y XZ plane."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dz = ez - sz
    length = math.hypot(dx, dz)
    if length <= 0.0:
        return
    pitch = -math.atan2(dz, dx)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_observation_wheel")

    steel = model.material("satin_steel", color=(0.72, 0.76, 0.78, 1.0))
    dark = model.material("dark_bearing", color=(0.10, 0.12, 0.14, 1.0))
    white = model.material("warm_white_composite", color=(0.92, 0.90, 0.84, 1.0))
    glass = model.material("blue_tinted_glass", color=(0.28, 0.63, 0.95, 0.45))

    support = model.part("support")
    support.visual(
        Box((0.78, 0.52, 0.08)),
        origin=Origin(xyz=(-1.80, -0.55, 0.04)),
        material=dark,
        name="foot_0",
    )
    support.visual(
        Box((0.78, 0.52, 0.08)),
        origin=Origin(xyz=(1.80, -0.55, 0.04)),
        material=dark,
        name="foot_1",
    )
    support.visual(
        Box((3.95, 0.18, 0.14)),
        origin=Origin(xyz=(0.0, -0.55, 0.15)),
        material=steel,
        name="base_tie",
    )
    _add_xz_beam(support, "inclined_leg_0", (-1.80, -0.55, 0.10), (0.0, -0.55, HUB_Z), 0.17, steel)
    _add_xz_beam(support, "inclined_leg_1", (1.80, -0.55, 0.10), (0.0, -0.55, HUB_Z), 0.17, steel)
    support.visual(
        Cylinder(radius=0.40, length=0.30),
        origin=Origin(xyz=(0.0, -0.55, HUB_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="bearing_housing",
    )
    support.visual(
        Cylinder(radius=0.13, length=0.255),
        origin=Origin(xyz=(0.0, -0.2775, HUB_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_stub",
    )

    rim_mesh = mesh_from_geometry(
        TorusGeometry(RIM_RADIUS, 0.040, radial_segments=24, tubular_segments=128).rotate_x(math.pi / 2.0),
        "slender_rim",
    )
    cabin_body_mesh = mesh_from_geometry(
        CapsuleGeometry(0.21, 0.36, radial_segments=24, height_segments=8).rotate_y(math.pi / 2.0),
        "rounded_capsule_cabin",
    )

    wheel = model.part("wheel")
    wheel.visual(rim_mesh, material=steel, name="slender_rim")
    wheel.visual(
        Cylinder(radius=0.54, length=0.30),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="broad_hub",
    )
    wheel.visual(
        Cylinder(radius=0.30, length=0.25),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_cap",
    )

    for i in range(20):
        theta = 2.0 * math.pi * i / 20.0
        spoke_len = RIM_RADIUS - 0.42
        spoke_center = 0.42 + spoke_len * 0.5
        wheel.visual(
            Box((spoke_len, 0.030, 0.026)),
            origin=_radial_origin(spoke_center, theta),
            material=steel,
            name=f"spoke_{i}",
        )

    hanger_length = PIVOT_RADIUS - RIM_RADIUS + 0.04
    hanger_center = RIM_RADIUS + hanger_length * 0.5 - 0.02
    for i in range(CAPSULE_COUNT):
        theta = 2.0 * math.pi * i / CAPSULE_COUNT
        wheel.visual(
            Box((hanger_length, 0.060, 0.060)),
            origin=_radial_origin(hanger_center, theta),
            material=dark,
            name=f"rim_arm_{i}",
        )
        wheel.visual(
            Box((0.12, CABIN_Y_OFFSET, 0.080)),
            origin=Origin(
                xyz=(
                    PIVOT_RADIUS * math.cos(theta),
                    CABIN_Y_OFFSET * 0.5,
                    PIVOT_RADIUS * math.sin(theta),
                ),
                rpy=(0.0, -theta, 0.0),
            ),
            material=dark,
            name=f"hanger_{i}",
        )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35),
    )

    for i in range(CAPSULE_COUNT):
        theta = 2.0 * math.pi * i / CAPSULE_COUNT
        cabin = model.part(f"cabin_{i}")
        cabin.visual(
            cabin_body_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.42)),
            material=white,
            name="rounded_body",
        )
        cabin.visual(
            Box((0.58, 0.020, 0.22)),
            origin=Origin(xyz=(0.0, -0.215, -0.40)),
            material=glass,
            name="front_window",
        )
        cabin.visual(
            Box((0.58, 0.020, 0.22)),
            origin=Origin(xyz=(0.0, 0.215, -0.40)),
            material=glass,
            name="rear_window",
        )
        cabin.visual(
            Box((0.64, 0.34, 0.050)),
            origin=Origin(xyz=(0.0, 0.0, -0.625)),
            material=dark,
            name="floor_sill",
        )
        cabin.visual(
            Box((0.060, 0.060, 0.270)),
            origin=Origin(xyz=(0.0, 0.120, -0.130)),
            material=dark,
            name="hanger_stem",
        )
        cabin.visual(
            Cylinder(radius=0.045, length=0.46),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="pivot_pin",
        )
        model.articulation(
            f"wheel_to_cabin_{i}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=cabin,
            origin=Origin(
                xyz=(PIVOT_RADIUS * math.cos(theta), CABIN_Y_OFFSET, PIVOT_RADIUS * math.sin(theta))
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("support_to_wheel")

    for i in range(CAPSULE_COUNT):
        cabin = object_model.get_part(f"cabin_{i}")
        ctx.allow_overlap(
            wheel,
            cabin,
            elem_a=f"hanger_{i}",
            elem_b="pivot_pin",
            reason="The capsule pivot pin is intentionally captured inside the short hanger end.",
        )
        ctx.expect_overlap(
            cabin,
            wheel,
            axes="xyz",
            elem_a="pivot_pin",
            elem_b=f"hanger_{i}",
            min_overlap=0.020,
            name=f"cabin_{i} pin is seated in its hanger",
        )
        ctx.expect_gap(
            cabin,
            wheel,
            axis="y",
            positive_elem="rounded_body",
            negative_elem="slender_rim",
            min_gap=0.10,
            max_gap=0.45,
            name=f"cabin_{i} is mounted outboard of the rim",
        )

    pivot_radii = []
    pivot_angles = []
    for i in range(CAPSULE_COUNT):
        joint = object_model.get_articulation(f"wheel_to_cabin_{i}")
        x, y, z = joint.origin.xyz
        pivot_radii.append(math.hypot(x, z))
        pivot_angles.append(math.atan2(z, x))
    sorted_angles = sorted((angle + 2.0 * math.pi) % (2.0 * math.pi) for angle in pivot_angles)
    angle_gaps = [
        (sorted_angles[(i + 1) % CAPSULE_COUNT] - sorted_angles[i]) % (2.0 * math.pi)
        for i in range(CAPSULE_COUNT)
    ]
    expected_gap = 2.0 * math.pi / CAPSULE_COUNT
    ctx.check(
        "capsule pivots are evenly spaced outside the rim",
        all(abs(r - PIVOT_RADIUS) < 1e-6 and r > RIM_RADIUS + 0.30 for r in pivot_radii)
        and all(abs(gap - expected_gap) < 1e-6 for gap in angle_gaps),
        details=f"radii={pivot_radii}, angle_gaps={angle_gaps}",
    )

    ctx.expect_gap(
        wheel,
        support,
        axis="y",
        positive_elem="broad_hub",
        negative_elem="axle_stub",
        min_gap=0.0,
        max_gap=0.002,
        name="rotating hub clears the fixed axle stub",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: math.pi / 4.0}):
        turned_pos = ctx.part_world_position(wheel)
        ctx.expect_gap(
            wheel,
            support,
            axis="y",
            positive_elem="broad_hub",
            negative_elem="axle_stub",
            min_gap=0.0,
            max_gap=0.002,
            name="rotating hub keeps side clearance",
        )
    ctx.check(
        "wheel spins about a fixed horizontal axle",
        rest_pos is not None and turned_pos is not None and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
