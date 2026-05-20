from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cylinder_y(part, name, radius, length, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _radial_box(part, name, radius, angle, size, material) -> None:
    """Place a box whose local +X is radial in the wheel XZ plane."""
    x = radius * math.cos(angle)
    z = radius * math.sin(angle)
    part.visual(
        Box(size),
        origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, -angle, 0.0)),
        material=material,
        name=name,
    )


def _sloped_beam(part, name, p0, p1, y, thickness, material) -> None:
    x0, z0 = p0
    x1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.sqrt(dx * dx + dz * dz)
    angle = math.atan2(dz, dx)
    _box(
        part,
        name,
        (length, thickness, thickness),
        ((x0 + x1) / 2.0, y, (z0 + z1) / 2.0),
        material,
        rpy=(0.0, -angle, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    oak = model.material("weathered_oak", rgba=(0.55, 0.34, 0.17, 1.0))
    endgrain = model.material("dark_endgrain", rgba=(0.30, 0.18, 0.09, 1.0))
    iron = model.material("blackened_iron", rgba=(0.04, 0.04, 0.04, 1.0))
    wetwood = model.material("wet_paddle_wood", rgba=(0.37, 0.22, 0.10, 1.0))
    stone = model.material("damp_stone", rgba=(0.38, 0.36, 0.32, 1.0))
    shadow = model.material("dark_water_shadow", rgba=(0.05, 0.09, 0.10, 1.0))

    base = model.part("base")
    # Ground skid frame: two long timbers tied together so it is one supported base.
    _box(base, "ground_sill_0", (2.10, 0.11, 0.10), (0.0, -0.56, 0.05), oak)
    _box(base, "ground_sill_1", (2.10, 0.11, 0.10), (0.0, 0.56, 0.05), oak)
    for i, x in enumerate((-0.70, 0.0, 0.70)):
        _box(base, f"cross_tie_{i}", (0.13, 1.23, 0.085), (x, 0.0, 0.045), endgrain)
    _box(base, "stone_water_bed", (1.58, 0.58, 0.045), (-0.03, 0.0, 0.075), stone)

    bearing_module = model.part("bearing_module")
    # Separable A-frame bearing carrier, fixed to but visually distinct from the base.
    for y, suffix in ((-0.50, "0"), (0.50, "1")):
        _box(bearing_module, f"side_sole_{suffix}", (2.05, 0.075, 0.08), (0.0, y, 0.14), oak)
        _sloped_beam(bearing_module, f"front_post_{suffix}", (-0.42, 0.16), (0.0, 0.55), y, 0.075, oak)
        _sloped_beam(bearing_module, f"rear_post_{suffix}", (0.42, 0.16), (0.0, 0.55), y, 0.075, oak)
        _box(bearing_module, f"head_block_{suffix}", (0.22, 0.09, 0.10), (0.0, y, 0.54), endgrain)
        bearing_mesh = mesh_from_geometry(
            TorusGeometry(0.068, 0.012, radial_segments=16, tubular_segments=32),
            f"bearing_ring_{suffix}",
        )
        bearing_module.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, y, 0.64), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"bearing_ring_{suffix}",
        )
        _box(bearing_module, f"saddle_plate_{suffix}", (0.24, 0.11, 0.035), (0.0, y, 0.525), iron)
        _box(bearing_module, f"journal_pad_{suffix}", (0.13, 0.10, 0.024), (0.0, y, 0.5915), iron)

    _box(bearing_module, "front_cross_beam", (0.10, 1.07, 0.075), (-0.93, 0.0, 0.145), oak)
    _box(bearing_module, "rear_cross_beam", (0.10, 1.07, 0.075), (0.93, 0.0, 0.145), oak)

    head_module = model.part("head_module")
    # Simple headrace/trough: a low undershot channel with side edges under the wheel.
    _box(head_module, "trough_floor", (1.54, 0.62, 0.020), (-0.05, 0.0, 0.1075), shadow)
    _box(head_module, "trough_edge_0", (1.54, 0.055, 0.125), (-0.05, -0.31, 0.180), oak)
    _box(head_module, "trough_edge_1", (1.54, 0.055, 0.125), (-0.05, 0.31, 0.180), oak)
    _box(head_module, "upstream_lip", (0.075, 0.66, 0.17), (-0.82, 0.0, 0.2025), endgrain)

    wheel = model.part("wheel")
    # The rotating assembly frame is centered on the axle; the fixed joint origin
    # places it at z=0.64 m in the bearing rings.
    _cylinder_y(wheel, "axle", 0.037, 1.12, (0.0, 0.0, 0.0), iron)
    _cylinder_y(wheel, "hub", 0.083, 0.38, (0.0, 0.0, 0.0), endgrain)
    for y, suffix in ((-0.205, "0"), (0.205, "1")):
        rim_mesh = mesh_from_geometry(
            TorusGeometry(0.395, 0.026, radial_segments=20, tubular_segments=64),
            f"outer_rim_{suffix}",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=wetwood,
            name=f"outer_rim_{suffix}",
        )
    for i in range(8):
        angle = i * math.tau / 8.0
        _radial_box(wheel, f"spoke_{i}", 0.215, angle, (0.58, 0.42, 0.045), oak)
    for i in range(12):
        angle = i * math.tau / 12.0
        _radial_box(wheel, f"paddle_{i}", 0.440, angle, (0.082, 0.46, 0.155), wetwood)

    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=bearing_module,
        origin=Origin(),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.FIXED,
        parent=base,
        child=head_module,
        origin=Origin(),
    )
    model.articulation(
        "bearing_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=bearing_module,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bearing_module = object_model.get_part("bearing_module")
    head_module = object_model.get_part("head_module")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("bearing_to_wheel")

    ctx.expect_contact(
        bearing_module,
        base,
        contact_tol=0.002,
        name="bearing module visibly seats on base timbers",
    )
    ctx.expect_contact(
        head_module,
        base,
        contact_tol=0.002,
        name="headrace feet visibly stand on the base",
    )
    ctx.expect_overlap(
        wheel,
        bearing_module,
        axes="y",
        min_overlap=0.9,
        name="axle spans between the two side-frame bearings",
    )
    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel rotates about a fixed horizontal axle",
        rest_pos is not None and turned_pos is not None and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    return ctx.report()


object_model = build_object_model()
