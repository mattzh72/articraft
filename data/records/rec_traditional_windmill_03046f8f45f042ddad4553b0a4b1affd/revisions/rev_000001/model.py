from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_round_member(part, a, b, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_lattice_blade(part, angle: float, material, index: int) -> None:
    # The hub part frame uses +X as the wind shaft.  Each blade is a small
    # wooden truss lying in the local YZ plane, proud of the front of the hub.
    x = 0.230
    radial = (0.0, math.sin(angle), math.cos(angle))
    tangent = (0.0, math.cos(angle), -math.sin(angle))

    def point(radius: float, offset: float = 0.0) -> tuple[float, float, float]:
        return (
            x,
            radial[1] * radius + tangent[1] * offset,
            radial[2] * radius + tangent[2] * offset,
        )

    inner = 0.135
    tip = 1.350
    half_width = 0.105

    _add_round_member(part, point(inner), point(tip), 0.020, material, f"blade_{index}_spar")
    _add_round_member(
        part,
        point(0.260, -half_width),
        point(1.240, -half_width * 0.72),
        0.010,
        material,
        f"blade_{index}_rail_0",
    )
    _add_round_member(
        part,
        point(0.260, half_width),
        point(1.240, half_width * 0.72),
        0.010,
        material,
        f"blade_{index}_rail_1",
    )
    for rung_index, radius in enumerate((0.340, 0.520, 0.700, 0.880, 1.060)):
        width = half_width * (1.0 - 0.22 * (radius - 0.26) / (tip - 0.26))
        _add_round_member(
            part,
            point(radius, -width),
            point(radius, width),
            0.007,
            material,
            f"blade_{index}_rung_{rung_index}",
        )
    _add_round_member(
        part,
        point(0.320, -half_width),
        point(1.120, half_width * 0.75),
        0.007,
        material,
        f"blade_{index}_diagonal_0",
    )
    _add_round_member(
        part,
        point(0.320, half_width),
        point(1.120, -half_width * 0.75),
        0.007,
        material,
        f"blade_{index}_diagonal_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("warm_stone", rgba=(0.66, 0.61, 0.52, 1.0))
    stone_dark = model.material("weathered_stone", rgba=(0.50, 0.47, 0.41, 1.0))
    whitewash = model.material("whitewashed_wood", rgba=(0.82, 0.78, 0.68, 1.0))
    roof = model.material("dark_shingle", rgba=(0.22, 0.16, 0.12, 1.0))
    wood = model.material("aged_oak", rgba=(0.55, 0.35, 0.18, 1.0))
    dark_wood = model.material("dark_oak", rgba=(0.30, 0.18, 0.10, 1.0))
    iron = model.material("blackened_iron", rgba=(0.07, 0.07, 0.065, 1.0))
    glass = model.material("dark_glass", rgba=(0.05, 0.06, 0.07, 1.0))

    tower = model.part("tower")
    tower_body = LatheGeometry(
        [
            (0.00, 0.18),
            (0.78, 0.18),
            (0.73, 0.48),
            (0.62, 1.25),
            (0.54, 2.10),
            (0.45, 2.92),
            (0.00, 2.92),
        ],
        segments=48,
    )
    tower.visual(
        Box((1.90, 1.90, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=stone_dark,
        name="square_plinth",
    )
    tower.visual(
        Cylinder(radius=0.86, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=stone,
        name="round_footing",
    )
    tower.visual(
        mesh_from_geometry(tower_body, "tapered_tower_body"),
        material=stone,
        name="tapered_body",
    )
    for index, (z, radius) in enumerate(((0.58, 0.73), (1.35, 0.62), (2.15, 0.53))):
        tower.visual(
            Cylinder(radius=radius, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=stone_dark,
            name=f"stone_belt_{index}",
        )
    tower.visual(
        Cylinder(radius=0.50, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.96)),
        material=stone_dark,
        name="top_bearing",
    )
    tower.visual(
        Box((0.020, 0.30, 0.48)),
        origin=Origin(xyz=(0.735, 0.0, 0.49)),
        material=dark_wood,
        name="front_door",
    )
    tower.visual(
        Box((0.018, 0.20, 0.26)),
        origin=Origin(xyz=(0.606, 0.0, 1.35)),
        material=glass,
        name="lower_window",
    )
    tower.visual(
        Box((0.016, 0.16, 0.22)),
        origin=Origin(xyz=(0.507, 0.0, 2.18)),
        material=glass,
        name="upper_window",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.56, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_wood,
        name="turntable",
    )
    cap.visual(
        Box((1.00, 0.82, 0.46)),
        origin=Origin(xyz=(0.18, 0.0, 0.31)),
        material=whitewash,
        name="cap_house",
    )
    cap.visual(
        Box((0.075, 0.66, 0.36)),
        origin=Origin(xyz=(0.700, 0.0, 0.36)),
        material=dark_wood,
        name="front_face",
    )
    cap.visual(
        Cylinder(radius=0.165, length=0.160),
        origin=Origin(xyz=(0.780, 0.0, 0.420), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="shaft_bearing",
    )
    cap.visual(
        Box((1.12, 0.55, 0.055)),
        origin=Origin(xyz=(0.18, 0.245, 0.640), rpy=(-0.45, 0.0, 0.0)),
        material=roof,
        name="roof_panel_0",
    )
    cap.visual(
        Box((1.12, 0.55, 0.055)),
        origin=Origin(xyz=(0.18, -0.245, 0.640), rpy=(0.45, 0.0, 0.0)),
        material=roof,
        name="roof_panel_1",
    )
    cap.visual(
        Cylinder(radius=0.035, length=1.18),
        origin=Origin(xyz=(0.18, 0.0, 0.765), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_wood,
        name="ridge_pole",
    )
    cap.visual(
        Box((0.030, 0.92, 0.06)),
        origin=Origin(xyz=(-0.34, 0.0, 0.56)),
        material=dark_wood,
        name="rear_eave",
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.205, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="rear_washer",
    )
    sail_hub.visual(
        Cylinder(radius=0.160, length=0.180),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_wood,
        name="hub_drum",
    )
    sail_hub.visual(
        Cylinder(radius=0.105, length=0.080),
        origin=Origin(xyz=(0.210, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_nose",
    )
    sail_hub.visual(
        Cylinder(radius=0.245, length=0.052),
        origin=Origin(xyz=(0.226, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_wood,
        name="blade_root_plate",
    )
    for index in range(4):
        _add_lattice_blade(sail_hub, index * math.pi / 2.0, wood, index)

    model.articulation(
        "cap_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 3.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.35),
    )
    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(0.860, 0.0, 0.420)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("sail_hub")
    cap_yaw = object_model.get_articulation("cap_yaw")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "cap has vertical continuous yaw",
        cap_yaw.articulation_type == ArticulationType.CONTINUOUS
        and tuple(cap_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={cap_yaw.articulation_type}, axis={cap_yaw.axis}",
    )
    ctx.check(
        "hub has continuous wind shaft spin",
        hub_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(hub_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={hub_spin.articulation_type}, axis={hub_spin.axis}",
    )
    ctx.expect_contact(
        cap,
        tower,
        elem_a="turntable",
        elem_b="top_bearing",
        contact_tol=0.002,
        name="cap turntable sits on tower bearing",
    )
    ctx.expect_gap(
        hub,
        cap,
        axis="x",
        positive_elem="rear_washer",
        negative_elem="shaft_bearing",
        max_gap=0.003,
        max_penetration=0.0,
        name="rotating hub seats against front bearing",
    )

    hub_rest = ctx.part_world_position(hub)
    with ctx.pose({hub_spin: math.pi / 2.0}):
        hub_spun = ctx.part_world_position(hub)
    ctx.check(
        "sail hub spins about its own shaft",
        hub_rest is not None
        and hub_spun is not None
        and math.dist(hub_rest, hub_spun) < 1e-6,
        details=f"rest={hub_rest}, spun={hub_spun}",
    )

    with ctx.pose({cap_yaw: math.pi / 2.0}):
        yawed_hub = ctx.part_world_position(hub)
    ctx.check(
        "cap yaw carries the front hub around tower axis",
        hub_rest is not None
        and yawed_hub is not None
        and yawed_hub[1] > 0.80
        and abs(yawed_hub[2] - hub_rest[2]) < 1e-6,
        details=f"rest={hub_rest}, yawed={yawed_hub}",
    )
    ctx.check(
        "rotating stage sits high above base",
        hub_rest is not None and hub_rest[2] > 3.35,
        details=f"hub_origin={hub_rest}",
    )

    return ctx.report()


object_model = build_object_model()
