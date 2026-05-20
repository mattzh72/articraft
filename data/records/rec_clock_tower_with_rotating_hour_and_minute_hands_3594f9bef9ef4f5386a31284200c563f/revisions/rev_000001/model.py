from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


CLOCK_CENTER_Z = 17.75
CLOCK_HUB_Y = 2.30


def _pointed_arch_profile(
    width: float,
    spring_height: float,
    *,
    z0: float = 0.0,
    samples: int = 8,
) -> list[tuple[float, float]]:
    """Closed 2-D profile for an equilateral Gothic pointed arch.

    The profile uses x as horizontal and the second coordinate as vertical.
    It starts on the lower left, travels counter-clockwise, and closes with a
    flat sill.
    """

    half = width / 2.0
    left_spring = (-half, z0 + spring_height)
    right_spring = (half, z0 + spring_height)

    pts: list[tuple[float, float]] = [(-half, z0), (half, z0), right_spring]

    # Right lancet curve: circle centered on the opposite spring point.
    for i in range(1, samples + 1):
        theta = (math.pi / 3.0) * (i / samples)
        pts.append((-half + width * math.cos(theta), z0 + spring_height + width * math.sin(theta)))

    # Left lancet curve: circle centered on the right spring point.
    for i in range(1, samples + 1):
        theta = (2.0 * math.pi / 3.0) + (math.pi / 3.0) * (i / samples)
        pts.append((half + width * math.cos(theta), z0 + spring_height + width * math.sin(theta)))

    return pts


def _gothic_opening_mesh(width: float, spring_height: float, depth: float) -> MeshGeometry:
    """Dark recessed pointed-arch panel, extruded along local Y."""

    geom = ExtrudeGeometry(_pointed_arch_profile(width, spring_height), depth, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return geom


def _gothic_frame_mesh(
    inner_width: float,
    inner_spring_height: float,
    *,
    wall: float,
    depth: float,
) -> MeshGeometry:
    """Stone trim ring around a pointed arch, with a real hole in the mesh."""

    inner = _pointed_arch_profile(inner_width, inner_spring_height, z0=wall)
    outer = _pointed_arch_profile(
        inner_width + 2.0 * wall,
        inner_spring_height + wall,
        z0=0.0,
    )
    geom = ExtrudeWithHolesGeometry(outer, [inner], depth, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return geom


def _tapered_square_shaft(bottom_width: float, top_width: float, height: float) -> MeshGeometry:
    """A simple eight-vertex frustum: square in plan and tapering upward."""

    geom = MeshGeometry()
    b = bottom_width / 2.0
    t = top_width / 2.0
    verts = [
        (-b, -b, 0.0),
        (b, -b, 0.0),
        (b, b, 0.0),
        (-b, b, 0.0),
        (-t, -t, height),
        (t, -t, height),
        (t, t, height),
        (-t, t, height),
    ]
    for x, y, z in verts:
        geom.add_vertex(x, y, z)
    for face in (
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ):
        geom.add_face(*face)
    return geom


def _square_pyramid(width: float, height: float) -> MeshGeometry:
    half = width / 2.0
    geom = MeshGeometry()
    for x, y, z in (
        (-half, -half, 0.0),
        (half, -half, 0.0),
        (half, half, 0.0),
        (-half, half, 0.0),
        (0.0, 0.0, height),
    ):
        geom.add_vertex(x, y, z)
    for face in ((0, 1, 2), (0, 2, 3), (0, 4, 1), (1, 4, 2), (2, 4, 3), (3, 4, 0)):
        geom.add_face(*face)
    return geom


def _add_belt_course(tower, z: float, width: float, *, name: str) -> None:
    """Four-piece stone band wrapping a square stage."""

    tower.visual(
        Box((width + 0.35, 0.18, 0.20)),
        origin=Origin(xyz=(0.0, width / 2.0 + 0.04, z)),
        material="limestone_edge",
        name=f"{name}_front",
    )
    tower.visual(
        Box((width + 0.35, 0.18, 0.20)),
        origin=Origin(xyz=(0.0, -width / 2.0 - 0.04, z)),
        material="limestone_edge",
        name=f"{name}_rear",
    )
    tower.visual(
        Box((0.18, width + 0.35, 0.20)),
        origin=Origin(xyz=(width / 2.0 + 0.04, 0.0, z)),
        material="limestone_edge",
        name=f"{name}_side_0",
    )
    tower.visual(
        Box((0.18, width + 0.35, 0.20)),
        origin=Origin(xyz=(-width / 2.0 - 0.04, 0.0, z)),
        material="limestone_edge",
        name=f"{name}_side_1",
    )


def _add_arch_assembly(
    tower,
    *,
    face: str,
    center: tuple[float, float, float],
    inner_width: float,
    inner_spring_height: float,
    wall: float,
    depth: float,
    prefix: str,
) -> None:
    """Add a dark pointed-arch recess and raised stone frame to one tower face."""

    x, y, z = center
    if face == "front":
        rpy = (0.0, 0.0, 0.0)
    elif face == "rear":
        rpy = (0.0, 0.0, math.pi)
    elif face == "side_0":
        rpy = (0.0, 0.0, -math.pi / 2.0)
    elif face == "side_1":
        rpy = (0.0, 0.0, math.pi / 2.0)
    else:
        raise ValueError(f"unknown face: {face}")

    tower.visual(
        mesh_from_geometry(_gothic_opening_mesh(inner_width, inner_spring_height, depth * 0.65), f"{prefix}_shadow"),
        origin=Origin(xyz=(x, y, z), rpy=rpy),
        material="deep_shadow",
        name=f"{prefix}_shadow",
    )
    tower.visual(
        mesh_from_geometry(
            _gothic_frame_mesh(inner_width, inner_spring_height, wall=wall, depth=depth),
            f"{prefix}_stone_frame",
        ),
        origin=Origin(xyz=(x, y, z - wall), rpy=rpy),
        material="limestone_edge",
        name=f"{prefix}_frame",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gothic_clock_tower")

    model.material("limestone", rgba=(0.58, 0.56, 0.50, 1.0))
    model.material("limestone_edge", rgba=(0.70, 0.68, 0.60, 1.0))
    model.material("deep_shadow", rgba=(0.025, 0.023, 0.022, 1.0))
    model.material("slate", rgba=(0.10, 0.12, 0.15, 1.0))
    model.material("clock_ivory", rgba=(0.92, 0.86, 0.68, 1.0))
    model.material("aged_bronze", rgba=(0.63, 0.45, 0.18, 1.0))
    model.material("blackened_iron", rgba=(0.01, 0.01, 0.012, 1.0))

    tower = model.part("tower")

    tower.visual(
        mesh_from_geometry(_tapered_square_shaft(5.2, 3.85, 16.0), "tapered_shaft"),
        material="limestone",
        name="tapered_shaft",
    )
    tower.visual(
        Box((5.8, 5.8, 0.75)),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material="limestone_edge",
        name="stepped_plinth",
    )

    # Corner buttresses visually stiffen the high square shaft and are embedded
    # just into the tapering core so the assembly reads as continuous masonry.
    for ix, x in enumerate((-2.48, 2.48)):
        for iy, y in enumerate((-2.48, 2.48)):
            tower.visual(
                Box((0.44, 0.44, 14.6)),
                origin=Origin(xyz=(x, y, 7.45)),
                material="limestone_edge",
                name=f"corner_buttress_{ix}_{iy}",
            )
            tower.visual(
                mesh_from_geometry(
                    ConeGeometry(0.33, 0.82, radial_segments=4, closed=True),
                    f"buttress_cap_mesh_{ix}_{iy}",
                ),
                origin=Origin(xyz=(x, y, 15.15), rpy=(0.0, 0.0, math.pi / 4.0)),
                material="limestone_edge",
                name=f"buttress_cap_{ix}_{iy}",
            )

    for idx, (z, width) in enumerate(((0.90, 5.20), (4.35, 4.82), (8.10, 4.50), (12.05, 4.18), (15.85, 3.90))):
        _add_belt_course(tower, z, width, name=f"shaft_band_{idx}")

    # Clock stage, belfry tier, parapet, roof and finial.
    tower.visual(
        Box((4.25, 4.25, 3.30)),
        origin=Origin(xyz=(0.0, 0.0, 17.65)),
        material="limestone",
        name="clock_stage",
    )
    _add_belt_course(tower, 16.10, 4.25, name="clock_sill")
    _add_belt_course(tower, 19.18, 4.25, name="belfry_sill")

    tower.visual(
        Box((4.05, 4.05, 4.45)),
        origin=Origin(xyz=(0.0, 0.0, 21.45)),
        material="limestone",
        name="belfry_tier",
    )
    _add_belt_course(tower, 23.65, 4.10, name="parapet_band")
    tower.visual(
        Box((4.55, 4.55, 0.48)),
        origin=Origin(xyz=(0.0, 0.0, 23.99)),
        material="limestone_edge",
        name="crenel_base",
    )
    for i, x in enumerate((-1.75, -0.55, 0.55, 1.75)):
        tower.visual(
            Box((0.50, 0.42, 0.72)),
            origin=Origin(xyz=(x, 2.32, 24.50)),
            material="limestone_edge",
            name=f"front_crenel_{i}",
        )
        tower.visual(
            Box((0.50, 0.42, 0.72)),
            origin=Origin(xyz=(x, -2.32, 24.50)),
            material="limestone_edge",
            name=f"rear_crenel_{i}",
        )
        tower.visual(
            Box((0.42, 0.50, 0.72)),
            origin=Origin(xyz=(2.32, x, 24.50)),
            material="limestone_edge",
            name=f"side_crenel_0_{i}",
        )
        tower.visual(
            Box((0.42, 0.50, 0.72)),
            origin=Origin(xyz=(-2.32, x, 24.50)),
            material="limestone_edge",
            name=f"side_crenel_1_{i}",
        )

    tower.visual(
        mesh_from_geometry(_square_pyramid(4.10, 4.95), "slate_spire"),
        origin=Origin(xyz=(0.0, 0.0, 24.15)),
        material="slate",
        name="slate_spire",
    )
    tower.visual(
        Cylinder(radius=0.055, length=1.05),
        origin=Origin(xyz=(0.0, 0.0, 29.55)),
        material="aged_bronze",
        name="finial_rod",
    )
    tower.visual(
        mesh_from_geometry(ConeGeometry(0.22, 0.56, radial_segments=16, closed=True), "finial_point_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 30.33)),
        material="aged_bronze",
        name="finial_point",
    )

    # Belfry openings on all four sides: dark lancet recesses inside raised
    # pointed-arch stone frames.
    _add_arch_assembly(
        tower,
        face="front",
        center=(0.0, 2.075, 19.78),
        inner_width=1.05,
        inner_spring_height=1.45,
        wall=0.16,
        depth=0.16,
        prefix="belfry_front",
    )
    _add_arch_assembly(
        tower,
        face="rear",
        center=(0.0, -2.075, 19.78),
        inner_width=1.05,
        inner_spring_height=1.45,
        wall=0.16,
        depth=0.16,
        prefix="belfry_rear",
    )
    _add_arch_assembly(
        tower,
        face="side_0",
        center=(2.075, 0.0, 19.78),
        inner_width=1.05,
        inner_spring_height=1.45,
        wall=0.16,
        depth=0.16,
        prefix="belfry_side_0",
    )
    _add_arch_assembly(
        tower,
        face="side_1",
        center=(-2.075, 0.0, 19.78),
        inner_width=1.05,
        inner_spring_height=1.45,
        wall=0.16,
        depth=0.16,
        prefix="belfry_side_1",
    )

    # Lower lancet windows on the shaft face reinforce the Gothic language.
    for i, z in enumerate((5.50, 9.70)):
        face_half = 2.60 + (1.925 - 2.60) * (z / 16.0)
        _add_arch_assembly(
            tower,
            face="front",
            center=(0.0, face_half + 0.055, z),
            inner_width=0.54,
            inner_spring_height=0.78,
            wall=0.09,
            depth=0.30,
            prefix=f"lancet_front_{i}",
        )

    # Round clock face and applied markers on the upper front face.
    tower.visual(
        Cylinder(radius=1.05, length=0.10),
        origin=Origin(xyz=(0.0, 2.155, CLOCK_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="clock_ivory",
        name="clock_face",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.10, 0.055, radial_segments=16, tubular_segments=48), "clock_rim"),
        origin=Origin(xyz=(0.0, 2.225, CLOCK_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="aged_bronze",
        name="clock_rim",
    )
    for i in range(12):
        theta = 2.0 * math.pi * i / 12.0
        radius = 0.86
        marker_length = 0.20 if i % 3 == 0 else 0.14
        marker_width = 0.060 if i % 3 == 0 else 0.038
        tower.visual(
            Box((marker_width, 0.032, marker_length)),
            origin=Origin(
                xyz=(radius * math.sin(theta), 2.215, CLOCK_CENTER_Z + radius * math.cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material="blackened_iron",
            name=f"hour_marker_{i}",
        )
    tower.visual(
        Cylinder(radius=0.055, length=0.28),
        origin=Origin(xyz=(0.0, 2.325, CLOCK_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="aged_bronze",
        name="clock_bushing",
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Box((0.13, 0.045, 0.65)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material="blackened_iron",
        name="hour_pointer",
    )
    hour_hand.visual(
        Box((0.18, 0.045, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material="blackened_iron",
        name="hour_tail",
    )
    hour_hand.visual(
        Cylinder(radius=0.19, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="aged_bronze",
        name="hour_hub",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Box((0.075, 0.045, 0.93)),
        origin=Origin(xyz=(0.0, 0.085, 0.525)),
        material="blackened_iron",
        name="minute_pointer",
    )
    minute_hand.visual(
        Box((0.11, 0.045, 0.30)),
        origin=Origin(xyz=(0.0, 0.085, -0.21)),
        material="blackened_iron",
        name="minute_tail",
    )
    minute_hand.visual(
        Cylinder(radius=0.145, length=0.05),
        origin=Origin(xyz=(0.0, 0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="aged_bronze",
        name="minute_hub_cap",
    )

    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hour_hand,
        origin=Origin(xyz=(0.0, CLOCK_HUB_Y, CLOCK_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=0.35, lower=0.0, upper=2.0 * math.pi),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=minute_hand,
        origin=Origin(xyz=(0.0, CLOCK_HUB_Y, CLOCK_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=1.2, lower=0.0, upper=2.0 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    ctx.check(
        "clock hands use concentric front-normal revolute axes",
        hour_joint.articulation_type == ArticulationType.REVOLUTE
        and minute_joint.articulation_type == ArticulationType.REVOLUTE
        and hour_joint.origin.xyz == minute_joint.origin.xyz
        and hour_joint.axis == minute_joint.axis == (0.0, 1.0, 0.0),
        details=f"hour={hour_joint.origin.xyz}, minute={minute_joint.origin.xyz}",
    )

    ctx.allow_overlap(
        tower,
        hour_hand,
        elem_a="clock_bushing",
        elem_b="hour_hub",
        reason="The fixed clock arbor is intentionally modeled as a small shaft captured through the hour-hand hub.",
    )
    ctx.allow_overlap(
        tower,
        minute_hand,
        elem_a="clock_bushing",
        elem_b="minute_hub_cap",
        reason="The same concentric clock arbor passes through the minute-hand hub cap to support the outer hand.",
    )
    ctx.expect_within(
        tower,
        hour_hand,
        axes="xz",
        inner_elem="clock_bushing",
        outer_elem="hour_hub",
        margin=0.0,
        name="clock arbor sits inside the hour hub bore footprint",
    )
    ctx.expect_overlap(
        tower,
        hour_hand,
        axes="y",
        elem_a="clock_bushing",
        elem_b="hour_hub",
        min_overlap=0.030,
        name="clock arbor retains the hour hand along its shaft",
    )
    ctx.expect_within(
        tower,
        minute_hand,
        axes="xz",
        inner_elem="clock_bushing",
        outer_elem="minute_hub_cap",
        margin=0.0,
        name="clock arbor sits inside the minute hub bore footprint",
    )
    ctx.expect_overlap(
        tower,
        minute_hand,
        axes="y",
        elem_a="clock_bushing",
        elem_b="minute_hub_cap",
        min_overlap=0.030,
        name="clock arbor retains the minute hand along its shaft",
    )
    ctx.expect_gap(
        minute_hand,
        hour_hand,
        axis="y",
        min_gap=0.020,
        positive_elem="minute_hub_cap",
        negative_elem="hour_hub",
        name="minute and hour hands are layered apart",
    )

    rest_hour = ctx.part_world_position(hour_hand)
    rest_minute = ctx.part_world_position(minute_hand)
    with ctx.pose({hour_joint: math.pi / 2.0, minute_joint: math.pi}):
        posed_hour = ctx.part_world_position(hour_hand)
        posed_minute = ctx.part_world_position(minute_hand)
        ctx.expect_within(
            tower,
            hour_hand,
            axes="xz",
            inner_elem="clock_bushing",
            outer_elem="hour_hub",
            margin=0.0,
            name="rotated hour hub stays concentric on the arbor",
        )
        ctx.expect_overlap(
            tower,
            hour_hand,
            axes="y",
            elem_a="clock_bushing",
            elem_b="hour_hub",
            min_overlap=0.030,
            name="rotated hour hand remains captured on the arbor",
        )
        ctx.expect_gap(
            minute_hand,
            hour_hand,
            axis="y",
            min_gap=0.020,
            positive_elem="minute_hub_cap",
            negative_elem="hour_hub",
            name="rotated clock hands keep separate layers",
        )

    ctx.check(
        "hand pivots remain at the clock hub while rotating",
        rest_hour is not None
        and posed_hour is not None
        and rest_minute is not None
        and posed_minute is not None
        and abs(rest_hour[0] - posed_hour[0]) < 1e-6
        and abs(rest_hour[2] - posed_hour[2]) < 1e-6
        and abs(rest_minute[0] - posed_minute[0]) < 1e-6
        and abs(rest_minute[2] - posed_minute[2]) < 1e-6,
        details=f"hour {rest_hour}->{posed_hour}, minute {rest_minute}->{posed_minute}",
    )

    return ctx.report()


object_model = build_object_model()
