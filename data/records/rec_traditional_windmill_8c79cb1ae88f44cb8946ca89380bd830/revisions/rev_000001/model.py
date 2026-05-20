from __future__ import annotations

import math

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
    mesh_from_cadquery,
)


def _tapered_tower_mesh() -> cq.Workplane:
    """A slightly tapered masonry smock-tower body in meters."""
    return (
        cq.Workplane("XY")
        .rect(1.42, 1.42)
        .workplane(offset=3.35)
        .rect(0.86, 0.86)
        .loft()
        .translate((0.0, 0.0, 0.15))
    )


def _gabled_roof_mesh(length: float, width: float, rise: float, base_z: float) -> cq.Workplane:
    """Triangular prism roof, authored with its ridge running along X."""
    return (
        cq.Workplane("YZ")
        .polyline([(-width / 2.0, 0.0), (0.0, rise), (width / 2.0, 0.0)])
        .close()
        .extrude(length)
        .translate((-length / 2.0, 0.0, base_z))
    )


def _bearing_ring_mesh() -> cq.Workplane:
    """Large front annular bearing housing with a real shaft opening."""
    outer = cq.Workplane("YZ").circle(0.245).extrude(0.40).translate((0.39, 0.0, 0.38))
    bore = cq.Workplane("YZ").circle(0.113).extrude(0.46).translate((0.36, 0.0, 0.38))
    return outer.cut(bore)


def _cap_house_mesh() -> cq.Workplane:
    """Wooden cap house with a through-bored shaft clearance."""
    house = cq.Workplane("XY").box(1.18, 0.94, 0.46).translate((0.08, 0.0, 0.35))
    shaft_clearance = cq.Workplane("YZ").circle(0.145).extrude(1.50).translate((-0.65, 0.0, 0.38))
    return house.cut(shaft_clearance)


def _add_lattice_blade(hub, angle: float, *, material: str) -> None:
    """Add one sail lattice in the hub frame. The shaft is local +X."""
    radial = (0.0, math.cos(angle), math.sin(angle))
    tangent = (0.0, -math.sin(angle), math.cos(angle))

    def yz_point(r: float, t: float) -> tuple[float, float, float]:
        return (
            0.16,
            radial[1] * r + tangent[1] * t,
            radial[2] * r + tangent[2] * t,
        )

    blade_len = 1.55
    rail_sep = 0.30
    rail_radius = rail_sep / 2.0
    root_offset = 0.10
    rail_center = root_offset + blade_len / 2.0

    # Tapered-looking pair of long wooden rails. They begin inside the hub disk,
    # so the lattice reads as mechanically fastened rather than floating.
    for idx, t in enumerate((-rail_radius, rail_radius)):
        hub.visual(
            Box((0.070, blade_len, 0.045)),
            origin=Origin(xyz=yz_point(rail_center, t), rpy=(angle, 0.0, 0.0)),
            material=material,
            name=f"blade_{int(round(angle * 1000))}_rail_{idx}",
        )

    # Cross battens divide the sail into rectangular openings.
    for idx, r in enumerate((0.22, 0.54, 0.88, 1.23, 1.55)):
        hub.visual(
            Box((0.075, 0.045, rail_sep + 0.10)),
            origin=Origin(xyz=yz_point(r, 0.0), rpy=(angle, 0.0, 0.0)),
            material=material,
            name=f"blade_{int(round(angle * 1000))}_cross_{idx}",
        )

    # Alternating diagonal braces make the sails read as latticework.
    for idx, (r0, r1) in enumerate(((0.25, 0.54), (0.54, 0.88), (0.88, 1.23), (1.23, 1.55))):
        dr = r1 - r0
        diagonal_angle = math.atan2(rail_sep, dr)
        length = math.hypot(dr, rail_sep)
        roll = angle + (diagonal_angle if idx % 2 == 0 else -diagonal_angle)
        t_center = 0.0
        r_center = (r0 + r1) / 2.0
        hub.visual(
            Box((0.050, length, 0.035)),
            origin=Origin(xyz=yz_point(r_center, t_center), rpy=(roll, 0.0, 0.0)),
            material=material,
            name=f"blade_{int(round(angle * 1000))}_brace_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    model.material("whitewashed_plaster", color=(0.78, 0.73, 0.62, 1.0))
    model.material("fieldstone", color=(0.35, 0.32, 0.29, 1.0))
    model.material("aged_oak", color=(0.45, 0.25, 0.12, 1.0))
    model.material("dark_timber", color=(0.22, 0.12, 0.06, 1.0))
    model.material("thatch", color=(0.55, 0.43, 0.22, 1.0))
    model.material("iron", color=(0.06, 0.06, 0.055, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((1.70, 1.70, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material="fieldstone",
        name="stone_plinth",
    )
    tower.visual(
        mesh_from_cadquery(_tapered_tower_mesh(), "tapered_tower"),
        material="whitewashed_plaster",
        name="tapered_body",
    )
    tower.visual(
        Cylinder(radius=0.50, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 3.44)),
        material="fieldstone",
        name="top_curb",
    )
    tower.visual(
        Box((0.08, 0.40, 0.62)),
        origin=Origin(xyz=(0.66, 0.0, 0.55)),
        material="dark_timber",
        name="front_door",
    )
    tower.visual(
        Box((0.10, 0.32, 0.28)),
        origin=Origin(xyz=(0.61, 0.0, 1.72)),
        material="aged_oak",
        name="front_window",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.54, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material="iron",
        name="yaw_turntable",
    )
    cap.visual(
        mesh_from_cadquery(_cap_house_mesh(), "bored_cap_house"),
        material="aged_oak",
        name="cap_house",
    )
    cap.visual(
        mesh_from_cadquery(_gabled_roof_mesh(1.30, 1.06, 0.36, 0.58), "cap_gabled_roof"),
        material="thatch",
        name="gabled_roof",
    )
    cap.visual(
        mesh_from_cadquery(_bearing_ring_mesh(), "front_bearing_ring"),
        material="iron",
        name="front_bearing",
    )
    cap.visual(
        Box((0.52, 0.11, 0.58)),
        origin=Origin(xyz=(0.56, 0.36, 0.34)),
        material="dark_timber",
        name="support_0",
    )
    cap.visual(
        Box((0.52, 0.11, 0.58)),
        origin=Origin(xyz=(0.56, -0.36, 0.34)),
        material="dark_timber",
        name="support_1",
    )
    cap.visual(
        Box((0.38, 0.22, 0.13)),
        origin=Origin(xyz=(0.50, 0.0, 0.11)),
        material="dark_timber",
        name="lower_bearing_saddle",
    )

    hub = model.part("sail_hub")
    hub.visual(
        Cylinder(radius=0.115, length=0.58),
        origin=Origin(xyz=(-0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="iron",
        name="shaft",
    )
    hub.visual(
        Cylinder(radius=0.24, length=0.18),
        origin=Origin(xyz=(0.13, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="iron",
        name="hub_drum",
    )
    hub.visual(
        Cylinder(radius=0.13, length=0.08),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_timber",
        name="wooden_spider",
    )
    for blade_angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        _add_lattice_blade(hub, blade_angle, material="aged_oak")

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 3.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.4),
    )
    model.articulation(
        "cap_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(0.79, 0.0, 0.38)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("sail_hub")
    yaw = object_model.get_articulation("tower_to_cap")
    shaft = object_model.get_articulation("cap_to_sail_hub")

    ctx.allow_overlap(
        cap,
        hub,
        elem_a="front_bearing",
        elem_b="shaft",
        reason="The iron windshaft is intentionally captured in the bearing bushing with slight modeled interference.",
    )

    ctx.check(
        "hub is continuous about shaft",
        shaft.articulation_type == ArticulationType.CONTINUOUS and tuple(shaft.axis) == (1.0, 0.0, 0.0),
        details=f"type={shaft.articulation_type}, axis={shaft.axis}",
    )
    ctx.check(
        "cap turns about vertical tower axis",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.expect_contact(
        tower,
        cap,
        elem_a="top_curb",
        elem_b="yaw_turntable",
        contact_tol=0.003,
        name="cap turntable sits on tower curb",
    )
    ctx.expect_within(
        hub,
        cap,
        axes="yz",
        inner_elem="shaft",
        outer_elem="front_bearing",
        margin=0.0,
        name="shaft passes through bearing opening centerline",
    )
    ctx.expect_overlap(
        hub,
        cap,
        axes="x",
        elem_a="shaft",
        elem_b="front_bearing",
        min_overlap=0.30,
        name="windshaft remains retained inside bearing",
    )
    rest_hub_position = ctx.part_world_position(hub)
    with ctx.pose({shaft: math.pi / 2.0}):
        turned_hub_position = ctx.part_world_position(hub)
    ctx.check(
        "hub rotation stays on shaft center",
        rest_hub_position is not None
        and turned_hub_position is not None
        and all(abs(a - b) < 0.001 for a, b in zip(rest_hub_position, turned_hub_position)),
        details=f"rest={rest_hub_position}, turned={turned_hub_position}",
    )

    return ctx.report()


object_model = build_object_model()
