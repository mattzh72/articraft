from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _tower_radius(z: float) -> float:
    """Outer radius of the tapered tower shell at world height z."""
    z0, z1 = 1.0, 7.35
    r0, r1 = 1.15, 0.66
    t = (z - z0) / (z1 - z0)
    return r0 + t * (r1 - r0)


def _frustum_band(z0: float, z1: float, proud: float = 0.018) -> LatheGeometry:
    """Thin lathed color band that follows the tapered tower."""
    r0 = _tower_radius(z0)
    r1 = _tower_radius(z1)
    return LatheGeometry(
        [
            (r0 - 0.012, z0),
            (r0 + proud, z0),
            (r1 + proud, z1),
            (r1 - 0.012, z1),
        ],
        segments=72,
    )


def _tangent_origin(radius: float, angle: float, z: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
        rpy=(0.0, 0.0, angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offshore_lighthouse")

    white = model.material("painted_white", rgba=(0.92, 0.90, 0.84, 1.0))
    red = model.material("weathered_red", rgba=(0.72, 0.06, 0.04, 1.0))
    concrete = model.material("sea_concrete", rgba=(0.47, 0.48, 0.46, 1.0))
    dark = model.material("blackened_metal", rgba=(0.03, 0.035, 0.035, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.47, 0.31, 0.13, 1.0))
    glass = model.material("greenish_glass", rgba=(0.42, 0.82, 0.95, 0.38))
    lens = model.material("warm_lens_glass", rgba=(1.0, 0.78, 0.18, 0.72))

    tower = model.part("tower")

    tower.visual(
        Cylinder(radius=2.05, length=1.05),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=concrete,
        name="sea_caisson",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=1.92, tube=0.07), "base_fender"),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=dark,
        name="base_fender",
    )

    tower_shell = LatheGeometry(
        [(0.0, 1.0), (1.15, 1.0), (0.66, 7.35), (0.0, 7.35)],
        segments=96,
    )
    tower.visual(
        mesh_from_geometry(tower_shell, "tower_shell"),
        material=white,
        name="tower_shell",
    )
    for name, z0, z1 in (
        ("red_band_0", 1.65, 2.18),
        ("red_band_1", 3.72, 4.20),
        ("red_band_2", 5.76, 6.20),
    ):
        tower.visual(
            mesh_from_geometry(_frustum_band(z0, z1), name),
            material=red,
            name=name,
        )

    tower.visual(
        Cylinder(radius=1.08, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 7.43)),
        material=concrete,
        name="gallery_deck",
    )
    tower.visual(
        Cylinder(radius=0.78, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 7.68)),
        material=white,
        name="lantern_floor",
    )

    # Gallery railing: a short circular guard rail around the lantern room.
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        tower.visual(
            Cylinder(radius=0.026, length=0.70),
            origin=Origin(xyz=(1.02 * math.cos(a), 1.02 * math.sin(a), 7.86)),
            material=dark,
            name=f"rail_post_{i}",
        )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=1.02, tube=0.024), "upper_gallery_rail"),
        origin=Origin(xyz=(0.0, 0.0, 8.20)),
        material=dark,
        name="upper_gallery_rail",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=1.02, tube=0.018), "lower_gallery_rail"),
        origin=Origin(xyz=(0.0, 0.0, 7.86)),
        material=dark,
        name="lower_gallery_rail",
    )

    # Compact octagonal lantern room: glass panels, metal mullions, and a
    # framed front opening reserved for the moving access hatch.
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=0.72, tube=0.035), "lantern_base_ring"),
        origin=Origin(xyz=(0.0, 0.0, 7.86)),
        material=dark,
        name="lantern_base_ring",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=0.72, tube=0.035), "lantern_top_ring"),
        origin=Origin(xyz=(0.0, 0.0, 9.21)),
        material=dark,
        name="lantern_top_ring",
    )
    for i in range(8):
        a = math.pi / 8.0 + i * math.pi / 4.0
        tower.visual(
            Box((0.060, 0.060, 1.42)),
            origin=_tangent_origin(0.72, a, 8.535),
            material=dark,
            name=f"mullion_{i}",
        )
    for i in range(8):
        a = i * math.pi / 4.0
        if i == 0:
            continue
        tower.visual(
            Box((0.026, 0.45, 1.36)),
            origin=_tangent_origin(0.715, a, 8.54),
            material=glass,
            name=f"glass_panel_{i}",
        )

    # Hatch surround on the front lantern wall.  The panel itself is a child
    # part hinged on the side jamb.
    tower.visual(
        Box((0.050, 0.060, 1.30)),
        origin=Origin(xyz=(0.715, -0.335, 8.51)),
        material=dark,
        name="hinge_jamb",
    )
    tower.visual(
        Box((0.050, 0.060, 1.30)),
        origin=Origin(xyz=(0.715, 0.225, 8.51)),
        material=dark,
        name="latch_jamb",
    )
    tower.visual(
        Box((0.050, 0.62, 0.080)),
        origin=Origin(xyz=(0.715, -0.055, 8.035)),
        material=dark,
        name="hatch_sill",
    )
    tower.visual(
        Box((0.050, 0.62, 0.080)),
        origin=Origin(xyz=(0.715, -0.055, 9.215)),
        material=dark,
        name="hatch_header",
    )

    roof = LatheGeometry(
        [(0.0, 9.21), (0.86, 9.21), (0.24, 9.86), (0.0, 9.86)],
        segments=96,
    )
    tower.visual(
        mesh_from_geometry(roof, "lantern_roof"),
        material=red,
        name="lantern_roof",
    )
    tower.visual(
        Cylinder(radius=0.13, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 10.04)),
        material=dark,
        name="beacon_pedestal",
    )

    beacon = model.part("beacon_head")
    beacon.visual(
        Cylinder(radius=0.26, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark,
        name="turntable_disk",
    )
    beacon.visual(
        Cylinder(radius=0.075, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=dark,
        name="turntable_post",
    )
    beacon.visual(
        Box((0.32, 0.36, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=dark,
        name="saddle_block",
    )
    beacon.visual(
        Box((0.48, 0.060, 0.30)),
        origin=Origin(xyz=(0.0, -0.14, 0.52)),
        material=dark,
        name="yoke_0",
    )
    beacon.visual(
        Box((0.48, 0.060, 0.30)),
        origin=Origin(xyz=(0.0, 0.14, 0.52)),
        material=dark,
        name="yoke_1",
    )
    beacon.visual(
        mesh_from_geometry(
            CapsuleGeometry(radius=0.13, length=0.50),
            "beacon_housing",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="beacon_housing",
    )
    beacon.visual(
        Cylinder(radius=0.112, length=0.045),
        origin=Origin(xyz=(0.39, 0.0, 0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="front_lens",
    )
    beacon.visual(
        Cylinder(radius=0.090, length=0.035),
        origin=Origin(xyz=(-0.38, 0.0, 0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="rear_lens",
    )

    hatch = model.part("access_hatch")
    hatch.visual(
        Box((0.055, 0.42, 1.02)),
        origin=Origin(xyz=(0.035, 0.21, 0.51)),
        material=red,
        name="hatch_panel",
    )
    hatch.visual(
        Cylinder(radius=0.035, length=1.04),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=dark,
        name="hatch_barrel",
    )
    hatch.visual(
        Cylinder(radius=0.035, length=0.035),
        origin=Origin(xyz=(0.078, 0.325, 0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="hatch_latch",
    )

    model.articulation(
        "beacon_turntable",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 10.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hatch,
        origin=Origin(xyz=(0.740, -0.270, 8.10)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon_head")
    hatch = object_model.get_part("access_hatch")
    turntable = object_model.get_articulation("beacon_turntable")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.check(
        "beacon joint is continuous",
        turntable.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={turntable.articulation_type}",
    )
    ctx.expect_gap(
        beacon,
        tower,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="beacon_pedestal",
        max_gap=0.002,
        max_penetration=0.001,
        name="turntable seats on pedestal",
    )
    ctx.expect_contact(
        hatch,
        tower,
        elem_a="hatch_barrel",
        elem_b="hinge_jamb",
        contact_tol=0.012,
        name="hatch barrel meets hinge jamb",
    )

    def _center_of(elem_aabb):
        if elem_aabb is None:
            return None
        lo, hi = elem_aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_lens = _center_of(ctx.part_element_world_aabb(beacon, elem="front_lens"))
    with ctx.pose({turntable: math.pi / 2.0}):
        turned_lens = _center_of(ctx.part_element_world_aabb(beacon, elem="front_lens"))
    ctx.check(
        "beacon head sweeps around turntable",
        rest_lens is not None
        and turned_lens is not None
        and turned_lens[1] > rest_lens[1] + 0.25,
        details=f"rest={rest_lens}, turned={turned_lens}",
    )

    rest_hatch = _center_of(ctx.part_element_world_aabb(hatch, elem="hatch_panel"))
    with ctx.pose({hatch_hinge: 1.20}):
        open_hatch = _center_of(ctx.part_element_world_aabb(hatch, elem="hatch_panel"))
    ctx.check(
        "access hatch opens outward",
        rest_hatch is not None
        and open_hatch is not None
        and open_hatch[0] > rest_hatch[0] + 0.12,
        details=f"rest={rest_hatch}, open={open_hatch}",
    )

    return ctx.report()


object_model = build_object_model()
