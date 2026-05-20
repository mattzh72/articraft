from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _flat_prism_from_xz_profile(
    profile: list[tuple[float, float]], thickness: float
) -> MeshGeometry:
    """Closed prism extruded along local Y from an X/Z outline."""
    geom = MeshGeometry()
    half = thickness / 2.0

    front = [geom.add_vertex(x, half, z) for x, z in profile]
    back = [geom.add_vertex(x, -half, z) for x, z in profile]
    n = len(profile)

    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])

    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])

    return geom


def _handle_shell() -> cq.Workplane:
    """Rounded one-piece handle shell with a real open blade/carrier channel."""
    shell = (
        cq.Workplane("XY")
        .box(0.158, 0.036, 0.022)
        .translate((-0.006, 0.0, 0.0))
        .edges()
        .fillet(0.0035)
    )

    blade_channel = (
        cq.Workplane("XY")
        .box(0.154, 0.014, 0.012)
        .translate((0.014, 0.0, 0.0))
    )
    top_slot = (
        cq.Workplane("XY")
        .box(0.128, 0.016, 0.028)
        .translate((0.002, 0.0, 0.013))
    )
    rear_lock_relief = (
        cq.Workplane("XY")
        .box(0.020, 0.018, 0.006)
        .translate((-0.070, 0.0, 0.008))
    )

    return shell.cut(blade_channel).cut(top_slot).cut(rear_lock_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_retractable_utility_knife")

    orange = model.material("safety_orange", color=(0.95, 0.33, 0.05, 1.0))
    black = model.material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))
    graphite = model.material("graphite_plastic", color=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("brushed_steel", color=(0.72, 0.74, 0.73, 1.0))
    dark_steel = model.material("darkened_steel", color=(0.34, 0.35, 0.34, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shell(), "handle_shell", tolerance=0.0007),
        material=orange,
        name="shell",
    )
    # Rubberized side grip inlays and visible screw heads are slightly seated in
    # the shell so they read as installed trim rather than floating decals.
    for side, y in (("near", 0.0187), ("far", -0.0187)):
        handle.visual(
            Box((0.080, 0.0020, 0.011)),
            origin=Origin(xyz=(-0.030, y, -0.001)),
            material=black,
            name=f"{side}_grip",
        )
        for x in (-0.065, 0.044):
            handle.visual(
                Cylinder(radius=0.0042, length=0.0020),
                origin=Origin(xyz=(x, y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
                name=f"{side}_screw_{'rear' if x < 0 else 'front'}",
            )

    # Fixed short guide rails at the nose.  The separate guard rides over these,
    # while the blade and carrier remain on their own central track below.
    for side, y in (("guide_0", 0.0140), ("guide_1", -0.0140)):
        handle.visual(
            Box((0.043, 0.0032, 0.0034)),
            origin=Origin(xyz=(0.070, y, 0.0127)),
            material=graphite,
            name=side,
        )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Box((0.090, 0.014, 0.006)),
        origin=Origin(xyz=(0.038, 0.0, 0.000)),
        material=dark_steel,
        name="rail",
    )
    carrier.visual(
        Box((0.018, 0.007, 0.011)),
        origin=Origin(xyz=(0.034, 0.0, 0.0075)),
        material=graphite,
        name="stem",
    )
    carrier.visual(
        Box((0.024, 0.013, 0.010)),
        origin=Origin(xyz=(0.034, 0.0, 0.0170)),
        material=black,
        name="thumb_tab",
    )
    for idx, x in enumerate((0.024, 0.029, 0.034, 0.039, 0.044)):
        carrier.visual(
            Box((0.0017, 0.014, 0.0030)),
            origin=Origin(xyz=(x, 0.0, 0.0227)),
            material=graphite,
            name=f"tab_ridge_{idx}",
        )
    blade_profile = [
        (0.055, -0.0040),
        (0.094, -0.0040),
        (0.112, 0.0000),
        (0.094, 0.0040),
        (0.055, 0.0040),
    ]
    carrier.visual(
        mesh_from_geometry(
            _flat_prism_from_xz_profile(blade_profile, 0.0030),
            "utility_blade",
        ),
        material=steel,
        name="blade",
    )

    guard = model.part("nose_guard")
    for side, y in (("cheek_0", 0.0122), ("cheek_1", -0.0122)):
        guard.visual(
            Box((0.030, 0.0030, 0.0170)),
            origin=Origin(xyz=(0.015, y, 0.0)),
            material=steel,
            name=side,
        )
    guard.visual(
        Box((0.030, 0.024, 0.0030)),
        origin=Origin(xyz=(0.015, 0.0, 0.0095)),
        material=steel,
        name="top_bridge",
    )
    guard.visual(
        Box((0.030, 0.024, 0.0030)),
        origin=Origin(xyz=(0.015, 0.0, -0.0095)),
        material=steel,
        name="bottom_bridge",
    )
    for side, y in (("shoe_0", 0.0140), ("shoe_1", -0.0140)):
        guard.visual(
            Box((0.024, 0.0042, 0.0030)),
            origin=Origin(xyz=(0.006, y, 0.0162)),
            material=dark_steel,
            name=side,
        )
        guard.visual(
            Box((0.006, 0.0040, 0.0050)),
            origin=Origin(xyz=(0.006, y, 0.0128)),
            material=dark_steel,
            name=f"{side}_post",
        )

    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.045),
    )
    model.articulation(
        "handle_to_guard",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=guard,
        origin=Origin(xyz=(0.076, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.25, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    guard = object_model.get_part("nose_guard")
    carrier_slide = object_model.get_articulation("handle_to_carrier")
    guard_slide = object_model.get_articulation("handle_to_guard")

    ctx.allow_overlap(
        carrier,
        handle,
        elem_a="rail",
        elem_b="shell",
        reason=(
            "The blade carrier rail is intentionally captured inside the "
            "handle's molded slot; the hollow shell mesh acts as the enclosing "
            "slide housing proxy."
        ),
    )

    ctx.expect_within(
        carrier,
        handle,
        axes="yz",
        inner_elem="rail",
        outer_elem="shell",
        margin=0.001,
        name="blade carrier rail sits inside the handle slot",
    )
    ctx.expect_overlap(
        carrier,
        handle,
        axes="x",
        elem_a="rail",
        elem_b="shell",
        min_overlap=0.070,
        name="carrier has retained length in the handle",
    )
    ctx.expect_gap(
        guard,
        handle,
        axis="x",
        positive_elem="cheek_0",
        negative_elem="shell",
        min_gap=0.001,
        max_gap=0.010,
        name="nose guard starts just ahead of the handle tip",
    )

    rest_carrier = ctx.part_world_position(carrier)
    rest_guard = ctx.part_world_position(guard)
    rest_blade = ctx.part_element_world_aabb(carrier, elem="blade")
    with ctx.pose({carrier_slide: 0.045, guard_slide: 0.018}):
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="rail",
            elem_b="shell",
            min_overlap=0.060,
            name="extended carrier remains captured in the handle slot",
        )
        extended_carrier = ctx.part_world_position(carrier)
        extended_guard = ctx.part_world_position(guard)
        extended_blade = ctx.part_element_world_aabb(carrier, elem="blade")

    ctx.check(
        "carrier slides forward along handle axis",
        rest_carrier is not None
        and extended_carrier is not None
        and extended_carrier[0] > rest_carrier[0] + 0.040,
        details=f"rest={rest_carrier}, extended={extended_carrier}",
    )
    ctx.check(
        "nose guard slides forward on its own short guide",
        rest_guard is not None
        and extended_guard is not None
        and extended_guard[0] > rest_guard[0] + 0.015,
        details=f"rest={rest_guard}, extended={extended_guard}",
    )
    ctx.check(
        "blade tip advances with carrier",
        rest_blade is not None
        and extended_blade is not None
        and extended_blade[1][0] > rest_blade[1][0] + 0.040,
        details=f"rest_blade={rest_blade}, extended_blade={extended_blade}",
    )

    return ctx.report()


object_model = build_object_model()
