from __future__ import annotations

import cadquery as cq
from math import pi

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


def _pod_body() -> cq.Workplane:
    """Boxy four-cell launch canister with real through-open tube mouths."""

    length = 1.46
    width = 0.54
    height = 0.42
    center_x = 0.12
    center_z = height / 2.0
    front_x = center_x + length / 2.0
    back_x = center_x - length / 2.0

    body = cq.Workplane("XY").box(length, width, height).translate((center_x, 0.0, center_z))

    tube_radius = 0.075
    muzzle_outer = 0.104
    tube_centers = [
        (-0.13, 0.14),
        (0.13, 0.14),
        (-0.13, 0.28),
        (0.13, 0.28),
    ]

    # Raised annular collars around each mouth make the rectangular pod read as
    # a launch canister pack rather than a plain box.
    for y, z in tube_centers:
        collar = (
            cq.Workplane("YZ", origin=(front_x, 0.0, 0.0))
            .center(y, z)
            .circle(muzzle_outer)
            .circle(tube_radius)
            .extrude(0.035)
        )
        body = body.union(collar)

    for y, z in tube_centers:
        cutter = (
            cq.Workplane("YZ", origin=(back_x - 0.06, 0.0, 0.0))
            .center(y, z)
            .circle(tube_radius)
            .extrude(length + 0.16)
        )
        body = body.cut(cutter)

    # A small chamfer keeps the pack from looking like an abstract sharp block
    # while preserving the military, fabricated-plate silhouette.
    return body.edges("|X").chamfer(0.012)


def _box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, radius, length, xyz, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_missile_launcher")

    model.material("olive_drab", rgba=(0.28, 0.34, 0.20, 1.0))
    model.material("dark_olive", rgba=(0.16, 0.20, 0.13, 1.0))
    model.material("gunmetal", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("bearing_steel", rgba=(0.44, 0.46, 0.45, 1.0))
    model.material("rubber_black", rgba=(0.02, 0.022, 0.02, 1.0))

    fixed_base = model.part("fixed_base")
    _cylinder(fixed_base, 0.60, 0.10, (0.0, 0.0, 0.05), "gunmetal", "floor_plate")
    _cylinder(fixed_base, 0.22, 0.38, (0.0, 0.0, 0.29), "dark_olive", "pedestal_column")
    _cylinder(fixed_base, 0.34, 0.07, (0.0, 0.0, 0.515), "bearing_steel", "bearing_seat")
    _box(fixed_base, (0.56, 0.060, 0.23), (0.0, 0.0, 0.215), "dark_olive", "rib_x")
    _box(fixed_base, (0.060, 0.56, 0.23), (0.0, 0.0, 0.215), "dark_olive", "rib_y")

    turntable = model.part("turntable")
    _cylinder(turntable, 0.36, 0.10, (0.0, 0.0, 0.05), "bearing_steel", "turntable_disk")
    _box(turntable, (0.70, 0.80, 0.12), (0.0, 0.0, 0.16), "olive_drab", "rotating_deck")
    _box(turntable, (0.18, 0.13, 0.66), (0.0, -0.36, 0.55), "olive_drab", "pitch_block_0")
    _box(turntable, (0.18, 0.13, 0.66), (0.0, 0.36, 0.55), "olive_drab", "pitch_block_1")
    _box(turntable, (0.12, 0.55, 0.10), (-0.20, 0.0, 0.27), "dark_olive", "rear_tie_beam")
    _box(turntable, (0.12, 0.55, 0.08), (0.22, 0.0, 0.26), "dark_olive", "front_tie_beam")

    cradle = model.part("cradle")
    _cylinder(
        cradle,
        0.055,
        0.90,
        (0.0, 0.0, 0.0),
        "bearing_steel",
        "trunnion_axle",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    _box(cradle, (1.05, 0.48, 0.10), (0.32, 0.0, 0.09), "dark_olive", "cradle_bed")
    _box(cradle, (0.20, 0.08, 0.20), (0.0, -0.24, 0.04), "dark_olive", "trunnion_lug_0")
    _box(cradle, (0.20, 0.08, 0.20), (0.0, 0.24, 0.04), "dark_olive", "trunnion_lug_1")
    _box(cradle, (0.12, 0.44, 0.055), (0.74, 0.0, 0.1125), "bearing_steel", "front_clamp")
    _box(cradle, (0.12, 0.44, 0.055), (-0.12, 0.0, 0.1125), "bearing_steel", "rear_clamp")

    launch_pod = model.part("launch_pod")
    launch_pod.visual(
        mesh_from_cadquery(_pod_body(), "canister_pack", tolerance=0.0012, angular_tolerance=0.08),
        origin=Origin(),
        material="olive_drab",
        name="canister_body",
    )
    _box(launch_pod, (1.36, 0.050, 0.050), (0.12, -0.295, 0.08), "gunmetal", "side_rail_0")
    _box(launch_pod, (1.36, 0.050, 0.050), (0.12, 0.295, 0.08), "gunmetal", "side_rail_1")
    _box(launch_pod, (0.20, 0.18, 0.045), (0.10, 0.0, 0.440), "bearing_steel", "top_lug")

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=fixed_base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.8, lower=-pi, upper=pi),
    )

    model.articulation(
        "turntable_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        # The pod extends mostly along +X, so -Y makes positive pitch raise
        # the front of the launcher.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.7, lower=-0.12, upper=0.95),
    )

    model.articulation(
        "cradle_to_launch_pod",
        ArticulationType.FIXED,
        parent=cradle,
        child=launch_pod,
        origin=Origin(xyz=(0.32, 0.0, 0.14)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed_base = object_model.get_part("fixed_base")
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    launch_pod = object_model.get_part("launch_pod")
    yaw = object_model.get_articulation("base_to_turntable")
    pitch = object_model.get_articulation("turntable_to_cradle")

    # The trunnion axle is intentionally captured inside the two solid bearing
    # block proxies.  This is a local, hidden shaft/bushing intersection rather
    # than a broad wrong-position collision.
    for block in ("pitch_block_0", "pitch_block_1"):
        ctx.allow_overlap(
            turntable,
            cradle,
            elem_a=block,
            elem_b="trunnion_axle",
            reason="The pitch trunnion shaft is intentionally captured inside the bearing block proxy.",
        )
        ctx.expect_within(
            cradle,
            turntable,
            axes="xz",
            inner_elem="trunnion_axle",
            outer_elem=block,
            margin=0.002,
            name=f"trunnion centered in {block}",
        )
        ctx.expect_overlap(
            cradle,
            turntable,
            axes="y",
            elem_a="trunnion_axle",
            elem_b=block,
            min_overlap=0.07,
            name=f"trunnion inserted through {block}",
        )

    ctx.expect_gap(
        turntable,
        fixed_base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_disk",
        negative_elem="bearing_seat",
        name="turntable seated on base bearing",
    )
    ctx.expect_overlap(
        turntable,
        fixed_base,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="bearing_seat",
        min_overlap=0.30,
        name="yaw turntable centered over pedestal",
    )
    ctx.expect_gap(
        launch_pod,
        cradle,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="canister_body",
        negative_elem="cradle_bed",
        name="canister pack rests on tilting cradle",
    )

    rest_pod = ctx.part_world_position(launch_pod)
    with ctx.pose({pitch: 0.75}):
        raised_pod = ctx.part_world_position(launch_pod)
    ctx.check(
        "positive pitch raises launcher",
        rest_pod is not None and raised_pod is not None and raised_pod[2] > rest_pod[2] + 0.05,
        details=f"rest={rest_pod}, pitched={raised_pod}",
    )

    rest_turntable = ctx.part_world_position(turntable)
    with ctx.pose({yaw: 1.2}):
        yawed_turntable = ctx.part_world_position(turntable)
    ctx.check(
        "yaw axis remains vertical at pedestal",
        rest_turntable is not None
        and yawed_turntable is not None
        and abs(yawed_turntable[2] - rest_turntable[2]) < 1e-6,
        details=f"rest={rest_turntable}, yawed={yawed_turntable}",
    )

    return ctx.report()


object_model = build_object_model()
