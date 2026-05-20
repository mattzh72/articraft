from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _canister_pack_mesh():
    """One-piece rectangular 2x2 launch pod with four through canister bores."""

    length = 1.15
    width = 0.54
    height = 0.36
    center_x = 0.46
    bottom_z = 0.10
    center_z = bottom_z + height / 2.0

    pod = cq.Workplane("XY").box(length, width, height).translate((center_x, 0.0, center_z))

    bore_w = 0.20
    bore_h = 0.13
    margin_y = (width - 2.0 * bore_w) / 3.0
    margin_z = (height - 2.0 * bore_h) / 3.0
    y_centers = (
        -width / 2.0 + margin_y + bore_w / 2.0,
        width / 2.0 - margin_y - bore_w / 2.0,
    )
    z_centers = (
        bottom_z + margin_z + bore_h / 2.0,
        bottom_z + 2.0 * margin_z + 1.5 * bore_h,
    )

    for y in y_centers:
        for z in z_centers:
            cutter = (
                cq.Workplane("XY")
                .box(length + 0.04, bore_w, bore_h)
                .translate((center_x, y, z))
            )
            pod = pod.cut(cutter)

    # Exterior strengthening bands and a small front brow are unioned into the
    # same mesh so the launch pod remains one manufactured, connected body.
    for x in (-0.02, 0.92):
        band = cq.Workplane("XY").box(0.055, width + 0.035, height).translate(
            (x, 0.0, center_z)
        )
        inner = cq.Workplane("XY").box(0.065, width - 0.060, height - 0.060).translate(
            (x, 0.0, center_z)
        )
        pod = pod.union(band.cut(inner))

    brow = cq.Workplane("XY").box(0.18, 0.20, 0.035).translate((0.75, 0.0, bottom_z + height + 0.0175))
    pod = pod.union(brow)

    return pod


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_missile_launcher")

    olive = Material("olive_drab_paint", rgba=(0.30, 0.36, 0.20, 1.0))
    dark_olive = Material("dark_olive_paint", rgba=(0.18, 0.23, 0.14, 1.0))
    gunmetal = Material("gunmetal", rgba=(0.12, 0.13, 0.13, 1.0))
    black = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    steel = Material("oiled_steel", rgba=(0.45, 0.46, 0.43, 1.0))

    fixed_base = model.part("fixed_base")
    fixed_base.visual(
        Box((0.78, 0.78, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_olive,
        name="ground_plinth",
    )
    fixed_base.visual(
        Cylinder(radius=0.32, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=olive,
        name="round_foot",
    )
    fixed_base.visual(
        Cylinder(radius=0.18, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=olive,
        name="pedestal_column",
    )
    fixed_base.visual(
        Cylinder(radius=0.26, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=gunmetal,
        name="fixed_bearing_race",
    )
    for i, (x, y) in enumerate(((0.27, 0.27), (-0.27, 0.27), (-0.27, -0.27), (0.27, -0.27))):
        fixed_base.visual(
            Cylinder(radius=0.035, length=0.018),
            origin=Origin(xyz=(x, y, 0.109)),
            material=steel,
            name=f"anchor_bolt_{i}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.30, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=gunmetal,
        name="rotating_disk",
    )
    turntable.visual(
        Box((0.46, 0.38, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_olive,
        name="turret_block",
    )
    for i, y in enumerate((-0.41, 0.41)):
        turntable.visual(
            Box((0.30, 0.08, 0.70)),
            origin=Origin(xyz=(0.0, y, 0.50)),
            material=olive,
            name=f"yoke_plate_{i}",
        )
        turntable.visual(
            Cylinder(radius=0.115, length=0.12),
            origin=Origin(xyz=(0.0, y * 0.85, 0.60), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"bearing_boss_{i}",
        )
    turntable.visual(
        Box((0.30, 0.74, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=olive,
        name="yoke_crossmember",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.055, length=0.68),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_bar",
    )
    cradle.visual(
        Box((0.18, 0.52, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 0.05)),
        material=gunmetal,
        name="pivot_saddle",
    )
    cradle.visual(
        Box((1.03, 0.52, 0.08)),
        origin=Origin(xyz=(0.43, 0.0, 0.06)),
        material=gunmetal,
        name="top_saddle_plate",
    )
    for i, y in enumerate((-0.315, 0.315)):
        cradle.visual(
            Box((0.78, 0.075, 0.045)),
            origin=Origin(xyz=(0.53, y * 0.905, 0.0775)),
            material=gunmetal,
            name=f"clamp_ledge_{i}",
        )
        cradle.visual(
            Box((0.78, 0.035, 0.045)),
            origin=Origin(xyz=(0.53, y, 0.1225)),
            material=steel,
            name=f"side_clamp_{i}",
        )

    canister_pack = model.part("canister_pack")
    canister_pack.visual(
        mesh_from_cadquery(_canister_pack_mesh(), "rectangular_canister_pack", tolerance=0.001),
        material=olive,
        name="pod_shell",
    )
    for i, y in enumerate((-0.18, 0.18)):
        canister_pack.visual(
            Box((0.82, 0.035, 0.030)),
            origin=Origin(xyz=(0.49, y, 0.475)),
            material=dark_olive,
            name=f"top_stiffener_{i}",
        )
    canister_pack.visual(
        Box((0.10, 0.18, 0.035)),
        origin=Origin(xyz=(0.75, 0.0, 0.5125)),
        material=black,
        name="lifting_lug",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=fixed_base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=1.2),
    )
    model.articulation(
        "turntable_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.7,
            lower=math.radians(-10.0),
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "cradle_to_canister_pack",
        ArticulationType.FIXED,
        parent=cradle,
        child=canister_pack,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    canister_pack = object_model.get_part("canister_pack")
    yaw = object_model.get_articulation("base_to_turntable")
    pitch = object_model.get_articulation("turntable_to_cradle")

    for boss in ("bearing_boss_0", "bearing_boss_1"):
        ctx.allow_overlap(
            turntable,
            cradle,
            elem_a=boss,
            elem_b="trunnion_bar",
            reason="The trunnion bar is intentionally captured inside the yoke bearing boss.",
        )
        ctx.expect_overlap(
            turntable,
            cradle,
            axes="y",
            elem_a=boss,
            elem_b="trunnion_bar",
            min_overlap=0.015,
            name=f"{boss} captures trunnion along pitch axis",
        )

    ctx.expect_contact(
        cradle,
        canister_pack,
        elem_a="top_saddle_plate",
        elem_b="pod_shell",
        contact_tol=0.002,
        name="canister pack sits on cradle saddle",
    )
    ctx.expect_overlap(
        canister_pack,
        cradle,
        axes="xy",
        elem_a="pod_shell",
        elem_b="top_saddle_plate",
        min_overlap=0.25,
        name="canister pack footprint is supported by saddle",
    )

    rest_aabb = ctx.part_world_aabb(canister_pack)
    with ctx.pose({pitch: math.radians(60.0)}):
        raised_aabb = ctx.part_world_aabb(canister_pack)
    ctx.check(
        "positive pitch raises launch pod nose",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.45,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    yaw_rest = ctx.part_world_position(turntable)
    with ctx.pose({yaw: math.pi / 2.0}):
        yaw_rotated = ctx.part_world_position(turntable)
        ctx.expect_origin_gap(
            canister_pack,
            object_model.get_part("fixed_base"),
            axis="z",
            min_gap=0.20,
            name="yawed canister remains above pedestal",
        )
    ctx.check(
        "yaw joint stays on vertical pedestal axis",
        yaw_rest is not None and yaw_rotated is not None and abs(yaw_rotated[2] - yaw_rest[2]) < 1e-6,
        details=f"rest={yaw_rest}, rotated={yaw_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
