from __future__ import annotations

import math

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
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rigid_yoke_tire_swing")

    wood = model.material("weathered_wood", rgba=(0.54, 0.34, 0.16, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.59, 0.60, 1.0))
    dark_steel = model.material("dark_pivot_steel", rgba=(0.10, 0.11, 0.12, 1.0))

    beam = model.part("beam")
    beam.visual(
        Box((1.60, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=wood,
        name="top_beam",
    )
    beam.visual(
        Box((0.86, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=galvanized,
        name="support_plate",
    )
    beam.visual(
        Box((0.04, 0.14, 0.17)),
        origin=Origin(xyz=(-0.40, 0.0, -0.005)),
        material=galvanized,
        name="pivot_cheek_0",
    )
    beam.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(-0.430, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="pivot_cap_0",
    )
    beam.visual(
        Box((0.04, 0.14, 0.17)),
        origin=Origin(xyz=(0.40, 0.0, -0.005)),
        material=galvanized,
        name="pivot_cheek_1",
    )
    beam.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.430, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="pivot_cap_1",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.035, length=0.76),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="top_barrel",
    )
    for i, x in enumerate((-0.30, 0.30)):
        yoke.visual(
            Box((0.045, 0.028, 0.50)),
            origin=Origin(xyz=(x, 0.0, -0.285)),
            material=galvanized,
            name=f"rigid_link_{i}",
        )
    lower_eye_mesh = mesh_from_geometry(
        TorusGeometry(0.035, 0.010, radial_segments=18, tubular_segments=32),
        "lower_hanger_eye",
    )
    yoke.visual(
        lower_eye_mesh,
        origin=Origin(xyz=(-0.30, 0.0, -0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="lower_eye_0",
    )
    yoke.visual(
        lower_eye_mesh,
        origin=Origin(xyz=(0.30, 0.0, -0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="lower_eye_1",
    )

    tire = model.part("tire")
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.42,
            0.14,
            inner_radius=0.22,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.07),
            tread=TireTread(style="block", depth=0.012, count=24, land_ratio=0.52),
            grooves=(
                TireGroove(center_offset=-0.030, width=0.010, depth=0.004),
                TireGroove(center_offset=0.030, width=0.010, depth=0.004),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.06),
            shoulder=TireShoulder(width=0.014, radius=0.006),
        ),
        "horizontal_tire_seat",
    )
    tire.visual(
        tire_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.13), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="tire_ring",
    )
    tire.visual(
        Cylinder(radius=0.027, length=0.76),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    for i, x in enumerate((-0.24, 0.24)):
        tire.visual(
            Box((0.035, 0.020, 0.090)),
            origin=Origin(xyz=(x, 0.0, -0.055)),
            material=galvanized,
            name=f"hanger_tab_{i}",
        )

    model.articulation(
        "beam_to_yoke",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=yoke,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "yoke_to_tire",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=tire,
        origin=Origin(xyz=(0.0, 0.0, -0.56)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    beam = object_model.get_part("beam")
    yoke = object_model.get_part("yoke")
    tire = object_model.get_part("tire")
    beam_joint = object_model.get_articulation("beam_to_yoke")
    tire_joint = object_model.get_articulation("yoke_to_tire")

    for eye_name in ("lower_eye_0", "lower_eye_1"):
        ctx.allow_overlap(
            tire,
            yoke,
            elem_a="axle",
            elem_b=eye_name,
            reason="The tire axle is intentionally captured in the yoke eye as a pinned hanger bearing.",
        )
        ctx.expect_overlap(
            tire,
            yoke,
            axes="xyz",
            elem_a="axle",
            elem_b=eye_name,
            min_overlap=0.015,
            name=f"axle captured by {eye_name}",
        )

    ctx.expect_contact(
        yoke,
        beam,
        elem_a="top_barrel",
        elem_b="pivot_cheek_0",
        contact_tol=0.001,
        name="top barrel bears on one beam cheek",
    )
    ctx.expect_contact(
        yoke,
        beam,
        elem_a="top_barrel",
        elem_b="pivot_cheek_1",
        contact_tol=0.001,
        name="top barrel bears on opposite beam cheek",
    )
    ctx.expect_gap(
        beam,
        tire,
        axis="z",
        min_gap=0.35,
        name="tire seat hangs well below the beam",
    )

    rest_pos = ctx.part_world_position(tire)
    with ctx.pose({beam_joint: 0.45}):
        swung_pos = ctx.part_world_position(tire)
    ctx.check(
        "beam pivot swings the yoke and tire forward",
        rest_pos is not None
        and swung_pos is not None
        and swung_pos[1] > rest_pos[1] + 0.20
        and swung_pos[2] > rest_pos[2] + 0.03,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    rest_aabb = ctx.part_element_world_aabb(tire, elem="tire_ring")
    with ctx.pose({tire_joint: 0.55}):
        tilted_aabb = ctx.part_element_world_aabb(tire, elem="tire_ring")
    rest_height = rest_aabb[1][2] - rest_aabb[0][2] if rest_aabb is not None else 0.0
    tilted_height = tilted_aabb[1][2] - tilted_aabb[0][2] if tilted_aabb is not None else 0.0
    ctx.check(
        "hanger pivots tilt the tire seat",
        tilted_height > rest_height + 0.18,
        details=f"rest_height={rest_height}, tilted_height={tilted_height}",
    )

    return ctx.report()


object_model = build_object_model()
