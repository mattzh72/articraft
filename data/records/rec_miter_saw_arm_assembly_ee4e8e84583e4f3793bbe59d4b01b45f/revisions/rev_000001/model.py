from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)
import cadquery as cq


def _toothed_disc(radius: float, tooth_depth: float, teeth: int, thickness: float) -> MeshGeometry:
    """Thin circular-saw blade mesh in the local XZ plane, with thickness on Y."""
    profile: list[tuple[float, float]] = []
    steps = teeth * 2
    for i in range(steps):
        angle = 2.0 * math.pi * i / steps
        r = radius + (tooth_depth if i % 2 == 0 else -0.002)
        profile.append((r * math.cos(angle), r * math.sin(angle)))
    arbor_hole = [
        (0.023 * math.cos(2.0 * math.pi * i / 32), 0.023 * math.sin(2.0 * math.pi * i / 32))
        for i in range(31, -1, -1)
    ]
    geom = ExtrudeWithHolesGeometry(profile, [arbor_hole], thickness, center=True)
    # ExtrudeGeometry is in XY with thickness on local Z; rotate it so the
    # circular blade lies in XZ and spins around local Y.
    geom.rotate_x(math.pi / 2.0)
    return geom


def _sector_plate(
    radius: float,
    start_deg: float,
    end_deg: float,
    thickness: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    """Filled circular sector plate in local XZ, with thickness on Y."""
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    profile: list[tuple[float, float]] = [(0.0, 0.0)]
    for i in range(segments + 1):
        t = start + (end - start) * (i / segments)
        profile.append((radius * math.cos(t), radius * math.sin(t)))
    geom = ExtrudeGeometry(profile, thickness, center=True)
    geom.rotate_x(math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="basic_chop_miter_saw")

    cast_gray = model.material("cast_gray", rgba=(0.34, 0.36, 0.37, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.05, 0.055, 0.06, 1.0))
    table_metal = model.material("brushed_table", rgba=(0.66, 0.68, 0.66, 1.0))
    yellow = model.material("saw_yellow", rgba=(0.95, 0.70, 0.12, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    guard_smoke = model.material("smoke_guard", rgba=(0.86, 0.77, 0.43, 0.55))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))

    # Root: fixed base, rear fence, and the forked pivot block that clips the arm.
    base = model.part("base")
    base.visual(
        Box((1.05, 0.55, 0.050)),
        origin=Origin(xyz=(-0.02, 0.0, 0.025)),
        material=cast_gray,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.245, length=0.0075),
        origin=Origin(xyz=(0.0, 0.0, 0.05375)),
        material=cast_gray,
        name="table_bearing_pad",
    )
    base.visual(
        Box((0.035, 0.68, 0.115)),
        origin=Origin(xyz=(-0.315, 0.0, 0.117)),
        material=dark_gray,
        name="rear_fence",
    )
    base.visual(
        Box((0.18, 0.34, 0.44)),
        origin=Origin(xyz=(-0.420, 0.0, 0.270)),
        material=cast_gray,
        name="pivot_block",
    )
    base.visual(
        Box((0.125, 0.050, 0.160)),
        origin=Origin(xyz=(-0.420, -0.140, 0.570)),
        material=cast_gray,
        name="pivot_ear_0",
    )
    base.visual(
        Box((0.125, 0.050, 0.160)),
        origin=Origin(xyz=(-0.420, 0.140, 0.570)),
        material=cast_gray,
        name="pivot_ear_1",
    )
    base.visual(
        Box((0.20, 0.080, 0.050)),
        origin=Origin(xyz=(-0.420, 0.0, 0.455)),
        material=cast_gray,
        name="yoke_bridge",
    )

    # Rotating miter table and the long front handle that makes the rotation visible.
    table = model.part("miter_table")
    table.visual(
        Cylinder(radius=0.275, length=0.045),
        origin=Origin(),
        material=table_metal,
        name="table_disc",
    )
    table.visual(
        Box((0.335, 0.034, 0.006)),
        origin=Origin(xyz=(0.050, 0.0, 0.0255)),
        material=dark_gray,
        name="kerf_insert",
    )
    table.visual(
        Box((0.185, 0.075, 0.035)),
        origin=Origin(xyz=(0.350, 0.0, -0.002)),
        material=table_metal,
        name="miter_handle",
    )

    # Arm/head assembly is one supported link: pivot tube, beam, motor, handle,
    # arbor boss, and the fixed upper blade shroud all move together.
    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.045, length=0.230),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="pivot_tube",
    )
    arm.visual(
        Box((0.590, 0.070, 0.065)),
        origin=Origin(xyz=(0.270, 0.0, -0.035)),
        material=yellow,
        name="arm_beam",
    )
    arm.visual(
        Cylinder(radius=0.115, length=0.220),
        origin=Origin(xyz=(0.570, -0.115, -0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yellow,
        name="motor_housing",
    )
    arm.visual(
        Box((0.085, 0.070, 0.230)),
        origin=Origin(xyz=(0.500, -0.175, 0.035), rpy=(0.0, 0.18, 0.0)),
        material=yellow,
        name="handle_post",
    )
    arm.visual(
        Box((0.120, 0.060, 0.050)),
        origin=Origin(xyz=(0.520, -0.175, 0.160)),
        material=rubber,
        name="grip",
    )
    arm.visual(
        Box((0.045, 0.020, 0.030)),
        origin=Origin(xyz=(0.565, -0.140, 0.060)),
        material=dark_gray,
        name="trigger",
    )
    arm.visual(
        Cylinder(radius=0.040, length=0.040),
        origin=Origin(xyz=(0.580, 0.015, -0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="arbor_boss",
    )
    arm.visual(
        Cylinder(radius=0.014, length=0.049),
        origin=Origin(xyz=(0.580, 0.0585, -0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="arbor_shaft",
    )
    arm.visual(
        mesh_from_geometry(_sector_plate(0.210, 12.0, 184.0, 0.050), "upper_blade_guard"),
        origin=Origin(xyz=(0.580, 0.010, -0.110)),
        material=yellow,
        name="upper_blade_guard",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(_toothed_disc(0.180, 0.012, 36, 0.008), "toothed_blade"),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material=blade_steel,
        name="toothed_blade",
    )
    blade.visual(
        Cylinder(radius=0.035, length=0.040),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="blade_hub",
    )

    lower_guard = model.part("lower_guard")
    lower_guard.visual(
        mesh_from_geometry(_sector_plate(0.205, -166.0, 20.0, 0.036), "lower_blade_guard"),
        origin=Origin(xyz=(0.0, 0.100, 0.0)),
        material=guard_smoke,
        name="lower_blade_guard",
    )
    lower_guard.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_smoke,
        name="guard_hub",
    )

    model.articulation(
        "miter_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "arm_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(-0.420, 0.0, 0.560)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=0.278),
    )
    model.articulation(
        "blade_arbor",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.580, 0.0, -0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=80.0),
    )
    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=lower_guard,
        origin=Origin(xyz=(0.580, 0.0, -0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-0.75, upper=0.05),
        mimic=Mimic(joint="arm_pivot", multiplier=-1.4, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    table = object_model.get_part("miter_table")
    arm = object_model.get_part("arm")
    blade = object_model.get_part("blade")
    lower_guard = object_model.get_part("lower_guard")
    miter = object_model.get_articulation("miter_axis")
    arm_pivot = object_model.get_articulation("arm_pivot")
    blade_arbor = object_model.get_articulation("blade_arbor")
    guard_hinge = object_model.get_articulation("guard_hinge")

    ctx.check(
        "blade has continuous arbor spin",
        blade_arbor.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_arbor.articulation_type}",
    )
    ctx.check(
        "lower guard follows arm pivot",
        guard_hinge.mimic is not None and guard_hinge.mimic.joint == "arm_pivot",
        details=f"mimic={guard_hinge.mimic}",
    )
    ctx.allow_overlap(
        arm,
        blade,
        elem_a="arbor_shaft",
        elem_b="blade_hub",
        reason="The driven arbor shaft is intentionally captured inside the blade hub.",
    )
    ctx.allow_overlap(
        arm,
        blade,
        elem_a="arbor_shaft",
        elem_b="toothed_blade",
        reason="The shaft passes through the blade's central arbor hole; the toothed disc is a simplified mesh at the bore.",
    )
    ctx.expect_within(
        arm,
        blade,
        axes="xz",
        inner_elem="arbor_shaft",
        outer_elem="blade_hub",
        margin=0.002,
        name="arbor shaft is centered through blade hub",
    )
    ctx.expect_overlap(
        arm,
        blade,
        axes="y",
        elem_a="arbor_shaft",
        elem_b="blade_hub",
        min_overlap=0.030,
        name="arbor shaft passes through blade hub",
    )
    ctx.expect_within(
        arm,
        blade,
        axes="xz",
        inner_elem="arbor_shaft",
        outer_elem="toothed_blade",
        margin=0.001,
        name="arbor shaft is centered in blade bore",
    )
    ctx.expect_gap(
        base,
        arm,
        axis="y",
        positive_elem="pivot_ear_1",
        negative_elem="pivot_tube",
        max_gap=0.004,
        max_penetration=0.001,
        name="positive yoke ear clips pivot tube",
    )
    ctx.expect_gap(
        arm,
        base,
        axis="y",
        positive_elem="pivot_tube",
        negative_elem="pivot_ear_0",
        max_gap=0.004,
        max_penetration=0.001,
        name="negative yoke ear clips pivot tube",
    )

    rest_blade_aabb = ctx.part_world_aabb(blade)
    rest_handle_aabb = ctx.part_element_world_aabb(table, elem="miter_handle")

    with ctx.pose({arm_pivot: 0.278}):
        lowered_blade_aabb = ctx.part_world_aabb(blade)
        ctx.expect_gap(
            blade,
            table,
            axis="z",
            max_gap=0.030,
            max_penetration=0.006,
            name="lowered blade reaches the table kerf",
        )

    with ctx.pose({miter: 0.55}):
        turned_handle_aabb = ctx.part_element_world_aabb(table, elem="miter_handle")

    if rest_blade_aabb is not None and lowered_blade_aabb is not None:
        rest_min_z = rest_blade_aabb[0][2]
        lowered_min_z = lowered_blade_aabb[0][2]
        ctx.check(
            "arm pivot chops blade downward",
            lowered_min_z < rest_min_z - 0.14,
            details=f"rest_min_z={rest_min_z:.3f}, lowered_min_z={lowered_min_z:.3f}",
        )
    else:
        ctx.fail("arm pivot chops blade downward", "could not measure blade AABBs")

    if rest_handle_aabb is not None and turned_handle_aabb is not None:
        rest_y = 0.5 * (rest_handle_aabb[0][1] + rest_handle_aabb[1][1])
        turned_y = 0.5 * (turned_handle_aabb[0][1] + turned_handle_aabb[1][1])
        ctx.check(
            "miter table rotation swings front handle",
            abs(turned_y - rest_y) > 0.10,
            details=f"rest_y={rest_y:.3f}, turned_y={turned_y:.3f}",
        )
    else:
        ctx.fail("miter table rotation swings front handle", "could not measure miter handle")

    return ctx.report()


object_model = build_object_model()
