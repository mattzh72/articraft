from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_pitch_roll_cartridge")

    cast_gray = model.material("cast_gray", color=(0.40, 0.43, 0.45, 1.0))
    dark_hardcoat = model.material("dark_hardcoat", color=(0.06, 0.07, 0.075, 1.0))
    satin_steel = model.material("satin_steel", color=(0.72, 0.72, 0.68, 1.0))
    output_blue = model.material("output_blue", color=(0.12, 0.24, 0.55, 1.0))
    bolt_black = model.material("black_fasteners", color=(0.01, 0.01, 0.012, 1.0))

    side_structure = model.part("side_structure")

    cheek_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.42, 0.88, 0.035, corner_segments=8),
        [_circle_profile(0.078, 64)],
        0.080,
        center=True,
    )
    cheek_plate.rotate_y(math.pi / 2.0)
    side_structure.visual(
        mesh_from_geometry(cheek_plate, "cheek_plate"),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=cast_gray,
        name="cheek_plate",
    )
    side_structure.visual(
        Box((0.46, 0.56, 0.060)),
        origin=Origin(xyz=(-0.020, 0.0, -0.470)),
        material=dark_hardcoat,
        name="base_foot",
    )
    for idx, y in enumerate((-0.175, 0.175)):
        side_structure.visual(
            Box((0.250, 0.035, 0.430)),
            origin=Origin(xyz=(-0.035, y, -0.250)),
            material=cast_gray,
            name=f"web_rib_{idx}",
        )

    bearing_ring = TorusGeometry(radius=0.096, tube=0.015, radial_segments=16, tubular_segments=48)
    bearing_ring.rotate_y(math.pi / 2.0)
    side_structure.visual(
        mesh_from_geometry(bearing_ring, "roll_bearing_ring"),
        origin=Origin(xyz=(-0.033, 0.0, 0.0)),
        material=satin_steel,
        name="roll_bearing_ring",
    )
    side_structure.visual(
        Cylinder(radius=0.075, length=0.010),
        origin=Origin(xyz=(-0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="roll_bearing_face",
    )
    for idx, (y, z) in enumerate(((-0.125, -0.125), (-0.125, 0.125), (0.125, -0.125), (0.125, 0.125))):
        side_structure.visual(
            Cylinder(radius=0.018, length=0.026),
            origin=Origin(xyz=(-0.030, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"cheek_bolt_{idx}",
        )

    roll_package = model.part("roll_package")
    roll_package.visual(
        Cylinder(radius=0.040, length=0.090),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="roll_spindle",
    )
    roll_package.visual(
        Cylinder(radius=0.075, length=0.010),
        origin=Origin(xyz=(-0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="roll_collar",
    )
    roll_package.visual(
        Cylinder(radius=0.128, length=0.080),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardcoat,
        name="roll_drum",
    )
    roll_package.visual(
        Box((0.050, 0.300, 0.220)),
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
        material=dark_hardcoat,
        name="yoke_bridge",
    )
    roll_package.visual(
        Box((0.150, 0.045, 0.205)),
        origin=Origin(xyz=(0.190, -0.145, 0.0)),
        material=dark_hardcoat,
        name="pitch_bearing_0",
    )
    roll_package.visual(
        Cylinder(radius=0.048, length=0.016),
        origin=Origin(xyz=(0.190, -0.142, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_bushing_0",
    )
    roll_package.visual(
        Box((0.150, 0.045, 0.205)),
        origin=Origin(xyz=(0.190, 0.145, 0.0)),
        material=dark_hardcoat,
        name="pitch_bearing_1",
    )
    roll_package.visual(
        Cylinder(radius=0.048, length=0.016),
        origin=Origin(xyz=(0.190, 0.142, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_bushing_1",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.030, length=0.248),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_trunnion",
    )
    pitch_cradle.visual(
        Box((0.075, 0.155, 0.080)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=output_blue,
        name="cradle_body",
    )
    for idx, z in enumerate((-0.042, 0.042)):
        pitch_cradle.visual(
            Box((0.135, 0.130, 0.022)),
            origin=Origin(xyz=(0.082, 0.0, z)),
            material=output_blue,
            name=f"cradle_rail_{idx}",
        )
    pitch_cradle.visual(
        Box((0.020, 0.150, 0.128)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=output_blue,
        name="output_flange",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.174, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="output_boss",
    )
    for idx, (y, z) in enumerate(((-0.048, -0.040), (-0.048, 0.040), (0.048, -0.040), (0.048, 0.040))):
        pitch_cradle.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(0.168, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"flange_bolt_{idx}",
        )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=side_structure,
        child=roll_package,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-math.radians(75.0), upper=math.radians(75.0)),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=roll_package,
        child=pitch_cradle,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-math.radians(45.0), upper=math.radians(45.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    side_structure = object_model.get_part("side_structure")
    roll_package = object_model.get_part("roll_package")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_axis = object_model.get_articulation("roll_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")

    ctx.allow_overlap(
        roll_package,
        pitch_cradle,
        elem_a="pitch_bearing_0",
        elem_b="pitch_trunnion",
        reason="The pitch trunnion is intentionally captured a millimetre into the left bearing block proxy.",
    )
    ctx.allow_overlap(
        roll_package,
        pitch_cradle,
        elem_a="pitch_bearing_1",
        elem_b="pitch_trunnion",
        reason="The pitch trunnion is intentionally captured a millimetre into the right bearing block proxy.",
    )

    ctx.check(
        "roll joint is a revolute x-axis",
        roll_axis.articulation_type == ArticulationType.REVOLUTE and tuple(roll_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={roll_axis.articulation_type}, axis={roll_axis.axis}",
    )
    pitch_dot = sum(float(a) * float(b) for a, b in zip(roll_axis.axis, pitch_axis.axis))
    ctx.check(
        "pitch joint is perpendicular revolute",
        pitch_axis.articulation_type == ArticulationType.REVOLUTE and abs(pitch_dot) < 1.0e-6,
        details=f"type={pitch_axis.articulation_type}, roll_axis={roll_axis.axis}, pitch_axis={pitch_axis.axis}",
    )

    ctx.expect_gap(
        roll_package,
        side_structure,
        axis="x",
        positive_elem="roll_collar",
        negative_elem="roll_bearing_face",
        max_gap=0.001,
        max_penetration=0.0005,
        name="roll collar is seated on cheek bearing face",
    )
    ctx.expect_gap(
        pitch_cradle,
        roll_package,
        axis="y",
        positive_elem="pitch_trunnion",
        negative_elem="pitch_bearing_0",
        max_gap=0.002,
        max_penetration=0.003,
        name="negative pitch trunnion is seated in bearing",
    )
    ctx.expect_gap(
        roll_package,
        pitch_cradle,
        axis="y",
        positive_elem="pitch_bearing_1",
        negative_elem="pitch_trunnion",
        max_gap=0.002,
        max_penetration=0.003,
        name="positive pitch trunnion is seated in bearing",
    )

    def aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        low, high = aabb
        return (float(low[2]) + float(high[2])) * 0.5

    def aabb_span_z(aabb) -> float | None:
        if aabb is None:
            return None
        low, high = aabb
        return float(high[2]) - float(low[2])

    rest_flange = ctx.part_element_world_aabb(pitch_cradle, elem="output_flange")
    rest_flange_z = aabb_center_z(rest_flange)
    rest_span_z = aabb_span_z(rest_flange)
    with ctx.pose({pitch_axis: pitch_axis.motion_limits.upper}):
        pitched_flange_z = aabb_center_z(ctx.part_element_world_aabb(pitch_cradle, elem="output_flange"))
    with ctx.pose({roll_axis: roll_axis.motion_limits.upper}):
        rolled_span_z = aabb_span_z(ctx.part_element_world_aabb(pitch_cradle, elem="output_flange"))

    ctx.check(
        "pitch motion tilts the output flange",
        rest_flange_z is not None
        and pitched_flange_z is not None
        and abs(pitched_flange_z - rest_flange_z) > 0.050,
        details=f"rest_z={rest_flange_z}, pitched_z={pitched_flange_z}",
    )
    ctx.check(
        "roll motion rotates the cartridge package",
        rest_span_z is not None and rolled_span_z is not None and rolled_span_z > rest_span_z + 0.030,
        details=f"rest_z_span={rest_span_z}, rolled_z_span={rolled_span_z}",
    )

    return ctx.report()


object_model = build_object_model()
