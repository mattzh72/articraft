from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, center: tuple[float, float] = (0.0, 0.0), segments: int = 40):
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_pitch_trunnion")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.22, 0.25, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.72, 0.48, 0.24, 1.0))
    output_blue = model.material("output_blue", rgba=(0.10, 0.30, 0.66, 1.0))
    bolt_black = model.material("bolt_black", rgba=(0.015, 0.015, 0.018, 1.0))

    pivot_x = 0.155
    pivot_z = 0.310
    cheek_center_x = 0.125
    cheek_center_z = pivot_z
    cheek_center_y = 0.120
    cheek_thickness = 0.036

    cheek_profile = rounded_rect_profile(0.245, 0.235, 0.018, corner_segments=8)
    cheek_hole = _circle_profile(
        0.031,
        center=(pivot_x - cheek_center_x, pivot_z - cheek_center_z),
        segments=48,
    )
    cheek_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(cheek_profile, [cheek_hole], cheek_thickness),
        "bored_side_cheek",
    )
    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.041, tube=0.0075, radial_segments=18, tubular_segments=48),
        "bearing_lip",
    )

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.030, 0.360, 0.460)),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=painted_steel,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.115, 0.390, 0.035)),
        origin=Origin(xyz=(0.035, 0.0, 0.0175)),
        material=painted_steel,
        name="ground_foot",
    )
    side_plate.visual(
        Box((0.205, 0.285, 0.026)),
        origin=Origin(xyz=(0.112, 0.0, 0.178)),
        material=painted_steel,
        name="lower_bridge",
    )
    side_plate.visual(
        Box((0.188, 0.105, 0.044)),
        origin=Origin(xyz=(0.094, 0.0, 0.238)),
        material=painted_steel,
        name="cartridge_spine",
    )
    side_plate.visual(
        Box((0.036, 0.250, 0.170)),
        origin=Origin(xyz=(0.018, 0.0, 0.298)),
        material=painted_steel,
        name="wall_boss",
    )
    side_plate.visual(
        cheek_mesh,
        origin=Origin(
            xyz=(cheek_center_x, cheek_center_y, cheek_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=painted_steel,
        name="side_cheek_pos",
    )
    side_plate.visual(
        cheek_mesh,
        origin=Origin(
            xyz=(cheek_center_x, -cheek_center_y, cheek_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=painted_steel,
        name="side_cheek_neg",
    )
    for y, name in ((0.144, "bearing_pos"), (-0.144, "bearing_neg")):
        side_plate.visual(
            bearing_mesh,
            origin=Origin(xyz=(pivot_x, y, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_bronze,
            name=name,
        )
    for y, z, name in (
        (-0.145, 0.075, "bolt_0"),
        (0.145, 0.075, "bolt_1"),
        (-0.145, 0.390, "bolt_2"),
        (0.145, 0.390, "bolt_3"),
    ):
        side_plate.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(0.020, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=name,
        )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.019, length=0.315),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_pin",
    )
    faceplate.visual(
        Cylinder(radius=0.041, length=0.174),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    faceplate.visual(
        Box((0.170, 0.075, 0.055)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=output_blue,
        name="output_neck",
    )
    faceplate.visual(
        Box((0.026, 0.190, 0.160)),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material=output_blue,
        name="faceplate_panel",
    )
    faceplate.visual(
        Cylinder(radius=0.034, length=0.024),
        origin=Origin(xyz=(0.202, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="output_boss",
    )
    for y, z, name in (
        (-0.060, -0.050, "face_bolt_0"),
        (0.060, -0.050, "face_bolt_1"),
        (-0.060, 0.050, "face_bolt_2"),
        (0.060, 0.050, "face_bolt_3"),
    ):
        faceplate.visual(
            Cylinder(radius=0.0085, length=0.009),
            origin=Origin(xyz=(0.196, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=name,
        )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=faceplate,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    faceplate = object_model.get_part("faceplate")
    pitch = object_model.get_articulation("pitch_axis")

    ctx.check(
        "single revolute pitch axis",
        len(object_model.articulations) == 1
        and pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 4) for v in pitch.axis) == (0.0, -1.0, 0.0),
        details=f"joints={len(object_model.articulations)}, type={pitch.articulation_type}, axis={pitch.axis}",
    )
    limits = pitch.motion_limits
    ctx.check(
        "pitch travel is trunnion limited",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < -0.5 and limits.upper > 0.5,
        details=f"limits={limits}",
    )

    ctx.allow_overlap(
        faceplate,
        side_plate,
        elem_a="trunnion_pin",
        elem_b="side_cheek_pos",
        reason="The trunnion pin is intentionally captured in the cheek bore; the cheek mesh acts as the bearing support around the shaft.",
    )
    ctx.allow_overlap(
        faceplate,
        side_plate,
        elem_a="trunnion_pin",
        elem_b="side_cheek_neg",
        reason="The trunnion pin is intentionally captured in the opposite cheek bore; this is the supported pitch-axis interface.",
    )

    ctx.expect_overlap(
        faceplate,
        side_plate,
        axes="xz",
        elem_a="trunnion_pin",
        elem_b="side_cheek_pos",
        min_overlap=0.018,
        name="pin aligns with positive cheek bore",
    )
    ctx.expect_overlap(
        faceplate,
        side_plate,
        axes="xz",
        elem_a="trunnion_pin",
        elem_b="side_cheek_neg",
        min_overlap=0.018,
        name="pin aligns with negative cheek bore",
    )
    ctx.expect_gap(
        side_plate,
        faceplate,
        axis="y",
        positive_elem="side_cheek_pos",
        negative_elem="hub_barrel",
        min_gap=0.010,
        name="hub clears positive cheek",
    )
    ctx.expect_gap(
        faceplate,
        side_plate,
        axis="y",
        positive_elem="hub_barrel",
        negative_elem="side_cheek_neg",
        min_gap=0.010,
        name="hub clears negative cheek",
    )

    rest_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_panel")
    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    with ctx.pose({pitch: 0.45}):
        raised_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_panel")
        raised_center_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5
    ctx.check(
        "positive pitch raises carried face",
        rest_center_z is not None and raised_center_z is not None and raised_center_z > rest_center_z + 0.050,
        details=f"rest_z={rest_center_z}, raised_z={raised_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
