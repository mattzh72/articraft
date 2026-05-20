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


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder_along(
    axis: tuple[float, float, float],
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    direction = cq.Vector(*axis)
    start = cq.Vector(*center) - direction.multiply(length / 2.0)
    solid = cq.Solid.makeCylinder(radius, length, start, direction)
    return cq.Workplane("XY").add(solid)


def _side_plate_shape(y_center: float, sign: float) -> cq.Workplane:
    """One base-mounted side support with a real trunnion clearance hole."""
    plate_thickness = 0.022
    boss_length = 0.016
    axis_z = 0.200

    plate = _cq_box((0.170, plate_thickness, 0.250), (0.0, y_center, 0.158))
    foot_flange = _cq_box((0.205, 0.060, 0.020), (0.0, y_center, 0.044))
    support = plate.union(foot_flange)

    outer_boss_center_y = y_center + sign * (plate_thickness / 2.0 + boss_length / 2.0 - 0.001)
    inner_boss_center_y = y_center - sign * (plate_thickness / 2.0 + 0.004 - 0.001)
    outer_boss = _cq_cylinder_along((0.0, sign, 0.0), 0.042, boss_length, (0.0, outer_boss_center_y, axis_z))
    inner_boss = _cq_cylinder_along((0.0, -sign, 0.0), 0.033, 0.008, (0.0, inner_boss_center_y, axis_z))
    support = support.union(outer_boss).union(inner_boss)

    # A horizontal through-hole makes the trunnion visually pass through the
    # support rather than intersect a solid side plate.
    hole = _cq_cylinder_along((0.0, sign, 0.0), 0.022, 0.070, (0.0, y_center, axis_z))
    # The boss/plate unions already provide a mechanically readable outline;
    # keep this unfilleted because the through-hole creates tiny edge fragments
    # that can make CadQuery's generic fillet builder fail.
    return support.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_axis_pitch_cradle")

    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.05, 0.22, 0.55, 1.0))
    black_plate = model.material("black_faceplate", rgba=(0.02, 0.02, 0.025, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.70, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.380, 0.420, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_slab",
    )
    for index, (y_center, sign) in enumerate(((-0.155, -1.0), (0.155, 1.0))):
        stand.visual(
            mesh_from_cadquery(_side_plate_shape(y_center, sign), f"side_plate_{index}"),
            material=dark_steel,
            name=f"side_plate_{index}",
        )

    for index, (x, y) in enumerate(((-0.145, -0.165), (-0.145, 0.165), (0.145, -0.165), (0.145, 0.165))):
        stand.visual(
            Box((0.052, 0.052, 0.010)),
            origin=Origin(xyz=(x, y, -0.005)),
            material=rubber,
            name=f"foot_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.015, length=0.400),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="trunnion_shaft",
    )
    for index, y in enumerate((-0.078, 0.078)):
        yoke.visual(
            Box((0.160, 0.020, 0.032)),
            origin=Origin(xyz=(0.070, y, 0.0)),
            material=blue_anodized,
            name=f"yoke_arm_{index}",
        )
        yoke.visual(
            Cylinder(radius=0.033, length=0.028),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blue_anodized,
            name=f"pivot_hub_{index}",
        )

    yoke.visual(
        Box((0.014, 0.146, 0.105)),
        origin=Origin(xyz=(0.154, 0.0, 0.0)),
        material=black_plate,
        name="faceplate",
    )
    yoke.visual(
        Box((0.006, 0.090, 0.050)),
        origin=Origin(xyz=(0.164, 0.0, 0.0)),
        material=Material("dark_recess", rgba=(0.0, 0.0, 0.0, 1.0)),
        name="front_pad",
    )
    for row, z in enumerate((-0.036, 0.036)):
        for col, y in enumerate((-0.045, 0.045)):
            yoke.visual(
                Cylinder(radius=0.006, length=0.005),
                origin=Origin(xyz=(0.1635, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brushed_metal,
                name=f"face_bolt_{row}_{col}",
            )
    for index, y in enumerate((-0.185, 0.185)):
        yoke.visual(
            Cylinder(radius=0.022, length=0.008),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"shaft_collar_{index}",
        )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    joint = object_model.get_articulation("pitch_trunnion")

    ctx.check(
        "single revolute pitch axis",
        joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_overlap(
        yoke,
        stand,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="side_plate_0",
        min_overlap=0.015,
        name="shaft passes through first side support",
    )
    ctx.expect_overlap(
        yoke,
        stand,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="side_plate_1",
        min_overlap=0.015,
        name="shaft passes through second side support",
    )
    ctx.expect_gap(
        yoke,
        stand,
        axis="z",
        positive_elem="faceplate",
        negative_elem="base_slab",
        min_gap=0.090,
        name="faceplate clears base at level pose",
    )

    rest_aabb = ctx.part_element_world_aabb(yoke, elem="faceplate")
    with ctx.pose({joint: 0.70}):
        pitched_aabb = ctx.part_element_world_aabb(yoke, elem="faceplate")
        ctx.expect_gap(
            yoke,
            stand,
            axis="z",
            positive_elem="faceplate",
            negative_elem="base_slab",
            min_gap=0.020,
            name="faceplate clears base at pitch limit",
        )
    if rest_aabb is not None and pitched_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
        pitched_center_z = (pitched_aabb[0][2] + pitched_aabb[1][2]) / 2.0
    else:
        rest_center_z = pitched_center_z = None
    ctx.check(
        "faceplate pitches about trunnion",
        rest_center_z is not None and pitched_center_z is not None and pitched_center_z < rest_center_z - 0.04,
        details=f"rest_z={rest_center_z}, pitched_z={pitched_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
