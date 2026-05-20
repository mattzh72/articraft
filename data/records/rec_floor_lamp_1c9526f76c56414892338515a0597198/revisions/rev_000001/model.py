from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _z_cylinder(radius: float, length: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, z0))


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_base_shape() -> cq.Workplane:
    disk = _z_cylinder(0.165, 0.028)
    stem = _z_cylinder(0.020, 0.060, 0.028)
    shoulder_block = _box((0.030, 0.016, 0.010), (0.0, 0.0, 0.089))
    ear_0 = _box((0.026, 0.008, 0.040), (0.0, -0.0115, 0.108))
    ear_1 = _box((0.026, 0.008, 0.040), (0.0, 0.0115, 0.108))
    return disk.union(stem).union(shoulder_block).union(ear_0).union(ear_1)


def _build_lower_arm_shape() -> cq.Workplane:
    hinge_barrel = _y_cylinder(0.008, 0.013)
    root_block = _box((0.016, 0.012, 0.018), (0.0, 0.0, 0.009))
    arm_tube = _z_cylinder(0.0105, 0.730, 0.018)
    elbow_block = _box((0.022, 0.022, 0.018), (0.0, 0.0, 0.756))
    ear_0 = _box((0.022, 0.006, 0.024), (0.0, -0.0095, 0.776))
    ear_1 = _box((0.022, 0.006, 0.024), (0.0, 0.0095, 0.776))
    return hinge_barrel.union(root_block).union(arm_tube).union(elbow_block).union(ear_0).union(ear_1)


def _build_upper_arm_shape() -> cq.Workplane:
    hinge_barrel = _y_cylinder(0.0065, 0.009)
    root_block = _box((0.014, 0.010, 0.016), (0.0, 0.0, 0.008))
    arm_tube = _z_cylinder(0.009, 0.470, 0.016)
    tip_block = _box((0.020, 0.018, 0.014), (0.0, 0.0, 0.492))
    ear_0 = _box((0.020, 0.005, 0.020), (0.0, -0.0075, 0.502))
    ear_1 = _box((0.020, 0.005, 0.020), (0.0, 0.0075, 0.502))
    return hinge_barrel.union(root_block).union(arm_tube).union(tip_block).union(ear_0).union(ear_1)


def _build_shade_mount_shape() -> cq.Workplane:
    hinge_barrel = _y_cylinder(0.0055, 0.007)
    connector = _z_cylinder(0.0045, 0.010)
    neck = _z_cylinder(0.014, 0.036, 0.010)
    rear_boss = _z_cylinder(0.020, 0.008, 0.044)
    knob_boss = _x_cylinder(0.007, 0.008).translate((0.018, 0.0, 0.030))
    rear_cap = (
        cq.Workplane("XY")
        .circle(0.086)
        .circle(0.012)
        .extrude(0.004)
        .translate((0.0, 0.0, 0.048))
    )
    return hinge_barrel.union(connector).union(neck).union(rear_boss).union(knob_boss).union(rear_cap)


def _build_shade_shell_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.090)
        .circle(0.082)
        .extrude(0.190)
        .translate((0.0, 0.0, 0.050))
    )


def _build_knob_shape() -> cq.Workplane:
    shaft = _z_cylinder(0.0045, 0.008)
    body = _z_cylinder(0.0105, 0.010, 0.008)
    face_rim = _z_cylinder(0.0125, 0.002, 0.018)
    return shaft.union(body).union(face_rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    dark_metal = model.material("dark_metal", rgba=(0.15, 0.15, 0.16, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.56, 0.54, 0.50, 1.0))
    shade_fabric = model.material("shade_fabric", rgba=(0.93, 0.91, 0.84, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.43, 0.36, 0.26, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base"),
        material=dark_metal,
        name="base_body",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_build_lower_arm_shape(), "lower_arm"),
        material=dark_metal,
        name="lower_arm_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_build_upper_arm_shape(), "upper_arm"),
        material=dark_metal,
        name="upper_arm_body",
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_build_shade_mount_shape(), "shade_mount"),
        material=brushed_metal,
        name="shade_mount",
    )
    shade.visual(
        mesh_from_cadquery(_build_shade_shell_shape(), "shade_shell"),
        material=shade_fabric,
        name="shade_shell",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_cadquery(_build_knob_shape(), "knob"),
        material=knob_finish,
        name="knob_body",
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.108), rpy=(0.0, 0.25, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.3, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.77635), rpy=(0.0, 0.72, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.85, upper=0.80),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, 0.506), rpy=(0.0, 0.88, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.70, upper=0.90),
    )
    model.articulation(
        "shade_to_knob",
        ArticulationType.CONTINUOUS,
        parent=shade,
        child=knob,
        origin=Origin(xyz=(0.022, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_hinge = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    shade_hinge = object_model.get_articulation("upper_arm_to_shade")
    knob_joint = object_model.get_articulation("shade_to_knob")

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    ctx.allow_overlap(
        base,
        lower_arm,
        elem_a="base_body",
        elem_b="lower_arm_body",
        reason="The shoulder knuckle is represented as a tight clevis around the lower arm hinge barrel, so the simplified solids intentionally occupy some shared hinge volume.",
    )
    ctx.allow_overlap(
        upper_arm,
        shade,
        elem_a="upper_arm_body",
        elem_b="shade_mount",
        reason="The lamp head tilt joint is modeled as compact nested knuckles, and the simplified mount proxy intentionally shares local hinge volume with the arm tip.",
    )
    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        elem_a="lower_arm_body",
        elem_b="upper_arm_body",
        reason="The elbow is represented as interleaved hinge knuckles, so the simplified solid elbow bodies intentionally share a small hinge-core volume.",
    )

    ctx.expect_origin_gap(
        lower_arm,
        base,
        axis="z",
        min_gap=0.10,
        name="lower arm hinge sits above the weighted base",
    )

    lower_rest = ctx.part_world_position(upper_arm)
    with ctx.pose({lower_hinge: 0.45}):
        lower_forward = ctx.part_world_position(upper_arm)
    ctx.check(
        "lower arm swings outward from the base hinge",
        lower_rest is not None
        and lower_forward is not None
        and lower_forward[0] > lower_rest[0] + 0.10
        and lower_forward[2] < lower_rest[2] - 0.02,
        details=f"rest={lower_rest}, forward={lower_forward}",
    )

    shade_rest = ctx.part_world_position(shade)
    with ctx.pose({elbow: 0.65}):
        shade_reached = ctx.part_world_position(shade)
    ctx.check(
        "upper arm elbow extends the lamp head outward",
        shade_rest is not None
        and shade_reached is not None
        and shade_reached[0] > shade_rest[0] + 0.04,
        details=f"rest={shade_rest}, reached={shade_reached}",
    )

    rest_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_hinge: 0.70}):
        tipped_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade hinge can tip the drum downward",
        rest_aabb is not None
        and tipped_aabb is not None
        and tipped_aabb[0][2] < rest_aabb[0][2] - 0.04,
        details=f"rest={rest_aabb}, tipped={tipped_aabb}",
    )

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "dimmer knob uses a continuous rotary joint",
        knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None
        and str(knob_joint.articulation_type).lower().endswith("continuous"),
        details=f"type={knob_joint.articulation_type}, limits={knob_limits}",
    )

    return ctx.report()


object_model = build_object_model()
