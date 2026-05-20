from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ARM_LENGTH = 0.54
HEAD_LINK_LENGTH = 0.15
HEAD_LENGTH = 0.33


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(center: tuple[float, float, float], radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length, both=True).translate(center)


def _z_cylinder(center: tuple[float, float, float], radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length, both=True).translate(center)


def _yoke(
    *,
    center_x: float,
    depth_x: float,
    cheek_thickness: float,
    gap_width: float,
    height_z: float,
    bridge_thickness_x: float,
    bridge_height_z: float,
    bore_radius: float,
) -> cq.Workplane:
    cheek_offset_y = gap_width / 2.0 + cheek_thickness / 2.0
    bridge_center_x = center_x - 0.5 * (depth_x - bridge_thickness_x)
    bridge_center_z = -height_z / 2.0 + bridge_height_z / 2.0

    yoke = _box((center_x, cheek_offset_y, 0.0), (depth_x, cheek_thickness, height_z))
    yoke = yoke.union(_box((center_x, -cheek_offset_y, 0.0), (depth_x, cheek_thickness, height_z)))
    yoke = yoke.union(
        _box(
            (bridge_center_x, 0.0, bridge_center_z),
            (bridge_thickness_x, gap_width + 2.0 * cheek_thickness, bridge_height_z),
        )
    )
    return yoke.cut(_y_cylinder((center_x, 0.0, 0.0), bore_radius, gap_width + 2.0 * cheek_thickness + 0.01))


def _clamp_body_shape() -> cq.Workplane:
    spine = _box((0.003, 0.0, -0.066), (0.028, 0.034, 0.108))
    top_jaw = _box((-0.047, 0.0, -0.020), (0.078, 0.034, 0.014))
    lower_carriage = _box((-0.026, 0.0, -0.096), (0.060, 0.034, 0.024))
    screw = _z_cylinder((-0.045, 0.0, -0.082), 0.006, 0.068)
    pressure_pad = _z_cylinder((-0.045, 0.0, -0.051), 0.011, 0.004)
    knob_wheel = _z_cylinder((-0.045, 0.0, -0.120), 0.018, 0.008)
    knob_crossbar = _y_cylinder((-0.045, 0.0, -0.120), 0.004, 0.044)

    clamp = spine.union(top_jaw)
    clamp = clamp.union(lower_carriage)
    clamp = clamp.union(screw)
    clamp = clamp.union(pressure_pad)
    clamp = clamp.union(knob_wheel)
    clamp = clamp.union(knob_crossbar)
    return clamp


def _clamp_yoke_shape() -> cq.Workplane:
    return _yoke(
        center_x=0.0,
        depth_x=0.026,
        cheek_thickness=0.010,
        gap_width=0.019,
        height_z=0.028,
        bridge_thickness_x=0.012,
        bridge_height_z=0.004,
        bore_radius=0.0045,
    )


def _arm_knuckle_shape() -> cq.Workplane:
    shoulder_boss = _y_cylinder((0.0, 0.0, 0.0), 0.0085, 0.017)
    root_neck = _box((0.021, 0.0, 0.0), (0.026, 0.014, 0.012))
    return shoulder_boss.union(root_neck)


def _arm_body_shape() -> cq.Workplane:
    beam = _box((0.285, 0.0, 0.0), (0.510, 0.022, 0.014)).edges("|X").fillet(0.0025)
    elbow_yoke = _yoke(
        center_x=ARM_LENGTH,
        depth_x=0.022,
        cheek_thickness=0.009,
        gap_width=0.016,
        height_z=0.024,
        bridge_thickness_x=0.010,
        bridge_height_z=0.003,
        bore_radius=0.0040,
    )
    return beam.union(elbow_yoke)


def _head_link_knuckle_shape() -> cq.Workplane:
    root_boss = _y_cylinder((0.0, 0.0, 0.0), 0.0075, 0.014)
    root_neck = _box((0.0175, 0.0, 0.0), (0.021, 0.011, 0.010))
    return root_boss.union(root_neck)


def _head_link_body_shape() -> cq.Workplane:
    beam = _box((0.087, 0.0, 0.0), (0.126, 0.018, 0.012)).edges("|X").fillet(0.0020)
    head_yoke = _yoke(
        center_x=HEAD_LINK_LENGTH,
        depth_x=0.020,
        cheek_thickness=0.009,
        gap_width=0.017,
        height_z=0.022,
        bridge_thickness_x=0.009,
        bridge_height_z=0.003,
        bore_radius=0.0040,
    )
    return beam.union(head_yoke)


def _lamp_head_knuckle_shape() -> cq.Workplane:
    rear_boss = _y_cylinder((0.0, 0.0, 0.0), 0.0075, 0.015)
    rear_neck = _box((0.0155, 0.0, 0.0), (0.017, 0.012, 0.010))
    return rear_boss.union(rear_neck)


def _lamp_head_body_shape() -> cq.Workplane:
    rear_housing = _box((0.044, 0.0, 0.0), (0.052, 0.044, 0.019)).edges("|X").fillet(0.0025)
    bar_body = _box((0.175, 0.0, 0.0), (0.310, 0.052, 0.013)).edges("|X").fillet(0.0035)
    return rear_housing.union(bar_body)


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="task_lamp")

    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    joint_black = model.material("joint_black", rgba=(0.12, 0.13, 0.14, 1.0))
    diffuser = model.material("diffuser", rgba=(0.90, 0.92, 0.88, 0.95))

    clamp = model.part("clamp")
    clamp.visual(
        mesh_from_cadquery(_clamp_body_shape(), "lamp_clamp_body"),
        material=graphite,
        name="clamp_shell",
    )
    clamp.visual(
        mesh_from_cadquery(_clamp_yoke_shape(), "lamp_clamp_yoke"),
        material=joint_black,
        name="shoulder_yoke",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_knuckle_shape(), "lamp_arm_knuckle"),
        material=joint_black,
        name="shoulder_knuckle",
    )
    arm.visual(
        mesh_from_cadquery(_arm_body_shape(), "lamp_arm_body"),
        material=graphite,
        name="arm_shell",
    )

    head_link = model.part("head_link")
    head_link.visual(
        mesh_from_cadquery(_head_link_knuckle_shape(), "lamp_head_link_knuckle"),
        material=joint_black,
        name="link_knuckle",
    )
    head_link.visual(
        mesh_from_cadquery(_head_link_body_shape(), "lamp_head_link_body"),
        material=joint_black,
        name="link_shell",
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        mesh_from_cadquery(_lamp_head_knuckle_shape(), "lamp_head_knuckle"),
        material=joint_black,
        name="head_knuckle",
    )
    lamp_head.visual(
        mesh_from_cadquery(_lamp_head_body_shape(), "lamp_head_body"),
        material=graphite,
        name="head_shell",
    )
    lamp_head.visual(
        Box((0.272, 0.030, 0.0025)),
        origin=Origin(xyz=(0.180, 0.0, -0.0055)),
        material=diffuser,
        name="diffuser",
    )

    model.articulation(
        "clamp_to_arm",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-0.95,
            upper=1.20,
        ),
    )
    model.articulation(
        "arm_to_head_link",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head_link,
        origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.8,
            lower=-1.10,
            upper=1.15,
        ),
    )
    model.articulation(
        "head_link_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=head_link,
        child=lamp_head,
        origin=Origin(xyz=(HEAD_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=-0.95,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp = object_model.get_part("clamp")
    arm = object_model.get_part("arm")
    head_link = object_model.get_part("head_link")
    lamp_head = object_model.get_part("lamp_head")
    shoulder = object_model.get_articulation("clamp_to_arm")
    elbow = object_model.get_articulation("arm_to_head_link")
    tilt = object_model.get_articulation("head_link_to_lamp_head")

    ctx.allow_overlap(
        clamp,
        arm,
        elem_a="shoulder_yoke",
        elem_b="shoulder_knuckle",
        reason="The shoulder hinge is represented as a simplified boss nested inside the clamp yoke.",
    )
    ctx.allow_overlap(
        arm,
        head_link,
        elem_a="arm_shell",
        elem_b="link_knuckle",
        reason="The elbow hinge uses a simplified nested knuckle inside the arm-end yoke.",
    )
    ctx.allow_overlap(
        head_link,
        lamp_head,
        elem_a="link_shell",
        elem_b="head_knuckle",
        reason="The lamp-head tilt hinge is simplified as a compact nested knuckle inside the forked tip.",
    )

    ctx.expect_origin_gap(
        lamp_head,
        clamp,
        axis="x",
        min_gap=0.66,
        name="lamp head reaches well out from the desk clamp",
    )

    rest_arm_shell = ctx.part_element_world_aabb(arm, elem="arm_shell")
    with ctx.pose({shoulder: 1.0}):
        raised_arm_shell = ctx.part_element_world_aabb(arm, elem="arm_shell")
    rest_arm_z = _aabb_center_z(rest_arm_shell)
    raised_arm_z = _aabb_center_z(raised_arm_shell)
    ctx.check(
        "shoulder hinge lifts the long arm upward",
        rest_arm_z is not None and raised_arm_z is not None and raised_arm_z > rest_arm_z + 0.18,
        details=f"rest_z={rest_arm_z}, raised_z={raised_arm_z}",
    )

    rest_head_shell = ctx.part_element_world_aabb(lamp_head, elem="head_shell")
    with ctx.pose({elbow: 0.95}):
        raised_head_shell = ctx.part_element_world_aabb(lamp_head, elem="head_shell")
    rest_head_z = _aabb_center_z(rest_head_shell)
    raised_head_z = _aabb_center_z(raised_head_shell)
    ctx.check(
        "tip hinge raises the head assembly",
        rest_head_z is not None and raised_head_z is not None and raised_head_z > rest_head_z + 0.10,
        details=f"rest_z={rest_head_z}, raised_z={raised_head_z}",
    )

    with ctx.pose({tilt: -0.75}):
        downward_head_shell = ctx.part_element_world_aabb(lamp_head, elem="head_shell")
    with ctx.pose({tilt: 0.55}):
        upward_head_shell = ctx.part_element_world_aabb(lamp_head, elem="head_shell")
    downward_head_z = _aabb_center_z(downward_head_shell)
    upward_head_z = _aabb_center_z(upward_head_shell)
    ctx.check(
        "led bar head tilts through a meaningful aiming range",
        downward_head_z is not None and upward_head_z is not None and upward_head_z > downward_head_z + 0.07,
        details=f"downward_z={downward_head_z}, upward_z={upward_head_z}",
    )

    return ctx.report()


object_model = build_object_model()
