from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ClevisBracketGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BASE_RADIUS = 0.14
BASE_THICKNESS = 0.02
PEDESTAL_RADIUS = 0.022
PEDESTAL_HEIGHT = 0.055

LOWER_ARM_LENGTH = 0.70
LOWER_ARM_RADIUS = 0.0105
UPPER_ARM_LENGTH = 0.43
UPPER_ARM_RADIUS = 0.009

SHOULDER_BARREL_RADIUS = 0.008
SHOULDER_BARREL_LENGTH = 0.028
ELBOW_BARREL_RADIUS = 0.0075
ELBOW_BARREL_LENGTH = 0.024

SHOULDER_BORE_CENTER_Z = 0.022
ELBOW_BORE_CENTER_Z = 0.022
SHADE_TRUNNION_CENTER_Z = 0.024

LOWER_REST_PITCH = 1.10
UPPER_REST_DROP = 0.65
SHADE_REST_DROP = 0.55


def _shade_body_shape() -> cq.Workplane:
    body_length = 0.19
    rear_overhang = 0.06
    rear_radius = 0.028
    front_radius = 0.057
    wall = 0.0028

    outer = (
        cq.Workplane("YZ")
        .circle(rear_radius)
        .workplane(offset=body_length)
        .circle(front_radius)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("YZ")
        .circle(rear_radius - wall)
        .workplane(offset=body_length)
        .circle(front_radius - wall)
        .loft(combine=True)
        .translate((wall * 1.8, 0.0, 0.0))
    )
    body = outer.cut(inner).translate((-rear_overhang, 0.0, 0.0))

    tail = (
        cq.Workplane("YZ")
        .circle(0.010)
        .extrude(0.020)
        .translate((-rear_overhang - 0.020, 0.0, 0.0))
    )
    return body.union(tail)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_task_lamp")

    model.material("matte_black", rgba=(0.12, 0.12, 0.14, 1.0))
    model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="matte_black",
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)),
        material="graphite",
        name="pedestal",
    )
    base.visual(
        mesh_from_geometry(
            ClevisBracketGeometry(
                (0.052, 0.046, 0.050),
                gap_width=0.030,
                bore_diameter=0.018,
                bore_center_z=SHOULDER_BORE_CENTER_Z,
                base_thickness=0.012,
                corner_radius=0.003,
                center=False,
            ),
            "shoulder_clevis",
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT)),
        material="graphite",
        name="shoulder_clevis",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=SHOULDER_BARREL_RADIUS, length=SHOULDER_BARREL_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="shoulder_barrel",
    )
    lower_arm.visual(
        Box((0.080, 0.016, 0.016)),
        origin=Origin(xyz=(0.040, 0.0, 0.000)),
        material="graphite",
        name="shoulder_web",
    )
    lower_arm.visual(
        Cylinder(radius=LOWER_ARM_RADIUS, length=0.58),
        origin=Origin(xyz=(0.355, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="graphite",
        name="lower_tube",
    )
    lower_arm.visual(
        Box((0.090, 0.018, 0.024)),
        origin=Origin(xyz=(0.655, 0.0, -0.012)),
        material="graphite",
        name="elbow_web",
    )
    lower_arm.visual(
        mesh_from_geometry(
            ClevisBracketGeometry(
                (0.060, 0.042, 0.046),
                gap_width=0.028,
                bore_diameter=0.016,
                bore_center_z=ELBOW_BORE_CENTER_Z,
                base_thickness=0.012,
                corner_radius=0.003,
                center=False,
            ),
            "elbow_clevis",
        ),
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, -ELBOW_BORE_CENTER_Z)),
        material="graphite",
        name="elbow_clevis",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=ELBOW_BARREL_RADIUS, length=ELBOW_BARREL_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="elbow_barrel",
    )
    upper_arm.visual(
        Box((0.075, 0.014, 0.014)),
        origin=Origin(xyz=(0.0375, 0.0, 0.000)),
        material="graphite",
        name="elbow_web",
    )
    upper_arm.visual(
        Cylinder(radius=UPPER_ARM_RADIUS, length=0.26),
        origin=Origin(xyz=(0.200, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="graphite",
        name="upper_tube",
    )
    upper_arm.visual(
        Box((0.056, 0.104, 0.012)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.072, 0.0, -0.038)),
        material="graphite",
        name="yoke_bridge",
    )
    upper_arm.visual(
        Box((0.036, 0.016, 0.050)),
        origin=Origin(xyz=(0.318, 0.0, -0.024)),
        material="graphite",
        name="yoke_support",
    )
    upper_arm.visual(
        Box((0.060, 0.008, 0.070)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.028, -0.045, -0.005)),
        material="graphite",
        name="yoke_arm_0",
    )
    upper_arm.visual(
        Box((0.060, 0.008, 0.070)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.028, 0.045, -0.005)),
        material="graphite",
        name="yoke_arm_1",
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_shade_body_shape(), "shade_body"),
        material="matte_black",
        name="shade_body",
    )
    shade.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.0, -0.029, 0.0)),
        material="graphite",
        name="left_hub",
    )
    shade.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.0, 0.029, 0.0)),
        material="graphite",
        name="right_hub",
    )
    shade.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="left_trunnion",
    )
    shade.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="right_trunnion",
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT + SHOULDER_BORE_CENTER_Z),
            rpy=(0.0, -LOWER_REST_PITCH, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.10,
            upper=0.55,
            effort=30.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, UPPER_REST_DROP, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.00,
            upper=1.00,
            effort=18.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, SHADE_REST_DROP, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.95,
            upper=0.80,
            effort=8.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    tilt = object_model.get_articulation("upper_arm_to_shade")

    ctx.allow_overlap(
        base,
        lower_arm,
        elem_a="shoulder_clevis",
        elem_b="shoulder_web",
        reason="The compact shoulder knuckle web is intentionally simplified as a nested member inside the clevis cheeks.",
    )
    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        elem_a="elbow_clevis",
        elem_b="elbow_web",
        reason="The elbow knuckle web is intentionally simplified as a nested member inside the lower arm clevis.",
    )
    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        elem_a="elbow_web",
        elem_b="elbow_barrel",
        reason="The elbow barrel and its compact knuckle web are intentionally represented as a nested hinge package.",
    )
    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        elem_a="elbow_web",
        elem_b="elbow_web",
        reason="Both compact elbow knuckle webs occupy the same simplified hinge pocket around the elbow axis.",
    )

    ctx.expect_gap(
        lower_arm,
        base,
        axis="z",
        positive_elem="lower_tube",
        negative_elem="base_plate",
        min_gap=0.02,
        name="raised lower arm clears the round base",
    )
    ctx.expect_gap(
        shade,
        base,
        axis="z",
        positive_elem="shade_body",
        negative_elem="base_plate",
        min_gap=0.55,
        name="lamp head is held well above the base",
    )
    ctx.expect_within(
        lower_arm,
        base,
        axes="yz",
        inner_elem="shoulder_barrel",
        outer_elem="shoulder_clevis",
        margin=0.006,
        name="shoulder barrel sits inside the base clevis envelope",
    )
    ctx.expect_within(
        upper_arm,
        lower_arm,
        axes="yz",
        inner_elem="elbow_barrel",
        outer_elem="elbow_clevis",
        margin=0.006,
        name="elbow barrel sits inside the lower arm clevis envelope",
    )
    ctx.expect_contact(
        shade,
        upper_arm,
        elem_a="left_trunnion",
        elem_b="yoke_arm_0",
        name="left trunnion seats against the left yoke arm",
    )
    ctx.expect_contact(
        upper_arm,
        shade,
        elem_a="yoke_arm_1",
        elem_b="right_trunnion",
        name="right trunnion seats against the right yoke arm",
    )

    rest_head = ctx.part_world_position(shade)
    shoulder_upper = shoulder.motion_limits.upper if shoulder.motion_limits is not None else None
    elbow_upper = elbow.motion_limits.upper if elbow.motion_limits is not None else None
    tilt_lower = tilt.motion_limits.lower if tilt.motion_limits is not None else None
    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None

    with ctx.pose({shoulder: shoulder_upper}):
        shoulder_raised = ctx.part_world_position(shade)
    ctx.check(
        "shoulder hinge lifts the lamp head",
        rest_head is not None
        and shoulder_raised is not None
        and shoulder_raised[2] > rest_head[2] + 0.14,
        details=f"rest={rest_head}, raised={shoulder_raised}",
    )

    with ctx.pose({elbow: elbow_upper}):
        elbow_raised = ctx.part_world_position(shade)
    ctx.check(
        "elbow hinge changes the lamp head height",
        rest_head is not None
        and elbow_raised is not None
        and elbow_raised[2] > rest_head[2] + 0.08,
        details=f"rest={rest_head}, elbow_raised={elbow_raised}",
    )

    with ctx.pose({tilt: tilt_lower}):
        shade_down = ctx.part_element_world_aabb(shade, elem="shade_body")
    with ctx.pose({tilt: tilt_upper}):
        shade_up = ctx.part_element_world_aabb(shade, elem="shade_body")
    ctx.check(
        "shade tilt changes the spotlight aim",
        shade_down is not None
        and shade_up is not None
        and shade_up[1][2] > shade_down[1][2] + 0.02,
        details=f"down={shade_down}, up={shade_up}",
    )

    return ctx.report()


object_model = build_object_model()
