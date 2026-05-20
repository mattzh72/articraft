from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CYLINDER_ALONG_Y = (math.pi / 2.0, 0.0, 0.0)
CYLINDER_ALONG_X = (0.0, math.pi / 2.0, 0.0)


def _add_barrel_section(
    part,
    *,
    radius: float,
    length: float,
    center_y: float,
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(0.0, center_y, 0.0), rpy=CYLINDER_ALONG_Y),
        material=material,
        name=name,
    )


def _make_tilt_frame_shape() -> cq.Workplane:
    hinge_tube = (
        cq.Workplane("YZ")
        .circle(0.0055)
        .extrude(0.052)
        .translate((-0.026, 0.0, 0.0))
    )
    bridge = cq.Workplane("XY").box(0.026, 0.010, 0.020).translate((0.0, 0.004, 0.0))
    front_frame = (
        cq.Workplane("XZ")
        .rect(0.058, 0.076)
        .rect(0.040, 0.050)
        .extrude(0.008)
    )
    rail_saddle = cq.Workplane("XY").box(0.036, 0.012, 0.016).translate((0.0, 0.012, 0.0))
    return hinge_tube.union(bridge).union(front_frame).union(rail_saddle)


def _make_front_block_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(0.072, 0.014, 0.084).translate((0.0, 0.009, 0.0))
    carriage = carriage.cut(
        cq.Workplane("XY").box(0.046, 0.018, 0.054).translate((0.0, 0.009, 0.0))
    )
    for x_pos in (-0.014, 0.014):
        carriage = carriage.cut(
            cq.Workplane("XY")
            .center(x_pos, 0.003)
            .circle(0.0048)
            .extrude(0.140)
            .translate((0.0, 0.0, -0.070))
        )

    front_barrel = cq.Workplane("XZ").circle(0.029).extrude(0.034).translate((0.0, 0.016, 0.0))
    front_rim = cq.Workplane("XZ").circle(0.0315).extrude(0.004).translate((0.0, 0.050, 0.0))
    front_block = carriage.union(front_barrel).union(front_rim)
    front_block = front_block.cut(
        cq.Workplane("XZ").circle(0.021).extrude(0.054).translate((0.0, 0.014, 0.0))
    )
    return front_block


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="perspective_control_lens")

    model.material("body_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("ring_rubber", rgba=(0.16, 0.16, 0.17, 1.0))
    model.material("metal_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("rail_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("knob_metal", rgba=(0.48, 0.49, 0.50, 1.0))
    model.material("front_glass", rgba=(0.22, 0.31, 0.35, 0.42))

    rear_body = model.part("rear_body")
    _add_barrel_section(
        rear_body,
        radius=0.031,
        length=0.006,
        center_y=0.003,
        material="metal_dark",
        name="mount_flange",
    )
    _add_barrel_section(
        rear_body,
        radius=0.024,
        length=0.006,
        center_y=0.009,
        material="metal_dark",
        name="mount_throat",
    )
    _add_barrel_section(
        rear_body,
        radius=0.0315,
        length=0.026,
        center_y=0.025,
        material="body_black",
        name="rear_barrel",
    )
    _add_barrel_section(
        rear_body,
        radius=0.036,
        length=0.020,
        center_y=0.048,
        material="ring_rubber",
        name="focus_ring",
    )
    _add_barrel_section(
        rear_body,
        radius=0.028,
        length=0.010,
        center_y=0.063,
        material="body_black",
        name="front_collar",
    )

    tilt_stage = model.part("tilt_stage")
    tilt_stage.visual(
        mesh_from_cadquery(_make_tilt_frame_shape(), "tilt_stage_frame"),
        material="body_black",
        name="frame",
    )
    for mount_index, x_pos in enumerate((-0.014, 0.014)):
        tilt_stage.visual(
            Box((0.010, 0.010, 0.020)),
            origin=Origin(xyz=(x_pos, 0.016, 0.0)),
            material="metal_dark",
            name=f"rail_mount_{mount_index}",
        )
    for rail_index, x_pos in enumerate((-0.014, 0.014)):
        tilt_stage.visual(
            Cylinder(radius=0.0043, length=0.062),
            origin=Origin(xyz=(x_pos, 0.024, 0.0)),
            material="rail_steel",
            name=f"rail_{rail_index}",
        )
    tilt_stage.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.029, 0.010, 0.0), rpy=CYLINDER_ALONG_X),
        material="metal_dark",
        name="boss_0",
    )
    tilt_stage.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.029, 0.010, 0.0), rpy=CYLINDER_ALONG_X),
        material="metal_dark",
        name="boss_1",
    )

    front_block = model.part("front_block")
    front_block.visual(
        Box((0.072, 0.012, 0.084)),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material="body_black",
        name="carriage",
    )
    front_block.visual(
        Cylinder(radius=0.029, length=0.034),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=CYLINDER_ALONG_Y),
        material="body_black",
        name="front_barrel",
    )
    front_block.visual(
        Cylinder(radius=0.0315, length=0.004),
        origin=Origin(xyz=(0.0, 0.051, 0.0), rpy=CYLINDER_ALONG_Y),
        material="metal_dark",
        name="front_rim",
    )
    front_block.visual(
        Cylinder(radius=0.0212, length=0.0025),
        origin=Origin(xyz=(0.0, 0.050, 0.0), rpy=CYLINDER_ALONG_Y),
        material="front_glass",
        name="front_glass",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.016,
            0.010,
            body_style="cylindrical",
            grip=KnobGrip(style="fluted", count=14, depth=0.0007),
            center=False,
        ),
        "side_lock_knob",
    )

    knob_0 = model.part("knob_0")
    knob_0.visual(
        Cylinder(radius=0.0036, length=0.006),
        origin=Origin(xyz=(-0.003, 0.0, 0.0), rpy=CYLINDER_ALONG_X),
        material="metal_dark",
        name="shaft",
    )
    knob_0.visual(
        knob_mesh,
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material="knob_metal",
        name="lock_knob",
    )

    knob_1 = model.part("knob_1")
    knob_1.visual(
        Cylinder(radius=0.0036, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=CYLINDER_ALONG_X),
        material="metal_dark",
        name="shaft",
    )
    knob_1.visual(
        knob_mesh,
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="knob_metal",
        name="lock_knob",
    )

    model.articulation(
        "rear_body_to_tilt_stage",
        ArticulationType.REVOLUTE,
        parent=rear_body,
        child=tilt_stage,
        origin=Origin(xyz=(0.0, 0.072, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=1.8,
            lower=-0.26,
            upper=0.26,
        ),
    )
    model.articulation(
        "tilt_stage_to_front_block",
        ArticulationType.PRISMATIC,
        parent=tilt_stage,
        child=front_block,
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=-0.012,
            upper=0.012,
        ),
    )
    model.articulation(
        "tilt_stage_to_knob_0",
        ArticulationType.CONTINUOUS,
        parent=tilt_stage,
        child=knob_0,
        origin=Origin(xyz=(-0.035, 0.010, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=5.0),
    )
    model.articulation(
        "tilt_stage_to_knob_1",
        ArticulationType.CONTINUOUS,
        parent=tilt_stage,
        child=knob_1,
        origin=Origin(xyz=(0.035, 0.010, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tilt_stage = object_model.get_part("tilt_stage")
    front_block = object_model.get_part("front_block")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")

    tilt_joint = object_model.get_articulation("rear_body_to_tilt_stage")
    shift_joint = object_model.get_articulation("tilt_stage_to_front_block")
    knob_joint_0 = object_model.get_articulation("tilt_stage_to_knob_0")
    knob_joint_1 = object_model.get_articulation("tilt_stage_to_knob_1")

    ctx.expect_overlap(
        front_block,
        tilt_stage,
        axes="x",
        min_overlap=0.050,
        name="front block stays centered on the tilt stage width",
    )
    ctx.expect_overlap(
        front_block,
        tilt_stage,
        axes="z",
        min_overlap=0.060,
        name="front block has substantial vertical engagement at rest",
    )
    ctx.expect_contact(
        knob_0,
        tilt_stage,
        elem_a="shaft",
        elem_b="boss_0",
        name="first lock knob is mounted on its side boss",
    )
    ctx.expect_contact(
        knob_1,
        tilt_stage,
        elem_a="shaft",
        elem_b="boss_1",
        name="second lock knob is mounted on its side boss",
    )

    rest_front_position = ctx.part_world_position(front_block)
    shift_upper = shift_joint.motion_limits.upper if shift_joint.motion_limits is not None else None
    tilt_upper = tilt_joint.motion_limits.upper if tilt_joint.motion_limits is not None else None

    shifted_front_position = None
    if shift_upper is not None:
        with ctx.pose({shift_joint: shift_upper}):
            ctx.expect_overlap(
                front_block,
                tilt_stage,
                axes="z",
                min_overlap=0.060,
                name="front block remains captured on the shift rail at max rise",
            )
            shifted_front_position = ctx.part_world_position(front_block)

    ctx.check(
        "front block shifts upward on the rail",
        rest_front_position is not None
        and shifted_front_position is not None
        and shifted_front_position[2] > rest_front_position[2] + 0.010,
        details=f"rest={rest_front_position}, shifted={shifted_front_position}",
    )

    tilted_front_position = None
    if tilt_upper is not None:
        with ctx.pose({tilt_joint: tilt_upper}):
            tilted_front_position = ctx.part_world_position(front_block)

    ctx.check(
        "tilt hinge lifts the front block at positive angle",
        rest_front_position is not None
        and tilted_front_position is not None
        and tilted_front_position[2] > rest_front_position[2] + 0.006,
        details=f"rest={rest_front_position}, tilted={tilted_front_position}",
    )

    with ctx.pose({knob_joint_0: 1.1, knob_joint_1: -0.9}):
        ctx.expect_contact(
            knob_0,
            tilt_stage,
            elem_a="shaft",
            elem_b="boss_0",
            name="first lock knob stays seated while rotating",
        )
        ctx.expect_contact(
            knob_1,
            tilt_stage,
            elem_a="shaft",
            elem_b="boss_1",
            name="second lock knob stays seated while rotating",
        )

    return ctx.report()


object_model = build_object_model()
