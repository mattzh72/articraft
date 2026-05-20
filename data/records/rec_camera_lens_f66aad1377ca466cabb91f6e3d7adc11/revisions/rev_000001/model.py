from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _x_cylinder(radius: float, x0: float, x1: float) -> cq.Workplane:
    return cq.Workplane("YZ").workplane(offset=x0).circle(radius).extrude(x1 - x0)


def _make_mount_stage() -> cq.Workplane:
    shape = _x_cylinder(0.032, 0.000, 0.004)
    shape = shape.union(_x_cylinder(0.028, 0.004, 0.022))
    shape = shape.union(_x_cylinder(0.026, 0.022, 0.025))

    top_strap = cq.Workplane("XY").box(0.016, 0.080, 0.006).translate((0.031, 0.0, 0.033))
    bottom_strap = cq.Workplane("XY").box(0.016, 0.080, 0.006).translate((0.031, 0.0, -0.033))
    positive_side = cq.Workplane("XY").box(0.016, 0.006, 0.060).translate((0.031, 0.038, 0.0))
    negative_side = cq.Workplane("XY").box(0.016, 0.006, 0.060).translate((0.031, -0.038, 0.0))
    knob_boss = cq.Workplane("XY").box(0.010, 0.008, 0.018).translate((0.034, 0.045, 0.0))
    bridge_block = cq.Workplane("XY").box(0.006, 0.074, 0.060).translate((0.024, 0.0, 0.0))
    shape = shape.union(top_strap).union(bottom_strap).union(positive_side).union(negative_side).union(knob_boss).union(bridge_block)

    tick_specs = (
        (-0.013, 0.0050),
        (-0.007, 0.0034),
        (0.000, 0.0068),
        (0.007, 0.0034),
        (0.013, 0.0050),
    )
    for z_pos, x_len in tick_specs:
        tick = cq.Workplane("XY").box(x_len, 0.0018, 0.0012).translate((0.028, 0.0419, z_pos))
        shape = shape.union(tick)

    return shape


def _make_optical_block() -> cq.Workplane:
    shape = _x_cylinder(0.024, -0.006, 0.008)
    shape = shape.union(cq.Workplane("XY").box(0.014, 0.050, 0.032).translate((0.004, 0.0, 0.0)))
    shape = shape.union(_x_cylinder(0.029, 0.008, 0.020))
    shape = shape.union(_x_cylinder(0.036, 0.020, 0.077))
    shape = shape.union(_x_cylinder(0.0385, 0.077, 0.091))
    shape = shape.union(_x_cylinder(0.041, 0.091, 0.095))
    shape = shape.union(cq.Workplane("XY").box(0.010, 0.006, 0.002).translate((0.070, 0.0, 0.037)))

    for x0 in (0.026, 0.030, 0.034, 0.038, 0.042, 0.046, 0.050, 0.054):
        shape = shape.union(_x_cylinder(0.0388, x0, x0 + 0.0015))

    shape = shape.cut(_x_cylinder(0.019, 0.014, 0.095))
    shape = shape.cut(_x_cylinder(0.0245, 0.060, 0.095))
    return shape
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_shift_lens")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))

    mount_stage = model.part("mount_stage")
    mount_stage.visual(
        mesh_from_cadquery(_make_mount_stage(), "mount_stage"),
        material=anodized_black,
        name="mount_stage",
    )

    optical_block = model.part("optical_block")
    optical_block.visual(
        mesh_from_cadquery(_make_optical_block(), "optical_block"),
        material=anodized_black,
        name="optical_block",
    )
    optical_block.visual(
        Box((0.010, 0.010, 0.018)),
        origin=Origin(xyz=(0.002, 0.030, 0.0)),
        material=anodized_black,
        name="hinge_pad_0",
    )
    optical_block.visual(
        Box((0.010, 0.010, 0.018)),
        origin=Origin(xyz=(0.002, -0.030, 0.0)),
        material=anodized_black,
        name="hinge_pad_1",
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        Cylinder(radius=0.0032, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
        material=satin_black,
        name="shaft",
    )
    tilt_knob.visual(
        Cylinder(radius=0.0085, length=0.011),
        origin=Origin(xyz=(0.0, 0.0115, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
        material=satin_black,
        name="knob_body",
    )
    tilt_knob.visual(
        Cylinder(radius=0.0100, length=0.002),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
        material=satin_black,
        name="knob_cap",
    )

    tilt_limits = MotionLimits(
        effort=2.0,
        velocity=1.5,
        lower=-0.14,
        upper=0.14,
    )

    model.articulation(
        "stage_to_optics",
        ArticulationType.REVOLUTE,
        parent=mount_stage,
        child=optical_block,
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=tilt_limits,
    )
    model.articulation(
        "stage_to_knob",
        ArticulationType.CONTINUOUS,
        parent=mount_stage,
        child=tilt_knob,
        origin=Origin(xyz=(0.034, 0.049, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount_stage = object_model.get_part("mount_stage")
    optical_block = object_model.get_part("optical_block")
    tilt_knob = object_model.get_part("tilt_knob")
    tilt_joint = object_model.get_articulation("stage_to_optics")
    knob_joint = object_model.get_articulation("stage_to_knob")

    ctx.allow_overlap(
        mount_stage,
        optical_block,
        elem_a="mount_stage",
        elem_b="hinge_pad_0",
        reason="The right tilt trunnion pad is modeled as a tight retained hinge contact inside the stage yoke.",
    )
    ctx.allow_overlap(
        mount_stage,
        optical_block,
        elem_a="mount_stage",
        elem_b="hinge_pad_1",
        reason="The left tilt trunnion pad is modeled as a tight retained hinge contact inside the stage yoke.",
    )

    with ctx.pose({tilt_joint: 0.0, knob_joint: 0.0}):
        ctx.expect_overlap(
            optical_block,
            mount_stage,
            axes="yz",
            min_overlap=0.045,
            name="optical block stays centered in the tilt stage",
        )
        ctx.expect_gap(
            tilt_knob,
            mount_stage,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            name="locking knob seats against the side boss",
        )
        ctx.expect_overlap(
            tilt_knob,
            mount_stage,
            axes="xz",
            min_overlap=0.010,
            name="locking knob aligns with the tilt boss",
        )

    tilt_limits = tilt_joint.motion_limits
    ctx.check(
        "tilt range matches a specialty lens",
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and tilt_limits.lower <= -0.12
        and 0.12 <= tilt_limits.upper <= 0.20,
        details=f"limits={tilt_limits}",
    )
    ctx.check(
        "locking knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={knob_joint.motion_limits}",
    )

    rest_aabb = ctx.part_world_aabb(optical_block)
    upper_tilt = tilt_limits.upper if tilt_limits is not None and tilt_limits.upper is not None else 0.14
    with ctx.pose({tilt_joint: upper_tilt}):
        tipped_aabb = ctx.part_world_aabb(optical_block)
        ctx.expect_overlap(
            optical_block,
            mount_stage,
            axes="y",
            min_overlap=0.030,
            name="tilted optics remain captured between stage sides",
        )

    ctx.check(
        "positive tilt lifts the front of the optical block",
        rest_aabb is not None
        and tipped_aabb is not None
        and tipped_aabb[1][2] > rest_aabb[1][2] + 0.010,
        details=f"rest={rest_aabb}, tipped={tipped_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
