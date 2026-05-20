from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)


def _tube_x(start_x: float, length: float, outer_radius: float, inner_radius: float):
    return (
        cq.Workplane("YZ", origin=(start_x, 0.0, 0.0))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _rear_mount_shape():
    rear_barrel = _tube_x(-0.062, 0.033, 0.0310, 0.0220)
    collar = _tube_x(-0.029, 0.013, 0.0285, 0.0215)
    stage = (
        cq.Workplane("YZ", origin=(-0.022, 0.0, 0.0))
        .rect(0.084, 0.074)
        .rect(0.056, 0.052)
        .extrude(0.028)
    )
    return rear_barrel.union(collar).union(stage)


def _front_block_shape():
    carriage = (
        cq.Workplane("YZ", origin=(0.0, 0.0, 0.0))
        .rect(0.068, 0.066)
        .rect(0.040, 0.038)
        .extrude(0.009)
    )
    shoulder = _tube_x(0.006, 0.014, 0.036, 0.030)
    barrel_main = _tube_x(0.018, 0.038, 0.040, 0.032)
    front_barrel = _tube_x(0.054, 0.022, 0.041, 0.031)
    front_rim = _tube_x(0.076, 0.010, 0.045, 0.033)
    return carriage.union(shoulder).union(barrel_main).union(front_barrel).union(front_rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shift_camera_lens")

    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    focus_rubber = model.material("focus_rubber", rgba=(0.17, 0.17, 0.18, 1.0))
    mount_steel = model.material("mount_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    rear_mount = model.part("rear_mount")
    rear_mount.visual(
        mesh_from_cadquery(_rear_mount_shape(), "rear_mount"),
        material=anodized_black,
        name="rear_body",
    )
    rear_mount.visual(
        mesh_from_cadquery(_tube_x(-0.066, 0.004, 0.0325, 0.0225), "mount_flange"),
        material=mount_steel,
        name="mount_flange",
    )
    rear_mount.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.004, 0.045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="knob_boss",
    )

    front_block = model.part("front_block")
    front_block.visual(
        mesh_from_cadquery(_front_block_shape(), "front_block"),
        material=anodized_black,
        name="front_body",
    )
    front_block.visual(
        mesh_from_cadquery(_tube_x(0.038, 0.016, 0.043, 0.032), "focus_ring"),
        material=focus_rubber,
        name="focus_ring",
    )

    model.articulation(
        "rear_to_front_shift",
        ArticulationType.PRISMATIC,
        parent=rear_mount,
        child=front_block,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.03,
            lower=-0.011,
            upper=0.011,
        ),
    )

    lock_knob = model.part("lock_knob")
    lock_knob.visual(
        Cylinder(radius=0.0030, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mount_steel,
        name="shaft",
    )
    lock_knob.visual(
        Cylinder(radius=0.0055, length=0.002),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mount_steel,
        name="washer",
    )
    lock_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.022,
                0.012,
                body_style="skirted",
                top_diameter=0.018,
                skirt=KnobSkirt(0.024, 0.003, flare=0.08),
                grip=KnobGrip(style="fluted", count=12, depth=0.0012),
                center=False,
            ),
            "lock_knob",
        ),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob",
    )

    model.articulation(
        "rear_to_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=rear_mount,
        child=lock_knob,
        origin=Origin(xyz=(-0.004, 0.048, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_mount = object_model.get_part("rear_mount")
    front_block = object_model.get_part("front_block")
    lock_knob = object_model.get_part("lock_knob")
    shift = object_model.get_articulation("rear_to_front_shift")
    knob_spin = object_model.get_articulation("rear_to_lock_knob")

    limits = shift.motion_limits
    rest_pos = ctx.part_world_position(front_block)

    ctx.expect_gap(
        front_block,
        rear_mount,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        name="front block sits just ahead of the shift stage",
    )
    ctx.expect_overlap(
        front_block,
        rear_mount,
        axes="yz",
        min_overlap=0.070,
        name="front block remains broadly supported by the stage at rest",
    )

    upper_pos = None
    lower_pos = None
    if limits is not None and limits.upper is not None:
        with ctx.pose({shift: limits.upper}):
            ctx.expect_gap(
                front_block,
                rear_mount,
                axis="x",
                max_gap=0.003,
                max_penetration=0.0,
                name="positive shift keeps the front block seated ahead of the stage",
            )
            ctx.expect_overlap(
                front_block,
                rear_mount,
                axes="yz",
                min_overlap=0.070,
                name="positive shift keeps broad stage overlap",
            )
            upper_pos = ctx.part_world_position(front_block)

    if limits is not None and limits.lower is not None:
        with ctx.pose({shift: limits.lower}):
            ctx.expect_gap(
                front_block,
                rear_mount,
                axis="x",
                max_gap=0.003,
                max_penetration=0.0,
                name="negative shift keeps the front block seated ahead of the stage",
            )
            ctx.expect_overlap(
                front_block,
                rear_mount,
                axes="yz",
                min_overlap=0.070,
                name="negative shift keeps broad stage overlap",
            )
            lower_pos = ctx.part_world_position(front_block)

    ctx.check(
        "front block reaches positive lateral shift",
        rest_pos is not None and upper_pos is not None and upper_pos[1] > rest_pos[1] + 0.009,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )
    ctx.check(
        "front block reaches negative lateral shift",
        rest_pos is not None and lower_pos is not None and lower_pos[1] < rest_pos[1] - 0.009,
        details=f"rest={rest_pos}, lower={lower_pos}",
    )

    ctx.expect_gap(
        lock_knob,
        rear_mount,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        name="lock knob seats against the side boss",
    )
    with ctx.pose({knob_spin: math.pi / 2.0}):
        ctx.expect_gap(
            lock_knob,
            rear_mount,
            axis="y",
            max_gap=0.0,
            max_penetration=0.0,
            name="rotated lock knob stays seated on its shaft boss",
        )

    return ctx.report()


object_model = build_object_model()
