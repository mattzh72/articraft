from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.94, 0.94, 0.92, 1.0))
    lid_white = model.material("lid_white", rgba=(0.93, 0.93, 0.91, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.72, 0.72, 0.73, 1.0))
    knob_grey = model.material("knob_grey", rgba=(0.18, 0.18, 0.19, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.92, 0.45, 0.14, 1.0))

    body_radius = 0.132
    body_height = 0.220
    lid_radius = 0.131
    dial_center_z = 0.094

    body_outer_profile = [
        (0.090, 0.000),
        (0.118, 0.012),
        (body_radius, 0.036),
        (body_radius, 0.192),
        (0.128, 0.210),
        (0.130, body_height),
    ]
    body_inner_profile = [
        (0.000, 0.004),
        (0.084, 0.012),
        (0.118, 0.036),
        (0.118, 0.206),
        (0.115, 0.214),
    ]
    body_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            body_outer_profile,
            body_inner_profile,
            segments=56,
        ),
        "body_shell",
    )

    lid_outer_profile = [
        (lid_radius, 0.000),
        (0.133, 0.003),
        (0.132, 0.014),
        (0.096, 0.028),
        (0.048, 0.036),
        (0.000, 0.040),
    ]
    lid_inner_profile = [
        (0.122, -0.003),
        (0.118, 0.004),
        (0.084, 0.014),
        (0.038, 0.020),
        (0.000, 0.022),
    ]
    lid_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            lid_outer_profile,
            lid_inner_profile,
            segments=56,
        ),
        "lid_shell",
    )

    body = model.part("body")
    body.visual(body_shell, material=shell_white, name="body_shell")
    body.visual(
        Box((0.022, 0.072, 0.098)),
        origin=Origin(xyz=(body_radius + 0.005, 0.0, dial_center_z)),
        material=trim_grey,
        name="control_housing",
    )
    body.visual(
        Box((0.026, 0.102, 0.016)),
        origin=Origin(xyz=(-0.121, 0.0, body_height - 0.006)),
        material=trim_grey,
        name="hinge_mount",
    )
    body.visual(
        Box((0.004, 0.024, 0.010)),
        origin=Origin(xyz=(body_radius + 0.0175, 0.0, dial_center_z + 0.034)),
        material=accent_orange,
        name="mode_mark",
    )

    lid = model.part("lid")
    lid.visual(
        lid_shell,
        origin=Origin(xyz=(0.128, 0.0, -0.006)),
        material=lid_white,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.106),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_grey,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.022, 0.106, 0.008)),
        origin=Origin(xyz=(0.008, 0.0, -0.002)),
        material=trim_grey,
        name="hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.128, -0.032, 0.046)),
        material=trim_grey,
        name="handle_post_0",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.128, 0.032, 0.046)),
        material=trim_grey,
        name="handle_post_1",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.076),
        origin=Origin(xyz=(0.128, 0.0, 0.059), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_grey,
        name="handle_grip",
    )
    lid.visual(
        Box((0.022, 0.074, 0.008)),
        origin=Origin(xyz=(0.128, 0.0, 0.036)),
        material=trim_grey,
        name="handle_base",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_grey,
        name="dial_base",
    )
    dial.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_grey,
        name="dial_grip",
    )
    dial.visual(
        Box((0.004, 0.010, 0.005)),
        origin=Origin(xyz=(0.024, 0.0, 0.014)),
        material=accent_orange,
        name="dial_pointer",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-body_radius, 0.0, body_height + 0.009)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(body_radius + 0.016, 0.0, dial_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("body_to_lid")

    closed_lid_aabb = None
    open_lid_aabb = None

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.012,
            max_penetration=0.004,
            name="lid skirt settles closely around the cooker rim when closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.18,
            name="lid covers the cooker opening when closed",
        )
        ctx.expect_gap(
            dial,
            body,
            axis="x",
            positive_elem="dial_base",
            negative_elem="control_housing",
            max_gap=0.002,
            max_penetration=0.0,
            name="dial seats against the front control housing",
        )
        ctx.expect_overlap(
            dial,
            body,
            axes="yz",
            elem_a="dial_base",
            elem_b="control_housing",
            min_overlap=0.04,
            name="dial stays centered on the front control housing",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)

    limits = lid_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({lid_hinge: limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
