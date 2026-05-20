from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_cctv_mast")

    galvanized = Material("galvanized_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    satin_head = Material("satin_camera_housing", rgba=(0.74, 0.76, 0.76, 1.0))
    black_glass = Material("black_lens_glass", rgba=(0.01, 0.015, 0.02, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.36, 0.36, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=galvanized,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.045, length=1.48),
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        material=galvanized,
        name="round_pole",
    )
    mast.visual(
        Cylinder(radius=0.085, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=galvanized,
        name="welded_flange",
    )
    mast.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 1.515)),
        material=galvanized,
        name="top_cap",
    )
    # Four compact welded webs and four dark bolt heads make the base read as a
    # real steel mast plate rather than a pole simply floating on a slab.
    for i, (x, y, yaw) in enumerate(
        (
            (0.060, 0.0, 0.0),
            (-0.060, 0.0, 0.0),
            (0.0, 0.060, math.pi / 2.0),
            (0.0, -0.060, math.pi / 2.0),
        )
    ):
        mast.visual(
            Box((0.018, 0.080, 0.110)),
            origin=Origin(xyz=(x, y, 0.080), rpy=(0.0, 0.0, yaw)),
            material=galvanized,
            name=f"weld_web_{i}",
        )
    for i, (x, y) in enumerate(
        ((0.125, 0.125), (-0.125, 0.125), (-0.125, -0.125), (0.125, -0.125))
    ):
        mast.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(xyz=(x, y, 0.031)),
            material=dark_steel,
            name=f"anchor_bolt_{i}",
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.075, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=galvanized,
        name="pan_bearing",
    )
    pan_head.visual(
        Cylinder(radius=0.060, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=galvanized,
        name="pan_motor_can",
    )
    pan_head.visual(
        Box((0.205, 0.060, 0.055)),
        origin=Origin(xyz=(0.1025, 0.0, 0.1725)),
        material=galvanized,
        name="support_arm",
    )
    pan_head.visual(
        Box((0.055, 0.220, 0.160)),
        origin=Origin(xyz=(0.165, 0.0, 0.170)),
        material=galvanized,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.220, 0.025, 0.220)),
        origin=Origin(xyz=(0.290, -0.085, 0.170)),
        material=galvanized,
        name="yoke_side_0",
    )
    pan_head.visual(
        Cylinder(radius=0.034, length=0.025),
        origin=Origin(xyz=(0.250, -0.110, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_bearing_0",
    )
    pan_head.visual(
        Box((0.220, 0.025, 0.220)),
        origin=Origin(xyz=(0.290, 0.085, 0.170)),
        material=galvanized,
        name="yoke_side_1",
    )
    pan_head.visual(
        Cylinder(radius=0.034, length=0.025),
        origin=Origin(xyz=(0.250, 0.110, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_bearing_1",
    )
    pan_head.visual(
        Box((0.210, 0.195, 0.035)),
        origin=Origin(xyz=(0.295, 0.0, 0.285)),
        material=galvanized,
        name="top_yoke_tie",
    )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.018, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_shaft",
    )
    camera.visual(
        Cylinder(radius=0.034, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_head,
        name="trunnion_boss",
    )
    camera.visual(
        Box((0.220, 0.110, 0.090)),
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        material=satin_head,
        name="camera_body",
    )
    camera.visual(
        Box((0.290, 0.130, 0.015)),
        origin=Origin(xyz=(0.165, 0.0, 0.0525)),
        material=galvanized,
        name="sun_shield",
    )
    camera.visual(
        Cylinder(radius=0.036, length=0.065),
        origin=Origin(xyz=(0.2825, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.319, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_glass,
        name="front_glass",
    )
    camera.visual(
        Box((0.030, 0.070, 0.040)),
        origin=Origin(xyz=(0.025, 0.0, -0.050)),
        material=dark_steel,
        name="cable_gland",
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.530)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.250, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.60, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.expect_gap(
        pan_head,
        mast,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="pan_bearing",
        negative_elem="top_cap",
        name="pan bearing rests on the mast cap",
    )
    ctx.expect_overlap(
        pan_head,
        mast,
        axes="xy",
        min_overlap=0.050,
        elem_a="pan_bearing",
        elem_b="top_cap",
        name="pan bearing centered on pole",
    )
    ctx.expect_contact(
        camera,
        pan_head,
        elem_a="tilt_shaft",
        elem_b="yoke_side_0",
        contact_tol=0.001,
        name="tilt shaft reaches one yoke cheek",
    )
    ctx.expect_contact(
        camera,
        pan_head,
        elem_a="tilt_shaft",
        elem_b="yoke_side_1",
        contact_tol=0.001,
        name="tilt shaft reaches opposite yoke cheek",
    )

    rest_pos = ctx.part_world_position(camera)
    with ctx.pose({pan_axis: math.pi / 2.0}):
        panned_pos = ctx.part_world_position(camera)
    ctx.check(
        "pan joint swings camera around the vertical pole",
        rest_pos is not None
        and panned_pos is not None
        and panned_pos[0] < rest_pos[0] - 0.20
        and panned_pos[1] > rest_pos[1] + 0.20,
        details=f"rest={rest_pos}, panned={panned_pos}",
    )

    lens_rest = ctx.part_element_world_aabb(camera, elem="front_glass")
    with ctx.pose({tilt_axis: 0.45}):
        lens_up = ctx.part_element_world_aabb(camera, elem="front_glass")
    ctx.check(
        "positive tilt raises the camera nose",
        lens_rest is not None
        and lens_up is not None
        and (lens_up[0][2] + lens_up[1][2]) > (lens_rest[0][2] + lens_rest[1][2]) + 0.050,
        details=f"rest={lens_rest}, tilted={lens_up}",
    )

    return ctx.report()


object_model = build_object_model()
