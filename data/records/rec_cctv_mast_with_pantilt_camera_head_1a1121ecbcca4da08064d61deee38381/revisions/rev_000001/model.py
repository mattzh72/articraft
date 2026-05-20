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
    model = ArticulatedObject(name="cluster_pole_cctv_mount")

    galvanized = Material("dark_galvanized_steel", color=(0.23, 0.25, 0.26, 1.0))
    hub_paint = Material("matte_black_hub", color=(0.04, 0.045, 0.05, 1.0))
    bearing_black = Material("black_bearing", color=(0.01, 0.012, 0.014, 1.0))
    camera_white = Material("warm_white_camera", color=(0.88, 0.86, 0.80, 1.0))
    lens_glass = Material("dark_glass", color=(0.0, 0.0, 0.0, 1.0))

    pole_hub = model.part("pole_hub")

    # Ground plate and round pole.
    pole_hub.visual(
        Cylinder(radius=0.24, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=galvanized,
        name="base_plate",
    )
    pole_hub.visual(
        Cylinder(radius=0.065, length=2.74),
        origin=Origin(xyz=(0.0, 0.0, 1.41)),
        material=galvanized,
        name="round_pole",
    )

    # Large radial hub at the pole top.
    pole_hub.visual(
        Cylinder(radius=0.18, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 2.82)),
        material=hub_paint,
        name="radial_hub",
    )
    pole_hub.visual(
        Cylinder(radius=0.205, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 2.9625)),
        material=hub_paint,
        name="top_cap",
    )

    arm_angles = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]
    arm_center_radius = 0.505
    arm_length = 0.82
    arm_end_radius = 0.88
    arm_z = 2.86
    pan_z = 2.90

    for i, angle in enumerate(arm_angles):
        ca = math.cos(angle)
        sa = math.sin(angle)
        pole_hub.visual(
            Cylinder(radius=0.035, length=arm_length),
            origin=Origin(
                xyz=(arm_center_radius * ca, arm_center_radius * sa, arm_z),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=galvanized,
            name=f"arm_{i}",
        )
        pole_hub.visual(
            Cylinder(radius=0.095, length=0.08),
            origin=Origin(xyz=(arm_end_radius * ca, arm_end_radius * sa, arm_z)),
            material=hub_paint,
            name=f"fixed_bearing_{i}",
        )

    for i, angle in enumerate(arm_angles):
        ca = math.cos(angle)
        sa = math.sin(angle)

        pan = model.part(f"pan_{i}")
        pan.visual(
            Cylinder(radius=0.080, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
            material=bearing_black,
            name="rotor_disc",
        )
        pan.visual(
            Box((0.32, 0.050, 0.030)),
            origin=Origin(xyz=(0.16, 0.0, 0.065)),
            material=hub_paint,
            name="radial_neck",
        )
        pan.visual(
            Box((0.055, 0.050, 0.140)),
            origin=Origin(xyz=(0.31, 0.0, 0.0)),
            material=hub_paint,
            name="drop_post",
        )
        pan.visual(
            Box((0.065, 0.240, 0.040)),
            origin=Origin(xyz=(0.31, 0.0, -0.050)),
            material=hub_paint,
            name="yoke_bridge",
        )
        pan.visual(
            Box((0.055, 0.025, 0.180)),
            origin=Origin(xyz=(0.31, 0.1075, -0.160)),
            material=hub_paint,
            name="yoke_cheek_0",
        )
        pan.visual(
            Box((0.055, 0.025, 0.180)),
            origin=Origin(xyz=(0.31, -0.1075, -0.160)),
            material=hub_paint,
            name="yoke_cheek_1",
        )

        model.articulation(
            f"bearing_{i}",
            ArticulationType.CONTINUOUS,
            parent=pole_hub,
            child=pan,
            origin=Origin(
                xyz=(arm_end_radius * ca, arm_end_radius * sa, pan_z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=2.5),
        )

        camera = model.part(f"camera_{i}")
        camera.visual(
            Box((0.260, 0.110, 0.100)),
            origin=Origin(xyz=(0.130, 0.0, 0.0)),
            material=camera_white,
            name="camera_body",
        )
        camera.visual(
            Box((0.315, 0.150, 0.014)),
            origin=Origin(xyz=(0.155, 0.0, 0.057)),
            material=camera_white,
            name="sun_hood",
        )
        camera.visual(
            Cylinder(radius=0.043, length=0.034),
            origin=Origin(xyz=(0.277, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lens_glass,
            name="front_lens",
        )
        camera.visual(
            Cylinder(radius=0.027, length=0.045),
            origin=Origin(xyz=(0.0, 0.0725, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bearing_black,
            name="tilt_pin_0",
        )
        camera.visual(
            Cylinder(radius=0.027, length=0.045),
            origin=Origin(xyz=(0.0, -0.0725, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bearing_black,
            name="tilt_pin_1",
        )
        camera.visual(
            Box((0.055, 0.118, 0.070)),
            origin=Origin(xyz=(0.015, 0.0, 0.0)),
            material=camera_white,
            name="rear_boss",
        )

        model.articulation(
            f"tilt_{i}",
            ArticulationType.REVOLUTE,
            parent=pan,
            child=camera,
            origin=Origin(xyz=(0.31, 0.0, -0.160)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.50, upper=1.00),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pan_joints = [object_model.get_articulation(f"bearing_{i}") for i in range(4)]
    tilt_joints = [object_model.get_articulation(f"tilt_{i}") for i in range(4)]
    cameras = [object_model.get_part(f"camera_{i}") for i in range(4)]
    pans = [object_model.get_part(f"pan_{i}") for i in range(4)]
    pole_hub = object_model.get_part("pole_hub")

    ctx.check(
        "four continuous pan bearings",
        len(pan_joints) == 4
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in pan_joints),
        details=str([j.articulation_type for j in pan_joints]),
    )
    ctx.check(
        "four limited tilt joints",
        len(tilt_joints) == 4
        and all(
            j.articulation_type == ArticulationType.REVOLUTE
            and j.motion_limits is not None
            and j.motion_limits.lower <= -0.45
            and j.motion_limits.upper >= 0.95
            for j in tilt_joints
        ),
        details=str([j.motion_limits for j in tilt_joints]),
    )

    for i in range(4):
        ctx.expect_gap(
            pans[i],
            pole_hub,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="rotor_disc",
            negative_elem=f"fixed_bearing_{i}",
            name=f"pan_{i} seated on fixed bearing",
        )
        ctx.expect_contact(
            cameras[i],
            pans[i],
            elem_a="tilt_pin_0",
            elem_b="yoke_cheek_0",
            contact_tol=0.002,
            name=f"camera_{i} side pin seated in yoke",
        )

    bearing_0 = pan_joints[0]
    camera_0 = cameras[0]
    tilt_0 = tilt_joints[0]
    pan_0 = pans[0]

    rest_pos = ctx.part_world_position(camera_0)
    with ctx.pose({bearing_0: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(camera_0)
    ctx.check(
        "continuous bearing sweeps camera around arm end",
        rest_pos is not None
        and turned_pos is not None
        and turned_pos[1] > rest_pos[1] + 0.25
        and turned_pos[0] < rest_pos[0] - 0.15,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    rest_aabb = ctx.part_world_aabb(camera_0)
    with ctx.pose({tilt_0: 0.75}):
        down_aabb = ctx.part_world_aabb(camera_0)
    ctx.check(
        "tilt joint pitches camera head downward",
        rest_aabb is not None
        and down_aabb is not None
        and down_aabb[0][2] < rest_aabb[0][2] - 0.08,
        details=f"rest={rest_aabb}, tilted={down_aabb}",
    )

    ctx.expect_origin_distance(
        pan_0,
        pole_hub,
        axes="xy",
        min_dist=0.84,
        max_dist=0.92,
        name="pan bearing sits at radial arm end",
    )

    return ctx.report()


object_model = build_object_model()
