from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_arm_corner_cctv_bracket")

    powder_coat = model.material("warm_white_powder_coat", rgba=(0.82, 0.83, 0.78, 1.0))
    hinge_metal = model.material("dark_zinc_hinge_metal", rgba=(0.18, 0.19, 0.18, 1.0))
    camera_body = model.material("matte_white_camera_body", rgba=(0.88, 0.89, 0.86, 1.0))
    lens_black = model.material("gloss_black_lens", rgba=(0.01, 0.012, 0.014, 1.0))
    screw_dark = model.material("black_screw_heads", rgba=(0.03, 0.03, 0.028, 1.0))

    plate = model.part("corner_plate")
    # Two perpendicular rectangular wall leaves make a rigid corner mount.
    plate.visual(
        Box((0.240, 0.025, 0.500)),
        origin=Origin(xyz=(0.120, -0.0125, 0.250)),
        material=powder_coat,
        name="wall_plate_0",
    )
    plate.visual(
        Box((0.025, 0.240, 0.500)),
        origin=Origin(xyz=(-0.0125, 0.120, 0.250)),
        material=powder_coat,
        name="wall_plate_1",
    )
    plate.visual(
        Box((0.040, 0.040, 0.500)),
        origin=Origin(xyz=(-0.002, -0.002, 0.250)),
        material=powder_coat,
        name="corner_spine",
    )

    # Fixed hinge bosses grow out of each wall face and carry the side-arm pivots.
    plate.visual(
        Box((0.070, 0.026, 0.062)),
        origin=Origin(xyz=(0.140, 0.013, 0.320)),
        material=powder_coat,
        name="arm_0_hinge_mount",
    )
    plate.visual(
        Box((0.026, 0.070, 0.062)),
        origin=Origin(xyz=(0.013, 0.140, 0.180)),
        material=powder_coat,
        name="arm_1_hinge_mount",
    )

    # Low-profile exposed screw heads on both rectangular wall leaves.
    for idx, (x, z) in enumerate(((0.060, 0.090), (0.180, 0.090), (0.060, 0.430), (0.180, 0.430))):
        plate.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, 0.003, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=screw_dark,
            name=f"screw_0_{idx}",
        )
    for idx, (y, z) in enumerate(((0.060, 0.070), (0.180, 0.070), (0.060, 0.410), (0.180, 0.410))):
        plate.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.003, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=screw_dark,
            name=f"screw_1_{idx}",
        )

    arm_specs = (
        # name suffix, hinge position, yaw so local +X points out from its wall face
        ("0", (0.140, 0.048, 0.320), pi / 2.0),
        ("1", (0.048, 0.140, 0.180), 0.0),
    )

    for suffix, hinge_xyz, hinge_yaw in arm_specs:
        arm = model.part(f"arm_{suffix}")
        arm.visual(
            Cylinder(radius=0.022, length=0.080),
            origin=Origin(),
            material=hinge_metal,
            name="hinge_barrel",
        )
        arm.visual(
            Box((0.265, 0.028, 0.024)),
            origin=Origin(xyz=(0.150, 0.0, 0.0)),
            material=powder_coat,
            name="arm_tube",
        )
        arm.visual(
            Cylinder(radius=0.030, length=0.014),
            origin=Origin(xyz=(0.292, 0.0, 0.016)),
            material=hinge_metal,
            name="pan_bearing_pad",
        )
        arm.visual(
            Box((0.050, 0.038, 0.012)),
            origin=Origin(xyz=(0.292, 0.0, 0.009)),
            material=powder_coat,
            name="end_saddle",
        )

        model.articulation(
            f"plate_to_arm_{suffix}",
            ArticulationType.REVOLUTE,
            parent=plate,
            child=arm,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, hinge_yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.75, upper=0.75),
        )

        pan = model.part(f"pan_{suffix}")
        pan.visual(
            Cylinder(radius=0.033, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=hinge_metal,
            name="pan_turntable",
        )
        pan.visual(
            Cylinder(radius=0.018, length=0.058),
            origin=Origin(xyz=(0.0, 0.0, 0.045)),
            material=powder_coat,
            name="pan_neck",
        )
        pan.visual(
            Box((0.080, 0.116, 0.012)),
            origin=Origin(xyz=(0.075, 0.0, 0.043)),
            material=powder_coat,
            name="yoke_base",
        )
        pan.visual(
            Box((0.046, 0.034, 0.014)),
            origin=Origin(xyz=(0.025, 0.0, 0.047)),
            material=powder_coat,
            name="yoke_rear_web",
        )
        pan.visual(
            Box((0.082, 0.012, 0.082)),
            origin=Origin(xyz=(0.080, 0.055, 0.080)),
            material=powder_coat,
            name="yoke_cheek_0",
        )
        pan.visual(
            Box((0.082, 0.012, 0.082)),
            origin=Origin(xyz=(0.080, -0.055, 0.080)),
            material=powder_coat,
            name="yoke_cheek_1",
        )
        pan.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.080, 0.0635, 0.085), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name="tilt_cap_0",
        )
        pan.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.080, -0.0635, 0.085), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name="tilt_cap_1",
        )

        model.articulation(
            f"arm_{suffix}_to_pan",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=pan,
            origin=Origin(xyz=(0.300, 0.0, 0.023)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=1.6, lower=-1.35, upper=1.35),
        )

        camera = model.part(f"camera_{suffix}")
        camera.visual(
            Cylinder(radius=0.030, length=0.112),
            origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=camera_body,
            name="camera_body",
        )
        camera.visual(
            Sphere(radius=0.030),
            origin=Origin(xyz=(0.002, 0.0, 0.0)),
            material=camera_body,
            name="rear_dome",
        )
        camera.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(0.121, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=lens_black,
            name="front_lens",
        )
        camera.visual(
            Box((0.070, 0.074, 0.013)),
            origin=Origin(xyz=(0.098, 0.0, 0.034)),
            material=camera_body,
            name="sun_visor",
        )
        camera.visual(
            Cylinder(radius=0.010, length=0.098),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name="tilt_trunnion",
        )

        model.articulation(
            f"pan_{suffix}_to_camera",
            ArticulationType.REVOLUTE,
            parent=pan,
            child=camera,
            origin=Origin(xyz=(0.080, 0.0, 0.085)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.4, lower=-0.55, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    plate = object_model.get_part("corner_plate")
    arm_0 = object_model.get_part("arm_0")
    arm_1 = object_model.get_part("arm_1")
    pan_0 = object_model.get_part("pan_0")
    camera_0 = object_model.get_part("camera_0")
    pan_1 = object_model.get_part("pan_1")
    camera_1 = object_model.get_part("camera_1")

    revolutes = [joint for joint in object_model.articulations if joint.articulation_type == ArticulationType.REVOLUTE]
    ctx.check("six requested revolute adjustments", len(revolutes) == 6, details=f"found {len(revolutes)}")

    ctx.expect_gap(
        arm_0,
        plate,
        axis="y",
        positive_elem="hinge_barrel",
        negative_elem="arm_0_hinge_mount",
        min_gap=-0.0005,
        max_gap=0.0005,
        name="arm 0 hinge sits at plate face",
    )
    ctx.expect_gap(
        arm_1,
        plate,
        axis="x",
        positive_elem="hinge_barrel",
        negative_elem="arm_1_hinge_mount",
        min_gap=-0.0005,
        max_gap=0.0005,
        name="arm 1 hinge sits at plate face",
    )

    for suffix, pan, camera, cradle_axis in (("0", pan_0, camera_0, "x"), ("1", pan_1, camera_1, "y")):
        ctx.expect_gap(
            pan,
            object_model.get_part(f"arm_{suffix}"),
            axis="z",
            positive_elem="pan_turntable",
            negative_elem="pan_bearing_pad",
            min_gap=-0.0005,
            max_gap=0.001,
            name=f"pan {suffix} turntable is seated above arm tip",
        )
        ctx.expect_within(
            camera,
            pan,
            axes=cradle_axis,
            inner_elem="camera_body",
            outer_elem="yoke_base",
            margin=0.0,
            name=f"camera {suffix} is cradled between yoke cheeks",
        )

    pan_joint = object_model.get_articulation("arm_0_to_pan")
    tilt_joint = object_model.get_articulation("pan_0_to_camera")
    rest_pan_pos = ctx.part_world_position(camera_0)
    rest_lens_aabb = ctx.part_element_world_aabb(camera_0, elem="front_lens")
    with ctx.pose({pan_joint: 0.60, tilt_joint: 0.45}):
        panned_pos = ctx.part_world_position(camera_0)
        tilted_lens_aabb = ctx.part_element_world_aabb(camera_0, elem="front_lens")

    rest_lens_z = None if rest_lens_aabb is None else (rest_lens_aabb[0][2] + rest_lens_aabb[1][2]) / 2.0
    tilted_lens_z = None if tilted_lens_aabb is None else (tilted_lens_aabb[0][2] + tilted_lens_aabb[1][2]) / 2.0
    ctx.check(
        "pan joint slews camera in plan",
        rest_pan_pos is not None
        and panned_pos is not None
        and ((panned_pos[0] - rest_pan_pos[0]) ** 2 + (panned_pos[1] - rest_pan_pos[1]) ** 2) ** 0.5 > 0.030,
        details=f"rest={rest_pan_pos}, panned={panned_pos}",
    )
    ctx.check(
        "tilt joint pitches lens downward",
        rest_lens_z is not None and tilted_lens_z is not None and tilted_lens_z < rest_lens_z - 0.010,
        details=f"rest_lens_z={rest_lens_z}, tilted_lens_z={tilted_lens_z}",
    )

    return ctx.report()


object_model = build_object_model()
