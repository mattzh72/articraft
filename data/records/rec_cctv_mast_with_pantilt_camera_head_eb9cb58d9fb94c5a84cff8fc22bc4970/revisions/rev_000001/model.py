from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="street_light_cctv_mount")

    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.68, 1.0))
    band_steel = model.material("band_steel", rgba=(0.44, 0.47, 0.50, 1.0))
    dark_housing = model.material("dark_housing", rgba=(0.20, 0.22, 0.24, 1.0))
    camera_gray = model.material("camera_gray", rgba=(0.80, 0.82, 0.84, 1.0))
    glass_black = model.material("glass_black", rgba=(0.08, 0.09, 0.10, 1.0))

    pole = model.part("pole")
    pole_profile = [
        (0.0, 0.0),
        (0.165, 0.0),
        (0.155, 1.8),
        (0.135, 4.6),
        (0.118, 6.5),
        (0.102, 7.02),
        (0.098, 7.12),
        (0.0, 7.12),
    ]
    pole.visual(
        mesh_from_geometry(LatheGeometry(pole_profile, segments=64), "tapered_pole"),
        material=galvanized,
        name="pole_shaft",
    )
    pole.visual(
        Cylinder(radius=0.125, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 7.11)),
        material=band_steel,
        name="top_band",
    )
    pole.visual(
        Cylinder(radius=0.100, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 7.24)),
        material=band_steel,
        name="pole_cap",
    )
    pole.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 7.28)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 3.64)),
    )

    arm = model.part("arm")
    brace_geom = tube_from_spline_points(
        [
            (0.05, 0.0, 0.05),
            (0.28, 0.0, 0.02),
            (0.56, 0.0, -0.01),
            (0.78, 0.0, -0.05),
        ],
        radius=0.022,
        samples_per_segment=12,
        radial_segments=16,
    )
    arm.visual(
        Cylinder(radius=0.140, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=band_steel,
        name="arm_turntable",
    )
    arm.visual(
        Cylinder(radius=0.068, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=band_steel,
        name="arm_hub",
    )
    arm.visual(
        Cylinder(radius=0.042, length=0.760),
        origin=Origin(xyz=(0.44, 0.0, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="main_arm_tube",
    )
    arm.visual(
        Box((0.090, 0.150, 0.200)),
        origin=Origin(xyz=(0.86, 0.0, 0.020)),
        material=band_steel,
        name="tip_hanger",
    )
    arm.visual(
        mesh_from_geometry(brace_geom, "arm_brace"),
        material=galvanized,
        name="brace_tube",
    )
    arm.inertial = Inertial.from_geometry(
        Box((1.02, 0.22, 0.34)),
        mass=22.0,
        origin=Origin(xyz=(0.46, 0.0, 0.08)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.070, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=band_steel,
        name="pan_flange",
    )
    pan_head.visual(
        Cylinder(radius=0.078, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=dark_housing,
        name="pan_bearing",
    )
    pan_head.visual(
        Box((0.050, 0.080, 0.110)),
        origin=Origin(xyz=(-0.045, 0.0, -0.145)),
        material=dark_housing,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.040, 0.240, 0.050)),
        origin=Origin(xyz=(-0.050, 0.0, -0.175)),
        material=dark_housing,
        name="yoke_crossbar",
    )
    pan_head.visual(
        Box((0.060, 0.012, 0.220)),
        origin=Origin(xyz=(0.0, 0.116, -0.225)),
        material=dark_housing,
        name="left_yoke_plate",
    )
    pan_head.visual(
        Box((0.060, 0.012, 0.220)),
        origin=Origin(xyz=(0.0, -0.116, -0.225)),
        material=dark_housing,
        name="right_yoke_plate",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.26, 0.42)),
        mass=6.0,
        origin=Origin(xyz=(-0.01, 0.0, -0.19)),
    )

    camera_body = model.part("camera_body")
    camera_body.visual(
        Box((0.280, 0.140, 0.145)),
        origin=Origin(xyz=(0.140, 0.0, 0.000)),
        material=camera_gray,
        name="camera_shell",
    )
    camera_body.visual(
        Box((0.240, 0.150, 0.028)),
        origin=Origin(xyz=(0.165, 0.0, 0.083)),
        material=camera_gray,
        name="sunshield",
    )
    camera_body.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.000, 0.090, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=band_steel,
        name="left_trunnion",
    )
    camera_body.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.000, -0.090, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=band_steel,
        name="right_trunnion",
    )
    camera_body.visual(
        Cylinder(radius=0.045, length=0.080),
        origin=Origin(xyz=(0.300, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_housing,
        name="lens_shroud",
    )
    camera_body.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.355, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_black,
        name="lens_glass",
    )
    camera_body.visual(
        Cylinder(radius=0.020, length=0.100),
        origin=Origin(xyz=(0.020, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_housing,
        name="rear_cable_boot",
    )
    camera_body.inertial = Inertial.from_geometry(
        Box((0.40, 0.18, 0.20)),
        mass=7.0,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    model.articulation(
        "pole_to_arm",
        ArticulationType.REVOLUTE,
        parent=pole,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 7.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.6,
            lower=-1.8,
            upper=1.8,
        ),
    )
    model.articulation(
        "arm_to_pan",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=pan_head,
        origin=Origin(xyz=(0.86, 0.0, -0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_body,
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.15,
            upper=1.20,
        ),
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

    pole = object_model.get_part("pole")
    arm = object_model.get_part("arm")
    pan_head = object_model.get_part("pan_head")
    camera_body = object_model.get_part("camera_body")

    arm_joint = object_model.get_articulation("pole_to_arm")
    pan_joint = object_model.get_articulation("arm_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")

    ctx.expect_contact(
        arm,
        pole,
        elem_a="arm_turntable",
        elem_b="pole_cap",
        name="arm turntable seats on pole cap",
    )
    ctx.expect_overlap(
        arm,
        pole,
        axes="xy",
        elem_a="arm_turntable",
        elem_b="pole_cap",
        min_overlap=0.18,
        name="arm turntable stays centered over pole cap",
    )
    ctx.expect_contact(
        pan_head,
        arm,
        elem_a="pan_flange",
        elem_b="tip_hanger",
        name="pan bearing flange hangs from arm tip bracket",
    )
    ctx.expect_gap(
        pan_head,
        camera_body,
        axis="z",
        positive_elem="pan_bearing",
        negative_elem="sunshield",
        min_gap=0.01,
        name="pan bearing stays above the camera sunshield",
    )
    ctx.expect_contact(
        camera_body,
        pan_head,
        elem_a="left_trunnion",
        elem_b="left_yoke_plate",
        name="camera tilt trunnion contacts left yoke plate",
    )
    ctx.expect_contact(
        camera_body,
        pan_head,
        elem_a="right_trunnion",
        elem_b="right_yoke_plate",
        name="camera tilt trunnion contacts right yoke plate",
    )

    def elem_center(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    arm_tip_rest = None
    arm_tip_swung = None
    lens_rest = None
    lens_panned = None
    lens_tilted = None

    with ctx.pose({arm_joint: 0.0, pan_joint: 0.0, tilt_joint: 0.0}):
        arm_tip_rest = elem_center(arm, "tip_hanger")
        lens_rest = elem_center(camera_body, "lens_shroud")

    with ctx.pose({arm_joint: 1.0, pan_joint: 0.0, tilt_joint: 0.0}):
        arm_tip_swung = elem_center(arm, "tip_hanger")

    with ctx.pose({arm_joint: 0.0, pan_joint: 1.0, tilt_joint: 0.0}):
        lens_panned = elem_center(camera_body, "lens_shroud")

    with ctx.pose({arm_joint: 0.0, pan_joint: 0.0, tilt_joint: 0.9}):
        lens_tilted = elem_center(camera_body, "lens_shroud")

    ctx.check(
        "arm revolute joint swings bracket around pole top",
        arm_tip_rest is not None
        and arm_tip_swung is not None
        and arm_tip_swung[1] > arm_tip_rest[1] + 0.45
        and arm_tip_swung[0] < arm_tip_rest[0] - 0.20,
        details=f"rest={arm_tip_rest}, swung={arm_tip_swung}",
    )
    ctx.check(
        "camera pan bearing rotates housing around vertical axis",
        lens_rest is not None
        and lens_panned is not None
        and lens_panned[1] > lens_rest[1] + 0.18
        and lens_panned[0] < lens_rest[0] - 0.08,
        details=f"rest={lens_rest}, panned={lens_panned}",
    )
    ctx.check(
        "positive tilt drives camera downward from the arm",
        lens_rest is not None
        and lens_tilted is not None
        and lens_tilted[2] < lens_rest[2] - 0.14,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
