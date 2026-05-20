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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_wall_cctv_mount")

    pillar_paint = model.material("pillar_paint", rgba=(0.86, 0.87, 0.88, 1.0))
    bracket_white = model.material("bracket_white", rgba=(0.93, 0.94, 0.95, 1.0))
    camera_white = model.material("camera_white", rgba=(0.95, 0.96, 0.97, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.21, 0.22, 0.24, 1.0))
    lens_black = model.material("lens_black", rgba=(0.07, 0.08, 0.09, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.34, 0.46, 0.52, 0.38))

    corner_mount = model.part("corner_mount")
    corner_mount.visual(
        Box((0.22, 0.22, 0.68)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=pillar_paint,
        name="corner_pillar",
    )
    corner_mount.visual(
        Box((0.004, 0.13, 0.18)),
        origin=Origin(xyz=(0.111, 0.077, 0.44)),
        material=bracket_white,
        name="x_wall_plate",
    )
    corner_mount.visual(
        Box((0.13, 0.004, 0.18)),
        origin=Origin(xyz=(0.077, 0.111, 0.44)),
        material=bracket_white,
        name="y_wall_plate",
    )
    corner_mount.visual(
        Box((0.05, 0.05, 0.18)),
        origin=Origin(xyz=(0.137, 0.137, 0.44)),
        material=bracket_white,
        name="corner_web",
    )
    corner_mount.visual(
        Box((0.13, 0.04, 0.03)),
        origin=Origin(xyz=(0.177, 0.177, 0.44), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=bracket_white,
        name="arm_beam",
    )
    corner_mount.visual(
        Box((0.10, 0.012, 0.07)),
        origin=Origin(xyz=(0.170, 0.170, 0.405), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=bracket_white,
        name="underslung_gusset",
    )
    corner_mount.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(0.225, 0.225, 0.447)),
        material=dark_metal,
        name="pan_base",
    )
    corner_mount.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.72)),
        mass=35.0,
        origin=Origin(xyz=(0.02, 0.02, 0.36)),
    )

    ball_head = model.part("ball_head")
    ball_head.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_metal,
        name="bearing_drum",
    )
    ball_head.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=dark_metal,
        name="bearing_cap",
    )
    ball_head.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=bracket_white,
        name="ball_shell",
    )
    ball_head.visual(
        Cylinder(radius=0.012, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_metal,
        name="ball_stem",
    )
    ball_head.visual(
        Box((0.012, 0.072, 0.020)),
        origin=Origin(xyz=(-0.006, 0.0, 0.060)),
        material=bracket_white,
        name="yoke_bridge",
    )
    ball_head.visual(
        Box((0.040, 0.008, 0.052)),
        origin=Origin(xyz=(0.016, 0.039, 0.060)),
        material=bracket_white,
        name="left_yoke_arm",
    )
    ball_head.visual(
        Box((0.040, 0.008, 0.052)),
        origin=Origin(xyz=(0.016, -0.039, 0.060)),
        material=bracket_white,
        name="right_yoke_arm",
    )
    ball_head.inertial = Inertial.from_geometry(
        Box((0.08, 0.09, 0.10)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    camera_housing = model.part("camera_housing")
    camera_housing.visual(
        Box((0.022, 0.056, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=camera_white,
        name="pivot_saddle",
    )
    camera_housing.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    camera_housing.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.0, -0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    camera_housing.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="rear_collar",
    )
    camera_housing.visual(
        Cylinder(radius=0.033, length=0.122),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="main_body",
    )
    camera_housing.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.148, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="camera_bezel",
    )
    camera_housing.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(0.157, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_window",
    )
    camera_housing.visual(
        Box((0.105, 0.060, 0.010)),
        origin=Origin(xyz=(0.092, 0.0, 0.033)),
        material=camera_white,
        name="sunshield",
    )
    camera_housing.visual(
        Box((0.030, 0.052, 0.006)),
        origin=Origin(xyz=(0.145, 0.0, 0.029)),
        material=camera_white,
        name="sunshield_lip",
    )
    camera_housing.inertial = Inertial.from_geometry(
        Box((0.18, 0.08, 0.09)),
        mass=1.6,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    model.articulation(
        "pan_bearing",
        ArticulationType.CONTINUOUS,
        parent=corner_mount,
        child=ball_head,
        origin=Origin(xyz=(0.225, 0.225, 0.454)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=ball_head,
        child=camera_housing,
        origin=Origin(xyz=(0.030, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-math.radians(70.0),
            upper=math.radians(30.0),
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

    corner_mount = object_model.get_part("corner_mount")
    ball_head = object_model.get_part("ball_head")
    camera_housing = object_model.get_part("camera_housing")
    pan_bearing = object_model.get_articulation("pan_bearing")
    camera_tilt = object_model.get_articulation("camera_tilt")

    ctx.expect_contact(
        ball_head,
        corner_mount,
        elem_a="bearing_drum",
        elem_b="pan_base",
        contact_tol=0.0015,
        name="pan drum seats on the bracket base",
    )
    ctx.expect_overlap(
        ball_head,
        corner_mount,
        axes="xy",
        elem_a="bearing_drum",
        elem_b="pan_base",
        min_overlap=0.050,
        name="pan bearing stays centered over the arm base",
    )
    ctx.expect_contact(
        camera_housing,
        ball_head,
        elem_a="left_trunnion",
        elem_b="left_yoke_arm",
        contact_tol=0.0015,
        name="left trunnion is supported by the yoke",
    )
    ctx.expect_contact(
        camera_housing,
        ball_head,
        elem_a="right_trunnion",
        elem_b="right_yoke_arm",
        contact_tol=0.0015,
        name="right trunnion is supported by the yoke",
    )

    rest_camera_pos = ctx.part_world_position(camera_housing)
    with ctx.pose({pan_bearing: math.pi / 2.0}):
        quarter_turn_camera_pos = ctx.part_world_position(camera_housing)
    ctx.check(
        "continuous pan swings the camera around the vertical bearing",
        rest_camera_pos is not None
        and quarter_turn_camera_pos is not None
        and quarter_turn_camera_pos[0] < rest_camera_pos[0] - 0.010
        and quarter_turn_camera_pos[1] > rest_camera_pos[1] + 0.010,
        details=f"rest={rest_camera_pos}, quarter_turn={quarter_turn_camera_pos}",
    )

    rest_bezel_aabb = ctx.part_element_world_aabb(camera_housing, elem="camera_bezel")
    with ctx.pose({camera_tilt: math.radians(20.0)}):
        raised_bezel_aabb = ctx.part_element_world_aabb(camera_housing, elem="camera_bezel")
    rest_bezel_z = None if rest_bezel_aabb is None else (rest_bezel_aabb[0][2] + rest_bezel_aabb[1][2]) * 0.5
    raised_bezel_z = None if raised_bezel_aabb is None else (raised_bezel_aabb[0][2] + raised_bezel_aabb[1][2]) * 0.5
    ctx.check(
        "positive tilt raises the camera nose",
        rest_bezel_z is not None and raised_bezel_z is not None and raised_bezel_z > rest_bezel_z + 0.020,
        details=f"rest_z={rest_bezel_z}, raised_z={raised_bezel_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
