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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_column_interior_cctv")

    pedestal_finish = model.material("pedestal_finish", rgba=(0.88, 0.89, 0.91, 1.0))
    housing_white = model.material("housing_white", rgba=(0.94, 0.95, 0.97, 1.0))
    bracket_dark = model.material("bracket_dark", rgba=(0.46, 0.48, 0.52, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.64, 0.67, 0.70, 1.0))
    camera_dark = model.material("camera_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    lens_black = model.material("lens_black", rgba=(0.05, 0.06, 0.07, 1.0))
    dome_smoke = model.material("dome_smoke", rgba=(0.16, 0.20, 0.24, 0.34))

    pedestal_support = model.part("pedestal_support")

    base_profile = [
        (0.0, 0.000),
        (0.050, 0.000),
        (0.063, 0.004),
        (0.069, 0.012),
        (0.066, 0.020),
        (0.056, 0.028),
        (0.043, 0.034),
        (0.033, 0.038),
        (0.0, 0.038),
    ]
    base_mesh = mesh_from_geometry(LatheGeometry(base_profile, segments=72), "pedestal_base")
    pedestal_support.visual(base_mesh, material=pedestal_finish, name="pedestal_base")
    pedestal_support.visual(
        Cylinder(radius=0.020, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=pedestal_finish,
        name="column_post",
    )
    pedestal_support.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
        material=bracket_dark,
        name="post_collar",
    )
    bearing_support_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0095, -0.022),
                (0.0140, -0.022),
                (0.0140, -0.004),
                (0.0240, -0.004),
                (0.0240, 0.000),
                (0.0105, 0.000),
                (0.0105, -0.004),
                (0.0095, -0.004),
            ],
            segments=72,
            closed=True,
        ),
        "bearing_support",
    )
    pedestal_support.visual(
        bearing_support_mesh,
        origin=Origin(xyz=(0.042, 0.0, 0.160)),
        material=bracket_dark,
        name="bearing_support",
    )
    pedestal_support.visual(
        Box((0.028, 0.008, 0.018)),
        origin=Origin(xyz=(0.018, -0.014, 0.149)),
        material=bracket_dark,
        name="left_bearing_gusset",
    )
    pedestal_support.visual(
        Box((0.028, 0.008, 0.018)),
        origin=Origin(xyz=(0.018, 0.014, 0.149)),
        material=bracket_dark,
        name="right_bearing_gusset",
    )

    left_bracket_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, -0.010, 0.142),
                (0.002, -0.011, 0.151),
                (0.006, -0.013, 0.160),
                (0.010, -0.016, 0.170),
                (0.012, -0.018, 0.178),
            ],
            radius=0.0045,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "left_bracket_arm",
    )
    pedestal_support.visual(left_bracket_mesh, material=bracket_dark, name="left_bracket_arm")
    right_bracket_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.010, 0.142),
                (0.002, 0.011, 0.151),
                (0.006, 0.013, 0.160),
                (0.010, 0.016, 0.170),
                (0.012, 0.018, 0.178),
            ],
            radius=0.0045,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "right_bracket_arm",
    )
    pedestal_support.visual(right_bracket_mesh, material=bracket_dark, name="right_bracket_arm")
    pedestal_support.visual(
        Box((0.024, 0.050, 0.014)),
        origin=Origin(xyz=(0.012, 0.0, 0.182)),
        material=bracket_dark,
        name="rear_mount_block",
    )
    pedestal_support.visual(
        Cylinder(radius=0.062, length=0.024),
        origin=Origin(xyz=(0.042, 0.0, 0.188)),
        material=housing_white,
        name="housing_cap",
    )

    dome_outer = [
        (0.022, 0.000),
        (0.031, -0.008),
        (0.043, -0.018),
        (0.053, -0.032),
        (0.060, -0.048),
        (0.062, -0.060),
    ]
    dome_inner = [
        (0.018, 0.000),
        (0.027, -0.008),
        (0.039, -0.019),
        (0.048, -0.033),
        (0.055, -0.048),
        (0.057, -0.060),
    ]
    dome_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            dome_outer,
            dome_inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "dome_shell",
    )
    pedestal_support.visual(
        dome_mesh,
        origin=Origin(xyz=(0.042, 0.0, 0.176)),
        material=dome_smoke,
        name="dome_shell",
    )
    pedestal_support.inertial = Inertial.from_geometry(
        Box((0.15, 0.15, 0.20)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    pan_carriage = model.part("pan_carriage")
    pan_carriage.visual(
        Cylinder(radius=0.024, length=0.008),
        material=bearing_metal,
        name="pan_bearing",
    )
    pan_carriage.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=bracket_dark,
        name="pan_spindle",
    )
    pan_carriage.visual(
        Box((0.012, 0.046, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, -0.029)),
        material=bracket_dark,
        name="yoke_bridge",
    )
    pan_carriage.visual(
        Box((0.012, 0.004, 0.034)),
        origin=Origin(xyz=(0.002, -0.021, -0.046)),
        material=bracket_dark,
        name="left_yoke_arm",
    )
    pan_carriage.visual(
        Box((0.012, 0.004, 0.034)),
        origin=Origin(xyz=(0.002, 0.021, -0.046)),
        material=bracket_dark,
        name="right_yoke_arm",
    )
    pan_carriage.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(
            xyz=(0.0, -0.023, -0.036),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bearing_metal,
        name="left_yoke_boss",
    )
    pan_carriage.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.023, -0.036),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bearing_metal,
        name="right_yoke_boss",
    )
    pan_carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.090),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
    )

    camera_module = model.part("camera_module")
    camera_module.visual(
        Box((0.034, 0.030, 0.024)),
        origin=Origin(xyz=(0.018, 0.0, -0.012)),
        material=camera_dark,
        name="camera_body",
    )
    camera_module.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(
            xyz=(-0.006, 0.0, -0.012),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=camera_dark,
        name="rear_housing",
    )
    camera_module.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(
            xyz=(0.038, 0.0, -0.012),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bracket_dark,
        name="lens_bezel",
    )
    camera_module.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.044, 0.0, -0.012)),
        material=lens_black,
        name="lens_glass",
    )
    camera_module.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(
            xyz=(0.0, -0.017, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bearing_metal,
        name="left_trunnion",
    )
    camera_module.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(
            xyz=(0.0, 0.017, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bearing_metal,
        name="right_trunnion",
    )
    camera_module.inertial = Inertial.from_geometry(
        Box((0.055, 0.040, 0.030)),
        mass=0.28,
        origin=Origin(xyz=(0.015, 0.0, -0.010)),
    )

    model.articulation(
        "housing_pan",
        ArticulationType.CONTINUOUS,
        parent=pedestal_support,
        child=pan_carriage,
        origin=Origin(xyz=(0.042, 0.0, 0.164)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_carriage,
        child=camera_module,
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        # Camera body extends along local +X from the tilt axis.
        # +Y makes positive q tilt the lens downward toward -Z.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.2,
            lower=math.radians(-30.0),
            upper=math.radians(65.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("pedestal_support")
    pan_carriage = object_model.get_part("pan_carriage")
    camera_module = object_model.get_part("camera_module")
    pan_joint = object_model.get_articulation("housing_pan")
    tilt_joint = object_model.get_articulation("camera_tilt")

    ctx.check(
        "pan joint is continuous",
        pan_joint.joint_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={pan_joint.joint_type}",
    )
    ctx.check(
        "tilt joint is revolute",
        tilt_joint.joint_type == ArticulationType.REVOLUTE,
        details=f"joint_type={tilt_joint.joint_type}",
    )

    ctx.expect_within(
        pan_carriage,
        support,
        axes="xy",
        outer_elem="dome_shell",
        margin=0.002,
        name="pan carriage stays inside dome footprint",
    )
    ctx.expect_within(
        camera_module,
        support,
        axes="xy",
        outer_elem="dome_shell",
        margin=0.002,
        name="camera module stays inside dome footprint at rest",
    )

    with ctx.pose({tilt_joint: math.radians(45.0)}):
        ctx.expect_within(
            camera_module,
            support,
            axes="xy",
            outer_elem="dome_shell",
            margin=0.002,
            name="camera module stays inside dome footprint when tilted",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_lens = aabb_center(ctx.part_element_world_aabb(camera_module, elem="lens_glass"))
    with ctx.pose({pan_joint: math.pi / 2.0}):
        turned_lens = aabb_center(ctx.part_element_world_aabb(camera_module, elem="lens_glass"))
    ctx.check(
        "pan rotation swings lens around vertical axis",
        rest_lens is not None
        and turned_lens is not None
        and turned_lens[1] > rest_lens[1] + 0.020
        and turned_lens[0] < rest_lens[0] - 0.020,
        details=f"rest={rest_lens}, turned={turned_lens}",
    )

    with ctx.pose({tilt_joint: 0.0}):
        level_lens = aabb_center(ctx.part_element_world_aabb(camera_module, elem="lens_glass"))
    with ctx.pose({tilt_joint: math.radians(45.0)}):
        lowered_lens = aabb_center(ctx.part_element_world_aabb(camera_module, elem="lens_glass"))
    ctx.check(
        "positive tilt lowers the lens",
        level_lens is not None
        and lowered_lens is not None
        and lowered_lens[2] < level_lens[2] - 0.010,
        details=f"level={level_lens}, lowered={lowered_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
