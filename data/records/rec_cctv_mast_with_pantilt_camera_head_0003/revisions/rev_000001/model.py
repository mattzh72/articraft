from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_upper_housing_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.023, -0.024),
            (0.027, -0.014),
            (0.028, 0.000),
            (0.023, 0.012),
            (0.015, 0.017),
            (0.006, 0.019),
        ],
        [
            (0.017, -0.022),
            (0.020, -0.013),
            (0.021, -0.001),
            (0.017, 0.010),
            (0.010, 0.014),
            (0.003, 0.016),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_dome_bubble_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.004, -0.051),
            (0.010, -0.048),
            (0.021, -0.041),
            (0.031, -0.032),
            (0.038, -0.022),
            (0.042, -0.010),
            (0.043, 0.000),
        ],
        [
            (0.0015, -0.048),
            (0.007, -0.045),
            (0.017, -0.038),
            (0.027, -0.030),
            (0.033, -0.021),
            (0.038, -0.010),
            (0.040, -0.001),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_cctv_arm", assets=ASSETS)

    bracket_gray = model.material("bracket_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    housing_white = model.material("housing_white", rgba=(0.87, 0.89, 0.91, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.36, 0.42, 0.48, 0.34))
    lens_glass = model.material("lens_glass", rgba=(0.18, 0.22, 0.26, 0.78))

    upper_shell_mesh = _save_mesh("cctv_upper_shell.obj", _build_upper_housing_shell())
    bubble_shell_mesh = _save_mesh("cctv_dome_bubble.obj", _build_dome_bubble_shell())

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.010, 0.150, 0.180)),
        origin=Origin(xyz=(0.005, 0.000, 0.090)),
        material=bracket_gray,
        name="wall_plate",
    )
    bracket.visual(
        Box((0.156, 0.040, 0.024)),
        origin=Origin(xyz=(0.088, 0.000, 0.126)),
        material=bracket_gray,
        name="arm_beam",
    )
    bracket.visual(
        Box((0.124, 0.034, 0.012)),
        origin=Origin(xyz=(0.058, 0.000, 0.082), rpy=(0.000, math.pi / 4.0, 0.000)),
        material=bracket_gray,
        name="angle_gusset",
    )
    bracket.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.160, 0.000, 0.131)),
        material=matte_black,
        name="pan_mount_pad",
    )
    for index, (y_pos, z_pos) in enumerate(((-0.042, 0.048), (0.042, 0.048), (-0.042, 0.132), (0.042, 0.132))):
        bracket.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(0.010, y_pos, z_pos), rpy=(0.000, math.pi / 2.0, 0.000)),
            material=matte_black,
            name=f"mount_screw_{index + 1}",
        )
    bracket.inertial = Inertial.from_geometry(
        Box((0.180, 0.160, 0.190)),
        mass=1.6,
        origin=Origin(xyz=(0.090, 0.000, 0.095)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, -0.010)),
        material=housing_white,
        name="pan_collar",
    )
    pan_head.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(xyz=(0.000, 0.000, -0.039)),
        material=housing_white,
        name="drop_stem",
    )
    pan_head.visual(
        Box((0.032, 0.084, 0.012)),
        origin=Origin(xyz=(0.014, 0.000, -0.064)),
        material=housing_white,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.012, 0.012, 0.052)),
        origin=Origin(xyz=(0.014, 0.036, -0.096)),
        material=housing_white,
        name="left_yoke_arm",
    )
    pan_head.visual(
        Box((0.012, 0.012, 0.052)),
        origin=Origin(xyz=(0.014, -0.036, -0.096)),
        material=housing_white,
        name="right_yoke_arm",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.050, 0.090, 0.130)),
        mass=0.5,
        origin=Origin(xyz=(0.010, 0.000, -0.060)),
    )

    camera_body = model.part("camera_body")
    camera_body.visual(
        Cylinder(radius=0.009, length=0.064),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=housing_white,
        name="tilt_axle",
    )
    camera_body.visual(
        Box((0.016, 0.008, 0.024)),
        origin=Origin(xyz=(0.006, 0.026, -0.002)),
        material=housing_white,
        name="left_tilt_cheek",
    )
    camera_body.visual(
        Box((0.016, 0.008, 0.024)),
        origin=Origin(xyz=(0.006, -0.026, -0.002)),
        material=housing_white,
        name="right_tilt_cheek",
    )
    camera_body.visual(
        Box((0.018, 0.028, 0.018)),
        origin=Origin(xyz=(0.014, 0.000, -0.012)),
        material=housing_white,
        name="neck_block",
    )
    camera_body.visual(
        upper_shell_mesh,
        origin=Origin(xyz=(0.040, 0.000, -0.014)),
        material=housing_white,
        name="upper_housing_shell",
    )
    camera_body.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.042, 0.000, -0.039)),
        material=housing_white,
        name="trim_ring",
    )
    camera_body.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(0.042, 0.000, -0.045)),
        material=matte_black,
        name="dome_ring",
    )
    camera_body.visual(
        bubble_shell_mesh,
        origin=Origin(xyz=(0.042, 0.000, -0.045)),
        material=smoked_clear,
        name="dome_bubble",
    )
    camera_body.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.038, 0.000, -0.032)),
        material=matte_black,
        name="sensor_mount",
    )
    camera_body.visual(
        Box((0.026, 0.020, 0.018)),
        origin=Origin(xyz=(0.048, 0.000, -0.048), rpy=(0.000, 0.000, 0.000)),
        material=matte_black,
        name="sensor_block",
    )
    camera_body.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.062, 0.000, -0.049), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=lens_glass,
        name="lens_barrel",
    )
    camera_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.026, 0.000, -0.032)),
    )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=pan_head,
        origin=Origin(xyz=(0.160, 0.000, 0.116)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=-3.05,
            upper=3.05,
        ),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_body,
        origin=Origin(xyz=(0.014, 0.000, -0.090)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.70,
            upper=0.52,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bracket = object_model.get_part("bracket")
    pan_head = object_model.get_part("pan_head")
    camera_body = object_model.get_part("camera_body")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")

    wall_plate = bracket.get_visual("wall_plate")
    pan_mount_pad = bracket.get_visual("pan_mount_pad")
    pan_collar = pan_head.get_visual("pan_collar")
    drop_stem = pan_head.get_visual("drop_stem")
    yoke_bridge = pan_head.get_visual("yoke_bridge")
    left_yoke_arm = pan_head.get_visual("left_yoke_arm")
    right_yoke_arm = pan_head.get_visual("right_yoke_arm")
    left_tilt_cheek = camera_body.get_visual("left_tilt_cheek")
    right_tilt_cheek = camera_body.get_visual("right_tilt_cheek")
    neck_block = camera_body.get_visual("neck_block")
    upper_housing_shell = camera_body.get_visual("upper_housing_shell")
    dome_ring = camera_body.get_visual("dome_ring")
    dome_bubble = camera_body.get_visual("dome_bubble")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "pan_joint_axis_is_vertical",
        tuple(pan_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical pan axis, got {pan_joint.axis!r}",
    )
    ctx.check(
        "tilt_joint_axis_is_horizontal",
        tuple(tilt_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected horizontal tilt axis, got {tilt_joint.axis!r}",
    )
    pan_limits = pan_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits
    ctx.check(
        "pan_joint_has_realistic_sweep",
        pan_limits is not None
        and pan_limits.lower is not None
        and pan_limits.upper is not None
        and pan_limits.lower <= -2.9
        and pan_limits.upper >= 2.9,
        f"Pan limits should allow near-full surveillance sweep, got {pan_limits!r}",
    )
    ctx.check(
        "tilt_joint_has_realistic_range",
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and tilt_limits.lower <= -0.65
        and tilt_limits.upper >= 0.45,
        f"Tilt limits should allow downward watch and slight upward bias, got {tilt_limits!r}",
    )

    ctx.expect_contact(
        pan_head,
        bracket,
        elem_a=pan_collar,
        elem_b=pan_mount_pad,
        name="pan_head_seated_on_mount_pad",
    )
    ctx.expect_overlap(
        pan_head,
        bracket,
        axes="xy",
        min_overlap=0.030,
        elem_a=pan_collar,
        elem_b=pan_mount_pad,
        name="pan_axis_centered_on_mount_pad",
    )
    ctx.expect_gap(
        camera_body,
        bracket,
        axis="x",
        min_gap=0.150,
        positive_elem=dome_ring,
        negative_elem=wall_plate,
        name="camera_hangs_away_from_wall",
    )
    ctx.expect_contact(
        camera_body,
        pan_head,
        elem_a=left_tilt_cheek,
        elem_b=left_yoke_arm,
        name="left_tilt_cheek_seated_in_yoke",
    )
    ctx.expect_contact(
        camera_body,
        pan_head,
        elem_a=right_tilt_cheek,
        elem_b=right_yoke_arm,
        name="right_tilt_cheek_seated_in_yoke",
    )
    ctx.expect_gap(
        camera_body,
        pan_head,
        axis="x",
        min_gap=0.006,
        max_gap=0.015,
        positive_elem=neck_block,
        negative_elem=drop_stem,
        name="neck_block_sits_ahead_of_drop_stem",
    )
    ctx.expect_gap(
        pan_head,
        camera_body,
        axis="z",
        min_gap=0.010,
        positive_elem=yoke_bridge,
        negative_elem=upper_housing_shell,
        name="upper_housing_sits_below_yoke_bridge",
    )
    ctx.expect_gap(
        pan_head,
        camera_body,
        axis="z",
        min_gap=0.020,
        positive_elem=pan_collar,
        negative_elem=dome_bubble,
        name="dome_sits_below_pan_axis",
    )
    with ctx.pose({pan_joint: 1.6}):
        ctx.expect_contact(
            pan_head,
            bracket,
            elem_a=pan_collar,
            elem_b=pan_mount_pad,
            name="pan_mount_stays_seated_when_swiveled",
        )
        ctx.expect_gap(
            camera_body,
            bracket,
            axis="x",
            min_gap=0.090,
            positive_elem=dome_ring,
            negative_elem=wall_plate,
            name="swiveled_camera_stays_off_wall",
        )
    with ctx.pose({tilt_joint: -0.60}):
        ctx.expect_contact(
            camera_body,
            pan_head,
            elem_a=left_tilt_cheek,
            elem_b=left_yoke_arm,
            name="left_tilt_contact_at_down_tilt",
        )
        ctx.expect_contact(
            camera_body,
            pan_head,
            elem_a=right_tilt_cheek,
            elem_b=right_yoke_arm,
            name="right_tilt_contact_at_down_tilt",
        )
        ctx.expect_gap(
            camera_body,
            bracket,
            axis="x",
            min_gap=0.160,
            positive_elem=dome_ring,
            negative_elem=wall_plate,
            name="down_tilt_keeps_dome_ahead_of_wall",
        )
        ctx.expect_gap(
            pan_head,
            camera_body,
            axis="z",
            min_gap=0.050,
            positive_elem=pan_collar,
            negative_elem=dome_bubble,
            name="down_tilt_keeps_dome_below_pan_axis",
        )
    with ctx.pose({tilt_joint: 0.42}):
        ctx.expect_contact(
            camera_body,
            pan_head,
            elem_a=left_tilt_cheek,
            elem_b=left_yoke_arm,
            name="left_tilt_contact_at_up_tilt",
        )
        ctx.expect_contact(
            camera_body,
            pan_head,
            elem_a=right_tilt_cheek,
            elem_b=right_yoke_arm,
            name="right_tilt_contact_at_up_tilt",
        )
        ctx.expect_gap(
            camera_body,
            bracket,
            axis="x",
            min_gap=0.130,
            positive_elem=dome_ring,
            negative_elem=wall_plate,
            name="up_tilt_keeps_camera_off_wall",
        )

    pan_limits = pan_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits
    if pan_limits is not None and pan_limits.lower is not None and pan_limits.upper is not None:
        with ctx.pose({pan_joint: pan_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pan_joint_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="pan_joint_lower_no_floating")
            ctx.expect_gap(
                camera_body,
                bracket,
                axis="x",
                min_gap=0.0,
                positive_elem=dome_ring,
                negative_elem=wall_plate,
                name="pan_joint_lower_keeps_dome_off_wall",
            )
        with ctx.pose({pan_joint: pan_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pan_joint_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="pan_joint_upper_no_floating")
            ctx.expect_gap(
                camera_body,
                bracket,
                axis="x",
                min_gap=0.0,
                positive_elem=dome_ring,
                negative_elem=wall_plate,
                name="pan_joint_upper_keeps_dome_off_wall",
            )
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt_joint: tilt_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_joint_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="tilt_joint_lower_no_floating")
            ctx.expect_gap(
                pan_head,
                camera_body,
                axis="z",
                min_gap=0.040,
                positive_elem=pan_collar,
                negative_elem=dome_bubble,
                name="tilt_joint_lower_keeps_dome_below_pan_axis",
            )
        with ctx.pose({tilt_joint: tilt_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_joint_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="tilt_joint_upper_no_floating")
            ctx.expect_gap(
                camera_body,
                bracket,
                axis="x",
                min_gap=0.120,
                positive_elem=dome_ring,
                negative_elem=wall_plate,
                name="tilt_joint_upper_keeps_dome_off_wall",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
