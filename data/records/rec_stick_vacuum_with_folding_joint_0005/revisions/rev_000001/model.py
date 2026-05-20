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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stick_vacuum_with_folding_joint", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    satin_charcoal = model.material("satin_charcoal", rgba=(0.28, 0.29, 0.31, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.65, 0.67, 0.70, 1.0))
    smoked_poly = model.material("smoked_poly", rgba=(0.46, 0.50, 0.54, 0.55))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    muted_bronze = model.material("muted_bronze", rgba=(0.53, 0.49, 0.42, 1.0))

    def xy_section(
        width_x: float,
        width_y: float,
        radius: float,
        z: float,
        *,
        x_offset: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x + x_offset, y, z) for x, y in rounded_rect_profile(width_x, width_y, radius)]

    def yz_section(
        width_y: float,
        height_z: float,
        radius: float,
        x: float,
        *,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_center) for y, z in rounded_rect_profile(width_y, height_z, radius)]

    floor_head = model.part("floor_head")

    head_shell_geom = section_loft(
        [
            yz_section(0.236, 0.028, 0.010, -0.052, z_center=0.018),
            yz_section(0.270, 0.043, 0.016, -0.008, z_center=0.022),
            yz_section(0.258, 0.038, 0.014, 0.055, z_center=0.020),
        ]
    )
    head_shell_mesh = mesh_from_geometry(head_shell_geom, ASSETS.mesh_path("floor_head_shell.obj"))
    floor_head.visual(head_shell_mesh, material=matte_graphite, name="head_shell")
    floor_head.visual(
        Box((0.034, 0.044, 0.024)),
        origin=Origin(xyz=(-0.028, 0.0, 0.043)),
        material=satin_charcoal,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.090, 0.020, 0.008)),
        origin=Origin(xyz=(-0.002, -0.076, 0.004)),
        material=soft_black,
        name="left_skid",
    )
    floor_head.visual(
        Box((0.090, 0.020, 0.008)),
        origin=Origin(xyz=(-0.002, 0.076, 0.004)),
        material=soft_black,
        name="right_skid",
    )
    floor_head.visual(
        Box((0.092, 0.182, 0.003)),
        origin=Origin(xyz=(0.004, 0.0, 0.0435)),
        material=muted_bronze,
        name="top_accent",
    )
    floor_head.visual(
        Box((0.016, 0.252, 0.010)),
        origin=Origin(xyz=(0.047, 0.0, 0.012)),
        material=soft_black,
        name="front_bumper",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.110, 0.270, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    head_yoke = model.part("head_yoke")
    head_yoke.visual(
        Cylinder(radius=0.0065, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="pitch_axle",
    )
    head_yoke.visual(
        Box((0.018, 0.006, 0.024)),
        origin=Origin(xyz=(0.002, -0.020, -0.006)),
        material=satin_charcoal,
        name="left_pitch_arm",
    )
    head_yoke.visual(
        Box((0.018, 0.006, 0.024)),
        origin=Origin(xyz=(0.002, 0.020, -0.006)),
        material=satin_charcoal,
        name="right_pitch_arm",
    )
    head_yoke.visual(
        Box((0.020, 0.024, 0.020)),
        origin=Origin(xyz=(-0.002, 0.0, 0.006)),
        material=matte_graphite,
        name="pivot_core",
    )
    head_yoke.visual(
        Cylinder(radius=0.016, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=satin_aluminum,
        name="mast_tube",
    )
    head_yoke.visual(
        Cylinder(radius=0.019, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=satin_charcoal,
        name="yaw_collar",
    )
    head_yoke.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=muted_bronze,
        name="yaw_trim_ring",
    )
    head_yoke.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.160)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_charcoal,
        name="yaw_socket",
    )
    lower_wand.visual(
        Cylinder(radius=0.0175, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.233)),
        material=satin_aluminum,
        name="lower_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.0195, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=satin_charcoal,
        name="fold_sleeve",
    )
    lower_wand.visual(
        Cylinder(radius=0.0065, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.436), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="fold_axle",
    )
    lower_wand.visual(
        Box((0.020, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, -0.019, 0.440)),
        material=satin_charcoal,
        name="left_fold_cheek",
    )
    lower_wand.visual(
        Box((0.020, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, 0.019, 0.440)),
        material=satin_charcoal,
        name="right_fold_cheek",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.460)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Box((0.028, 0.032, 0.032)),
        origin=Origin(xyz=(0.002, 0.0, 0.014)),
        material=satin_charcoal,
        name="fold_knuckle",
    )
    upper_wand.visual(
        Cylinder(radius=0.0165, length=0.312),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=satin_aluminum,
        name="upper_tube",
    )
    upper_wand.visual(
        Box((0.005, 0.028, 0.218)),
        origin=Origin(xyz=(0.011, 0.0, 0.152)),
        material=muted_bronze,
        name="signal_strip",
    )
    upper_wand.visual(
        Cylinder(radius=0.020, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.304)),
        material=satin_charcoal,
        name="body_receiver_collar",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.340)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    body = model.part("body")

    body_shell_geom = section_loft(
        [
            xy_section(0.062, 0.056, 0.018, 0.036, x_offset=-0.010),
            xy_section(0.094, 0.070, 0.026, 0.098, x_offset=-0.004),
            xy_section(0.102, 0.074, 0.024, 0.162, x_offset=0.002),
            xy_section(0.076, 0.060, 0.020, 0.236, x_offset=-0.012),
        ]
    )
    body_shell_mesh = mesh_from_geometry(body_shell_geom, ASSETS.mesh_path("motor_body_shell.obj"))
    rear_spine_geom = section_loft(
        [
            xy_section(0.066, 0.060, 0.018, 0.022, x_offset=-0.018),
            xy_section(0.074, 0.064, 0.020, 0.090, x_offset=-0.024),
            xy_section(0.054, 0.042, 0.016, 0.184, x_offset=-0.044),
            xy_section(0.044, 0.034, 0.014, 0.258, x_offset=-0.055),
        ]
    )
    rear_spine_mesh = mesh_from_geometry(rear_spine_geom, ASSETS.mesh_path("rear_spine_shell.obj"))
    trigger_housing_geom = section_loft(
        [
            xy_section(0.032, 0.030, 0.010, 0.148, x_offset=-0.036),
            xy_section(0.038, 0.034, 0.012, 0.192, x_offset=-0.043),
            xy_section(0.026, 0.026, 0.008, 0.228, x_offset=-0.049),
        ]
    )
    trigger_housing_mesh = mesh_from_geometry(
        trigger_housing_geom,
        ASSETS.mesh_path("trigger_housing.obj"),
    )
    body.visual(body_shell_mesh, material=matte_graphite, name="motor_shell")
    body.visual(
        Cylinder(radius=0.021, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=satin_charcoal,
        name="wand_receiver",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.122),
        origin=Origin(xyz=(0.042, 0.0, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_poly,
        name="dust_cup",
    )
    body.visual(
        rear_spine_mesh,
        material=satin_charcoal,
        name="rear_spine_shell",
    )
    body.visual(
        trigger_housing_mesh,
        material=matte_graphite,
        name="trigger_housing",
    )
    body.visual(
        Box((0.030, 0.021, 0.034)),
        origin=Origin(xyz=(-0.040, 0.0, 0.184)),
        material=soft_black,
        name="trigger_pad",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(-0.008, 0.0, 0.245)),
        material=satin_charcoal,
        name="exhaust_neck",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.016),
        origin=Origin(xyz=(-0.008, 0.0, 0.259)),
        material=muted_bronze,
        name="exhaust_cap",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.130, 0.080, 0.290)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    brush_roll = model.part("brush_roll")
    brush_roll.visual(
        Cylinder(radius=0.016, length=0.220),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="roller_body",
    )
    brush_roll.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, -0.112, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_charcoal,
        name="left_roller_cap",
    )
    brush_roll.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.112, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_charcoal,
        name="right_roller_cap",
    )
    brush_roll.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.220),
        mass=0.18,
        origin=Origin(),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=0.018, length=0.011),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.010, length=0.013),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=muted_bronze,
        name="hub_cap",
    )
    left_wheel.inertial = Inertial.from_geometry(Cylinder(radius=0.018, length=0.011), mass=0.06)

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=0.018, length=0.011),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.010, length=0.013),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=muted_bronze,
        name="hub_cap",
    )
    right_wheel.inertial = Inertial.from_geometry(Cylinder(radius=0.018, length=0.011), mass=0.06)

    head_pitch = model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=floor_head,
        child=head_yoke,
        origin=Origin(xyz=(-0.022, 0.0, 0.043), rpy=(0.0, -0.20, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.30,
        ),
    )

    wand_yaw = model.articulation(
        "wand_yaw",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-0.80,
            upper=0.80,
        ),
    )

    fold_joint = model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_wand,
        origin=Origin(xyz=(0.0, 0.0, 0.436)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.10,
        ),
    )

    model.articulation(
        "upper_wand_to_body",
        ArticulationType.FIXED,
        parent=upper_wand,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
    )

    model.articulation(
        "head_to_brush_roll",
        ArticulationType.CONTINUOUS,
        parent=floor_head,
        child=brush_roll,
        origin=Origin(xyz=(0.006, 0.0, 0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )

    model.articulation(
        "head_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=floor_head,
        child=left_wheel,
        origin=Origin(xyz=(-0.030, -0.103, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=30.0),
    )

    model.articulation(
        "head_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=floor_head,
        child=right_wheel,
        origin=Origin(xyz=(-0.030, 0.103, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    floor_head = object_model.get_part("floor_head")
    head_yoke = object_model.get_part("head_yoke")
    lower_wand = object_model.get_part("lower_wand")
    upper_wand = object_model.get_part("upper_wand")
    body = object_model.get_part("body")
    brush_roll = object_model.get_part("brush_roll")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    head_pitch = object_model.get_articulation("head_pitch")
    wand_yaw = object_model.get_articulation("wand_yaw")
    fold_joint = object_model.get_articulation("fold_joint")
    head_to_brush_roll = object_model.get_articulation("head_to_brush_roll")
    head_to_left_wheel = object_model.get_articulation("head_to_left_wheel")
    head_to_right_wheel = object_model.get_articulation("head_to_right_wheel")

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        floor_head,
        head_yoke,
        reason="Exposed pitch axle passes through the floor head neck block and captured yoke arms.",
    )
    ctx.allow_overlap(
        head_yoke,
        lower_wand,
        reason="The yaw socket is captured over the swivel collar for a realistic neck pivot.",
    )
    ctx.allow_overlap(
        lower_wand,
        upper_wand,
        reason="The folding knuckle rotates on a through-axle seated inside the clevis cheeks.",
    )
    ctx.allow_overlap(
        upper_wand,
        body,
        reason="The upper tube seats into the motor body's receiver sleeve.",
    )
    ctx.allow_overlap(
        floor_head,
        brush_roll,
        reason="The brush roll occupies the suction cavity inside the floor head.",
    )
    ctx.allow_overlap(
        floor_head,
        left_wheel,
        reason="The left wheel axle sits inside the floor head wheel pocket.",
    )
    ctx.allow_overlap(
        floor_head,
        right_wheel,
        reason="The right wheel axle sits inside the floor head wheel pocket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "joint_axes_are_realistic",
        head_pitch.axis == (0.0, 1.0, 0.0)
        and wand_yaw.axis == (0.0, 0.0, 1.0)
        and fold_joint.axis == (0.0, 1.0, 0.0)
        and head_to_brush_roll.axis == (0.0, 1.0, 0.0)
        and head_to_left_wheel.axis == (0.0, 1.0, 0.0)
        and head_to_right_wheel.axis == (0.0, 1.0, 0.0),
        "Expected pitch about Y, wand swivel about Z, fold about Y, and rolling hardware about Y.",
    )

    ctx.expect_contact(floor_head, head_yoke)
    ctx.expect_contact(head_yoke, lower_wand)
    ctx.expect_contact(lower_wand, upper_wand)
    ctx.expect_contact(upper_wand, body)
    ctx.expect_contact(floor_head, brush_roll)
    ctx.expect_contact(floor_head, left_wheel)
    ctx.expect_contact(floor_head, right_wheel)

    ctx.expect_gap(body, floor_head, axis="z", min_gap=0.78)
    ctx.expect_overlap(brush_roll, floor_head, axes="y", min_overlap=0.18)
    ctx.expect_origin_gap(right_wheel, left_wheel, axis="y", min_gap=0.18, max_gap=0.24)

    rest_body = ctx.part_world_position(body)
    if rest_body is None:
        ctx.fail("body_world_position_available", "Unable to resolve rest-pose body world position.")
    else:
        fold_limits = fold_joint.motion_limits
        if fold_limits is None or fold_limits.upper is None:
            ctx.fail("fold_joint_limits_present", "Fold joint is missing an upper motion limit.")
        else:
            with ctx.pose({fold_joint: fold_limits.upper}):
                folded_body = ctx.part_world_position(body)
                ctx.check(
                    "fold_joint_changes_posture",
                    folded_body is not None
                    and folded_body[2] < rest_body[2] - 0.08
                    and folded_body[0] > rest_body[0] + 0.20,
                    "" if folded_body is None else (
                        f"Expected the folded motor body to drop and swing forward. "
                        f"rest={rest_body}, folded={folded_body}"
                    ),
                )
                ctx.expect_contact(lower_wand, upper_wand, name="fold_joint_stays_captured")
                ctx.fail_if_isolated_parts(name="fold_joint_upper_no_floating")
                ctx.fail_if_parts_overlap_in_current_pose(name="fold_joint_upper_no_unintended_overlap")

    pitch_limits = head_pitch.motion_limits
    if pitch_limits is None or pitch_limits.lower is None or pitch_limits.upper is None:
        ctx.fail("head_pitch_limits_present", "Head pitch joint is missing bounded motion limits.")
    else:
        with ctx.pose({head_pitch: pitch_limits.lower}):
            ctx.expect_contact(floor_head, head_yoke, name="head_pitch_lower_contact")
            ctx.fail_if_isolated_parts(name="head_pitch_lower_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name="head_pitch_lower_no_unintended_overlap")
        with ctx.pose({head_pitch: pitch_limits.upper}):
            ctx.expect_contact(floor_head, head_yoke, name="head_pitch_upper_contact")
            ctx.fail_if_isolated_parts(name="head_pitch_upper_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name="head_pitch_upper_no_unintended_overlap")

    yaw_limits = wand_yaw.motion_limits
    if yaw_limits is None or yaw_limits.lower is None or yaw_limits.upper is None:
        ctx.fail("wand_yaw_limits_present", "Wand yaw joint is missing bounded motion limits.")
    else:
        with ctx.pose({wand_yaw: yaw_limits.lower}):
            ctx.expect_contact(head_yoke, lower_wand, name="wand_yaw_lower_contact")
            ctx.fail_if_isolated_parts(name="wand_yaw_lower_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name="wand_yaw_lower_no_unintended_overlap")
        with ctx.pose({wand_yaw: yaw_limits.upper}):
            ctx.expect_contact(head_yoke, lower_wand, name="wand_yaw_upper_contact")
            ctx.fail_if_isolated_parts(name="wand_yaw_upper_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name="wand_yaw_upper_no_unintended_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
