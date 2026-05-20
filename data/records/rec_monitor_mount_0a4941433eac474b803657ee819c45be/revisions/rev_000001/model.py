from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


POST_RADIUS = 0.0165
POST_HEIGHT = 0.43
COLLAR_Z = 0.295

PRIMARY_LENGTH = 0.335
SECONDARY_LENGTH = 0.235
SECONDARY_OFFSET_Y = 0.055
PAN_TO_TILT_X = 0.088

TILT_SHAFT_RADIUS = 0.0075


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _rounded_box(
    length: float,
    width: float,
    height: float,
    center: tuple[float, float, float],
    fillet: float,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(length, width, height)
    if fillet > 0.0:
        shape = shape.edges("|X").fillet(fillet)
    return shape.translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate(
        (center[0], center[1], center[2] - length / 2.0)
    )


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate(
        (center[0], center[1] - length / 2.0, center[2])
    )


def _ring(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    *,
    center_xy: tuple[float, float] = (0.0, 0.0),
) -> cq.Workplane:
    height = z_max - z_min
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((center_xy[0], center_xy[1], z_min))
    )


def _annulus_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_z(outer_radius, length, center).cut(_cyl_z(inner_radius, length + 0.0006, center))


def _anchor_body_shape() -> cq.Workplane:
    clamp_frame = (
        cq.Workplane("XZ")
        .moveTo(-0.050, 0.012)
        .lineTo(0.046, 0.012)
        .lineTo(0.046, 0.002)
        .lineTo(-0.020, 0.002)
        .lineTo(-0.020, -0.062)
        .lineTo(0.024, -0.062)
        .lineTo(0.024, -0.080)
        .lineTo(-0.050, -0.080)
        .close()
        .extrude(0.072)
        .translate((0.0, -0.036, 0.0))
    )

    gusset = (
        cq.Workplane("XZ")
        .moveTo(-0.010, 0.010)
        .lineTo(0.020, 0.010)
        .lineTo(-0.012, -0.032)
        .close()
        .extrude(0.056)
        .translate((0.0, -0.028, 0.0))
    )

    post_boss = _cyl_z(0.034, 0.020, (0.0, 0.0, 0.021))
    post = _cyl_z(POST_RADIUS, POST_HEIGHT, (0.0, 0.0, 0.012 + POST_HEIGHT / 2.0))

    collar_lower = _cyl_z(0.0185, 0.003, (0.0, 0.0, COLLAR_Z - 0.0035))
    collar_upper = _cyl_z(0.0175, 0.002, (0.0, 0.0, COLLAR_Z - 0.007))
    top_cap = _cyl_z(0.020, 0.004, (0.0, 0.0, 0.014 + POST_HEIGHT))

    screw_housing = _box(0.030, 0.038, 0.022, (-0.004, 0.0, -0.009))
    screw = _cyl_z(0.008, 0.082, (0.0, 0.0, -0.048))
    pressure_pad = _cyl_z(0.017, 0.004, (0.0, 0.0, -0.091))
    t_handle = _box(0.056, 0.008, 0.008, (0.0, 0.0, -0.098))

    shape = clamp_frame.union(gusset)
    for extra in (
        post_boss,
        post,
        collar_lower,
        collar_upper,
        top_cap,
        screw_housing,
        screw,
        pressure_pad,
        t_handle,
    ):
        shape = shape.union(extra)
    return shape


def _anchor_trim_shape() -> cq.Workplane:
    return _cyl_z(0.026, 0.002, (0.0, 0.0, COLLAR_Z + 0.017))


def _primary_arm_main_shape() -> cq.Workplane:
    collar_hub = _annulus_z(0.022, POST_RADIUS + 0.0012, 0.006, (0.0, 0.0, 0.003))
    shoulder = _rounded_box(0.050, 0.046, 0.022, (0.050, 0.0, 0.017), 0.005)
    beam = _rounded_box(0.205, 0.036, 0.016, (0.180, 0.0, 0.032), 0.004)
    elbow_body = _rounded_box(0.040, 0.042, 0.022, (0.300, 0.0, 0.030), 0.005)
    elbow_column = _rounded_box(0.020, 0.026, 0.010, (0.323, 0.0, 0.041), 0.003)
    elbow_flange = _cyl_z(0.0145, 0.004, (PRIMARY_LENGTH, 0.0, 0.044))

    shape = collar_hub.union(shoulder).union(beam).union(elbow_body)
    for extra in (elbow_column, elbow_flange):
        shape = shape.union(extra)
    return shape


def _primary_arm_cover_shape() -> cq.Workplane:
    return _rounded_box(0.150, 0.028, 0.003, (0.180, 0.0, 0.0415), 0.001)


def _primary_arm_trim_shape() -> cq.Workplane:
    elbow_cap = _cyl_z(0.013, 0.002, (PRIMARY_LENGTH, 0.0, 0.047))
    nose_cap = _rounded_box(0.012, 0.030, 0.006, (0.030, 0.0, 0.035), 0.001)
    return elbow_cap.union(nose_cap)


def _secondary_arm_main_shape() -> cq.Workplane:
    base_disc = _cyl_z(0.0145, 0.004, (0.0, 0.0, 0.002))
    shoulder = _rounded_box(0.052, 0.060, 0.040, (0.040, 0.028, 0.028), 0.004)
    beam = _rounded_box(0.118, 0.028, 0.014, (0.136, SECONDARY_OFFSET_Y, 0.058), 0.003)
    distal_body = _rounded_box(
        0.030,
        0.032,
        0.020,
        (0.206, SECONDARY_OFFSET_Y, 0.056),
        0.0035,
    )
    pan_flange = _cyl_z(0.012, 0.004, (SECONDARY_LENGTH, SECONDARY_OFFSET_Y, 0.066))

    shape = base_disc.union(shoulder).union(beam).union(distal_body)
    for extra in (pan_flange,):
        shape = shape.union(extra)
    return shape


def _secondary_arm_cover_shape() -> cq.Workplane:
    return _rounded_box(0.082, 0.022, 0.003, (0.136, SECONDARY_OFFSET_Y, 0.0665), 0.001)


def _secondary_arm_trim_shape() -> cq.Workplane:
    return _rounded_box(0.030, 0.020, 0.002, (0.210, SECONDARY_OFFSET_Y, 0.065), 0.0008)


def _head_pan_bracket_shape() -> cq.Workplane:
    base_disc = _cyl_z(0.0125, 0.004, (0.0, 0.0, 0.002))
    spine = _rounded_box(0.046, 0.018, 0.010, (0.034, 0.0, 0.010), 0.002)
    mast = _rounded_box(0.018, 0.018, 0.054, (PAN_TO_TILT_X - 0.010, 0.0, 0.031), 0.0018)

    ear_left = _box(0.008, 0.008, 0.046, (PAN_TO_TILT_X - 0.005, 0.040, 0.042)).cut(
        _cyl_y(TILT_SHAFT_RADIUS * 0.72, 0.014, (PAN_TO_TILT_X - 0.002, 0.040, 0.042))
    )
    ear_right = _box(0.008, 0.008, 0.046, (PAN_TO_TILT_X - 0.005, -0.040, 0.042)).cut(
        _cyl_y(TILT_SHAFT_RADIUS * 0.72, 0.014, (PAN_TO_TILT_X - 0.002, -0.040, 0.042))
    )

    lower_tie = _rounded_box(0.014, 0.050, 0.008, (PAN_TO_TILT_X - 0.012, 0.0, 0.019), 0.0015)
    shape = base_disc.union(spine).union(mast).union(ear_left).union(ear_right).union(lower_tie)
    return shape


def _head_pan_trim_shape() -> cq.Workplane:
    return _rounded_box(0.020, 0.016, 0.004, (0.022, 0.0, 0.016), 0.001)


def _vesa_frame_shape() -> cq.Workplane:
    pin_radius = 0.003
    axle = _cyl_y(pin_radius, 0.084, (0.0, 0.0, 0.0))
    left_stub = _cyl_y(pin_radius, 0.012, (0.004, 0.040, 0.0))
    right_stub = _cyl_y(pin_radius, 0.012, (0.004, -0.040, 0.0))
    hub = _rounded_box(0.014, 0.024, 0.024, (0.014, 0.0, 0.0), 0.0012)
    stem = _rounded_box(0.058, 0.012, 0.012, (0.046, 0.0, 0.0), 0.0012)
    plate_mount = _rounded_box(0.022, 0.034, 0.034, (0.082, 0.0, 0.0), 0.0014)
    brace_upper = _rounded_box(0.036, 0.010, 0.010, (0.056, 0.0, 0.028), 0.001)
    brace_lower = _rounded_box(0.036, 0.010, 0.010, (0.056, 0.0, -0.028), 0.001)
    frame = (
        cq.Workplane("YZ")
        .rect(0.140, 0.140)
        .rect(0.070, 0.070)
        .extrude(0.005)
        .translate((0.120, 0.0, 0.0))
    )

    shape = (
        axle.union(left_stub)
        .union(right_stub)
        .union(hub)
        .union(stem)
        .union(plate_mount)
        .union(brace_upper)
        .union(brace_lower)
        .union(frame)
    )
    for y_sign in (-0.050, 0.050):
        for z_sign in (-0.050, 0.050):
            shape = shape.cut(cq.Workplane("YZ").center(y_sign, z_sign).circle(0.003).extrude(0.020).translate((-0.010, 0.0, 0.0)))
    return shape


def _vesa_trim_shape() -> cq.Workplane:
    return _rounded_box(0.004, 0.076, 0.076, (0.126, 0.0, 0.0), 0.001)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_post_monitor_arm")

    model.material("powder_graphite", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("anodized_post", rgba=(0.36, 0.38, 0.41, 1.0))

    clamp_post = model.part("clamp_post")
    clamp_post.visual(
        mesh_from_cadquery(_anchor_body_shape(), "clamp_post_body"),
        material="anodized_post",
        name="body",
    )
    clamp_post.visual(
        mesh_from_cadquery(_anchor_trim_shape(), "clamp_post_trim"),
        material="trim_black",
        name="trim",
    )

    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        mesh_from_cadquery(_primary_arm_main_shape(), "primary_arm_main"),
        material="powder_graphite",
        name="main",
    )
    primary_arm.visual(
        mesh_from_cadquery(_primary_arm_cover_shape(), "primary_arm_cover"),
        material="trim_black",
        name="cover",
    )
    primary_arm.visual(
        mesh_from_cadquery(_primary_arm_trim_shape(), "primary_arm_trim"),
        material="steel",
        name="trim",
    )

    secondary_arm = model.part("secondary_arm")
    secondary_arm.visual(
        mesh_from_cadquery(_secondary_arm_main_shape(), "secondary_arm_main"),
        material="powder_graphite",
        name="main",
    )
    secondary_arm.visual(
        mesh_from_cadquery(_secondary_arm_cover_shape(), "secondary_arm_cover"),
        material="trim_black",
        name="cover",
    )
    secondary_arm.visual(
        mesh_from_cadquery(_secondary_arm_trim_shape(), "secondary_arm_trim"),
        material="steel",
        name="trim",
    )

    head_pan = model.part("head_pan")
    head_pan.visual(
        mesh_from_cadquery(_head_pan_bracket_shape(), "head_pan_bracket"),
        material="steel",
        name="bracket",
    )
    head_pan.visual(
        mesh_from_cadquery(_head_pan_trim_shape(), "head_pan_trim"),
        material="trim_black",
        name="trim",
    )

    vesa_head = model.part("vesa_head")
    vesa_head.visual(
        mesh_from_cadquery(_vesa_frame_shape(), "vesa_head_frame"),
        material="powder_graphite",
        name="frame",
    )
    vesa_head.visual(
        mesh_from_cadquery(_vesa_trim_shape(), "vesa_head_trim"),
        material="steel",
        name="hub_trim",
    )

    model.articulation(
        "collar_yaw",
        ArticulationType.REVOLUTE,
        parent=clamp_post,
        child=primary_arm,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z + 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.6, lower=-2.75, upper=2.75),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-2.45, upper=2.45),
    )
    model.articulation(
        "head_pan_yaw",
        ArticulationType.REVOLUTE,
        parent=secondary_arm,
        child=head_pan,
        origin=Origin(xyz=(SECONDARY_LENGTH, SECONDARY_OFFSET_Y, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_pan,
        child=vesa_head,
        origin=Origin(xyz=(PAN_TO_TILT_X - 0.0045, 0.0, 0.042)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.4, lower=-0.35, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp_post = object_model.get_part("clamp_post")
    primary_arm = object_model.get_part("primary_arm")
    secondary_arm = object_model.get_part("secondary_arm")
    head_pan = object_model.get_part("head_pan")
    vesa_head = object_model.get_part("vesa_head")

    collar_yaw = object_model.get_articulation("collar_yaw")
    elbow_yaw = object_model.get_articulation("elbow_yaw")
    head_pan_yaw = object_model.get_articulation("head_pan_yaw")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
        head_pan,
        vesa_head,
        elem_a="bracket",
        elem_b="frame",
        reason="Tilt clevis is modeled as a close nested pivot fit, so the frame hub and fork read as one supported hinge assembly.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(clamp_post, primary_arm, name="collar_joint_is_supported")
    ctx.expect_contact(primary_arm, secondary_arm, name="elbow_joint_is_supported")
    ctx.expect_contact(secondary_arm, head_pan, name="pan_joint_is_supported")
    ctx.expect_contact(head_pan, vesa_head, name="tilt_joint_is_supported")

    ctx.expect_origin_gap(
        primary_arm,
        clamp_post,
        axis="z",
        min_gap=0.26,
        name="working_arm_sits_above_grounded_clamp",
    )
    ctx.expect_overlap(
        head_pan,
        vesa_head,
        axes="yz",
        min_overlap=0.050,
        name="vesa_frame_stays_centered_in_yoke",
    )

    with ctx.pose({collar_yaw: 0.15, elbow_yaw: 2.15, head_pan_yaw: 0.25, head_tilt: 0.1}):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_stays_clear")

    with ctx.pose({collar_yaw: -1.7, elbow_yaw: 1.25, head_pan_yaw: 1.1, head_tilt: -0.25}):
        ctx.fail_if_parts_overlap_in_current_pose(name="side_swing_pose_stays_clear")

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=40,
        name="articulation_paths_clear_through_motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
