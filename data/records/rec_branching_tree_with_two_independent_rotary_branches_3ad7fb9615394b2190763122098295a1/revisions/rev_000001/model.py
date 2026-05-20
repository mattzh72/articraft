from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


POST_RADIUS = 0.028
POST_HEIGHT = 0.82
BASE_THICKNESS = 0.018
PEDESTAL_HEIGHT = 0.03

LOWER_COLLAR_Z = 0.31
UPPER_COLLAR_Z = 0.57
COLLAR_HEIGHT = 0.046
COLLAR_OUTER_RADIUS = 0.055
PIVOT_OFFSET = 0.108
ARM_BOSS_WIDTH = 0.028
CHEEK_THICKNESS = 0.010


def _make_frame() -> cq.Workplane:
    base_span = 0.46
    foot_width = 0.082
    pedestal_radius = 0.068

    base_x = cq.Workplane("XY").box(base_span, foot_width, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )
    base_y = cq.Workplane("XY").box(foot_width, base_span, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )
    pedestal = cq.Workplane("XY").cylinder(PEDESTAL_HEIGHT, pedestal_radius).translate(
        (0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)
    )
    post = cq.Workplane("XY").cylinder(POST_HEIGHT, POST_RADIUS).translate(
        (0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT + POST_HEIGHT / 2.0)
    )

    rib_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.034, 0.0),
                (0.034, 0.0),
                (0.010, 0.16),
                (-0.010, 0.16),
            ]
        )
        .close()
        .extrude(0.008, both=True)
        .translate((0.0, 0.0, BASE_THICKNESS + 0.004))
    )
    ribs = (
        rib_profile.union(rib_profile.rotate((0, 0, 0), (0, 0, 1), 90))
        .union(rib_profile.rotate((0, 0, 0), (0, 0, 1), 45))
        .union(rib_profile.rotate((0, 0, 0), (0, 0, 1), -45))
    )

    return base_x.union(base_y).union(pedestal).union(post).union(ribs)


def _make_hub(orientation_deg: float) -> cq.Workplane:
    support_depth = 0.018
    support_height = 0.042
    pivot_radius = 0.023
    plate_thickness = 0.010
    shell_clearance = 0.0012
    saddle_start = POST_RADIUS + 0.002
    collar_span = COLLAR_OUTER_RADIUS - POST_RADIUS + 0.010
    spine_start = POST_RADIUS + 0.010
    spine_length = PIVOT_OFFSET - spine_start

    collar_shell = cq.Workplane("XY").cylinder(COLLAR_HEIGHT, COLLAR_OUTER_RADIUS).cut(
        cq.Workplane("XY").cylinder(COLLAR_HEIGHT + 0.004, POST_RADIUS + shell_clearance)
    )
    collar_shell = collar_shell.intersect(
        cq.Workplane("XY")
        .box(collar_span, 2.0 * COLLAR_OUTER_RADIUS, COLLAR_HEIGHT + 0.004)
        .translate((saddle_start + collar_span / 2.0, 0.0, 0.0))
    )

    spine = cq.Workplane("XY").box(spine_length, support_depth, support_height).translate(
        (spine_start + spine_length / 2.0, 0.0, 0.0)
    )

    cheek = (
        cq.Workplane("XZ")
        .circle(pivot_radius)
        .extrude(plate_thickness)
        .translate((PIVOT_OFFSET, -plate_thickness, 0.0))
    )
    brace = (
        cq.Workplane("XZ")
        .polyline(
            [
                (spine_start, -support_height / 2.0),
                (spine_start, support_height / 2.0),
                (PIVOT_OFFSET - 0.008, 0.015),
                (PIVOT_OFFSET - 0.008, -0.015),
            ]
        )
        .close()
        .extrude(plate_thickness)
        .translate((0.0, -plate_thickness, 0.0))
    )

    hub = collar_shell.union(spine).union(cheek).union(brace)
    return hub.rotate((0, 0, 0), (0, 0, 1), orientation_deg)


def _make_pad_arm() -> cq.Workplane:
    beam_thickness = 0.012
    beam_height = 0.022
    arm_length = 0.30

    beam = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -0.014),
                (0.050, -beam_height / 2.0),
                (0.22, -beam_height / 2.0),
                (arm_length, -0.010),
                (arm_length, 0.010),
                (0.22, beam_height / 2.0),
                (0.050, beam_height / 2.0),
                (0.0, 0.014),
            ]
        )
        .close()
        .extrude(beam_thickness)
    )
    root_lug = cq.Workplane("XZ").center(0.016, 0.0).circle(0.016).extrude(beam_thickness)
    neck = cq.Workplane("XY").box(0.028, beam_thickness, 0.022).translate(
        (arm_length + 0.006, beam_thickness / 2.0, 0.0)
    )
    pad = cq.Workplane("XY").box(0.050, beam_thickness, 0.044).translate(
        (arm_length + 0.020, beam_thickness / 2.0, 0.0)
    )

    return beam.union(root_lug).union(neck).union(pad)


def _make_fork_arm() -> cq.Workplane:
    beam_thickness = 0.012
    beam_height = 0.020
    arm_length = 0.27
    fork_length = 0.070
    fork_outer_height = 0.048
    fork_slot_height = 0.022

    beam = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.0, -0.014),
                (0.050, -beam_height / 2.0),
                (0.20, -beam_height / 2.0),
                (arm_length, -0.010),
                (arm_length, 0.010),
                (0.20, beam_height / 2.0),
                (0.050, beam_height / 2.0),
                (0.0, 0.014),
            ]
        )
        .close()
        .extrude(beam_thickness)
        .translate((-beam_thickness, 0.0, 0.0))
    )
    root_lug = (
        cq.Workplane("YZ")
        .center(0.016, 0.0)
        .circle(0.016)
        .extrude(beam_thickness)
        .translate((-beam_thickness, 0.0, 0.0))
    )
    fork_block = cq.Workplane("XZ").box(beam_thickness, fork_length, fork_outer_height).translate(
        (-beam_thickness / 2.0, arm_length + 0.025, 0.0)
    )
    fork_slot = cq.Workplane("XZ").box(
        beam_thickness + 0.004,
        fork_length - 0.014,
        fork_slot_height,
    ).translate((-beam_thickness / 2.0, arm_length + 0.031, 0.0))

    return beam.union(root_lug).union(fork_block).cut(fork_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_motion_study")

    frame_mat = model.material("frame_graphite", rgba=(0.22, 0.24, 0.26, 1.0))
    hub_mat = model.material("hub_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    lower_arm_mat = model.material("arm_blue", rgba=(0.28, 0.38, 0.56, 1.0))
    upper_arm_mat = model.material("arm_warm_gray", rgba=(0.60, 0.57, 0.52, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.46, 0.082, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=frame_mat,
        name="base_x",
    )
    frame.visual(
        Box((0.082, 0.46, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=frame_mat,
        name="base_y",
    )
    frame.visual(
        Cylinder(radius=0.068, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)),
        material=frame_mat,
        name="pedestal",
    )
    frame.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT + POST_HEIGHT / 2.0)
        ),
        material=frame_mat,
        name="central_post",
    )
    frame.visual(
        Box((0.044, 0.020, 0.034)),
        origin=Origin(xyz=(-0.050, 0.0, LOWER_COLLAR_Z)),
        material=frame_mat,
        name="lower_mount_tab",
    )
    frame.visual(
        Box((0.020, 0.044, 0.034)),
        origin=Origin(xyz=(0.0, -0.050, UPPER_COLLAR_Z)),
        material=frame_mat,
        name="upper_mount_tab",
    )

    lower_hub = model.part("lower_hub")
    lower_hub.visual(
        Box((0.012, 0.072, COLLAR_HEIGHT)),
        origin=Origin(xyz=(-0.078, 0.0, 0.0)),
        material=hub_mat,
        name="rear_plate",
    )
    lower_hub.visual(
        Box((0.128, 0.010, COLLAR_HEIGHT)),
        origin=Origin(xyz=(-0.014, 0.034, 0.0)),
        material=hub_mat,
        name="upper_collar_bar",
    )
    lower_hub.visual(
        Box((0.128, 0.010, COLLAR_HEIGHT)),
        origin=Origin(xyz=(-0.014, -0.034, 0.0)),
        material=hub_mat,
        name="lower_collar_bar",
    )
    lower_hub.visual(
        Box((0.018, 0.058, COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=hub_mat,
        name="front_collar_bar",
    )
    lower_hub.visual(
        Box((0.022, 0.014, 0.032)),
        origin=Origin(xyz=(0.073, 0.0, 0.0)),
        material=hub_mat,
        name="pivot_spine",
    )
    lower_hub.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(PIVOT_OFFSET, -0.005, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="pivot_cheek",
    )
    lower_hub.visual(
        Box((0.002, 0.004, 0.026)),
        origin=Origin(xyz=(0.085, -0.008, 0.0)),
        material=hub_mat,
        name="cheek_bridge",
    )

    upper_hub = model.part("upper_hub")
    upper_hub.visual(
        Box((0.072, 0.012, COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.078, 0.0)),
        material=hub_mat,
        name="rear_plate",
    )
    upper_hub.visual(
        Box((0.010, 0.128, COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.034, -0.014, 0.0)),
        material=hub_mat,
        name="right_collar_bar",
    )
    upper_hub.visual(
        Box((0.010, 0.128, COLLAR_HEIGHT)),
        origin=Origin(xyz=(-0.034, -0.014, 0.0)),
        material=hub_mat,
        name="left_collar_bar",
    )
    upper_hub.visual(
        Box((0.058, 0.018, COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material=hub_mat,
        name="front_collar_bar",
    )
    upper_hub.visual(
        Box((0.014, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, 0.073, 0.0)),
        material=hub_mat,
        name="pivot_spine",
    )
    upper_hub.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.005, PIVOT_OFFSET, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_mat,
        name="pivot_cheek",
    )
    upper_hub.visual(
        Box((0.004, 0.002, 0.026)),
        origin=Origin(xyz=(-0.008, 0.085, 0.0)),
        material=hub_mat,
        name="cheek_bridge",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=lower_arm_mat,
        name="root_lug",
    )
    lower_arm.visual(
        Box((0.236, 0.012, 0.018)),
        origin=Origin(xyz=(0.134, 0.006, 0.0)),
        material=lower_arm_mat,
        name="main_beam",
    )
    lower_arm.visual(
        Box((0.036, 0.012, 0.014)),
        origin=Origin(xyz=(0.270, 0.006, 0.0)),
        material=lower_arm_mat,
        name="tip_neck",
    )
    lower_arm.visual(
        Box((0.050, 0.012, 0.042)),
        origin=Origin(xyz=(0.313, 0.006, 0.0)),
        material=lower_arm_mat,
        name="pad_tip",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=upper_arm_mat,
        name="root_lug",
    )
    upper_arm.visual(
        Box((0.012, 0.216, 0.018)),
        origin=Origin(xyz=(0.006, 0.124, 0.0)),
        material=upper_arm_mat,
        name="main_beam",
    )
    upper_arm.visual(
        Box((0.012, 0.028, 0.046)),
        origin=Origin(xyz=(0.006, 0.246, 0.0)),
        material=upper_arm_mat,
        name="fork_bridge",
    )
    upper_arm.visual(
        Box((0.012, 0.080, 0.012)),
        origin=Origin(xyz=(0.006, 0.300, 0.017)),
        material=upper_arm_mat,
        name="upper_tine",
    )
    upper_arm.visual(
        Box((0.012, 0.080, 0.012)),
        origin=Origin(xyz=(0.006, 0.300, -0.017)),
        material=upper_arm_mat,
        name="lower_tine",
    )

    model.articulation(
        "frame_to_lower_hub",
        ArticulationType.FIXED,
        parent=frame,
        child=lower_hub,
        origin=Origin(xyz=(0.0, 0.0, LOWER_COLLAR_Z)),
    )
    model.articulation(
        "frame_to_upper_hub",
        ArticulationType.FIXED,
        parent=frame,
        child=upper_hub,
        origin=Origin(xyz=(0.0, 0.0, UPPER_COLLAR_Z)),
    )
    model.articulation(
        "lower_hub_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=lower_hub,
        child=lower_arm,
        origin=Origin(xyz=(PIVOT_OFFSET, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=-1.05,
            upper=1.15,
        ),
    )
    model.articulation(
        "upper_hub_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=upper_hub,
        child=upper_arm,
        origin=Origin(xyz=(0.0, PIVOT_OFFSET, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.95,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lower_hub = object_model.get_part("lower_hub")
    upper_hub = object_model.get_part("upper_hub")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_joint = object_model.get_articulation("lower_hub_to_lower_arm")
    upper_joint = object_model.get_articulation("upper_hub_to_upper_arm")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(lower_hub, frame, name="lower_hub_mounts_to_frame")
    ctx.expect_contact(upper_hub, frame, name="upper_hub_mounts_to_frame")
    ctx.expect_contact(lower_arm, lower_hub, name="lower_arm_supported_by_hub")
    ctx.expect_contact(upper_arm, upper_hub, name="upper_arm_supported_by_hub")

    with ctx.pose({lower_joint: 0.0}):
        lower_closed = ctx.part_world_aabb(lower_arm)
    with ctx.pose({lower_joint: 0.95}):
        lower_open = ctx.part_world_aabb(lower_arm)
    ctx.check(
        "lower_arm_positive_motion_lifts_tip",
        lower_closed is not None
        and lower_open is not None
        and (lower_open[0][2] + lower_open[1][2]) / 2.0
        > (lower_closed[0][2] + lower_closed[1][2]) / 2.0 + 0.08,
        details=f"closed={lower_closed}, open={lower_open}",
    )

    with ctx.pose({upper_joint: 0.0}):
        upper_closed = ctx.part_world_aabb(upper_arm)
    with ctx.pose({upper_joint: 0.90}):
        upper_open = ctx.part_world_aabb(upper_arm)
    ctx.check(
        "upper_arm_positive_motion_lifts_tip",
        upper_closed is not None
        and upper_open is not None
        and (upper_open[0][2] + upper_open[1][2]) / 2.0
        > (upper_closed[0][2] + upper_closed[1][2]) / 2.0 + 0.04,
        details=f"closed={upper_closed}, open={upper_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
