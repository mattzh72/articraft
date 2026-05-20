from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_LEN = 0.186
FRAME_W = 0.100
FRAME_H = 0.130
PLATE_T = 0.016
RAIL_T = 0.014

FRAME_SIDE_X = FRAME_LEN / 2.0 - PLATE_T / 2.0
FRAME_INNER_FACE_X = FRAME_LEN / 2.0 - PLATE_T

CARTRIDGE_T = 0.022
CARTRIDGE_W = 0.064
CARTRIDGE_CENTER_X = FRAME_INNER_FACE_X - CARTRIDGE_T / 2.0
CARTRIDGE_INNER_FACE_X = CARTRIDGE_CENTER_X - CARTRIDGE_T / 2.0

JOURNAL_R = 0.0085
HUB_R = 0.013
SHOULDER_R = 0.0135
SHOULDER_T = 0.004
JOURNAL_LEN = CARTRIDGE_T
MAIN_SHAFT_LEN = 2.0 * CARTRIDGE_INNER_FACE_X
HUB_LEN = 0.044
MOUNT_PAD_L = 0.028
MOUNT_PAD_W = 0.022
MOUNT_PAD_T = 0.004
MOUNT_PAD_CENTER_Z = HUB_R
MOUNT_PAD_TOP_Z = MOUNT_PAD_CENTER_Z + MOUNT_PAD_T / 2.0

CLAMP_L = MOUNT_PAD_L
CLAMP_W = MOUNT_PAD_W
CLAMP_T = 0.005
WEB_L = 0.010
WEB_W = 0.022
WEB_H = 0.020
TOP_PLATE_L = 0.026
TOP_PLATE_W = 0.030
TOP_PLATE_T = 0.004
BRACKET_ORIGIN_Z = MOUNT_PAD_TOP_Z + CLAMP_T / 2.0


def _bearing_cartridge_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(CARTRIDGE_T, CARTRIDGE_W, CARTRIDGE_W)
    bore = cq.Workplane("YZ").circle(0.011).extrude(CARTRIDGE_T + 0.010, both=True)

    pocket_points = [
        (-0.018, -0.018),
        (-0.018, 0.018),
        (0.018, -0.018),
        (0.018, 0.018),
    ]
    front_pockets = (
        cq.Workplane("YZ")
        .pushPoints(pocket_points)
        .circle(0.0045)
        .extrude(0.0035)
        .translate((CARTRIDGE_T / 2.0 - 0.00175, 0.0, 0.0))
    )
    back_pockets = (
        cq.Workplane("YZ")
        .pushPoints(pocket_points)
        .circle(0.0045)
        .extrude(0.0035)
        .translate((-CARTRIDGE_T / 2.0 + 0.00175, 0.0, 0.0))
    )

    return body.cut(bore).cut(front_pockets).cut(back_pockets)


def _clamp_block_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(CLAMP_L, CLAMP_W, CLAMP_T)
        .edges("|Z")
        .fillet(0.0012)
    )


def _web_rib_shape() -> cq.Workplane:
    profile = [
        (-WEB_L / 2.0, -WEB_H / 2.0),
        (WEB_L / 2.0, -WEB_H / 2.0),
        (WEB_L * 0.32, WEB_H / 2.0),
        (-WEB_L * 0.32, WEB_H / 2.0),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(WEB_W / 2.0, both=True)


def _top_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(TOP_PLATE_L, TOP_PLATE_W, TOP_PLATE_T)
        .edges("|Z")
        .fillet(0.001)
    )
    return (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.007, 0.0), (0.007, 0.0)])
        .hole(0.0045, depth=TOP_PLATE_T + 0.001)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_roll_axis_rotary_module", assets=ASSETS)

    frame_gray = model.material("frame_gray", rgba=(0.25, 0.28, 0.31, 1.0))
    cartridge_aluminum = model.material("cartridge_aluminum", rgba=(0.74, 0.76, 0.80, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    bracket_black = model.material("bracket_black", rgba=(0.16, 0.17, 0.19, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((PLATE_T, FRAME_W, FRAME_H)),
        origin=Origin(xyz=(-FRAME_SIDE_X, 0.0, 0.0)),
        material=frame_gray,
        name="left_plate",
    )
    support_frame.visual(
        Box((PLATE_T, FRAME_W, FRAME_H)),
        origin=Origin(xyz=(FRAME_SIDE_X, 0.0, 0.0)),
        material=frame_gray,
        name="right_plate",
    )
    support_frame.visual(
        Box((FRAME_LEN - 2.0 * PLATE_T, FRAME_W, RAIL_T)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_H / 2.0 - RAIL_T / 2.0)),
        material=frame_gray,
        name="top_rail",
    )
    support_frame.visual(
        Box((FRAME_LEN - 2.0 * PLATE_T, FRAME_W, RAIL_T)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_H / 2.0 + RAIL_T / 2.0)),
        material=frame_gray,
        name="bottom_rail",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((FRAME_LEN, FRAME_W, FRAME_H)),
        mass=3.4,
    )

    bearing_mesh = mesh_from_cadquery(_bearing_cartridge_shape(), "bearing_cartridge.obj", assets=ASSETS)

    left_bearing = model.part("left_bearing_cartridge")
    left_bearing.visual(bearing_mesh, material=cartridge_aluminum, name="housing")
    left_bearing.inertial = Inertial.from_geometry(
        Box((CARTRIDGE_T, CARTRIDGE_W, CARTRIDGE_W)),
        mass=0.22,
    )

    right_bearing = model.part("right_bearing_cartridge")
    right_bearing.visual(bearing_mesh, material=cartridge_aluminum, name="housing")
    right_bearing.inertial = Inertial.from_geometry(
        Box((CARTRIDGE_T, CARTRIDGE_W, CARTRIDGE_W)),
        mass=0.22,
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
        origin=Origin(xyz=(-CARTRIDGE_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="left_journal",
    )
    shaft.visual(
        Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
        origin=Origin(xyz=(CARTRIDGE_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="right_journal",
    )
    shaft.visual(
        Cylinder(radius=SHOULDER_R, length=SHOULDER_T),
        origin=Origin(
            xyz=(-CARTRIDGE_INNER_FACE_X + SHOULDER_T / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=shaft_steel,
        name="left_shoulder",
    )
    shaft.visual(
        Cylinder(radius=SHOULDER_R, length=SHOULDER_T),
        origin=Origin(
            xyz=(CARTRIDGE_INNER_FACE_X - SHOULDER_T / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=shaft_steel,
        name="right_shoulder",
    )
    shaft.visual(
        Cylinder(radius=0.010, length=MAIN_SHAFT_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="center_hub",
    )
    shaft.visual(
        Box((MOUNT_PAD_L, MOUNT_PAD_W, MOUNT_PAD_T)),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_PAD_CENTER_Z)),
        material=shaft_steel,
        name="mount_pad",
    )
    shaft.inertial = Inertial.from_geometry(
        Box((FRAME_LEN - 0.020, 0.028, 0.028)),
        mass=0.68,
    )

    payload_bracket = model.part("payload_bracket")
    payload_bracket.visual(
        mesh_from_cadquery(_clamp_block_shape(), "payload_clamp_block.obj", assets=ASSETS),
        material=bracket_black,
        name="clamp_block",
    )
    payload_bracket.visual(
        mesh_from_cadquery(_web_rib_shape(), "payload_web_rib.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, CLAMP_T / 2.0 + WEB_H / 2.0)),
        material=bracket_black,
        name="web_rib",
    )
    payload_bracket.visual(
        mesh_from_cadquery(_top_plate_shape(), "payload_top_plate.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, CLAMP_T / 2.0 + WEB_H + TOP_PLATE_T / 2.0)),
        material=bracket_black,
        name="top_plate",
    )
    payload_bracket.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_L, TOP_PLATE_W, CLAMP_T + WEB_H + TOP_PLATE_T)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    model.articulation(
        "frame_to_left_bearing",
        ArticulationType.FIXED,
        parent=support_frame,
        child=left_bearing,
        origin=Origin(xyz=(-CARTRIDGE_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_right_bearing",
        ArticulationType.FIXED,
        parent=support_frame,
        child=right_bearing,
        origin=Origin(xyz=(CARTRIDGE_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_shaft_roll",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "shaft_to_payload_bracket",
        ArticulationType.FIXED,
        parent=shaft,
        child=payload_bracket,
        origin=Origin(xyz=(0.0, 0.0, BRACKET_ORIGIN_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support_frame = object_model.get_part("support_frame")
    left_bearing = object_model.get_part("left_bearing_cartridge")
    right_bearing = object_model.get_part("right_bearing_cartridge")
    shaft = object_model.get_part("shaft")
    payload_bracket = object_model.get_part("payload_bracket")
    roll = object_model.get_articulation("frame_to_shaft_roll")

    left_plate = support_frame.get_visual("left_plate")
    right_plate = support_frame.get_visual("right_plate")
    top_rail = support_frame.get_visual("top_rail")
    bottom_rail = support_frame.get_visual("bottom_rail")
    left_journal = shaft.get_visual("left_journal")
    right_journal = shaft.get_visual("right_journal")
    left_shoulder = shaft.get_visual("left_shoulder")
    right_shoulder = shaft.get_visual("right_shoulder")
    mount_pad = shaft.get_visual("mount_pad")
    clamp_block = payload_bracket.get_visual("clamp_block")
    top_plate = payload_bracket.get_visual("top_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.expect_contact(
        left_bearing,
        support_frame,
        elem_b=left_plate,
        name="left_bearing_cartridge_is_mounted_to_left_plate",
    )
    ctx.expect_contact(
        right_bearing,
        support_frame,
        elem_b=right_plate,
        name="right_bearing_cartridge_is_mounted_to_right_plate",
    )
    ctx.expect_origin_distance(
        left_bearing,
        right_bearing,
        axes="yz",
        max_dist=0.001,
        name="bearing_cartridges_share_a_common_axis",
    )
    ctx.expect_origin_distance(
        left_bearing,
        right_bearing,
        axes="x",
        min_dist=0.131,
        max_dist=0.133,
        name="bearing_cartridge_spacing_matches_frame_span",
    )
    ctx.expect_origin_distance(
        shaft,
        support_frame,
        axes="yz",
        max_dist=0.001,
        name="shaft_axis_is_centered_in_frame",
    )
    ctx.expect_contact(
        shaft,
        left_bearing,
        elem_a=left_shoulder,
        name="left_shaft_shoulder_seats_against_left_bearing",
    )
    ctx.expect_contact(
        shaft,
        right_bearing,
        elem_a=right_shoulder,
        name="right_shaft_shoulder_seats_against_right_bearing",
    )
    ctx.expect_overlap(
        shaft,
        left_bearing,
        axes="yz",
        min_overlap=0.016,
        elem_a=left_journal,
        name="left_journal_is_coaxially_carried_by_left_bearing",
    )
    ctx.expect_overlap(
        shaft,
        right_bearing,
        axes="yz",
        min_overlap=0.016,
        elem_a=right_journal,
        name="right_journal_is_coaxially_carried_by_right_bearing",
    )
    ctx.expect_contact(
        payload_bracket,
        shaft,
        elem_a=clamp_block,
        elem_b=mount_pad,
        name="payload_bracket_is_clamped_to_mount_pad",
    )
    ctx.expect_overlap(
        payload_bracket,
        shaft,
        axes="xy",
        min_overlap=0.020,
        elem_a=clamp_block,
        elem_b=mount_pad,
        name="payload_bracket_is_centered_on_shaft_mount_pad",
    )

    with ctx.pose({roll: 0.0}):
        ctx.expect_gap(
            support_frame,
            payload_bracket,
            axis="z",
            min_gap=0.004,
            positive_elem=top_rail,
            negative_elem=top_plate,
            name="payload_bracket_clears_top_rail_at_zero_roll",
        )

    with ctx.pose({roll: math.pi / 2.0}):
        ctx.expect_within(
            payload_bracket,
            support_frame,
            axes="y",
            margin=0.004,
            inner_elem=top_plate,
            name="payload_bracket_stays_within_frame_width_at_quarter_turn",
        )

    with ctx.pose({roll: math.pi}):
        ctx.expect_gap(
            payload_bracket,
            support_frame,
            axis="z",
            min_gap=0.004,
            positive_elem=top_plate,
            negative_elem=bottom_rail,
            name="payload_bracket_clears_bottom_rail_at_half_turn",
        )

    with ctx.pose({roll: 0.0}):
        p0 = ctx.part_world_position(payload_bracket)
    with ctx.pose({roll: math.pi / 2.0}):
        p90 = ctx.part_world_position(payload_bracket)
    with ctx.pose({roll: math.pi}):
        p180 = ctx.part_world_position(payload_bracket)

    motion_ok = (
        p0 is not None
        and p90 is not None
        and p180 is not None
        and abs(p0[0]) < 0.001
        and abs(p0[1]) < 0.001
        and abs(p0[2] - BRACKET_ORIGIN_Z) < 0.002
        and abs(p90[0]) < 0.001
        and abs(p90[1] + BRACKET_ORIGIN_Z) < 0.002
        and abs(p90[2]) < 0.002
        and abs(p180[0]) < 0.001
        and abs(p180[1]) < 0.002
        and abs(p180[2] + BRACKET_ORIGIN_Z) < 0.002
    )
    ctx.check(
        "payload_bracket_follows_roll_axis_about_the_shaft_centerline",
        motion_ok,
        details=f"p0={p0}, p90={p90}, p180={p180}, radial_offset={BRACKET_ORIGIN_Z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
