from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BRACKET_W = 0.068
BRACKET_H = 0.155
BRACKET_D = 0.015

LINK_1_LEN = 0.120
LINK_2_LEN = 0.115
PAN_NECK_LEN = 0.036

HINGE_R = 0.011
OUTER_BARREL_H = 0.012
CENTER_BARREL_H = 0.014
OUTER_BARREL_Z = (CENTER_BARREL_H + OUTER_BARREL_H) / 2.0

LINK_W = 0.010
LINK_H = 0.012
LINK_SIDE_OFFSET = 0.020
CONNECTOR_LEN = 0.020
CONNECTOR_H = 0.010
BODY_START = 0.016
BODY_END_MARGIN = 0.024
DISTAL_NECK_LEN = 0.020
DISTAL_NECK_H = 0.009

PITCH_R = 0.010
PITCH_CENTER_LEN = 0.032
PITCH_OUTER_LEN = 0.015
PITCH_OUTER_X = (PITCH_CENTER_LEN + PITCH_OUTER_LEN) / 2.0

FRAME_W = 0.240
FRAME_H = 0.160
FRAME_T = 0.006
FRAME_BAR = 0.025
FRAME_OFFSET = 0.028
FRAME_HUB_W = 0.086
FRAME_HUB_H = 0.110
FRAME_HUB_D = 0.014
VESA_X = 0.200
VESA_Z = 0.100


def z_cylinder(radius: float, height: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((x, y, z - height / 2.0))
    )


def x_cylinder(radius: float, length: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((x - length / 2.0, y, z))
    )


def half_z_pivot(radius: float, height: float, *, y_center: float, side: str) -> cq.Workplane:
    clip_len = radius + 0.002
    clip = cq.Workplane("XY").box(2.0 * radius + 0.004, clip_len, height + 0.004)
    if side == "front":
        clip = clip.translate((0.0, y_center + clip_len / 2.0, 0.0))
    else:
        clip = clip.translate((0.0, y_center - clip_len / 2.0, 0.0))
    return z_cylinder(radius, height, center=(0.0, y_center, 0.0)).intersect(clip)


def half_x_pivot(radius: float, length: float, *, y_center: float, side: str) -> cq.Workplane:
    clip_len = radius + 0.002
    clip = cq.Workplane("XY").box(length + 0.004, clip_len, 2.0 * radius + 0.004)
    if side == "front":
        clip = clip.translate((0.0, y_center + clip_len / 2.0, 0.0))
    else:
        clip = clip.translate((0.0, y_center - clip_len / 2.0, 0.0))
    return x_cylinder(radius, length, center=(0.0, y_center, 0.0)).intersect(clip)


def make_wall_bracket() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BRACKET_W, BRACKET_D, BRACKET_H)
        .translate((-0.012, -(HINGE_R + BRACKET_D / 2.0), 0.0))
        .edges("|Y")
        .fillet(0.005)
    )
    mounting_holes = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (-0.016, 0.050),
                (0.016, 0.050),
                (-0.016, -0.050),
                (0.016, -0.050),
            ]
        )
        .circle(0.0045)
        .extrude(BRACKET_D + 0.004)
        .translate((0.0, -(HINGE_R + BRACKET_D + 0.002), 0.0))
    )
    spine = cq.Workplane("XY").box(0.024, 0.018, 0.034).translate((-0.008, -0.009, 0.0))
    barrel = z_cylinder(HINGE_R, CENTER_BARREL_H, center=(0.0, 0.0, 0.0))
    gusset = cq.Workplane("XY").box(0.020, 0.010, 0.055).translate((-0.006, -0.015, 0.0))
    return plate.cut(mounting_holes).union(spine).union(gusset).union(barrel)


def make_swivel_link(length: float, side: float) -> cq.Workplane:
    x_mid = side * (LINK_SIDE_OFFSET / 2.0)
    x_strap = side * LINK_SIDE_OFFSET

    upper_barrel = z_cylinder(HINGE_R, OUTER_BARREL_H, center=(0.0, 0.0, OUTER_BARREL_Z))
    lower_barrel = z_cylinder(HINGE_R, OUTER_BARREL_H, center=(0.0, 0.0, -OUTER_BARREL_Z))
    upper_connector = cq.Workplane("XY").box(LINK_SIDE_OFFSET + LINK_W, 0.018, CONNECTOR_H).translate(
        (x_mid, 0.009, OUTER_BARREL_Z)
    )
    lower_connector = cq.Workplane("XY").box(LINK_SIDE_OFFSET + LINK_W, 0.018, CONNECTOR_H).translate(
        (x_mid, 0.009, -OUTER_BARREL_Z)
    )

    body_len = length - 0.036
    body = (
        cq.Workplane("XY")
        .box(LINK_W, body_len, LINK_H)
        .translate((x_strap, 0.018 + body_len / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.002)
    )
    distal_connector = cq.Workplane("XY").box(LINK_SIDE_OFFSET + LINK_W, 0.018, DISTAL_NECK_H).translate(
        (x_mid, length - 0.009, 0.0)
    )
    distal_barrel = z_cylinder(HINGE_R, CENTER_BARREL_H, center=(0.0, length, 0.0))

    return (
        upper_barrel.union(lower_barrel)
        .union(upper_connector)
        .union(lower_connector)
        .union(body)
        .union(distal_connector)
        .union(distal_barrel)
    )


def make_head_yoke() -> cq.Workplane:
    x_mid = LINK_SIDE_OFFSET / 2.0
    x_strap = LINK_SIDE_OFFSET

    upper_barrel = z_cylinder(HINGE_R, OUTER_BARREL_H, center=(0.0, 0.0, OUTER_BARREL_Z))
    lower_barrel = z_cylinder(HINGE_R, OUTER_BARREL_H, center=(0.0, 0.0, -OUTER_BARREL_Z))
    upper_connector = cq.Workplane("XY").box(LINK_SIDE_OFFSET + 0.010, 0.018, CONNECTOR_H).translate(
        (x_mid, 0.009, OUTER_BARREL_Z)
    )
    lower_connector = cq.Workplane("XY").box(LINK_SIDE_OFFSET + 0.010, 0.018, CONNECTOR_H).translate(
        (x_mid, 0.009, -OUTER_BARREL_Z)
    )

    center_body_len = PAN_NECK_LEN - 0.028
    center_body = (
        cq.Workplane("XY")
        .box(0.010, center_body_len, 0.014)
        .translate((x_strap, 0.014 + center_body_len / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.0018)
    )
    pitch_support = cq.Workplane("XY").box(0.030, 0.014, 0.024).translate((0.0, PAN_NECK_LEN - 0.007, 0.0))
    side_web = cq.Workplane("XY").box(LINK_SIDE_OFFSET + 0.010, 0.016, 0.014).translate((x_mid, PAN_NECK_LEN - 0.010, 0.0))
    pitch_barrel = x_cylinder(PITCH_R, PITCH_CENTER_LEN, center=(0.0, PAN_NECK_LEN, 0.0))

    return (
        upper_barrel.union(lower_barrel)
        .union(upper_connector)
        .union(lower_connector)
        .union(center_body)
        .union(side_web)
        .union(pitch_support)
        .union(pitch_barrel)
    )


def make_vesa_frame() -> cq.Workplane:
    pitch_receiver = half_x_pivot(PITCH_R, PITCH_CENTER_LEN, y_center=0.0, side="front")
    left_hub = cq.Workplane("XY").box(0.014, 0.016, 0.056).translate((-PITCH_OUTER_X, 0.012, 0.0))
    right_hub = cq.Workplane("XY").box(0.014, 0.016, 0.056).translate((PITCH_OUTER_X, 0.012, 0.0))

    hub_plate = cq.Workplane("XY").box(FRAME_HUB_W, FRAME_HUB_D, FRAME_HUB_H).translate(
        (0.0, FRAME_OFFSET + 0.004, 0.0)
    )
    ring_outer = cq.Workplane("XY").box(FRAME_W, FRAME_T, FRAME_H).translate((0.0, FRAME_OFFSET + FRAME_T / 2.0, 0.0))
    ring_inner = cq.Workplane("XY").box(FRAME_W - 2.0 * FRAME_BAR, FRAME_T + 0.002, FRAME_H - 2.0 * FRAME_BAR).translate(
        (0.0, FRAME_OFFSET + FRAME_T / 2.0, 0.0)
    )
    ring = ring_outer.cut(ring_inner)
    vesa_holes = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (-VESA_X / 2.0, VESA_Z / 2.0),
                (VESA_X / 2.0, VESA_Z / 2.0),
                (-VESA_X / 2.0, -VESA_Z / 2.0),
                (VESA_X / 2.0, -VESA_Z / 2.0),
            ]
        )
        .circle(0.0035)
        .extrude(FRAME_T + 0.004)
        .translate((0.0, FRAME_OFFSET - 0.002, 0.0))
    )

    return pitch_receiver.union(left_hub).union(right_hub).union(hub_plate).union(ring.cut(vesa_holes))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_full_motion_wall_mount")

    model.material("bracket_black", color=(0.14, 0.14, 0.15, 1.0))
    model.material("arm_black", color=(0.17, 0.17, 0.18, 1.0))
    model.material("head_black", color=(0.13, 0.13, 0.14, 1.0))
    model.material("frame_black", color=(0.12, 0.12, 0.13, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        mesh_from_cadquery(make_wall_bracket(), "wall_bracket"),
        material="bracket_black",
        name="wall_bracket_body",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_W, BRACKET_D + HINGE_R, BRACKET_H)),
        mass=0.75,
        origin=Origin(xyz=(0.0, -(HINGE_R + BRACKET_D) / 2.0, 0.0)),
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(make_swivel_link(LINK_1_LEN, 1.0), "link_1"),
        material="arm_black",
        name="link_1_body",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_W, LINK_1_LEN + 2.0 * HINGE_R, 0.040)),
        mass=0.42,
        origin=Origin(xyz=(0.0, LINK_1_LEN / 2.0, 0.0)),
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(make_swivel_link(LINK_2_LEN, -1.0), "link_2"),
        material="arm_black",
        name="link_2_body",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_W, LINK_2_LEN + 2.0 * HINGE_R, 0.040)),
        mass=0.38,
        origin=Origin(xyz=(0.0, LINK_2_LEN / 2.0, 0.0)),
    )

    head_yoke = model.part("head_yoke")
    head_yoke.visual(
        mesh_from_cadquery(make_head_yoke(), "head_yoke"),
        material="head_black",
        name="head_yoke_body",
    )
    head_yoke.inertial = Inertial.from_geometry(
        Box((0.050, PAN_NECK_LEN + 0.020, 0.040)),
        mass=0.30,
        origin=Origin(xyz=(0.0, PAN_NECK_LEN / 2.0, 0.0)),
    )

    vesa_frame = model.part("vesa_frame")
    vesa_frame.visual(
        mesh_from_cadquery(make_vesa_frame(), "vesa_frame"),
        material="frame_black",
        name="vesa_frame_body",
    )
    vesa_frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, FRAME_OFFSET + FRAME_HUB_D, FRAME_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
    )

    model.articulation(
        "wall_swivel",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=link_1,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-1.7, upper=1.7),
    )
    model.articulation(
        "elbow_swivel",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, LINK_1_LEN, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.1, lower=-3.05, upper=3.05),
    )
    model.articulation(
        "head_pan",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=head_yoke,
        origin=Origin(xyz=(0.0, LINK_2_LEN, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.2, lower=-3.05, upper=3.05),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=vesa_frame,
        origin=Origin(xyz=(0.0, PAN_NECK_LEN, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=-0.35, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    head_yoke = object_model.get_part("head_yoke")
    vesa_frame = object_model.get_part("vesa_frame")

    wall_swivel = object_model.get_articulation("wall_swivel")
    elbow_swivel = object_model.get_articulation("elbow_swivel")
    head_pan_joint = object_model.get_articulation("head_pan")
    head_pitch = object_model.get_articulation("head_pitch")

    ctx.allow_overlap(
        wall_bracket,
        link_1,
        reason="shared wall-side hinge barrels occupy the same revolute axis volume",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        reason="nested elbow hinge barrels intentionally share coaxial joint space",
    )
    ctx.allow_overlap(
        link_2,
        head_yoke,
        reason="pan hinge barrels intentionally overlap at the shared swivel axis",
    )
    ctx.allow_overlap(
        head_yoke,
        vesa_frame,
        reason="pitch trunnion and receiver are modeled as interleaved coaxial hinge hardware",
    )

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

    for part_name in ("wall_bracket", "link_1", "link_2", "head_yoke", "vesa_frame"):
        ctx.check(f"part present: {part_name}", object_model.get_part(part_name) is not None)

    ctx.check(
        "wall swivel axis is vertical",
        wall_swivel.axis == (0.0, 0.0, 1.0),
        details=f"axis={wall_swivel.axis}",
    )
    ctx.check(
        "elbow swivel axis is vertical",
        elbow_swivel.axis == (0.0, 0.0, 1.0),
        details=f"axis={elbow_swivel.axis}",
    )
    ctx.check(
        "head pan axis is vertical",
        head_pan_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={head_pan_joint.axis}",
    )
    ctx.check(
        "head pitch axis is horizontal",
        head_pitch.axis == (1.0, 0.0, 0.0),
        details=f"axis={head_pitch.axis}",
    )

    ctx.expect_contact(link_1, wall_bracket, name="wall bracket touches first link")
    ctx.expect_contact(link_1, link_2, name="first link touches second link")
    ctx.expect_contact(link_2, head_yoke, name="second link touches pan head")
    ctx.expect_contact(head_yoke, vesa_frame, name="pan head touches vesa frame")

    folded_pose = {
        wall_swivel: -0.41,
        elbow_swivel: 2.24,
        head_pan_joint: -1.02,
        head_pitch: 0.0,
    }

    with ctx.pose(folded_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded pose has no overlaps")
        ctx.expect_gap(
            vesa_frame,
            wall_bracket,
            axis="y",
            min_gap=0.008,
            max_gap=0.080,
            name="folded frame stays close to wall",
        )
        folded_pos = ctx.part_world_position(vesa_frame)

    with ctx.pose({wall_swivel: 0.0, elbow_swivel: 0.0, head_pan_joint: 0.0, head_pitch: 0.0}):
        ctx.expect_gap(
            vesa_frame,
            wall_bracket,
            axis="y",
            min_gap=0.220,
            name="deployed frame reaches outward",
        )
        deployed_pos = ctx.part_world_position(vesa_frame)

    if folded_pos is not None and deployed_pos is not None:
        ctx.check(
            "deployment adds meaningful reach",
            deployed_pos[1] - folded_pos[1] > 0.100,
            details=f"folded_y={folded_pos[1]:.4f}, deployed_y={deployed_pos[1]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
