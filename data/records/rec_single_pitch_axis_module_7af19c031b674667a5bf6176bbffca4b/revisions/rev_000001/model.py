from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


FOOT_X = 0.28
FOOT_Y = 0.22
FOOT_H = 0.03

SHOULDER_X = 0.095
SHOULDER_Y = 0.115
SHOULDER_H = 0.045

COLUMN_X = 0.075
COLUMN_Y = 0.092
COLUMN_H = 0.055

OUTER_Y = 0.194
INNER_GAP = 0.15
CHEEK_T = (OUTER_Y - INNER_GAP) / 2.0
YOKE_BASE_Z = FOOT_H + SHOULDER_H + COLUMN_H - 0.004
CHEEK_X = 0.034
CHEEK_H = 0.11
BACK_BRIDGE_X = 0.055
BACK_BRIDGE_Y = 0.15
BACK_BRIDGE_Z = 0.045
BACK_BRIDGE_CENTER_X = -0.028

TRUNNION_AXIS_X = 0.0
TRUNNION_AXIS_Z = 0.22
CHEEK_BORE_RADIUS = 0.014
TRUNNION_SHAFT_RADIUS = 0.01
TRUNNION_COLLAR_RADIUS = 0.018
TRUNNION_COLLAR_T = 0.003

HEAD_BODY_X = 0.102
HEAD_BODY_Y = 0.104
HEAD_BODY_Z = 0.078
HEAD_BODY_CENTER_X = 0.096
HEAD_BODY_CENTER_Z = 0.002
HEAD_NECK_X = 0.042
HEAD_NECK_Y = 0.05
HEAD_NECK_Z = 0.042
HEAD_NECK_CENTER_X = 0.048
HEAD_HUB_X = 0.022
HEAD_HUB_Y = 0.04
HEAD_HUB_Z = 0.04
HEAD_HUB_CENTER_X = 0.018


def _make_pedestal() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_X, FOOT_Y, FOOT_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )

    shoulder = (
        cq.Workplane("XY")
        .box(SHOULDER_X, SHOULDER_Y, SHOULDER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, FOOT_H))
    )

    column = (
        cq.Workplane("XY")
        .box(COLUMN_X, COLUMN_Y, COLUMN_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.01)
        .translate((0.0, 0.0, FOOT_H + SHOULDER_H))
    )

    cheek_y_center = INNER_GAP / 2.0 + CHEEK_T / 2.0
    cheek_z_center = TRUNNION_AXIS_Z - 0.014

    back_bridge = (
        cq.Workplane("XY")
        .box(BACK_BRIDGE_X, BACK_BRIDGE_Y, BACK_BRIDGE_Z)
        .edges("|Z")
        .fillet(0.008)
        .translate((BACK_BRIDGE_CENTER_X, 0.0, YOKE_BASE_Z + BACK_BRIDGE_Z / 2.0))
    )

    left_cheek_blank = (
        cq.Workplane("XY")
        .box(CHEEK_X, CHEEK_T, CHEEK_H)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, cheek_y_center, cheek_z_center))
    )

    right_cheek_blank = (
        cq.Workplane("XY")
        .box(CHEEK_X, CHEEK_T, CHEEK_H)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, -cheek_y_center, cheek_z_center))
    )

    left_bore = (
        cq.Workplane("XZ")
        .workplane(offset=INNER_GAP / 2.0)
        .center(TRUNNION_AXIS_X, TRUNNION_AXIS_Z)
        .circle(CHEEK_BORE_RADIUS)
        .extrude(CHEEK_T)
    )

    right_bore = (
        cq.Workplane("XZ")
        .workplane(offset=-(INNER_GAP / 2.0 + CHEEK_T))
        .center(TRUNNION_AXIS_X, TRUNNION_AXIS_Z)
        .circle(CHEEK_BORE_RADIUS)
        .extrude(CHEEK_T)
    )

    return (
        foot.union(shoulder)
        .union(column)
        .union(back_bridge)
        .union(left_cheek_blank.cut(left_bore))
        .union(right_cheek_blank.cut(right_bore))
    )


def _make_head() -> cq.Workplane:
    axle = (
        cq.Workplane("XZ")
        .workplane(offset=-(OUTER_Y / 2.0))
        .center(0.0, 0.0)
        .circle(TRUNNION_SHAFT_RADIUS)
        .extrude(OUTER_Y)
    )

    left_collar = (
        cq.Workplane("XZ")
        .workplane(offset=OUTER_Y / 2.0)
        .center(0.0, 0.0)
        .circle(TRUNNION_COLLAR_RADIUS)
        .extrude(TRUNNION_COLLAR_T)
    )

    right_collar = (
        cq.Workplane("XZ")
        .workplane(offset=-(OUTER_Y / 2.0 + TRUNNION_COLLAR_T))
        .center(0.0, 0.0)
        .circle(TRUNNION_COLLAR_RADIUS)
        .extrude(TRUNNION_COLLAR_T)
    )

    hub = (
        cq.Workplane("XY")
        .box(HEAD_HUB_X, HEAD_HUB_Y, HEAD_HUB_Z)
        .edges("|Y")
        .fillet(0.01)
        .translate((HEAD_HUB_CENTER_X, 0.0, 0.0))
    )

    neck = (
        cq.Workplane("XY")
        .box(HEAD_NECK_X, HEAD_NECK_Y, HEAD_NECK_Z)
        .edges("|Y")
        .fillet(0.01)
        .translate((HEAD_NECK_CENTER_X, 0.0, 0.0))
    )

    body = (
        cq.Workplane("XY")
        .box(HEAD_BODY_X, HEAD_BODY_Y, HEAD_BODY_Z)
        .edges("|Y")
        .fillet(0.016)
        .translate((HEAD_BODY_CENTER_X, 0.0, HEAD_BODY_CENTER_Z))
    )

    bezel = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_BODY_CENTER_X + HEAD_BODY_X / 2.0 - 0.004)
        .circle(0.034)
        .extrude(0.02)
    )

    recess = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_BODY_CENTER_X + HEAD_BODY_X / 2.0 - 0.002)
        .circle(0.024)
        .extrude(0.02)
    )

    crown = (
        cq.Workplane("XY")
        .box(0.075, 0.11, 0.018)
        .edges("|Y")
        .fillet(0.008)
        .translate((0.03, 0.0, 0.032))
    )

    return (
        axle.union(left_collar)
        .union(right_collar)
        .union(hub)
        .union(neck)
        .union(body)
        .union(bezel)
        .union(crown)
        .cut(recess)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_trunnion_module")

    base_color = model.material("base_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    head_color = model.material("head_finish", rgba=(0.60, 0.63, 0.67, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_make_pedestal(), "pedestal_shell"),
        origin=Origin(),
        material=base_color,
        name="pedestal_shell",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((FOOT_X, FOOT_Y, 0.26)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head(), "head_shell"),
        origin=Origin(),
        material=head_color,
        name="head_shell",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.13, INNER_GAP, 0.09)),
        mass=2.2,
        origin=Origin(xyz=(0.05, 0.0, -0.005)),
    )

    model.articulation(
        "pedestal_to_head_pitch",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(TRUNNION_AXIS_X, 0.0, TRUNNION_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-0.75,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("pedestal_to_head_pitch")

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
        head,
        pedestal,
        reason=(
            "the trunnion spindle is intentionally captured inside the pedestal's "
            "side-cheek bearing bores, so the pitch module has installed bearing overlap"
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "pitch_axis_is_horizontal",
        pitch.axis == (0.0, -1.0, 0.0),
        details=f"expected pitch axis (0, -1, 0), got {pitch.axis}",
    )
    ctx.check(
        "pitch_limits_span_level_pose",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower < 0.0 < pitch.motion_limits.upper,
        details="pitch joint should allow both slight down-tilt and up-tilt from level",
    )
    ctx.expect_origin_gap(
        head,
        pedestal,
        axis="z",
        min_gap=0.21,
        max_gap=0.23,
        name="trunnion_axis_height",
    )

    with ctx.pose({pitch: 0.0}):
        ctx.expect_contact(head, pedestal, name="closed_pose_trunnion_contact")
        ctx.expect_overlap(head, pedestal, axes="xy", min_overlap=0.05, name="head_over_pedestal")

        closed_aabb = ctx.part_element_world_aabb(head, elem="head_shell")
        if closed_aabb is None:
            ctx.fail("head_shell_aabb_closed", "missing world AABB for head_shell in level pose")
            closed_center_z = None
        else:
            closed_center_z = 0.5 * (closed_aabb[0][2] + closed_aabb[1][2])

    with ctx.pose({pitch: pitch.motion_limits.upper}):
        ctx.expect_contact(head, pedestal, name="open_pose_trunnion_contact")

        open_aabb = ctx.part_element_world_aabb(head, elem="head_shell")
        if open_aabb is None:
            ctx.fail("head_shell_aabb_open", "missing world AABB for head_shell in raised pose")
        elif closed_center_z is not None:
            open_center_z = 0.5 * (open_aabb[0][2] + open_aabb[1][2])
            ctx.check(
                "positive_pitch_raises_head",
                open_center_z > closed_center_z + 0.02,
                details=(
                    f"expected raised-pitch head center z to increase by > 0.02 m; "
                    f"closed={closed_center_z:.4f}, open={open_center_z:.4f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
