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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_roll_axis_spindle")

    steel_dark = model.material("steel_dark", rgba=(0.30, 0.33, 0.36, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.56, 0.58, 0.60, 1.0))
    machined_face = model.material("machined_face", rgba=(0.77, 0.79, 0.81, 1.0))

    foot_x = 0.30
    foot_y = 0.22
    foot_t = 0.045

    column_x = 0.10
    column_y = 0.09
    column_h = 0.175

    saddle_x = 0.17
    saddle_y = 0.11
    saddle_t = 0.018

    cheek_t = 0.030
    cheek_y = 0.11
    cheek_z = 0.100
    inner_gap = 0.090
    shaft_axis_z = foot_t + column_h + saddle_t + cheek_z / 2.0

    cheek_center_x = inner_gap / 2.0 + cheek_t / 2.0

    shaft_r = 0.018
    shaft_len = inner_gap - 0.008
    flange_r = 0.042
    flange_t = 0.022
    flange_hole_r = 0.0055
    hub_r = 0.024
    hub_t = 0.036
    thrust_r = 0.030
    thrust_t = 0.004
    left_inner_face_x = -inner_gap / 2.0
    right_inner_face_x = inner_gap / 2.0

    pedestal = model.part("pedestal")

    foot = (
        cq.Workplane("XY")
        .box(foot_x, foot_y, foot_t)
        .translate((0.0, 0.0, foot_t / 2.0))
        .edges("|Z")
        .fillet(0.010)
        .faces(">Z")
        .workplane()
        .rect(foot_x - 0.080, foot_y - 0.080, forConstruction=True)
        .vertices()
        .hole(0.016)
    )
    pedestal.visual(mesh_from_cadquery(foot, "pedestal_foot"), material=steel_dark, name="foot")

    column = (
        cq.Workplane("XY")
        .box(column_x, column_y, column_h)
        .translate((0.0, 0.0, foot_t + column_h / 2.0))
        .edges("|Z")
        .fillet(0.008)
    )
    pedestal.visual(mesh_from_cadquery(column, "pedestal_column"), material=steel_dark, name="column")

    saddle = (
        cq.Workplane("XY")
        .box(saddle_x, saddle_y, saddle_t)
        .translate((0.0, 0.0, foot_t + column_h + saddle_t / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )
    pedestal.visual(mesh_from_cadquery(saddle, "pedestal_saddle"), material=steel_dark, name="saddle")

    def make_cheek(center_x: float, side: str) -> cq.Workplane:
        cheek = (
            cq.Workplane("XY")
            .box(cheek_t, cheek_y, cheek_z)
            .translate((center_x, 0.0, shaft_axis_z))
            .edges("|Z")
            .fillet(0.004)
        )

        if side == "left":
            outer_boss_start_x = center_x - cheek_t / 2.0 - 0.010
            recess_start_x = left_inner_face_x - 0.010
        else:
            outer_boss_start_x = center_x + cheek_t / 2.0
            recess_start_x = right_inner_face_x

        outer_boss = (
            cq.Workplane("YZ")
            .circle(0.032)
            .extrude(0.010)
            .translate((outer_boss_start_x, 0.0, shaft_axis_z))
        )
        inner_recess = (
            cq.Workplane("YZ")
            .circle(0.026)
            .extrude(0.010)
            .translate((recess_start_x, 0.0, shaft_axis_z))
        )
        return cheek.union(outer_boss).cut(inner_recess)

    left_cheek = make_cheek(-cheek_center_x, "left")
    right_cheek = make_cheek(cheek_center_x, "right")
    pedestal.visual(
        mesh_from_cadquery(left_cheek, "pedestal_left_cheek"),
        material=steel_dark,
        name="left_cheek",
    )
    pedestal.visual(
        mesh_from_cadquery(right_cheek, "pedestal_right_cheek"),
        material=steel_dark,
        name="right_cheek",
    )

    pedestal.inertial = Inertial.from_geometry(
        Box((foot_x, foot_y, shaft_axis_z + cheek_z / 2.0)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (shaft_axis_z + cheek_z / 2.0) / 2.0)),
    )

    spindle_head = model.part("spindle_head")
    spindle_body = (
        cq.Workplane("YZ")
        .circle(shaft_r)
        .extrude(shaft_len)
        .translate((-shaft_len / 2.0, 0.0, 0.0))
    )
    flange = (
        cq.Workplane("YZ")
        .circle(flange_r)
        .extrude(flange_t)
        .translate((-flange_t / 2.0, 0.0, 0.0))
    )
    bolt_circle_r = 0.026
    for angle_deg in (0.0, 120.0, 240.0):
        angle_rad = math.radians(angle_deg)
        hole_y = bolt_circle_r * math.cos(angle_rad)
        hole_z = bolt_circle_r * math.sin(angle_rad)
        hole = (
            cq.Workplane("YZ")
            .circle(flange_hole_r)
            .extrude(flange_t + 0.008)
            .translate((-(flange_t + 0.008) / 2.0, hole_y, hole_z))
        )
        flange = flange.cut(hole)
    central_hub = (
        cq.Workplane("YZ")
        .circle(hub_r)
        .extrude(hub_t)
        .translate((-hub_t / 2.0, 0.0, 0.0))
    )
    left_thrust = (
        cq.Workplane("YZ")
        .circle(thrust_r)
        .extrude(thrust_t)
        .translate((left_inner_face_x, 0.0, 0.0))
    )
    right_thrust = (
        cq.Workplane("YZ")
        .circle(thrust_r)
        .extrude(thrust_t)
        .translate((right_inner_face_x - thrust_t, 0.0, 0.0))
    )
    spindle_shape = spindle_body.union(flange).union(central_hub).union(left_thrust).union(right_thrust)
    spindle_head.visual(
        mesh_from_cadquery(spindle_shape, "spindle_head_body"),
        material=steel_mid,
        name="head_body",
    )
    spindle_head.visual(
        Box((0.010, 0.018, 0.028)),
        origin=Origin(xyz=(0.011, 0.026, 0.0)),
        material=machined_face,
        name="index_pad",
    )
    spindle_head.inertial = Inertial.from_geometry(
        Box((shaft_len, flange_r * 2.0, flange_r * 2.0)),
        mass=3.2,
        origin=Origin(),
    )

    model.articulation(
        "pedestal_to_spindle_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=spindle_head,
        origin=Origin(xyz=(0.0, 0.0, shaft_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    spindle_head = object_model.get_part("spindle_head")
    roll_joint = object_model.get_articulation("pedestal_to_spindle_head")

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

    axis = roll_joint.axis
    limits = roll_joint.motion_limits
    ctx.check(
        "roll_joint_aligned_with_shaft_x_axis",
        abs(abs(axis[0]) - 1.0) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2]) < 1e-9,
        details=f"axis={axis}",
    )
    ctx.check(
        "roll_joint_has_bidirectional_spin_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0
        and limits.upper > 0.0,
        details=f"limits={limits}",
    )

    spindle_pos = ctx.part_world_position(spindle_head)
    ctx.check(
        "spindle_axis_sits_on_pedestal_centerline",
        spindle_pos is not None
        and abs(spindle_pos[0]) < 1e-6
        and abs(spindle_pos[1]) < 1e-6
        and 0.26 < spindle_pos[2] < 0.30,
        details=f"spindle_pos={spindle_pos}",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

    with ctx.pose({roll_joint: 0.0}):
        closed_pad = aabb_center(ctx.part_element_world_aabb(spindle_head, elem="index_pad"))

    with ctx.pose({roll_joint: math.pi / 2.0}):
        quarter_pad = aabb_center(ctx.part_element_world_aabb(spindle_head, elem="index_pad"))
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_quarter_turn")

    ctx.check(
        "index_pad_tracks_roll_motion",
        closed_pad is not None
        and quarter_pad is not None
        and closed_pad[1] > 0.015
        and abs(closed_pad[2] - spindle_pos[2]) < 0.004
        and abs(quarter_pad[1]) < 0.004
        and quarter_pad[2] > spindle_pos[2] + 0.015,
        details=f"closed_pad={closed_pad}, quarter_pad={quarter_pad}, spindle_pos={spindle_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
