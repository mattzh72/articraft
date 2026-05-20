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


SUPPORT_LENGTH = 0.24
SUPPORT_WIDTH = 0.08
SUPPORT_THICKNESS = 0.014
MOUNT_HOLE_RADIUS = 0.007

BOSS_RADIUS = 0.032
BOSS_DROP = 0.020
SHAFT_RADIUS = 0.015
SHAFT_LENGTH = 0.305

COLLAR_RADIUS = 0.032
COLLAR_THICKNESS = 0.010
OUTER_STAGE_Z = -0.040
MID_STAGE_Z = -0.130
INNER_STAGE_Z = -0.220

HANGER_OUTER_RADIUS = 0.028
HANGER_INNER_RADIUS = SHAFT_RADIUS + 0.001
HANGER_THICKNESS = 0.004
SLEEVE_OUTER_RADIUS = 0.024
SLEEVE_HEIGHT = 0.028
RIM_HEIGHT = 0.020
RIM_TOP_Z = -0.012
SPOKE_THICKNESS = 0.010
SPOKE_WIDTH = 0.018
SPOKE_CENTER_Z = -0.018


def annulus(outer_radius: float, inner_radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(height)
    ring = outer.cut(inner)
    if z0:
        ring = ring.translate((0.0, 0.0, z0))
    return ring


def cylinder(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    body = cq.Workplane("XY").circle(radius).extrude(height)
    if z0:
        body = body.translate((0.0, 0.0, z0))
    return body


def make_frame_components() -> dict[str, cq.Workplane]:
    top_support = (
        cq.Workplane("XY")
        .rect(SUPPORT_LENGTH, SUPPORT_WIDTH)
        .extrude(SUPPORT_THICKNESS)
        .edges("|Z")
        .fillet(0.018)
    )
    hole_cutter = (
        cq.Workplane("XY")
        .pushPoints([(-0.078, 0.0), (0.078, 0.0)])
        .circle(MOUNT_HOLE_RADIUS)
        .extrude(SUPPORT_THICKNESS + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    top_support = top_support.cut(hole_cutter)

    boss = cylinder(BOSS_RADIUS, SUPPORT_THICKNESS + BOSS_DROP, -BOSS_DROP)
    shaft = cylinder(SHAFT_RADIUS, SHAFT_LENGTH, -SHAFT_LENGTH)
    outer_collar = cylinder(COLLAR_RADIUS, COLLAR_THICKNESS, OUTER_STAGE_Z)
    mid_collar = cylinder(COLLAR_RADIUS, COLLAR_THICKNESS, MID_STAGE_Z)
    inner_collar = cylinder(COLLAR_RADIUS, COLLAR_THICKNESS, INNER_STAGE_Z)
    tail_cap = cylinder(0.022, 0.012, -SHAFT_LENGTH)

    return {
        "top_support": top_support,
        "center_boss": boss,
        "shaft": shaft,
        "collar_outer": outer_collar,
        "collar_mid": mid_collar,
        "collar_inner": inner_collar,
        "tail_cap": tail_cap,
    }


def make_stage_mesh(
    *,
    outer_radius: float,
    rim_width: float,
    tab_angle_deg: float,
    tab_length: float,
    tab_width: float,
    mesh_name: str,
):
    rim_inner_radius = outer_radius - rim_width
    rim_bottom_z = RIM_TOP_Z - RIM_HEIGHT

    hanger = annulus(HANGER_OUTER_RADIUS, HANGER_INNER_RADIUS, HANGER_THICKNESS, -HANGER_THICKNESS)
    sleeve = annulus(
        SLEEVE_OUTER_RADIUS,
        HANGER_INNER_RADIUS,
        SLEEVE_HEIGHT,
        -(HANGER_THICKNESS + SLEEVE_HEIGHT),
    )
    rim = annulus(outer_radius, rim_inner_radius, RIM_HEIGHT, rim_bottom_z)

    stage = hanger.union(sleeve).union(rim)

    spoke_length = rim_inner_radius - SLEEVE_OUTER_RADIUS
    spoke_center_x = SLEEVE_OUTER_RADIUS + spoke_length / 2.0
    base_spoke = (
        cq.Workplane("XY")
        .box(spoke_length, SPOKE_WIDTH, SPOKE_THICKNESS)
        .translate((spoke_center_x, 0.0, SPOKE_CENTER_Z))
    )
    for angle in (0.0, 90.0, 180.0, 270.0):
        stage = stage.union(base_spoke.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle))

    tab = (
        cq.Workplane("XY")
        .box(tab_length, tab_width, RIM_HEIGHT - 0.004)
        .translate((outer_radius + tab_length / 2.0 - 0.006, 0.0, rim_bottom_z + RIM_HEIGHT / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), tab_angle_deg)
    )
    stage = stage.union(tab)

    return mesh_from_cadquery(stage, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_coaxial_stage_stack")

    support_mat = model.material("support_gray", color=(0.19, 0.20, 0.22))
    steel_mat = model.material("steel", color=(0.66, 0.68, 0.71))
    outer_stage_mat = model.material("outer_stage_finish", color=(0.39, 0.42, 0.46))
    mid_stage_mat = model.material("mid_stage_finish", color=(0.56, 0.58, 0.61))
    inner_stage_mat = model.material("inner_stage_finish", color=(0.42, 0.37, 0.31))

    frame = model.part("frame")
    frame_components = make_frame_components()
    frame.visual(
        mesh_from_cadquery(frame_components["top_support"], "top_support"),
        material=support_mat,
        name="top_support",
    )
    frame.visual(
        mesh_from_cadquery(frame_components["center_boss"], "center_boss"),
        material=support_mat,
        name="center_boss",
    )
    frame.visual(
        mesh_from_cadquery(frame_components["shaft"], "center_shaft"),
        material=steel_mat,
        name="shaft",
    )
    frame.visual(
        mesh_from_cadquery(frame_components["collar_outer"], "collar_outer"),
        material=steel_mat,
        name="collar_outer",
    )
    frame.visual(
        mesh_from_cadquery(frame_components["collar_mid"], "collar_mid"),
        material=steel_mat,
        name="collar_mid",
    )
    frame.visual(
        mesh_from_cadquery(frame_components["collar_inner"], "collar_inner"),
        material=steel_mat,
        name="collar_inner",
    )
    frame.visual(
        mesh_from_cadquery(frame_components["tail_cap"], "tail_cap"),
        material=steel_mat,
        name="tail_cap",
    )

    outer_stage = model.part("outer_stage")
    outer_stage.visual(
        make_stage_mesh(
            outer_radius=0.205,
            rim_width=0.028,
            tab_angle_deg=0.0,
            tab_length=0.030,
            tab_width=0.016,
            mesh_name="outer_stage_shell",
        ),
        material=outer_stage_mat,
        name="outer_stage_shell",
    )

    mid_stage = model.part("mid_stage")
    mid_stage.visual(
        make_stage_mesh(
            outer_radius=0.160,
            rim_width=0.026,
            tab_angle_deg=132.0,
            tab_length=0.026,
            tab_width=0.014,
            mesh_name="mid_stage_shell",
        ),
        material=mid_stage_mat,
        name="mid_stage_shell",
    )

    inner_stage = model.part("inner_stage")
    inner_stage.visual(
        make_stage_mesh(
            outer_radius=0.112,
            rim_width=0.024,
            tab_angle_deg=252.0,
            tab_length=0.022,
            tab_width=0.012,
            mesh_name="inner_stage_shell",
        ),
        material=inner_stage_mat,
        name="inner_stage_shell",
    )

    stage_limits = MotionLimits(effort=12.0, velocity=2.5, lower=-pi, upper=pi)
    model.articulation(
        "frame_to_outer_stage",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=outer_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=stage_limits,
    )
    model.articulation(
        "frame_to_mid_stage",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=mid_stage,
        origin=Origin(xyz=(0.0, 0.0, MID_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-pi, upper=pi),
    )
    model.articulation(
        "frame_to_inner_stage",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    outer_stage = object_model.get_part("outer_stage")
    mid_stage = object_model.get_part("mid_stage")
    inner_stage = object_model.get_part("inner_stage")

    outer_joint = object_model.get_articulation("frame_to_outer_stage")
    mid_joint = object_model.get_articulation("frame_to_mid_stage")
    inner_joint = object_model.get_articulation("frame_to_inner_stage")

    outer_collar = frame.get_visual("collar_outer")
    mid_collar = frame.get_visual("collar_mid")
    inner_collar = frame.get_visual("collar_inner")

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

    shared_axis_ok = all(j.axis == (0.0, 0.0, 1.0) for j in (outer_joint, mid_joint, inner_joint))
    shared_xy_ok = len({j.origin.xyz[:2] for j in (outer_joint, mid_joint, inner_joint)}) == 1
    stacked_z_ok = outer_joint.origin.xyz[2] > mid_joint.origin.xyz[2] > inner_joint.origin.xyz[2]
    ctx.check(
        "coaxial_stage_articulations_share_one_vertical_axis",
        shared_axis_ok and shared_xy_ok and stacked_z_ok,
        details=(
            f"axes={[outer_joint.axis, mid_joint.axis, inner_joint.axis]}, "
            f"origins={[outer_joint.origin.xyz, mid_joint.origin.xyz, inner_joint.origin.xyz]}"
        ),
    )

    ctx.expect_contact(outer_stage, frame, elem_b=outer_collar, name="outer_stage_is_carried_by_outer_collar")
    ctx.expect_contact(mid_stage, frame, elem_b=mid_collar, name="mid_stage_is_carried_by_mid_collar")
    ctx.expect_contact(inner_stage, frame, elem_b=inner_collar, name="inner_stage_is_carried_by_inner_collar")

    ctx.expect_gap(outer_stage, mid_stage, axis="z", min_gap=0.05, name="outer_stage_clears_mid_stage")
    ctx.expect_gap(mid_stage, inner_stage, axis="z", min_gap=0.05, name="mid_stage_clears_inner_stage")

    with ctx.pose({outer_joint: 0.95, mid_joint: -1.10, inner_joint: 1.45}):
        ctx.expect_contact(
            outer_stage,
            frame,
            elem_b=outer_collar,
            name="outer_stage_remains_hung_when_rotated",
        )
        ctx.expect_contact(
            mid_stage,
            frame,
            elem_b=mid_collar,
            name="mid_stage_remains_hung_when_rotated",
        )
        ctx.expect_contact(
            inner_stage,
            frame,
            elem_b=inner_collar,
            name="inner_stage_remains_hung_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
