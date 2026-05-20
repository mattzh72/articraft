from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


JOINT_GAP = 0.088
CHEEK_THICKNESS = 0.020
OUTER_BOSS_THICKNESS = 0.012
HUB_RADIUS = 0.034
PIN_BOSS_RADIUS = 0.044
CHEEK_LENGTH = 0.090

LINK1_LENGTH = 0.460
LINK2_LENGTH = 0.360
LINK3_BODY_LENGTH = 0.225


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] + (length / 2.0), center[2]))
    )


def _y_profile_prism(
    points: list[tuple[float, float]],
    thickness: float,
    center_y: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, center_y + (thickness / 2.0), 0.0))
    )


def _outer_boss_y() -> float:
    return (JOINT_GAP / 2.0) + CHEEK_THICKNESS + (OUTER_BOSS_THICKNESS / 2.0) - 0.001


def _half_y_cylinder(
    radius: float,
    length: float,
    axis_x: float,
    center_y: float,
    side: str,
) -> cq.Workplane:
    cylinder = _y_cylinder(radius, length, (axis_x, center_y, 0.0))
    clip_length_x = radius + 0.004
    clip = cq.Workplane("XY").box(clip_length_x, length + 0.010, (2.0 * radius) + 0.012)
    x_sign = 1.0 if side == "positive" else -1.0
    clip = clip.translate((axis_x + (x_sign * clip_length_x / 2.0), center_y, 0.0))
    return cylinder.intersect(clip)


def _root_lug(root_length: float, beam_depth: float) -> cq.Workplane:
    lug_pad = _half_y_cylinder(HUB_RADIUS, JOINT_GAP, 0.0, 0.0, "positive")
    lug_block = cq.Workplane("XY").box(root_length, JOINT_GAP, beam_depth).translate(
        (root_length / 2.0, 0.0, 0.0)
    )
    return lug_pad.union(lug_block)


def _make_base() -> cq.Workplane:
    outer_cheek_y = (JOINT_GAP / 2.0) + (CHEEK_THICKNESS / 2.0)
    outer_boss_y = _outer_boss_y()

    pedestal = cq.Workplane("XY").box(0.230, 0.190, 0.100).translate((-0.125, 0.0, -0.120))
    top_saddle = cq.Workplane("XY").box(0.130, 0.150, 0.035).translate((-0.075, 0.0, -0.055))
    rear_tie = cq.Workplane("XY").box(0.070, JOINT_GAP, 0.025).translate((-0.095, 0.0, 0.038))

    left_cheek = cq.Workplane("XY").box(CHEEK_LENGTH, CHEEK_THICKNESS, 0.160).translate(
        (-CHEEK_LENGTH / 2.0, outer_cheek_y, -0.006)
    )
    right_cheek = cq.Workplane("XY").box(CHEEK_LENGTH, CHEEK_THICKNESS, 0.160).translate(
        (-CHEEK_LENGTH / 2.0, -outer_cheek_y, -0.006)
    )

    gusset_profile = [(-0.155, -0.074), (-0.070, -0.074), (-0.012, -0.010), (-0.095, -0.010)]
    left_gusset = _y_profile_prism(gusset_profile, 0.016, outer_cheek_y)
    right_gusset = _y_profile_prism(gusset_profile, 0.016, -outer_cheek_y)

    left_boss = _half_y_cylinder(PIN_BOSS_RADIUS, OUTER_BOSS_THICKNESS, 0.0, outer_boss_y, "negative")
    right_boss = _half_y_cylinder(PIN_BOSS_RADIUS, OUTER_BOSS_THICKNESS, 0.0, -outer_boss_y, "negative")

    base = pedestal.union(top_saddle)
    base = base.union(rear_tie)
    base = base.union(left_cheek).union(right_cheek)
    base = base.union(left_gusset).union(right_gusset)
    base = base.union(left_boss).union(right_boss)
    return base


def _make_box_link(length: float, beam_width: float, beam_depth: float) -> cq.Workplane:
    outer_cheek_y = (JOINT_GAP / 2.0) + (CHEEK_THICKNESS / 2.0)
    outer_boss_y = _outer_boss_y()
    beam_start_x = 0.052
    beam_end_x = length - 0.026
    beam_length = beam_end_x - beam_start_x
    beam_half_y = beam_width / 2.0
    cheek_inner_y = JOINT_GAP / 2.0
    clevis_height = max(beam_depth + 0.030, (2.0 * HUB_RADIUS) + 0.050)
    clevis_x_length = 0.082
    root_length = 0.062

    proximal_hub = _root_lug(root_length, beam_depth)
    beam = cq.Workplane("XY").box(beam_length, beam_width, beam_depth).translate(
        (beam_start_x + (beam_length / 2.0), 0.0, 0.0)
    )

    left_cheek = cq.Workplane("XY").box(clevis_x_length, CHEEK_THICKNESS, clevis_height).translate(
        (length - (clevis_x_length / 2.0), outer_cheek_y, 0.0)
    )
    right_cheek = cq.Workplane("XY").box(clevis_x_length, CHEEK_THICKNESS, clevis_height).translate(
        (length - (clevis_x_length / 2.0), -outer_cheek_y, 0.0)
    )

    cheek_bridge_thickness = (cheek_inner_y - beam_half_y) + 0.006
    cheek_bridge_center_y = (cheek_inner_y + beam_half_y) / 2.0
    cheek_bridge_length = 0.092
    cheek_bridge_x = length - (cheek_bridge_length / 2.0) - 0.004
    left_bridge = cq.Workplane("XY").box(
        cheek_bridge_length, cheek_bridge_thickness, beam_depth
    ).translate((cheek_bridge_x, cheek_bridge_center_y, 0.0))
    right_bridge = cq.Workplane("XY").box(
        cheek_bridge_length, cheek_bridge_thickness, beam_depth
    ).translate((cheek_bridge_x, -cheek_bridge_center_y, 0.0))

    left_boss = _half_y_cylinder(PIN_BOSS_RADIUS, OUTER_BOSS_THICKNESS, length, outer_boss_y, "negative")
    right_boss = _half_y_cylinder(PIN_BOSS_RADIUS, OUTER_BOSS_THICKNESS, length, -outer_boss_y, "negative")

    link = proximal_hub.union(beam)
    link = link.union(left_cheek).union(right_cheek)
    link = link.union(left_bridge).union(right_bridge)
    link = link.union(left_boss).union(right_boss)
    return link


def _make_terminal_link(body_length: float, beam_width: float, beam_depth: float) -> cq.Workplane:
    beam_start_x = 0.052
    beam_end_x = body_length - 0.004
    beam_length = beam_end_x - beam_start_x
    tab_width = 0.052
    tab_depth = 0.074
    tab_block_length = 0.058
    tab_nose_radius = 0.031
    root_length = 0.062
    tab_center_x = body_length + 0.018
    nose_center_x = body_length + 0.030

    proximal_hub = _root_lug(root_length, beam_depth)
    beam = cq.Workplane("XY").box(beam_length, beam_width, beam_depth).translate(
        (beam_start_x + (beam_length / 2.0), 0.0, 0.0)
    )
    tab_block = cq.Workplane("XY").box(tab_block_length, tab_width, tab_depth).translate(
        (tab_center_x, 0.0, 0.0)
    )
    tab_nose = _y_cylinder(tab_nose_radius, tab_width, (nose_center_x, 0.0, 0.0))
    hole = _y_cylinder(0.013, tab_width + 0.010, (nose_center_x, 0.0, 0.0))

    link = proximal_hub.union(beam).union(tab_block).union(tab_nose)
    return link.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_three_joint_chain")

    base_gray = model.material("base_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    arm_yellow = model.material("arm_yellow", rgba=(0.79, 0.63, 0.18, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base(), "base"), material=base_gray, name="base_shell")

    link1 = model.part("link1")
    link1.visual(
        mesh_from_cadquery(_make_box_link(LINK1_LENGTH, 0.076, 0.126), "link1"),
        material=arm_yellow,
        name="link1_shell",
    )

    link2 = model.part("link2")
    link2.visual(
        mesh_from_cadquery(_make_box_link(LINK2_LENGTH, 0.072, 0.110), "link2"),
        material=arm_yellow,
        name="link2_shell",
    )

    link3 = model.part("link3")
    link3.visual(
        mesh_from_cadquery(_make_terminal_link(LINK3_BODY_LENGTH, 0.060, 0.090), "link3"),
        material=arm_yellow,
        name="link3_shell",
    )

    model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.2, lower=-0.90, upper=1.10),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-1.60, upper=1.25),
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-1.30, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    j1 = object_model.get_articulation("base_to_link1")
    j2 = object_model.get_articulation("link1_to_link2")
    j3 = object_model.get_articulation("link2_to_link3")

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

    ctx.expect_contact(link1, base, name="link1_supported_by_base_cheeks")
    ctx.expect_contact(link2, link1, name="link2_supported_by_link1_clevis")
    ctx.expect_contact(link3, link2, name="link3_supported_by_link2_clevis")

    parallel_revolute = all(
        joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, -1.0, 0.0)
        for joint in (j1, j2, j3)
    )
    ctx.check(
        "parallel_revolute_joint_axes",
        parallel_revolute,
        "All three serial joints should be revolute and share a common -Y hinge axis.",
    )

    link2_rest = ctx.part_world_position(link2)
    link3_rest = ctx.part_world_position(link3)
    reach_ok = (
        link2_rest is not None
        and link3_rest is not None
        and link2_rest[0] > 0.40
        and link3_rest[0] > (link2_rest[0] + 0.25)
        and abs(link2_rest[2]) < 0.02
        and abs(link3_rest[2]) < 0.02
    )
    ctx.check(
        "rest_pose_reads_as_forward_serial_chain",
        reach_ok,
        "The rest pose should extend the links forward in a planar chain.",
    )

    with ctx.pose({j1: 0.55, j2: 0.45, j3: 0.30}):
        link3_raised = ctx.part_world_position(link3)
    lifted_ok = (
        link3_rest is not None
        and link3_raised is not None
        and link3_raised[2] > (link3_rest[2] + 0.22)
        and link3_raised[0] > 0.15
    )
    ctx.check(
        "positive_joint_motion_lifts_terminal_link",
        lifted_ok,
        "Positive revolute motion should raise the chain rather than driving it downward.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
