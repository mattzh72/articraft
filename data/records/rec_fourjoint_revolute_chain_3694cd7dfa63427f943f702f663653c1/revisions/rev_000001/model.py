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


LINK_PLATE_THICKNESS = 0.006
LINK_BOSS_PROUD = 0.003
CHEEK_THICKNESS = 0.008
CHEEK_BOSS_PROUD = 0.004
PIN_RADIUS = 0.0048
LINK_BOSS_RADIUS = 0.017
CHEEK_BOSS_RADIUS = 0.022
RAISED_BOSS_RADIUS = 0.012
WEB_WIDTH = 0.018
TAB_WIDTH = 0.015
LINK_LENGTHS = (0.092, 0.082, 0.072)
TAB_LENGTH = 0.046


def _extrude_on_side(
    profile: cq.Workplane,
    thickness: float,
    side: int,
    *,
    offset: float = 0.0,
) -> cq.Workplane:
    solid = profile.extrude(thickness)
    if side > 0:
        y_min = offset
    else:
        y_min = -(offset + thickness)
    return solid.translate((0.0, y_min + thickness, 0.0))


def _disk_profile(x: float, z: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XZ").center(x, z).circle(radius)


def _rect_profile(cx: float, cz: float, sx: float, sz: float) -> cq.Workplane:
    return cq.Workplane("XZ").center(cx, cz).rect(sx, sz)


def _through_hole(x: float, z: float, radius: float, depth: float = 0.05) -> cq.Workplane:
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(depth).translate((0.0, -depth / 2.0, 0.0))


def _box_cut(cx: float, cz: float, sx: float, sz: float, depth: float = 0.05) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, depth, sz).translate((cx, 0.0, cz))


def _make_link(length: float, side: int) -> cq.Workplane:
    prox_boss = _extrude_on_side(_disk_profile(0.0, 0.0, LINK_BOSS_RADIUS), LINK_PLATE_THICKNESS, side)
    dist_boss = _extrude_on_side(_disk_profile(length, 0.0, LINK_BOSS_RADIUS), LINK_PLATE_THICKNESS, side)
    web = _extrude_on_side(
        _rect_profile(length / 2.0, 0.0, length - 1.1 * LINK_BOSS_RADIUS, WEB_WIDTH),
        LINK_PLATE_THICKNESS,
        side,
    )
    prox_raised = _extrude_on_side(
        _disk_profile(0.0, 0.0, RAISED_BOSS_RADIUS),
        LINK_BOSS_PROUD,
        side,
        offset=LINK_PLATE_THICKNESS,
    )
    dist_raised = _extrude_on_side(
        _disk_profile(length, 0.0, RAISED_BOSS_RADIUS),
        LINK_BOSS_PROUD,
        side,
        offset=LINK_PLATE_THICKNESS,
    )

    link = prox_boss.union(dist_boss).union(web).union(prox_raised).union(dist_raised)

    slot_length = max(length - (2.35 * LINK_BOSS_RADIUS), 0.012)
    link = link.cut(_through_hole(0.0, 0.0, PIN_RADIUS))
    link = link.cut(_through_hole(length, 0.0, PIN_RADIUS))
    link = link.cut(_box_cut(length / 2.0, 0.0, slot_length, WEB_WIDTH * 0.45))
    return link


def _make_end_tab(length: float, side: int) -> cq.Workplane:
    tip_radius = 0.010
    root_boss = _extrude_on_side(_disk_profile(0.0, 0.0, LINK_BOSS_RADIUS), LINK_PLATE_THICKNESS, side)
    tab_web = _extrude_on_side(
        _rect_profile(length / 2.0, 0.0, length, TAB_WIDTH),
        LINK_PLATE_THICKNESS,
        side,
    )
    tip = _extrude_on_side(_disk_profile(length, 0.0, tip_radius), LINK_PLATE_THICKNESS, side)
    raised = _extrude_on_side(
        _disk_profile(0.0, 0.0, RAISED_BOSS_RADIUS),
        LINK_BOSS_PROUD,
        side,
        offset=LINK_PLATE_THICKNESS,
    )

    tab = root_boss.union(tab_web).union(tip).union(raised)
    tab = tab.cut(_through_hole(0.0, 0.0, PIN_RADIUS))
    tab = tab.cut(_through_hole(length * 0.72, 0.0, 0.0042))
    return tab


def _make_cheek() -> cq.Workplane:
    joint_boss = _extrude_on_side(_disk_profile(0.0, 0.0, CHEEK_BOSS_RADIUS), CHEEK_THICKNESS, 1)
    upright = _extrude_on_side(_rect_profile(-0.018, -0.034, 0.046, 0.092), CHEEK_THICKNESS, 1)
    foot = _extrude_on_side(_rect_profile(-0.032, -0.072, 0.084, 0.024), CHEEK_THICKNESS, 1)
    shoulder = _extrude_on_side(_rect_profile(-0.004, -0.010, 0.032, 0.030), CHEEK_THICKNESS, 1)
    raised = _extrude_on_side(
        _disk_profile(0.0, 0.0, 0.015),
        CHEEK_BOSS_PROUD,
        1,
        offset=CHEEK_THICKNESS,
    )

    cheek = joint_boss.union(upright).union(foot).union(shoulder).union(raised)
    cheek = cheek.cut(_through_hole(0.0, 0.0, PIN_RADIUS))
    cheek = cheek.cut(_through_hole(-0.046, -0.072, 0.0045))
    cheek = cheek.cut(_through_hole(-0.018, -0.072, 0.0045))
    cheek = cheek.cut(_box_cut(-0.024, -0.032, 0.014, 0.030))
    return cheek


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_four_joint_chain")

    cheek_finish = model.material("cheek_finish", rgba=(0.21, 0.23, 0.26, 1.0))
    link_finish = model.material("link_finish", rgba=(0.67, 0.69, 0.72, 1.0))
    tab_finish = model.material("tab_finish", rgba=(0.44, 0.46, 0.49, 1.0))

    cheek = model.part("cheek")
    cheek.visual(
        mesh_from_cadquery(_make_cheek(), "cheek_plate"),
        material=cheek_finish,
        name="body",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_make_link(LINK_LENGTHS[0], side=-1), "link_1_plate"),
        material=link_finish,
        name="body",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_make_link(LINK_LENGTHS[1], side=1), "link_2_plate"),
        material=link_finish,
        name="body",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_make_link(LINK_LENGTHS[2], side=-1), "link_3_plate"),
        material=link_finish,
        name="body",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_make_end_tab(TAB_LENGTH, side=1), "end_tab_plate"),
        material=tab_finish,
        name="body",
    )

    common_limits = MotionLimits(effort=12.0, velocity=2.5, lower=-1.6, upper=1.6)

    model.articulation(
        "joint_1",
        ArticulationType.REVOLUTE,
        parent=cheek,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "joint_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "joint_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "joint_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(LINK_LENGTHS[2], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cheek = object_model.get_part("cheek")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_tab = object_model.get_part("end_tab")
    joint_1 = object_model.get_articulation("joint_1")
    joint_2 = object_model.get_articulation("joint_2")
    joint_3 = object_model.get_articulation("joint_3")
    joint_4 = object_model.get_articulation("joint_4")

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

    for parent, child, label in (
        (cheek, link_1, "joint_1_support_contact"),
        (link_1, link_2, "joint_2_support_contact"),
        (link_2, link_3, "joint_3_support_contact"),
        (link_3, end_tab, "joint_4_support_contact"),
    ):
        ctx.expect_contact(parent, child, name=label)

    joints = (joint_1, joint_2, joint_3, joint_4)
    ctx.check(
        "four_parallel_revolute_axes",
        all(j.articulation_type == ArticulationType.REVOLUTE and tuple(j.axis) == (0.0, -1.0, 0.0) for j in joints),
        "all four joints should be revolute and share the same supported -Y axis",
    )
    ctx.check(
        "joint_limits_allow_bidirectional_bend",
        all(
            j.motion_limits is not None
            and j.motion_limits.lower is not None
            and j.motion_limits.upper is not None
            and j.motion_limits.lower < 0.0 < j.motion_limits.upper
            for j in joints
        ),
        "each joint should bend both forward and back from the straight pose",
    )

    closed_end_tab = ctx.part_world_position(end_tab)
    closed_end_tab_aabb = ctx.part_world_aabb(end_tab)
    ctx.check(
        "extended_rest_pose",
        closed_end_tab_aabb is not None and closed_end_tab_aabb[1][0] > sum(LINK_LENGTHS) + TAB_LENGTH * 0.95,
        "the chain should read as a straight extended side-plate assembly in the rest pose",
    )

    with ctx.pose(joint_1=0.85):
        lifted_end_tab = ctx.part_world_position(end_tab)
        ctx.check(
            "joint_1_lifts_outboard_chain",
            closed_end_tab is not None
            and lifted_end_tab is not None
            and lifted_end_tab[2] > closed_end_tab[2] + 0.16,
            "positive motion on the first revolute joint should lift the chain upward",
        )

    with ctx.pose(joint_1=0.55, joint_2=-0.70, joint_3=0.60, joint_4=0.35):
        ctx.expect_contact(cheek, link_1, name="joint_1_contact_bent")
        ctx.expect_contact(link_1, link_2, name="joint_2_contact_bent")
        ctx.expect_contact(link_2, link_3, name="joint_3_contact_bent")
        ctx.expect_contact(link_3, end_tab, name="joint_4_contact_bent")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
