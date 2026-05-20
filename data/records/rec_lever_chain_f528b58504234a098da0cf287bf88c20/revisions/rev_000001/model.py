from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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


BASE_THICKNESS = 0.010
BASE_LENGTH = 0.080
BASE_WIDTH = 0.040
JOINT_HEIGHT = 0.045

EAR_THICKNESS = 0.004
LINK_THICKNESS = 0.008
FORK_GAP = LINK_THICKNESS
FORK_SPAN = FORK_GAP + 2.0 * EAR_THICKNESS
WEB_WIDTH = 0.012
BOSS_RADIUS = 0.010
PIN_RADIUS = 0.0035
HOLE_RADIUS = 0.0039
CLEVIS_LENGTH = 0.018

LINK_1_LENGTH = 0.082
LINK_2_LENGTH = 0.070
LINK_3_LENGTH = 0.058


def _y_cylinder(radius: float, length: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _clevis_block(joint_x: float, *, with_pin: bool) -> cq.Workplane:
    block = cq.Workplane("XY").box(CLEVIS_LENGTH, FORK_SPAN, WEB_WIDTH).translate((joint_x - CLEVIS_LENGTH / 2.0, 0.0, 0.0))
    slot = cq.Workplane("XY").box(CLEVIS_LENGTH - 0.003, FORK_GAP, WEB_WIDTH + 0.004).translate(
        (joint_x - (CLEVIS_LENGTH - 0.003) / 2.0, 0.0, 0.0)
    )
    clevis = block.cut(slot).cut(_y_cylinder(HOLE_RADIUS, FORK_SPAN + 0.004, center=(joint_x, 0.0, 0.0)))
    if with_pin:
        clevis = clevis.union(_y_cylinder(PIN_RADIUS, FORK_SPAN, center=(joint_x, 0.0, 0.0)))
    return clevis


def _make_clevis_link(length: float) -> cq.Workplane:
    web_start = BOSS_RADIUS * 0.80
    web_end = length - CLEVIS_LENGTH + 0.001
    web_length = max(web_end - web_start, 0.012)

    root = _y_cylinder(BOSS_RADIUS, LINK_THICKNESS, center=(0.0, 0.0, 0.0)).cut(
        _y_cylinder(HOLE_RADIUS, LINK_THICKNESS + 0.004, center=(0.0, 0.0, 0.0))
    )
    web = cq.Workplane("XY").box(web_length, LINK_THICKNESS, WEB_WIDTH).translate((web_start + web_length / 2.0, 0.0, 0.0))
    lightening_cut = cq.Workplane("XY").box(max(web_length - 0.020, 0.010), LINK_THICKNESS + 0.002, 0.004).translate(
        (web_start + web_length / 2.0, 0.0, 0.0)
    )
    return root.union(web).union(_clevis_block(length, with_pin=True)).cut(lightening_cut)


def _make_terminal_link(length: float) -> cq.Workplane:
    web_start = BOSS_RADIUS * 0.80
    pad_length = 0.014
    web_end = length - pad_length + 0.001
    web_length = max(web_end - web_start, 0.012)

    root = _y_cylinder(BOSS_RADIUS, LINK_THICKNESS, center=(0.0, 0.0, 0.0)).cut(
        _y_cylinder(HOLE_RADIUS, LINK_THICKNESS + 0.004, center=(0.0, 0.0, 0.0))
    )
    web = cq.Workplane("XY").box(web_length, LINK_THICKNESS, WEB_WIDTH).translate((web_start + web_length / 2.0, 0.0, 0.0))
    pad = cq.Workplane("XY").box(pad_length, LINK_THICKNESS, 0.016).translate((length - pad_length / 2.0, 0.0, 0.0))
    return root.union(web).union(pad)


def _make_end_tab() -> cq.Workplane:
    mount = cq.Workplane("XY").box(0.006, LINK_THICKNESS, 0.016).translate((0.003, 0.0, 0.0))
    flag = cq.Workplane("XY").box(0.028, LINK_THICKNESS, 0.018).translate((0.020, 0.0, 0.0))
    tip_hole = _y_cylinder(0.0032, LINK_THICKNESS + 0.003, center=(0.029, 0.0, 0.0))
    return mount.union(flag).cut(tip_hole)


def _make_base_lug() -> cq.Workplane:
    foot = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS).translate((-0.025, 0.0, BASE_THICKNESS / 2.0))
    foot = foot.cut(
        cq.Workplane("XY")
        .pushPoints([(-0.045, 0.0), (-0.005, 0.0)])
        .circle(0.004)
        .extrude(BASE_THICKNESS + 0.002)
        .translate((0.0, 0.0, -0.001))
    )

    post_height = JOINT_HEIGHT - BASE_THICKNESS - WEB_WIDTH / 2.0
    post = cq.Workplane("XY").box(0.014, 0.016, post_height).translate((-0.018, 0.0, BASE_THICKNESS + post_height / 2.0))
    brace = (
        cq.Workplane("XZ")
        .moveTo(-0.040, BASE_THICKNESS)
        .lineTo(-0.040, BASE_THICKNESS + 0.010)
        .lineTo(-0.018, JOINT_HEIGHT - WEB_WIDTH / 2.0)
        .lineTo(-0.012, JOINT_HEIGHT - WEB_WIDTH / 2.0)
        .lineTo(-0.012, BASE_THICKNESS)
        .close()
        .extrude(0.016 / 2.0, both=True)
    )
    clevis = _clevis_block(0.0, with_pin=True).translate((0.0, 0.0, JOINT_HEIGHT))

    return foot.union(post).union(brace).union(clevis)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_link_lever_chain")

    plate_thickness = 0.008
    ear_thickness = 0.003
    fork_gap = plate_thickness
    fork_outer = fork_gap + 2.0 * ear_thickness
    ear_center_y = fork_gap / 2.0 + ear_thickness / 2.0
    bar_height = 0.012
    boss_radius = 0.006
    bridge_len = 0.010
    ear_len = 0.008
    joint_axis_rpy = (1.5707963267948966, 0.0, 0.0)

    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.16, 0.17, 0.19, 1.0))
    painted_tab = model.material("painted_tab", rgba=(0.73, 0.21, 0.16, 1.0))

    base_lug = model.part("base_lug")
    base_lug.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(-0.025, 0.0, BASE_THICKNESS / 2.0)),
        material=dark_steel,
        name="base_foot",
    )
    base_post_height = JOINT_HEIGHT - BASE_THICKNESS - bar_height / 2.0
    base_lug.visual(
        Box((0.014, 0.014, base_post_height)),
        origin=Origin(xyz=(-0.020, 0.0, BASE_THICKNESS + base_post_height / 2.0)),
        material=dark_steel,
        name="base_post",
    )
    base_lug.visual(
        Box((bridge_len, fork_outer, bar_height)),
        origin=Origin(xyz=(-0.0125, 0.0, JOINT_HEIGHT)),
        material=dark_steel,
        name="base_bridge",
    )
    base_lug.visual(
        Box((ear_len, ear_thickness, bar_height)),
        origin=Origin(xyz=(-0.004, -ear_center_y, JOINT_HEIGHT)),
        material=dark_steel,
        name="base_ear_left",
    )
    base_lug.visual(
        Box((ear_len, ear_thickness, bar_height)),
        origin=Origin(xyz=(-0.004, ear_center_y, JOINT_HEIGHT)),
        material=dark_steel,
        name="base_ear_right",
    )

    link_1 = model.part("link_1")
    for part_obj, length, material, prefix, terminal in (
        (link_1, LINK_1_LENGTH, satin_steel, "link_1", False),
    ):
        bar_len = length - boss_radius - bridge_len - ear_len
        part_obj.visual(
            Cylinder(radius=boss_radius, length=plate_thickness),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=joint_axis_rpy),
            material=material,
            name=f"{prefix}_root_boss",
        )
        part_obj.visual(
            Box((bar_len, plate_thickness, bar_height)),
            origin=Origin(xyz=(boss_radius + bar_len / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_bar",
        )
        part_obj.visual(
            Box((bridge_len, fork_outer, bar_height)),
            origin=Origin(xyz=(length - ear_len - bridge_len / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_bridge",
        )
        part_obj.visual(
            Box((ear_len, ear_thickness, bar_height)),
            origin=Origin(xyz=(length - ear_len / 2.0, -ear_center_y, 0.0)),
            material=material,
            name=f"{prefix}_ear_left",
        )
        part_obj.visual(
            Box((ear_len, ear_thickness, bar_height)),
            origin=Origin(xyz=(length - ear_len / 2.0, ear_center_y, 0.0)),
            material=material,
            name=f"{prefix}_ear_right",
        )

    link_2 = model.part("link_2")
    for part_obj, length, material, prefix, terminal in (
        (link_2, LINK_2_LENGTH, black_oxide, "link_2", False),
    ):
        bar_len = length - boss_radius - bridge_len - ear_len
        part_obj.visual(
            Cylinder(radius=boss_radius, length=plate_thickness),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=joint_axis_rpy),
            material=material,
            name=f"{prefix}_root_boss",
        )
        part_obj.visual(
            Box((bar_len, plate_thickness, bar_height)),
            origin=Origin(xyz=(boss_radius + bar_len / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_bar",
        )
        part_obj.visual(
            Box((bridge_len, fork_outer, bar_height)),
            origin=Origin(xyz=(length - ear_len - bridge_len / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_bridge",
        )
        part_obj.visual(
            Box((ear_len, ear_thickness, bar_height)),
            origin=Origin(xyz=(length - ear_len / 2.0, -ear_center_y, 0.0)),
            material=material,
            name=f"{prefix}_ear_left",
        )
        part_obj.visual(
            Box((ear_len, ear_thickness, bar_height)),
            origin=Origin(xyz=(length - ear_len / 2.0, ear_center_y, 0.0)),
            material=material,
            name=f"{prefix}_ear_right",
        )

    link_3 = model.part("link_3")
    link_3.visual(
        Cylinder(radius=boss_radius, length=plate_thickness),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=joint_axis_rpy),
        material=satin_steel,
        name="link_3_root_boss",
    )
    tip_len = 0.014
    link_3.visual(
        Box((LINK_3_LENGTH - boss_radius - tip_len, plate_thickness, bar_height)),
        origin=Origin(xyz=(boss_radius + (LINK_3_LENGTH - boss_radius - tip_len) / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="link_3_bar",
    )
    link_3.visual(
        Box((tip_len, plate_thickness, 0.016)),
        origin=Origin(xyz=(LINK_3_LENGTH - tip_len / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="link_3_tip",
    )
    link_3.visual(
        Box((0.010, plate_thickness, 0.010)),
        origin=Origin(xyz=(LINK_3_LENGTH + 0.005, 0.0, 0.0)),
        material=painted_tab,
        name="link_3_tab_stem",
    )
    link_3.visual(
        Box((0.022, plate_thickness, 0.018)),
        origin=Origin(xyz=(LINK_3_LENGTH + 0.018, 0.0, 0.0)),
        material=painted_tab,
        name="link_3_tab_flag",
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base_lug,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, JOINT_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.90),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=3.0, lower=-1.00, upper=1.00),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.95, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_lug = object_model.get_part("base_lug")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_3_tab_flag = link_3.get_visual("link_3_tab_flag")

    base_to_link_1 = object_model.get_articulation("base_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")

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

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (base_lug, link_1, link_2, link_3)),
        "Expected base lug and three lever links.",
    )

    for joint in (base_to_link_1, link_1_to_link_2, link_2_to_link_3):
        ctx.check(
            f"{joint.name}_is_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"{joint.name} should be a revolute joint.",
        )
        ctx.check(
            f"{joint.name}_axis_parallel",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis should be parallel to world Y.",
        )

    ctx.check(
        "free_end_tab_present",
        link_3_tab_flag is not None,
        "The last lever should carry a visible free-end tab.",
    )

    with ctx.pose({base_to_link_1: 0.0, link_1_to_link_2: 0.0, link_2_to_link_3: 0.0}):
        ctx.expect_contact(base_lug, link_1, name="base_lug_supports_link_1")
        ctx.expect_contact(link_1, link_2, name="link_1_connects_to_link_2")
        ctx.expect_contact(link_2, link_3, name="link_2_connects_to_link_3")

        ctx.expect_origin_gap(
            link_1,
            base_lug,
            axis="z",
            min_gap=JOINT_HEIGHT - 0.0005,
            max_gap=JOINT_HEIGHT + 0.0005,
            name="first_joint_height",
        )
        ctx.expect_origin_gap(
            link_3,
            base_lug,
            axis="x",
            min_gap=LINK_1_LENGTH + LINK_2_LENGTH - 0.0005,
            max_gap=LINK_1_LENGTH + LINK_2_LENGTH + 0.0005,
            name="third_link_root_reach",
        )
        ctx.expect_origin_distance(link_1, link_2, axes="z", max_dist=0.0005, name="link_1_link_2_coplanar_height")
        ctx.expect_origin_distance(link_2, link_3, axes="z", max_dist=0.0005, name="link_2_link_3_coplanar_height")

        tab_aabb = ctx.part_element_world_aabb(link_3, elem="link_3_tab_flag")
        ctx.check(
            "neutral_tab_extends_past_last_joint",
            tab_aabb is not None and tab_aabb[1][0] >= LINK_1_LENGTH + LINK_2_LENGTH + LINK_3_LENGTH + 0.020,
            "Free-end tab should visibly project beyond the third link's distal joint.",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
    for joint in (base_to_link_1, link_1_to_link_2, link_2_to_link_3):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=False,
    )
    ctx.fail_if_isolated_parts(max_pose_samples=16, name="sampled_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
