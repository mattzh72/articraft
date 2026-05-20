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


CHEEK_PLATE_THICKNESS = 0.008
CHEEK_BOSS_EXTRA = 0.004
BOSS_FUSE_OVERLAP = 0.0005
CHEEK_TOTAL_THICKNESS = CHEEK_PLATE_THICKNESS + CHEEK_BOSS_EXTRA - BOSS_FUSE_OVERLAP

LINK_PLATE_THICKNESS = 0.005
LINK_BOSS_EXTRA = 0.003
LINK_TOTAL_THICKNESS = LINK_PLATE_THICKNESS + LINK_BOSS_EXTRA - BOSS_FUSE_OVERLAP

TAB_PLATE_THICKNESS = 0.0045
TAB_BOSS_EXTRA = 0.0025
TAB_TOTAL_THICKNESS = TAB_PLATE_THICKNESS + TAB_BOSS_EXTRA - BOSS_FUSE_OVERLAP

LINK_A_SPAN = 0.074
LINK_B_SPAN = 0.064
TAB_SPAN = 0.038

LINK_WEB_WIDTH = 0.016
LINK_BOSS_DIAMETER = 0.026
LINK_PAD_DIAMETER = 0.030
TAB_TIP_DIAMETER = 0.018
CHEEK_PIVOT_DIAMETER = 0.036
CHEEK_PAD_DIAMETER = 0.030


def _disc_on_xz(x: float, z: float, radius: float, thickness: float) -> cq.Workplane:
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(thickness).translate((0.0, thickness, 0.0))


def _rect_on_xz(
    center_x: float, center_z: float, size_x: float, size_z: float, thickness: float
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .rect(size_x, size_z)
        .extrude(thickness)
        .translate((0.0, thickness, 0.0))
    )


def _link_shape(span: float, web_width: float, plate_t: float, boss_extra: float) -> cq.Workplane:
    boss_r = LINK_BOSS_DIAMETER * 0.5
    pad_r = LINK_PAD_DIAMETER * 0.5

    body = _rect_on_xz(span * 0.5, 0.0, span, web_width, plate_t)
    body = body.union(_disc_on_xz(0.0, 0.0, boss_r, plate_t))
    body = body.union(_disc_on_xz(span, 0.0, boss_r, plate_t))

    first_pad = _disc_on_xz(0.0, 0.0, pad_r, boss_extra).translate((0.0, plate_t - BOSS_FUSE_OVERLAP, 0.0))
    second_pad = _disc_on_xz(span, 0.0, pad_r, boss_extra).translate((0.0, plate_t - BOSS_FUSE_OVERLAP, 0.0))
    return body.union(first_pad).union(second_pad)


def _end_tab_shape() -> cq.Workplane:
    pivot_r = LINK_BOSS_DIAMETER * 0.5
    pad_r = LINK_PAD_DIAMETER * 0.5
    tip_r = TAB_TIP_DIAMETER * 0.5

    body = _rect_on_xz(TAB_SPAN * 0.5, 0.0, TAB_SPAN, 0.014, TAB_PLATE_THICKNESS)
    body = body.union(_disc_on_xz(0.0, 0.0, pivot_r, TAB_PLATE_THICKNESS))
    body = body.union(_disc_on_xz(TAB_SPAN, 0.0, tip_r, TAB_PLATE_THICKNESS))
    pivot_pad = _disc_on_xz(0.0, 0.0, pad_r, TAB_BOSS_EXTRA).translate(
        (0.0, TAB_PLATE_THICKNESS - BOSS_FUSE_OVERLAP, 0.0)
    )
    return body.union(pivot_pad)


def _cheek_shape() -> cq.Workplane:
    pivot_r = CHEEK_PIVOT_DIAMETER * 0.5
    pad_r = CHEEK_PAD_DIAMETER * 0.5

    pivot = _disc_on_xz(0.0, 0.0, pivot_r, CHEEK_PLATE_THICKNESS)
    rear = _disc_on_xz(-0.055, 0.0, 0.015, CHEEK_PLATE_THICKNESS)
    upper = _disc_on_xz(-0.024, 0.028, 0.012, CHEEK_PLATE_THICKNESS)
    lower = _disc_on_xz(-0.032, -0.030, 0.014, CHEEK_PLATE_THICKNESS)

    spine = _rect_on_xz(-0.028, 0.0, 0.056, 0.024, CHEEK_PLATE_THICKNESS)
    upper_web = _rect_on_xz(-0.037, 0.015, 0.030, 0.022, CHEEK_PLATE_THICKNESS)
    lower_web = _rect_on_xz(-0.041, -0.016, 0.028, 0.030, CHEEK_PLATE_THICKNESS)

    cheek = pivot.union(rear).union(upper).union(lower).union(spine).union(upper_web).union(lower_web)
    pivot_pad = _disc_on_xz(0.0, 0.0, pad_r, CHEEK_BOSS_EXTRA).translate(
        (0.0, CHEEK_PLATE_THICKNESS - BOSS_FUSE_OVERLAP, 0.0)
    )
    return cheek.union(pivot_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_revolute_chain")

    cheek_mat = model.material("cheek_steel", color=(0.35, 0.37, 0.40))
    link_mat = model.material("link_steel", color=(0.72, 0.74, 0.76))
    tab_mat = model.material("end_tab_finish", color=(0.20, 0.22, 0.24))

    cheek = model.part("cheek")
    cheek.visual(
        mesh_from_cadquery(_cheek_shape(), "cheek_body"),
        material=cheek_mat,
        name="body",
    )

    link_a = model.part("link_a")
    link_a.visual(
        mesh_from_cadquery(
            _link_shape(LINK_A_SPAN, LINK_WEB_WIDTH, LINK_PLATE_THICKNESS, LINK_BOSS_EXTRA),
            "link_a_body",
        ),
        material=link_mat,
        name="body",
    )

    link_b = model.part("link_b")
    link_b.visual(
        mesh_from_cadquery(
            _link_shape(LINK_B_SPAN, LINK_WEB_WIDTH * 0.95, LINK_PLATE_THICKNESS, LINK_BOSS_EXTRA),
            "link_b_body",
        ),
        material=link_mat,
        name="body",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_end_tab_shape(), "end_tab_body"),
        material=tab_mat,
        name="body",
    )

    model.articulation(
        "cheek_to_link_a",
        ArticulationType.REVOLUTE,
        parent=cheek,
        child=link_a,
        origin=Origin(xyz=(0.0, CHEEK_TOTAL_THICKNESS, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-2.2, upper=2.2),
    )
    model.articulation(
        "link_a_to_link_b",
        ArticulationType.REVOLUTE,
        parent=link_a,
        child=link_b,
        origin=Origin(xyz=(LINK_A_SPAN, LINK_TOTAL_THICKNESS, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "link_b_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_b,
        child=end_tab,
        origin=Origin(xyz=(LINK_B_SPAN, LINK_TOTAL_THICKNESS, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-2.6, upper=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cheek = object_model.get_part("cheek")
    link_a = object_model.get_part("link_a")
    link_b = object_model.get_part("link_b")
    end_tab = object_model.get_part("end_tab")

    joint_a = object_model.get_articulation("cheek_to_link_a")
    joint_b = object_model.get_articulation("link_a_to_link_b")
    joint_c = object_model.get_articulation("link_b_to_end_tab")

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

    joint_axes_parallel = all(joint.axis == (0.0, -1.0, 0.0) for joint in (joint_a, joint_b, joint_c))
    ctx.check(
        "joint axes are parallel supported y-axes",
        joint_axes_parallel,
        f"expected all axes to be (0, -1, 0), got {[joint.axis for joint in (joint_a, joint_b, joint_c)]}",
    )

    ctx.expect_contact(cheek, link_a, name="cheek supports first link at the first boss")
    ctx.expect_contact(link_a, link_b, name="first link supports second link at the second boss")
    ctx.expect_contact(link_b, end_tab, name="second link supports end tab at the third boss")

    tol = 1e-6
    ctx.expect_origin_gap(
        link_a,
        cheek,
        axis="y",
        min_gap=CHEEK_TOTAL_THICKNESS - tol,
        max_gap=CHEEK_TOTAL_THICKNESS + tol,
        name="first link is layered just outside the cheek",
    )
    ctx.expect_origin_gap(
        link_b,
        link_a,
        axis="y",
        min_gap=LINK_TOTAL_THICKNESS - tol,
        max_gap=LINK_TOTAL_THICKNESS + tol,
        name="second link is layered outside the first link",
    )
    ctx.expect_origin_gap(
        end_tab,
        link_b,
        axis="y",
        min_gap=LINK_TOTAL_THICKNESS - tol,
        max_gap=LINK_TOTAL_THICKNESS + tol,
        name="end tab is layered outside the second link",
    )

    with ctx.pose({joint_a: 0.0}):
        rest_tip = ctx.part_world_position(end_tab)
    with ctx.pose({joint_a: 0.6}):
        raised_tip = ctx.part_world_position(end_tab)
    ctx.check(
        "positive first-joint rotation lifts the chain",
        rest_tip is not None and raised_tip is not None and raised_tip[2] > rest_tip[2] + 0.04,
        f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
