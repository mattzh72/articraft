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


LINK_THICK = 0.012
LINK1_LEN = 0.110
LINK2_LEN = 0.090
END_TAB_LEN = 0.040


def _box(length: float, width: float, height: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _triangular_rib(
    x0: float,
    z0: float,
    x1: float,
    z1: float,
    x2: float,
    z2: float,
    *,
    y_center: float,
    thickness: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline([(x0, z0), (x1, z1), (x2, z2)])
        .close()
        .extrude(thickness)
        .translate((0.0, y_center - thickness / 2.0, 0.0))
    )


def make_root_bracket() -> cq.Workplane:
    base = _box(0.090, 0.060, 0.010, center=(-0.006, 0.0, -0.063))
    upright = _box(0.014, 0.050, 0.054, center=(-0.030, 0.0, -0.031))
    saddle = _box(0.040, 0.034, 0.006, center=(0.000, 0.0, -0.003))
    front_pad = _box(0.014, 0.028, 0.010, center=(0.018, 0.0, -0.005))
    left_rib = _triangular_rib(-0.030, -0.006, -0.030, -0.050, 0.000, -0.006, y_center=0.016, thickness=0.006)
    right_rib = _triangular_rib(-0.030, -0.006, -0.030, -0.050, 0.000, -0.006, y_center=-0.022, thickness=0.006)

    bracket = base.union(upright).union(saddle).union(front_pad).union(left_rib).union(right_rib)

    for x_hole in (-0.028, 0.016):
        for y_hole in (-0.017, 0.017):
            cutter = cq.Workplane("XY").circle(0.0032).extrude(0.012).translate((x_hole, y_hole, -0.070))
            bracket = bracket.cut(cutter)

    return bracket


def make_link1() -> cq.Workplane:
    base_block = _box(0.028, 0.028, LINK_THICK, center=(0.014, 0.0, 0.0))
    arm = _box(LINK1_LEN - 0.028, 0.020, LINK_THICK, center=(0.028 + (LINK1_LEN - 0.028) / 2.0, 0.0, 0.0))
    left_ear = _box(0.012, 0.007, 0.018, center=(LINK1_LEN - 0.006, 0.0085, 0.0))
    right_ear = _box(0.012, 0.007, 0.018, center=(LINK1_LEN - 0.006, -0.0085, 0.0))
    return base_block.union(arm).union(left_ear).union(right_ear)


def make_link2() -> cq.Workplane:
    proximal_lug = _box(0.012, 0.009, 0.016, center=(0.006, 0.0, 0.0))
    arm = _box(LINK2_LEN - 0.012, 0.018, LINK_THICK, center=(0.012 + (LINK2_LEN - 0.012) / 2.0, 0.0, 0.0))
    left_ear = _box(0.012, 0.007, 0.018, center=(LINK2_LEN - 0.006, 0.0085, 0.0))
    right_ear = _box(0.012, 0.007, 0.018, center=(LINK2_LEN - 0.006, -0.0085, 0.0))
    return proximal_lug.union(arm).union(left_ear).union(right_ear)


def make_end_tab() -> cq.Workplane:
    proximal_lug = _box(0.012, 0.009, 0.016, center=(0.006, 0.0, 0.0))
    tab = _box(END_TAB_LEN - 0.012, 0.020, LINK_THICK, center=(0.012 + (END_TAB_LEN - 0.012) / 2.0, 0.0, 0.0))
    tip = _box(0.012, 0.024, LINK_THICK, center=(END_TAB_LEN - 0.006, 0.0, 0.0))
    end_tab = proximal_lug.union(tab).union(tip)
    hole = cq.Workplane("YZ").circle(0.0035).extrude(0.020).translate((END_TAB_LEN - 0.010, 0.0, 0.0))
    return end_tab.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_joint_revolute_chain")

    bracket_mat = model.material("bracket_gray", rgba=(0.32, 0.34, 0.36, 1.0))
    link_mat = model.material("link_aluminum", rgba=(0.73, 0.76, 0.80, 1.0))
    tip_mat = model.material("end_tab_dark", rgba=(0.48, 0.52, 0.56, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(make_root_bracket(), "root_bracket"),
        material=bracket_mat,
        name="bracket_body",
    )

    link1 = model.part("link1")
    link1.visual(
        mesh_from_cadquery(make_link1(), "link1"),
        material=link_mat,
        name="link1_body",
    )

    link2 = model.part("link2")
    link2.visual(
        mesh_from_cadquery(make_link2(), "link2"),
        material=link_mat,
        name="link2_body",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(make_end_tab(), "end_tab"),
        material=tip_mat,
        name="end_tab_body",
    )

    model.articulation(
        "bracket_to_link1",
        ArticulationType.FIXED,
        parent=root_bracket,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, LINK_THICK / 2.0)),
    )

    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.15, upper=1.55),
    )

    model.articulation(
        "link2_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=end_tab,
        origin=Origin(xyz=(LINK2_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.35, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    end_tab = object_model.get_part("end_tab")

    mount = object_model.get_articulation("bracket_to_link1")
    joint1 = object_model.get_articulation("link1_to_link2")
    joint2 = object_model.get_articulation("link2_to_end_tab")

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
        "chain_has_two_parallel_revolute_joints",
        joint1.articulation_type == ArticulationType.REVOLUTE
        and joint2.articulation_type == ArticulationType.REVOLUTE
        and tuple(joint1.axis) == (0.0, -1.0, 0.0)
        and tuple(joint2.axis) == (0.0, -1.0, 0.0)
        and mount.articulation_type == ArticulationType.FIXED,
        "Expected one fixed root mount and two serial revolute joints sharing the same -Y axis.",
    )

    ctx.expect_contact(
        link1,
        root_bracket,
        contact_tol=0.0005,
        name="link1_seats_on_root_bracket",
    )
    ctx.expect_overlap(
        link1,
        root_bracket,
        axes="xy",
        min_overlap=0.020,
        name="root_bracket_has_real_mounting_footprint",
    )

    with ctx.pose({joint1: 0.0, joint2: 0.0}):
        ctx.expect_overlap(
            link1,
            link2,
            axes="yz",
            min_overlap=0.010,
            name="first_hinge_blocks_share_axis_envelope",
        )
        ctx.expect_overlap(
            link2,
            end_tab,
            axes="yz",
            min_overlap=0.009,
            name="second_hinge_blocks_share_axis_envelope",
        )

    with ctx.pose({joint1: 0.85, joint2: 0.75}):
        ctx.expect_origin_gap(
            end_tab,
            link1,
            axis="z",
            min_gap=0.045,
            name="positive_joint_motion_lifts_end_tab",
        )
        ctx.expect_origin_gap(
            end_tab,
            root_bracket,
            axis="x",
            min_gap=0.140,
            name="open_chain_reaches_forward_of_bracket",
        )
        ctx.expect_origin_distance(
            end_tab,
            root_bracket,
            axes="y",
            max_dist=0.001,
            name="chain_motion_stays_in_one_plane",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
