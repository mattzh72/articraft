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


PLATE_THICKNESS = 0.010
LINK_WEB_WIDTH = 0.014
TOP_EYE_RADIUS = 0.010
TOP_EYE_LENGTH = 0.012
LINK_PITCH = 0.115
BOTTOM_EAR_RADIUS = 0.009
HINGE_BORE_RADIUS = 0.0048
BRACKET_TOP_WIDTH = 0.108
BRACKET_TOP_DEPTH = 0.048
BRACKET_TOP_THICKNESS = 0.008
BRACKET_DROP = 0.060
END_TAB_DROP = 0.058
EAR_LENGTH = 0.008
EAR_OFFSET_Y = 0.010
TOP_NECK_HEIGHT = 0.014
MAIN_STRAP_HEIGHT = 0.076
MAIN_STRAP_CENTER_Z = -0.052
LINK_CLEVIS_BRIDGE_HEIGHT = 0.008
LINK_CLEVIS_BRIDGE_CENTER_Z = -0.091
LINK_SIDE_ARM_HEIGHT = 0.028
LINK_SIDE_ARM_CENTER_Z = -0.102
EAR_WIDTH = 0.008
BRACKET_SIDE_ARM_HEIGHT = 0.058
BRACKET_SIDE_ARM_CENTER_Z = -0.033
END_TAB_STEM_HEIGHT = 0.038
END_TAB_STEM_CENTER_Z = -0.033
END_TAB_CONNECTOR_HEIGHT = 0.014
END_TAB_CONNECTOR_CENTER_Z = -0.051
END_PAD_RADIUS = 0.012


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def cylinder_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def cylinder_z(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def make_link_body(*, bottom_fork: bool) -> cq.Workplane:
    body = (
        cylinder_y(TOP_EYE_RADIUS, TOP_EYE_LENGTH, (0.0, 0.0, 0.0))
        .union(box_at((0.022, TOP_EYE_LENGTH, TOP_NECK_HEIGHT), (0.0, 0.0, -TOP_NECK_HEIGHT / 2.0)))
        .union(box_at((LINK_WEB_WIDTH, TOP_EYE_LENGTH, MAIN_STRAP_HEIGHT), (0.0, 0.0, MAIN_STRAP_CENTER_Z)))
    )

    if bottom_fork:
        body = body.union(
            box_at((0.018, 0.018, LINK_CLEVIS_BRIDGE_HEIGHT), (0.0, 0.0, LINK_CLEVIS_BRIDGE_CENTER_Z))
        )
        for side_sign in (-1.0, 1.0):
            body = body.union(
                box_at(
                    (EAR_WIDTH, EAR_LENGTH, LINK_SIDE_ARM_HEIGHT),
                    (0.0, side_sign * EAR_OFFSET_Y, LINK_SIDE_ARM_CENTER_Z),
                )
            )
            body = body.union(
                cylinder_y(
                    BOTTOM_EAR_RADIUS,
                    EAR_LENGTH,
                    (0.0, side_sign * EAR_OFFSET_Y, -LINK_PITCH),
                )
            )
    else:
        body = body.union(cylinder_y(0.015, TOP_EYE_LENGTH, (0.0, 0.0, -LINK_PITCH)))

    body = body.cut(cylinder_y(HINGE_BORE_RADIUS, TOP_EYE_LENGTH + 0.004, (0.0, 0.0, 0.0)))
    if not bottom_fork:
        body = body.cut(cylinder_y(HINGE_BORE_RADIUS, TOP_EYE_LENGTH + 0.004, (0.0, 0.0, -LINK_PITCH)))
    return body


def make_support_bracket() -> cq.Workplane:
    bracket = box_at((BRACKET_TOP_WIDTH, BRACKET_TOP_DEPTH, BRACKET_TOP_THICKNESS), (0.0, 0.0, 0.0))

    for side_sign in (-1.0, 1.0):
        bracket = bracket.union(
            box_at(
                (EAR_WIDTH, EAR_LENGTH, BRACKET_SIDE_ARM_HEIGHT),
                (0.0, side_sign * EAR_OFFSET_Y, BRACKET_SIDE_ARM_CENTER_Z),
            )
        )
        bracket = bracket.union(
            cylinder_y(
                BOTTOM_EAR_RADIUS,
                EAR_LENGTH,
                (0.0, side_sign * EAR_OFFSET_Y, -BRACKET_DROP),
            )
        )

    bracket = (
        bracket.cut(cylinder_z(0.0045, BRACKET_TOP_THICKNESS + 0.004, (-0.030, 0.0, 0.0)))
        .cut(cylinder_z(0.0045, BRACKET_TOP_THICKNESS + 0.004, (0.030, 0.0, 0.0)))
    )
    return bracket


def make_end_tab() -> cq.Workplane:
    tab = (
        cylinder_y(TOP_EYE_RADIUS, TOP_EYE_LENGTH, (0.0, 0.0, 0.0))
        .union(box_at((0.022, TOP_EYE_LENGTH, TOP_NECK_HEIGHT), (0.0, 0.0, -TOP_NECK_HEIGHT / 2.0)))
        .union(box_at((LINK_WEB_WIDTH, TOP_EYE_LENGTH, END_TAB_STEM_HEIGHT), (0.0, 0.0, END_TAB_STEM_CENTER_Z)))
        .union(box_at((0.012, TOP_EYE_LENGTH, END_TAB_CONNECTOR_HEIGHT), (0.0, 0.0, END_TAB_CONNECTOR_CENTER_Z)))
        .union(cylinder_y(END_PAD_RADIUS, TOP_EYE_LENGTH, (0.0, 0.0, -END_TAB_DROP)))
        .cut(cylinder_y(HINGE_BORE_RADIUS, TOP_EYE_LENGTH + 0.004, (0.0, 0.0, 0.0)))
        .cut(cylinder_y(0.0048, TOP_EYE_LENGTH + 0.004, (0.0, 0.0, -END_TAB_DROP)))
    )
    return tab


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_four_joint_chain")

    bracket_mat = model.material("zinc_plated_bracket", color=(0.63, 0.65, 0.68))
    link_mat = model.material("black_oxide_link", color=(0.30, 0.32, 0.35))
    tab_mat = model.material("dark_end_tab", color=(0.22, 0.24, 0.27))

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        mesh_from_cadquery(make_support_bracket(), "support_bracket"),
        origin=Origin(),
        material=bracket_mat,
        name="support_shell",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(make_link_body(bottom_fork=True), "link_1"),
        origin=Origin(),
        material=link_mat,
        name="link_shell",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(make_link_body(bottom_fork=True), "link_2"),
        origin=Origin(),
        material=link_mat,
        name="link_shell",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(make_link_body(bottom_fork=True), "link_3"),
        origin=Origin(),
        material=link_mat,
        name="link_shell",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(make_end_tab(), "end_tab"),
        origin=Origin(),
        material=tab_mat,
        name="tab_shell",
    )

    joint_limits = MotionLimits(effort=18.0, velocity=2.5, lower=-1.9, upper=1.9)

    model.articulation(
        "support_to_link_1",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, -BRACKET_DROP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -LINK_PITCH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(0.0, 0.0, -LINK_PITCH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_3_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(0.0, 0.0, -LINK_PITCH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_bracket = object_model.get_part("support_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_tab = object_model.get_part("end_tab")
    hinge_1 = object_model.get_articulation("support_to_link_1")
    hinge_2 = object_model.get_articulation("link_1_to_link_2")
    hinge_3 = object_model.get_articulation("link_2_to_link_3")
    hinge_4 = object_model.get_articulation("link_3_to_end_tab")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        support_bracket,
        link_1,
        reason="Adjacent hinge knuckles are modeled as tight interleaved barrels with zero-clearance visual nesting at the support joint.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        reason="Adjacent chain links use compact interleaved hinge barrels that intentionally occupy the same local hinge envelope in the visual model.",
    )
    ctx.allow_overlap(
        link_2,
        link_3,
        reason="Adjacent chain links use compact interleaved hinge barrels that intentionally occupy the same local hinge envelope in the visual model.",
    )
    ctx.allow_overlap(
        link_3,
        end_tab,
        reason="The end tab shares the final hinge barrel envelope with the last link as a tight visual hinge representation.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected_part_count",
        len(object_model.parts) == 5,
        f"expected 5 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "parallel_hinge_axes",
        all(joint.axis == (0.0, -1.0, 0.0) for joint in (hinge_1, hinge_2, hinge_3, hinge_4)),
        "all four revolute joints should share the same -Y hinge axis",
    )

    with ctx.pose():
        ctx.expect_contact(support_bracket, link_1, contact_tol=0.0005, name="support_to_link_1_contact")
        ctx.expect_contact(link_1, link_2, contact_tol=0.0005, name="link_1_to_link_2_contact")
        ctx.expect_contact(link_2, link_3, contact_tol=0.0005, name="link_2_to_link_3_contact")
        ctx.expect_contact(link_3, end_tab, contact_tol=0.0005, name="link_3_to_end_tab_contact")

        ctx.expect_origin_gap(
            support_bracket,
            link_1,
            axis="z",
            min_gap=0.045,
            max_gap=0.075,
            name="link_1_hangs_below_support",
        )
        ctx.expect_origin_gap(
            link_1,
            link_2,
            axis="z",
            min_gap=0.100,
            max_gap=0.130,
            name="link_2_hangs_below_link_1",
        )
        ctx.expect_origin_gap(
            link_2,
            link_3,
            axis="z",
            min_gap=0.100,
            max_gap=0.130,
            name="link_3_hangs_below_link_2",
        )
        ctx.expect_origin_gap(
            link_3,
            end_tab,
            axis="z",
            min_gap=0.100,
            max_gap=0.130,
            name="end_tab_hangs_below_link_3",
        )

        rest_tab_pos = ctx.part_world_position(end_tab)

    with ctx.pose({hinge_1: 0.70, hinge_2: 0.55, hinge_3: 0.40, hinge_4: 0.25}):
        swung_tab_pos = ctx.part_world_position(end_tab)
        ctx.check(
            "positive_joint_motion_swings_chain_forward",
            rest_tab_pos is not None
            and swung_tab_pos is not None
            and swung_tab_pos[0] > rest_tab_pos[0] + 0.10
            and swung_tab_pos[2] > rest_tab_pos[2] + 0.06,
            f"rest_tab={rest_tab_pos}, swung_tab={swung_tab_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
