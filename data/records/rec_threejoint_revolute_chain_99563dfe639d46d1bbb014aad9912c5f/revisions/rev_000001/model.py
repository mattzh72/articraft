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


BASE_PLATE_LENGTH = 0.090
BASE_PLATE_WIDTH = 0.060
BASE_PLATE_THICKNESS = 0.012
WEB_THICKNESS = 0.014
WEB_HEIGHT = 0.060

HINGE_AXIS_Z = 0.065
WEB_WIDTH = 0.032

BARREL_RADIUS = 0.007
BARREL_LENGTH = 0.008
FORK_OUTER_WIDTH = 0.030
FORK_GAP_WIDTH = 0.010
CHEEK_THICKNESS = 0.010
CHEEK_LENGTH = 0.014
CHEEK_HEIGHT = 0.026
BRIDGE_LENGTH = 0.018
BRIDGE_HEIGHT = 0.016

ARM_WIDTH = 0.016
ARM_HEIGHT = 0.012
BEAM_START_X = 0.006
DISTAL_BRIDGE_OFFSET = 0.014

LINK_1_PITCH = 0.130
LINK_2_PITCH = 0.105
END_TAB_NECK_LENGTH = 0.026
END_TAB_PAD_LENGTH = 0.024
END_TAB_TIP_LENGTH = 0.018


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _hinge_barrel(center_x: float, center_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(BARREL_RADIUS)
        .extrude(BARREL_LENGTH)
        .translate((center_x, -BARREL_LENGTH / 2.0, center_z))
    )


def _fork_end(joint_x: float, center_z: float) -> cq.Workplane:
    cheek_offset_y = (FORK_GAP_WIDTH / 2.0) + (CHEEK_THICKNESS / 2.0)
    bridge = _box(
        BRIDGE_LENGTH,
        FORK_OUTER_WIDTH,
        BRIDGE_HEIGHT,
        (joint_x - DISTAL_BRIDGE_OFFSET, 0.0, center_z),
    )
    left_cheek = _box(
        CHEEK_LENGTH,
        CHEEK_THICKNESS,
        CHEEK_HEIGHT,
        (joint_x - (CHEEK_LENGTH / 2.0), -cheek_offset_y, center_z),
    )
    right_cheek = _box(
        CHEEK_LENGTH,
        CHEEK_THICKNESS,
        CHEEK_HEIGHT,
        (joint_x - (CHEEK_LENGTH / 2.0), cheek_offset_y, center_z),
    )
    return bridge.union(left_cheek).union(right_cheek)


def _make_root_bracket() -> cq.Workplane:
    bracket = _box(
        0.075,
        BASE_PLATE_WIDTH,
        BASE_PLATE_THICKNESS,
        (-0.0425, 0.0, BASE_PLATE_THICKNESS / 2.0),
    )
    bracket = bracket.union(
        _box(
            WEB_THICKNESS,
            WEB_WIDTH,
            0.050,
            (-0.030, 0.0, 0.035),
        )
    )
    bracket = bracket.union(_fork_end(joint_x=0.0, center_z=HINGE_AXIS_Z))

    bracket = bracket.cut(
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(BASE_PLATE_THICKNESS + 0.004)
        .translate((-0.028, 0.0, -0.002))
    )
    bracket = bracket.cut(
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(BASE_PLATE_THICKNESS + 0.004)
        .translate((0.018, 0.0, -0.002))
    )

    return bracket


def _make_link(pitch: float) -> cq.Workplane:
    beam_end_x = pitch - (DISTAL_BRIDGE_OFFSET - (BRIDGE_LENGTH / 2.0))
    beam_length = beam_end_x - BEAM_START_X
    link = _hinge_barrel(0.0, 0.0)
    link = link.union(
        _box(
            beam_length,
            ARM_WIDTH,
            ARM_HEIGHT,
            ((BEAM_START_X + beam_end_x) / 2.0, 0.0, 0.0),
        )
    )
    link = link.union(_fork_end(joint_x=pitch, center_z=0.0))
    return link


def _make_end_tab() -> cq.Workplane:
    tab = _hinge_barrel(0.0, 0.0)
    tab = tab.union(
        _box(
            END_TAB_NECK_LENGTH,
            ARM_WIDTH,
            ARM_HEIGHT,
            (0.019, 0.0, 0.0),
        )
    )
    tab = tab.union(
        _box(
            END_TAB_PAD_LENGTH,
            0.018,
            0.010,
            (0.044, 0.0, 0.0),
        )
    )
    tab = tab.union(
        _box(
            END_TAB_TIP_LENGTH,
            0.024,
            0.012,
            (0.065, 0.0, 0.0),
        )
    )
    return tab


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_revolute_chain")

    model.material("bracket_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("link_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("tool_black", rgba=(0.12, 0.13, 0.14, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_make_root_bracket(), "root_bracket"),
        material="bracket_steel",
        name="bracket_body",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_make_link(LINK_1_PITCH), "link_1"),
        material="link_aluminum",
        name="link_1_body",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_make_link(LINK_2_PITCH), "link_2"),
        material="link_aluminum",
        name="link_2_body",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_make_end_tab(), "end_tab"),
        material="tool_black",
        name="end_tab_body",
    )

    model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-1.10,
            upper=1.20,
        ),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.8,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "link_2_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_tab,
        origin=Origin(xyz=(LINK_2_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.2,
            lower=-1.55,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_tab = object_model.get_part("end_tab")

    root_to_link_1 = object_model.get_articulation("root_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_end_tab = object_model.get_articulation("link_2_to_end_tab")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        root_bracket,
        link_1,
        reason="Compact hinge root uses a simplified shared knuckle/pin volume instead of separate pin hardware.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        reason="Middle hinge is modeled as interleaved compact knuckles with an intentionally shared hinge-core volume.",
    )
    ctx.allow_overlap(
        link_2,
        end_tab,
        reason="Distal hinge uses the same compact shared knuckle simplification for the end tab pivot.",
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
        "all_joint_axes_parallel",
        all(
            articulation.axis == (0.0, -1.0, 0.0)
            for articulation in (root_to_link_1, link_1_to_link_2, link_2_to_end_tab)
        ),
        details="All three serial hinges should rotate about the same -Y axis.",
    )

    ctx.expect_contact(root_bracket, link_1, name="root bracket supports link 1")
    ctx.expect_contact(link_1, link_2, name="link 1 supports link 2")
    ctx.expect_contact(link_2, end_tab, name="link 2 supports end tab")

    ctx.expect_overlap(
        root_bracket,
        link_1,
        axes="yz",
        min_overlap=0.020,
        name="root hinge overlaps in hinge plane",
    )
    ctx.expect_overlap(
        link_1,
        link_2,
        axes="yz",
        min_overlap=0.020,
        name="middle hinge overlaps in hinge plane",
    )
    ctx.expect_overlap(
        link_2,
        end_tab,
        axes="yz",
        min_overlap=0.012,
        name="distal hinge overlaps in hinge plane",
    )

    rest_end_tab_origin = ctx.part_world_position(end_tab)
    rest_end_tab_aabb = ctx.part_element_world_aabb(end_tab, elem="end_tab_body")

    with ctx.pose(
        {
            root_to_link_1: 0.80,
            link_1_to_link_2: 0.0,
            link_2_to_end_tab: 0.0,
        }
    ):
        raised_origin = ctx.part_world_position(end_tab)
    ctx.check(
        "first_joint_positive_motion_lifts_chain",
        rest_end_tab_origin is not None
        and raised_origin is not None
        and raised_origin[2] > rest_end_tab_origin[2] + 0.10,
        details="Positive motion on the root hinge should raise the downstream chain upward.",
    )

    with ctx.pose(
        {
            root_to_link_1: 0.0,
            link_1_to_link_2: 0.80,
            link_2_to_end_tab: 0.0,
        }
    ):
        elbow_raised_origin = ctx.part_world_position(end_tab)
    ctx.check(
        "second_joint_positive_motion_lifts_end_tab_origin",
        rest_end_tab_origin is not None
        and elbow_raised_origin is not None
        and elbow_raised_origin[2] > rest_end_tab_origin[2] + 0.04,
        details="Positive motion on the middle hinge should raise the distal chain.",
    )

    with ctx.pose(
        {
            root_to_link_1: 0.0,
            link_1_to_link_2: 0.0,
            link_2_to_end_tab: 0.90,
        }
    ):
        wrist_raised_aabb = ctx.part_element_world_aabb(end_tab, elem="end_tab_body")
    ctx.check(
        "third_joint_positive_motion_pitches_tab_upward",
        rest_end_tab_aabb is not None
        and wrist_raised_aabb is not None
        and wrist_raised_aabb[1][2] > rest_end_tab_aabb[1][2] + 0.018,
        details="Positive motion on the distal hinge should rotate the compact end tab upward.",
    )

    with ctx.pose(
        {
            root_to_link_1: 0.35,
            link_1_to_link_2: 0.25,
            link_2_to_end_tab: 0.25,
        }
    ):
        ctx.expect_contact(root_bracket, link_1, name="root hinge stays mounted in articulated pose")
        ctx.expect_contact(link_1, link_2, name="middle hinge stays mounted in articulated pose")
        ctx.expect_contact(link_2, end_tab, name="distal hinge stays mounted in articulated pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
