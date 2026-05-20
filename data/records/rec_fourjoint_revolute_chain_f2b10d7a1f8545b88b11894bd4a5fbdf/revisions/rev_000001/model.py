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


PIN_RADIUS = 0.0045
JOINT_OUTER_RADIUS = 0.0125
LINK_THICKNESS = 0.010
EAR_THICKNESS = 0.005
CLEVIS_GAP = LINK_THICKNESS
CLEVIS_WIDTH = CLEVIS_GAP + 2.0 * EAR_THICKNESS
ARM_HEIGHT = 0.014
JAW_LENGTH = 0.026
JAW_OPEN_LENGTH = 0.015
HINGE_BLOCK_LENGTH = 0.022

LINK_1_LENGTH = 0.074
LINK_2_LENGTH = 0.068
LINK_3_LENGTH = 0.060
END_TAB_LENGTH = 0.034
ROOT_HINGE_X = 0.016


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, center[1], 0.0))
    )


def _slot_cut(length: float, height: float, thickness: float, center_x: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, 0.0)
        .slot2D(length, height)
        .extrude(thickness / 2.0, both=True)
    )


def _keep_x_ge(shape: cq.Workplane, x_min: float) -> cq.Workplane:
    return shape.intersect(_box((2.0, 0.20, 0.20), (x_min + 1.0, 0.0, 0.0)))


def _keep_x_le(shape: cq.Workplane, x_max: float) -> cq.Workplane:
    return shape.intersect(_box((2.0, 0.20, 0.20), (x_max - 1.0, 0.0, 0.0)))


def make_link_with_distal_clevis(length: float) -> cq.Workplane:
    eye_block_length = 0.018
    ear_length = 0.018
    rib_length = 0.010
    arm_start_x = 0.006
    arm_end_x = length - ear_length
    arm_length = arm_end_x - arm_start_x
    ear_center_y = CLEVIS_GAP / 2.0 + EAR_THICKNESS / 2.0

    link = _keep_x_ge(_cylinder_y(JOINT_OUTER_RADIUS, LINK_THICKNESS, (0.0, 0.0, 0.0)), 0.0)
    link = link.union(_box((eye_block_length, LINK_THICKNESS, ARM_HEIGHT + 0.004), (eye_block_length / 2.0, 0.0, 0.0)))
    link = link.union(_box((arm_length, LINK_THICKNESS, ARM_HEIGHT), (arm_start_x + arm_length / 2.0, 0.0, 0.0)))
    link = link.union(
        _box((rib_length, CLEVIS_WIDTH, ARM_HEIGHT * 0.9), (length - ear_length - rib_length / 2.0, 0.0, 0.0))
    )

    for sign in (-1.0, 1.0):
        y = sign * ear_center_y
        ear = _box((ear_length, EAR_THICKNESS, 2.0 * JOINT_OUTER_RADIUS), (length - ear_length / 2.0, y, 0.0))
        ear = ear.union(_keep_x_le(_cylinder_y(JOINT_OUTER_RADIUS, EAR_THICKNESS, (length, y, 0.0)), length))
        link = link.union(ear)

    link = link.cut(_cylinder_y(PIN_RADIUS, LINK_THICKNESS + 0.002, (0.0, 0.0, 0.0)))
    link = link.cut(_cylinder_y(PIN_RADIUS, CLEVIS_WIDTH + 0.002, (length, 0.0, 0.0)))

    window_length = arm_length - 0.010
    if window_length > 0.016:
        link = link.cut(
            _slot_cut(
                window_length,
                ARM_HEIGHT * 0.48,
                LINK_THICKNESS + 0.001,
                arm_start_x + arm_length / 2.0,
            )
        )

    return link.clean()


def make_end_tab(length: float) -> cq.Workplane:
    eye_block_length = 0.018
    tab_tip_radius = 0.0085
    tab_bar_length = length + 0.014

    end_tab = _keep_x_ge(_cylinder_y(JOINT_OUTER_RADIUS, LINK_THICKNESS, (0.0, 0.0, 0.0)), 0.0)
    end_tab = end_tab.union(
        _box((eye_block_length, LINK_THICKNESS, ARM_HEIGHT + 0.004), (eye_block_length / 2.0, 0.0, 0.0))
    )
    end_tab = end_tab.union(
        _box((tab_bar_length, LINK_THICKNESS, ARM_HEIGHT), (0.008 + tab_bar_length / 2.0, 0.0, 0.0))
    )
    end_tab = end_tab.union(_box((0.016, LINK_THICKNESS, 0.018), (length + 0.014, 0.0, 0.0)))
    end_tab = end_tab.union(_cylinder_y(tab_tip_radius, LINK_THICKNESS, (length + 0.022, 0.0, 0.0)))
    end_tab = end_tab.cut(_cylinder_y(PIN_RADIUS, LINK_THICKNESS + 0.001, (0.0, 0.0, 0.0)))
    end_tab = end_tab.cut(_cylinder_y(0.0032, LINK_THICKNESS + 0.001, (length + 0.020, 0.0, 0.0)))
    return end_tab.clean()


def make_root_bracket() -> cq.Workplane:
    ear_length = 0.018
    spine_x = -0.036
    ear_center_y = CLEVIS_GAP / 2.0 + EAR_THICKNESS / 2.0

    bracket = _box((0.012, LINK_THICKNESS, 0.074), (spine_x, 0.0, 0.004))
    bracket = bracket.union(_box((0.068, LINK_THICKNESS, 0.012), (-0.022, 0.0, -0.033)))
    bracket = bracket.union(
        cq.Workplane("XZ")
        .polyline([(-0.050, -0.027), (-0.018, -0.027), (-0.018, 0.022), (-0.034, 0.022)])
        .close()
        .extrude(LINK_THICKNESS, both=True)
    )
    bracket = bracket.union(_box((0.030, LINK_THICKNESS, 0.020), (-0.014, 0.0, 0.0)))
    bracket = bracket.union(_box((0.016, LINK_THICKNESS, 0.014), (0.004, 0.0, 0.0)))

    for sign in (-1.0, 1.0):
        y = sign * ear_center_y
        bracket = bracket.union(
            _box((ear_length, EAR_THICKNESS, 2.0 * JOINT_OUTER_RADIUS), (ROOT_HINGE_X - ear_length / 2.0, y, 0.0))
        )
        bracket = bracket.union(
            _keep_x_le(_cylinder_y(JOINT_OUTER_RADIUS, EAR_THICKNESS, (ROOT_HINGE_X, y, 0.0)), ROOT_HINGE_X)
        )

    bracket = bracket.cut(_cylinder_y(PIN_RADIUS, CLEVIS_WIDTH + 0.002, (ROOT_HINGE_X, 0.0, 0.0)))
    bracket = bracket.cut(_cylinder_y(0.0035, LINK_THICKNESS + 0.002, (-0.040, 0.0, -0.033)))
    bracket = bracket.cut(_cylinder_y(0.0035, LINK_THICKNESS + 0.002, (-0.022, 0.0, -0.033)))
    return bracket.clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_revolute_chain")

    model.material("bracket_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("link_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("end_tab_steel", rgba=(0.55, 0.58, 0.62, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(make_root_bracket(), "root_bracket"),
        origin=Origin(),
        material="bracket_steel",
        name="bracket_body",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(make_link_with_distal_clevis(LINK_1_LENGTH), "link_1"),
        origin=Origin(),
        material="link_steel",
        name="link_1_body",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(make_link_with_distal_clevis(LINK_2_LENGTH), "link_2"),
        origin=Origin(),
        material="link_steel",
        name="link_2_body",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(make_link_with_distal_clevis(LINK_3_LENGTH), "link_3"),
        origin=Origin(),
        material="link_steel",
        name="link_3_body",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(make_end_tab(END_TAB_LENGTH), "end_tab"),
        origin=Origin(),
        material="end_tab_steel",
        name="end_tab_body",
    )

    common_limits = MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=0.35)

    model.articulation(
        "bracket_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=link_1,
        origin=Origin(xyz=(ROOT_HINGE_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_3_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.8, lower=0.0, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_tab = object_model.get_part("end_tab")

    joint_1 = object_model.get_articulation("bracket_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_link_3")
    joint_4 = object_model.get_articulation("link_3_to_end_tab")

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

    ctx.expect_contact(root_bracket, link_1, name="root_joint_has_physical_contact")
    ctx.expect_contact(link_1, link_2, name="joint_2_has_physical_contact")
    ctx.expect_contact(link_2, link_3, name="joint_3_has_physical_contact")
    ctx.expect_contact(link_3, end_tab, name="joint_4_has_physical_contact")

    ctx.expect_origin_gap(link_2, link_1, axis="x", min_gap=LINK_1_LENGTH - 0.001, max_gap=LINK_1_LENGTH + 0.001)
    ctx.expect_origin_gap(link_3, link_2, axis="x", min_gap=LINK_2_LENGTH - 0.001, max_gap=LINK_2_LENGTH + 0.001)
    ctx.expect_origin_gap(end_tab, link_3, axis="x", min_gap=LINK_3_LENGTH - 0.001, max_gap=LINK_3_LENGTH + 0.001)

    joints = (joint_1, joint_2, joint_3, joint_4)
    ctx.check(
        "all_revolute_axes_parallel",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, -1.0, 0.0) for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    rest_end = ctx.part_world_position(end_tab)
    with ctx.pose({joint_1: 0.35, joint_2: 0.25, joint_3: 0.20, joint_4: 0.15}):
        folded_end = ctx.part_world_position(end_tab)
    folds_up = (
        rest_end is not None
        and folded_end is not None
        and folded_end[2] > rest_end[2] + 0.08
        and abs(folded_end[1] - rest_end[1]) < 1e-4
    )
    ctx.check(
        "positive_joint_motion_lifts_chain_in_plane",
        folds_up,
        details=f"rest_end={rest_end}, folded_end={folded_end}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
