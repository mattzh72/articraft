from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


RAIL_LENGTH = 0.340
RAIL_WIDTH = 0.074
BASE_THICKNESS = 0.016
GUIDE_LENGTH = RAIL_LENGTH - 0.048
GUIDE_WIDTH = 0.038
GUIDE_HEIGHT = 0.028
GUIDE_CENTER_Z = BASE_THICKNESS + (GUIDE_HEIGHT / 2.0)

END_HOUSING_LENGTH = 0.024
END_HOUSING_WIDTH = 0.066
END_HOUSING_HEIGHT = 0.044

WAY_LENGTH = GUIDE_LENGTH
WAY_WIDTH = 0.012
WAY_HEIGHT = 0.004
WAY_Y = 0.027

STOP_PAD_LENGTH = 0.006
STOP_PAD_WIDTH = 0.048
STOP_PAD_HEIGHT = 0.024
STOP_PAD_Z = BASE_THICKNESS + (STOP_PAD_HEIGHT / 2.0)
LEFT_STOP_PAD_X = -((RAIL_LENGTH / 2.0) - END_HOUSING_LENGTH - (STOP_PAD_LENGTH / 2.0))
RIGHT_STOP_PAD_X = (RAIL_LENGTH / 2.0) - END_HOUSING_LENGTH - (STOP_PAD_LENGTH / 2.0)

CARRIAGE_LENGTH = 0.102
CARRIAGE_WIDTH = 0.110
CARRIAGE_HEIGHT = 0.054
CARRIAGE_CENTER_Z = 0.006
CARRIAGE_SIDE_WALL = 0.010
CARRIAGE_SIDE_WALL_HEIGHT = 0.030
CARRIAGE_SIDE_WALL_Z = 0.041
CARRIAGE_TOP_BRIDGE_LENGTH = 0.086
CARRIAGE_TOP_BRIDGE_THICKNESS = 0.012
CARRIAGE_TOP_BRIDGE_Z = 0.058

OUTER_RIB_THICKNESS = 0.005
OUTER_RIB_LENGTH = 0.038
OUTER_RIB_HEIGHT = 0.016

WIPER_THICKNESS = 0.004
WIPER_SIDE_WIDTH = 0.006
WIPER_SIDE_HEIGHT = 0.028
WIPER_TOP_WIDTH = 0.082
WIPER_TOP_HEIGHT = 0.006
WIPER_TOP_Z = 0.019
WIPER_SIDE_Z = 0.001

SHOE_LENGTH = 0.072
SHOE_WIDTH = WAY_WIDTH
SHOE_HEIGHT = WAY_HEIGHT
SHOE_CENTER_Z = (BASE_THICKNESS + WAY_HEIGHT + (SHOE_HEIGHT / 2.0)) - GUIDE_CENTER_Z
CLEVIS_DEPTH = 0.018
CLEVIS_THICKNESS = 0.014
CLEVIS_GAP = 0.040
CLEVIS_HEIGHT = 0.020
CLEVIS_CENTER_Y = (CLEVIS_GAP / 2.0) + (CLEVIS_THICKNESS / 2.0)

HINGE_X = (CARRIAGE_LENGTH / 2.0) + (CLEVIS_DEPTH / 2.0)
HINGE_Z = 0.058

TAB_BARREL_RADIUS = 0.0055
TAB_BARREL_LENGTH = CLEVIS_GAP
TAB_BLADE_WIDTH = 0.020
TAB_BLADE_THICKNESS = 0.008
TAB_BLADE_LENGTH = 0.018
TAB_ARM_LENGTH = 0.026
TAB_BLADE_X = 0.024
TAB_FOOT_WIDTH = 0.030
TAB_FOOT_HEIGHT = 0.008

SLIDE_LOWER = -0.085
SLIDE_UPPER = 0.060
TAB_LOWER = -1.050
TAB_UPPER = 0.250
TAB_CLEAR_POSE = -0.950


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _rail_shell_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.0025)
    )

    guide = (
        cq.Workplane("XY")
        .box(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + (GUIDE_HEIGHT / 2.0)))
        .edges("|X and >Z")
        .fillet(0.0020)
    )

    left_housing = (
        cq.Workplane("XY")
        .box(END_HOUSING_LENGTH, END_HOUSING_WIDTH, END_HOUSING_HEIGHT)
        .translate(
            (
                -((RAIL_LENGTH / 2.0) - (END_HOUSING_LENGTH / 2.0)),
                0.0,
                END_HOUSING_HEIGHT / 2.0,
            )
        )
    )
    right_housing = left_housing.translate((RAIL_LENGTH - END_HOUSING_LENGTH, 0.0, 0.0))

    left_cap = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-END_HOUSING_LENGTH / 2.0, 0.0),
                (END_HOUSING_LENGTH / 2.0, 0.0),
                (END_HOUSING_LENGTH / 2.0, END_HOUSING_HEIGHT - 0.006),
                (0.0, END_HOUSING_HEIGHT),
                (-END_HOUSING_LENGTH / 2.0, END_HOUSING_HEIGHT - 0.006),
            ]
        )
        .close()
        .extrude((END_HOUSING_WIDTH - 0.010) / 2.0, both=True)
        .translate(
            (
                -((RAIL_LENGTH / 2.0) - (END_HOUSING_LENGTH / 2.0)),
                0.0,
                0.0,
            )
        )
    )
    right_cap = left_cap.translate((RAIL_LENGTH - END_HOUSING_LENGTH, 0.0, 0.0))

    return _union_all(base, guide, left_housing, right_housing, left_cap, right_cap)


def _carriage_shell_shape() -> cq.Workplane:
    top_bridge = (
        cq.Workplane("XY")
        .box(CARRIAGE_TOP_BRIDGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_TOP_BRIDGE_THICKNESS)
        .translate((0.0, 0.0, CARRIAGE_TOP_BRIDGE_Z))
    )

    left_wall = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_SIDE_WALL, CARRIAGE_SIDE_WALL_HEIGHT)
        .translate((0.0, 0.050, CARRIAGE_SIDE_WALL_Z))
    )
    right_wall = left_wall.translate((0.0, -0.100, 0.0))

    left_shoulder = (
        cq.Workplane("XY")
        .box(SHOE_LENGTH, 0.024, 0.008)
        .translate((0.0, 0.038, 0.022))
    )
    right_shoulder = left_shoulder.translate((0.0, -0.076, 0.0))

    left_hanger = (
        cq.Workplane("XY")
        .box(SHOE_LENGTH, 0.006, 0.028)
        .translate((0.0, WAY_Y, 0.006))
    )
    right_hanger = left_hanger.translate((0.0, -2.0 * WAY_Y, 0.0))

    left_front_rib = (
        cq.Workplane("XY")
        .box(OUTER_RIB_LENGTH, OUTER_RIB_THICKNESS, OUTER_RIB_HEIGHT)
        .translate((0.021, 0.0525, 0.036))
    )
    left_rear_rib = left_front_rib.translate((-0.044, 0.0, 0.0))
    right_front_rib = left_front_rib.translate((0.0, -0.105, 0.0))
    right_rear_rib = left_rear_rib.translate((0.0, -0.105, 0.0))

    left_nose = (
        cq.Workplane("XY")
        .box(0.018, 0.010, 0.016)
        .translate((0.050, 0.043, 0.054))
    )
    right_nose = left_nose.translate((0.0, -0.086, 0.0))

    left_cheek = (
        cq.Workplane("XY")
        .box(CLEVIS_DEPTH, CLEVIS_THICKNESS, CLEVIS_HEIGHT)
        .translate((HINGE_X, CLEVIS_CENTER_Y, HINGE_Z))
    )
    right_cheek = left_cheek.translate((0.0, -2.0 * CLEVIS_CENTER_Y, 0.0))
    left_gusset = (
        cq.Workplane("XY")
        .box(0.016, 0.010, 0.014)
        .translate((0.053, 0.036, 0.053))
    )
    right_gusset = left_gusset.translate((0.0, -0.072, 0.0))

    return _union_all(
        top_bridge,
        left_wall,
        right_wall,
        left_shoulder,
        right_shoulder,
        left_hanger,
        right_hanger,
        left_front_rib,
        left_rear_rib,
        right_front_rib,
        right_rear_rib,
        left_nose,
        right_nose,
        left_cheek,
        right_cheek,
        left_gusset,
        right_gusset,
    )


def _carriage_wiper_shape() -> cq.Workplane:
    front_top = (
        cq.Workplane("XY")
        .box(WIPER_THICKNESS, 0.050, 0.010)
        .translate(((CARRIAGE_LENGTH / 2.0) - (WIPER_THICKNESS / 2.0), 0.0, 0.047))
    )
    front_left = (
        cq.Workplane("XY")
        .box(WIPER_THICKNESS, 0.010, 0.014)
        .translate(((CARRIAGE_LENGTH / 2.0) - (WIPER_THICKNESS / 2.0), 0.028, 0.006))
    )
    front_right = front_left.translate((0.0, -0.056, 0.0))
    rear_offset = -(CARRIAGE_LENGTH - WIPER_THICKNESS)
    rear_top = front_top.translate((rear_offset, 0.0, 0.0))
    rear_left = front_left.translate((rear_offset, 0.0, 0.0))
    rear_right = front_right.translate((rear_offset, 0.0, 0.0))
    return _union_all(front_top, front_left, front_right, rear_top, rear_left, rear_right)


def _support_tab_barrel_shape() -> cq.Workplane:
    return cq.Workplane("XZ").circle(TAB_BARREL_RADIUS).extrude(TAB_BARREL_LENGTH / 2.0, both=True)


def _support_tab_body_shape() -> cq.Workplane:
    neck = (
        cq.Workplane("XY")
        .box(0.008, 0.018, 0.008)
        .translate((0.009, 0.0, -0.001))
    )

    arm = (
        cq.Workplane("XY")
        .box(TAB_ARM_LENGTH, TAB_BLADE_WIDTH, TAB_BLADE_THICKNESS)
        .translate((0.024, 0.0, -0.001))
    )

    foot = (
        cq.Workplane("XY")
        .box(0.014, TAB_FOOT_WIDTH, TAB_FOOT_HEIGHT)
        .translate((0.040, 0.0, -0.006))
    )

    web = (
        cq.Workplane("XY")
        .box(0.018, 0.014, 0.006)
        .translate((0.032, 0.0, -0.004))
    )

    blade = (
        cq.Workplane("XY")
        .box(TAB_BLADE_THICKNESS, TAB_BLADE_WIDTH, TAB_BLADE_LENGTH)
        .translate((0.045, 0.0, -0.014))
    )

    return _union_all(neck, arm, foot, web, blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_transfer_axis")

    model.material("rail_body_finish", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_finish", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("hardened_way", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("polymer_wiper", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("tab_finish", rgba=(0.64, 0.66, 0.69, 1.0))

    rail_body = model.part("rail_body")
    rail_body.visual(
        mesh_from_cadquery(_rail_shell_shape(), "transfer_axis_rail_shell"),
        material="rail_body_finish",
        name="rail_shell",
    )
    rail_body.visual(
        Box((WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT)),
        origin=Origin(xyz=(0.0, WAY_Y, BASE_THICKNESS + (WAY_HEIGHT / 2.0))),
        material="hardened_way",
        name="left_way",
    )
    rail_body.visual(
        Box((WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT)),
        origin=Origin(xyz=(0.0, -WAY_Y, BASE_THICKNESS + (WAY_HEIGHT / 2.0))),
        material="hardened_way",
        name="right_way",
    )
    rail_body.visual(
        Box((STOP_PAD_LENGTH, STOP_PAD_WIDTH, STOP_PAD_HEIGHT)),
        origin=Origin(xyz=(LEFT_STOP_PAD_X, 0.0, STOP_PAD_Z)),
        material="polymer_wiper",
        name="left_stop_pad",
    )
    rail_body.visual(
        Box((STOP_PAD_LENGTH, STOP_PAD_WIDTH, STOP_PAD_HEIGHT)),
        origin=Origin(xyz=(RIGHT_STOP_PAD_X, 0.0, STOP_PAD_Z)),
        material="polymer_wiper",
        name="right_stop_pad",
    )
    rail_body.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, END_HOUSING_HEIGHT)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, END_HOUSING_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shell_shape(), "transfer_axis_carriage_shell"),
        material="carriage_finish",
        name="carriage_shell",
    )
    carriage.visual(
        mesh_from_cadquery(_carriage_wiper_shape(), "transfer_axis_wiper_frames"),
        material="polymer_wiper",
        name="wiper_frames",
    )
    carriage.visual(
        Box((SHOE_LENGTH, SHOE_WIDTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, WAY_Y, SHOE_CENTER_Z)),
        material="polymer_wiper",
        name="left_shoe",
    )
    carriage.visual(
        Box((SHOE_LENGTH, SHOE_WIDTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, -WAY_Y, SHOE_CENTER_Z)),
        material="polymer_wiper",
        name="right_shoe",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH + CLEVIS_DEPTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=2.4,
        origin=Origin(xyz=(0.008, 0.0, 0.006)),
    )

    support_tab = model.part("support_tab")
    support_tab.visual(
        mesh_from_cadquery(_support_tab_barrel_shape(), "transfer_axis_support_tab_barrel"),
        material="tab_finish",
        name="tab_barrel",
    )
    support_tab.visual(
        mesh_from_cadquery(_support_tab_body_shape(), "transfer_axis_support_tab_body"),
        material="tab_finish",
        name="tab_body",
    )
    support_tab.inertial = Inertial.from_geometry(
        Box((0.018, TAB_BLADE_WIDTH, TAB_BLADE_LENGTH)),
        mass=0.35,
        origin=Origin(xyz=(0.008, 0.0, -(TAB_BLADE_LENGTH / 2.0))),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_body,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.45,
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
        ),
    )
    model.articulation(
        "carriage_to_support_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=support_tab,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=TAB_LOWER,
            upper=TAB_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_body = object_model.get_part("rail_body")
    carriage = object_model.get_part("carriage")
    support_tab = object_model.get_part("support_tab")
    slide = object_model.get_articulation("rail_to_carriage")
    tab_hinge = object_model.get_articulation("carriage_to_support_tab")

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
    ctx.allow_overlap(
        carriage,
        support_tab,
        elem_a="carriage_shell",
        elem_b="tab_barrel",
        reason="hinge barrel is intentionally captured between clevis cheeks; cheek bore relief is abstracted in the mesh shell",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_parts_present",
        rail_body is not None and carriage is not None and support_tab is not None,
        "rail_body, carriage, and support_tab must all exist",
    )
    ctx.check(
        "slide_axis_is_longitudinal",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected slide axis (1,0,0), got {slide.axis}",
    )
    ctx.check(
        "tab_hinge_axis_is_supported_local_y",
        tuple(tab_hinge.axis) == (0.0, 1.0, 0.0),
        f"expected tab hinge axis (0,1,0), got {tab_hinge.axis}",
    )

    ctx.expect_contact(
        carriage,
        rail_body,
        elem_a="left_shoe",
        elem_b="left_way",
        name="left_shoe_runs_on_left_way",
    )
    ctx.expect_contact(
        carriage,
        rail_body,
        elem_a="right_shoe",
        elem_b="right_way",
        name="right_shoe_runs_on_right_way",
    )
    ctx.expect_contact(
        support_tab,
        carriage,
        elem_a="tab_barrel",
        elem_b="carriage_shell",
        name="support_tab_is_carried_by_clevis",
    )
    ctx.expect_overlap(
        carriage,
        rail_body,
        axes="yz",
        elem_a="carriage_shell",
        elem_b="rail_shell",
        min_overlap=0.020,
        name="carriage_visually_wraps_fixed_rail",
    )

    with ctx.pose({slide: SLIDE_LOWER, tab_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_pose_clear_of_body")
        ctx.expect_gap(
            carriage,
            rail_body,
            axis="x",
            positive_elem="carriage_shell",
            negative_elem="left_stop_pad",
            min_gap=0.0035,
            name="rear_stop_clearance",
        )

    with ctx.pose({slide: SLIDE_UPPER, tab_hinge: TAB_CLEAR_POSE}):
        ctx.fail_if_parts_overlap_in_current_pose(name="forward_open_pose_clear_of_body")
        ctx.expect_gap(
            rail_body,
            carriage,
            axis="x",
            positive_elem="right_stop_pad",
            negative_elem="carriage_shell",
            min_gap=0.0045,
            name="forward_stop_clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
