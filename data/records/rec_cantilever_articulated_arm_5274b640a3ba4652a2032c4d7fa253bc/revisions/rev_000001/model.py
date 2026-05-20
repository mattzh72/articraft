from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LENGTH = 0.62
BASE_WIDTH = 0.42
BASE_THICKNESS = 0.065
PLINTH_LENGTH = 0.40
PLINTH_WIDTH = 0.26
PLINTH_HEIGHT = 0.095
COLUMN_LENGTH = 0.23
COLUMN_WIDTH = 0.19
COLUMN_HEIGHT = 0.60

SHOULDER_X = 0.31
SHOULDER_Z = BASE_THICKNESS + PLINTH_HEIGHT + COLUMN_HEIGHT + 0.055
SHOULDER_OUTER_WIDTH = 0.118
SHOULDER_PLATE_THICKNESS = 0.022
SHOULDER_GAP = SHOULDER_OUTER_WIDTH - 2.0 * SHOULDER_PLATE_THICKNESS
SHOULDER_BOSS_RADIUS = 0.056

UPPER_LINK_LENGTH = 0.64
UPPER_LINK_WIDTH = 0.056
UPPER_ROOT_RADIUS = 0.048

ELBOW_OUTER_WIDTH = 0.086
ELBOW_PLATE_THICKNESS = 0.020
ELBOW_GAP = ELBOW_OUTER_WIDTH - 2.0 * ELBOW_PLATE_THICKNESS
ELBOW_BOSS_RADIUS = 0.046

FOREARM_LENGTH = 0.42
FOREARM_WIDTH = 0.044
FOREARM_ROOT_RADIUS = 0.041

WRIST_HOUSING_OUTER_RADIUS = 0.060
WRIST_HOUSING_INNER_RADIUS = 0.0415
WRIST_HOUSING_LENGTH = 0.052
TOOL_HUB_RADIUS = WRIST_HOUSING_INNER_RADIUS
TOOL_FLANGE_RADIUS = 0.072
TOOL_FLANGE_THICKNESS = 0.016
TOOL_NOSE_RADIUS = 0.028
TOOL_NOSE_LENGTH = 0.020


def _pedestal_structure_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.030)
        .edges(">Z")
        .fillet(0.010)
    )

    plinth = (
        cq.Workplane("XY")
        .box(PLINTH_LENGTH, PLINTH_WIDTH, PLINTH_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + PLINTH_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.022)
    )

    column = (
        cq.Workplane("XY")
        .box(COLUMN_LENGTH, COLUMN_WIDTH, COLUMN_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + PLINTH_HEIGHT + COLUMN_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.028)
    )

    shoulder_body = (
        cq.Workplane("XY")
        .box(0.150, 0.150, 0.165)
        .translate((SHOULDER_X - 0.120, 0.0, SHOULDER_Z - 0.015))
        .edges("|Y")
        .fillet(0.016)
    )

    offset_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.060, BASE_THICKNESS + PLINTH_HEIGHT + 0.14),
                (-0.020, BASE_THICKNESS + PLINTH_HEIGHT + COLUMN_HEIGHT - 0.06),
                (SHOULDER_X - 0.200, SHOULDER_Z - 0.110),
                (SHOULDER_X - 0.070, SHOULDER_Z - 0.095),
                (SHOULDER_X - 0.070, SHOULDER_Z - 0.040),
                (SHOULDER_X - 0.205, SHOULDER_Z - 0.030),
                (-0.030, BASE_THICKNESS + PLINTH_HEIGHT + COLUMN_HEIGHT - 0.11),
                (-0.060, BASE_THICKNESS + PLINTH_HEIGHT + 0.20),
            ]
        )
        .close()
        .extrude(0.064, both=True)
    )

    yoke_center_y = SHOULDER_GAP / 2.0 + SHOULDER_PLATE_THICKNESS / 2.0
    yoke_plate = (
        cq.Workplane("XY")
        .box(0.095, SHOULDER_PLATE_THICKNESS, 0.145)
        .translate((SHOULDER_X + 0.010, yoke_center_y, SHOULDER_Z))
    )
    yoke_boss = (
        cq.Workplane("XZ")
        .center(SHOULDER_X, SHOULDER_Z)
        .circle(0.048)
        .extrude(SHOULDER_PLATE_THICKNESS)
        .translate((0.0, yoke_center_y - SHOULDER_PLATE_THICKNESS / 2.0 - 0.001, 0.0))
    )
    left_plate = yoke_plate.union(yoke_boss)
    right_plate = left_plate.mirror("XZ")

    top_bridge = (
        cq.Workplane("XY")
        .box(0.050, SHOULDER_OUTER_WIDTH, 0.022)
        .translate((SHOULDER_X - 0.030, 0.0, SHOULDER_Z + 0.082))
    )

    pedestal = (
        base_plate.union(plinth)
        .union(column)
        .union(shoulder_body)
        .union(offset_web)
        .union(left_plate)
        .union(right_plate)
        .union(top_bridge)
    )

    return pedestal


def _pedestal_cap_shape() -> cq.Workplane:
    cap_thickness = 0.006
    cap_center_y = SHOULDER_OUTER_WIDTH / 2.0 + cap_thickness / 2.0 - 0.001
    cap = (
        cq.Workplane("XZ")
        .center(SHOULDER_X, SHOULDER_Z)
        .circle(0.030)
        .extrude(cap_thickness)
        .translate((0.0, cap_center_y - cap_thickness / 2.0, 0.0))
    )
    return cap.union(cap.mirror("XZ"))


def _upper_link_structure_shape() -> cq.Workplane:
    shoulder_hub_width = SHOULDER_GAP - 0.010
    shoulder_hub = (
        cq.Workplane("XZ")
        .circle(0.032)
        .extrude(shoulder_hub_width / 2.0, both=True)
    )

    main_beam = (
        cq.Workplane("XY")
        .box(0.470, 0.050, 0.082)
        .translate((0.295, 0.0, 0.0))
        .edges("|Y")
        .fillet(0.010)
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.340, 0.028, 0.022)
        .translate((0.300, 0.0, -0.032))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(0.220, 0.024, 0.018)
        .translate((0.240, 0.0, 0.028))
    )

    clevis_center_y = ELBOW_GAP / 2.0 + ELBOW_PLATE_THICKNESS / 2.0
    elbow_transition = (
        cq.Workplane("XY")
        .box(0.060, ELBOW_OUTER_WIDTH, 0.072)
        .translate((UPPER_LINK_LENGTH - 0.095, 0.0, 0.0))
    )
    elbow_plate = (
        cq.Workplane("XY")
        .box(0.090, ELBOW_PLATE_THICKNESS, 0.112)
        .translate((UPPER_LINK_LENGTH - 0.005, clevis_center_y, 0.0))
    )
    elbow_boss = (
        cq.Workplane("XZ")
        .center(UPPER_LINK_LENGTH, 0.0)
        .circle(0.040)
        .extrude(ELBOW_PLATE_THICKNESS)
        .translate((0.0, clevis_center_y - ELBOW_PLATE_THICKNESS / 2.0 - 0.001, 0.0))
    )
    left_plate = elbow_plate.union(elbow_boss)
    right_plate = left_plate.mirror("XZ")

    return (
        shoulder_hub.union(main_beam)
        .union(lower_rib)
        .union(top_rib)
        .union(elbow_transition)
        .union(left_plate)
        .union(right_plate)
    )


def _upper_link_cover_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.240, 0.032, 0.020)
        .translate((0.255, 0.0, 0.034))
        .edges("|Y")
        .fillet(0.008)
    )


def _upper_link_cap_shape() -> cq.Workplane:
    cap_thickness = 0.006
    cap_y = ELBOW_OUTER_WIDTH / 2.0 + cap_thickness / 2.0 - 0.001
    cap = (
        cq.Workplane("XZ")
        .center(UPPER_LINK_LENGTH, 0.0)
        .circle(0.029)
        .extrude(cap_thickness)
        .translate((0.0, cap_y - cap_thickness / 2.0, 0.0))
    )
    return cap.union(cap.mirror("XZ"))


def _forearm_structure_shape() -> cq.Workplane:
    elbow_hub_width = ELBOW_GAP - 0.010
    elbow_hub = (
        cq.Workplane("XZ")
        .circle(0.032)
        .extrude(elbow_hub_width / 2.0, both=True)
    )

    main_beam = (
        cq.Workplane("XY")
        .box(0.275, 0.040, 0.070)
        .translate((0.195, 0.0, 0.0))
        .edges("|Y")
        .fillet(0.008)
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.190, 0.024, 0.018)
        .translate((0.190, 0.0, -0.028))
    )
    wrist_bridge = (
        cq.Workplane("XY")
        .box(0.060, FOREARM_WIDTH, 0.072)
        .translate((FOREARM_LENGTH - 0.070, 0.0, 0.0))
    )
    wrist_ring_outer = (
        cq.Workplane("YZ")
        .circle(0.055)
        .extrude(0.046, both=True)
        .translate((FOREARM_LENGTH, 0.0, 0.0))
    )
    wrist_ring_inner = (
        cq.Workplane("YZ")
        .circle(0.034)
        .extrude(0.048, both=True)
        .translate((FOREARM_LENGTH, 0.0, 0.0))
    )
    wrist_ring = wrist_ring_outer.cut(wrist_ring_inner)

    return elbow_hub.union(main_beam).union(lower_rib).union(wrist_bridge).union(wrist_ring)


def _forearm_cover_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.170, 0.026, 0.018)
        .translate((0.190, 0.0, 0.028))
        .edges("|Y")
        .fillet(0.007)
    )


def _tool_flange_structure_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("YZ")
        .circle(0.029)
        .extrude(0.030)
        .translate((-0.010, 0.0, 0.0))
    )
    flange = (
        cq.Workplane("YZ")
        .circle(TOOL_FLANGE_RADIUS)
        .extrude(TOOL_FLANGE_THICKNESS)
        .translate((0.026, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("YZ")
        .circle(TOOL_NOSE_RADIUS)
        .extrude(TOOL_NOSE_LENGTH)
        .translate(
            (
                0.026 + TOOL_FLANGE_THICKNESS,
                0.0,
                0.0,
            )
        )
    )
    orientation_lug = (
        cq.Workplane("XY")
        .box(0.012, 0.022, 0.012)
        .translate((0.034, 0.0, 0.042))
    )

    body = hub.union(flange).union(nose).union(orientation_lug)

    bolt_cuts = (
        cq.Workplane("YZ")
        .pushPoints([(0.040, 0.0), (-0.040, 0.0), (0.0, 0.040), (0.0, -0.040)])
        .circle(0.0055)
        .extrude(0.080, both=True)
        .translate((0.026 + TOOL_FLANGE_THICKNESS / 2.0, 0.0, 0.0))
    )
    key_flat = (
        cq.Workplane("XY")
        .box(0.020, 0.040, 0.020)
        .translate((0.026 + TOOL_FLANGE_THICKNESS / 2.0, 0.0, 0.064))
    )
    return body.cut(bolt_cuts).cut(key_flat)


def _tool_flange_face_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(0.064)
        .circle(0.048)
        .extrude(0.004 / 2.0, both=True)
        .translate((0.026 + TOOL_FLANGE_THICKNESS + 0.001, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_arm")

    model.material("cast_iron", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("arm_gray", rgba=(0.54, 0.57, 0.61, 1.0))
    model.material("cover_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("machined_cap", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("graphite", rgba=(0.24, 0.25, 0.28, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_structure_shape(), "pedestal_structure"),
        material="cast_iron",
        name="pedestal_structure",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.36, 0.28, 0.86)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_link_structure_shape(), "upper_link_structure"),
        material="arm_gray",
        name="upper_link_structure",
    )
    upper_link.visual(
        mesh_from_cadquery(_upper_link_cover_shape(), "upper_link_cover"),
        material="cover_black",
        name="upper_link_cover",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.66, 0.10, 0.16)),
        mass=12.0,
        origin=Origin(xyz=(0.32, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_structure_shape(), "forearm_structure"),
        material="arm_gray",
        name="forearm_structure",
    )
    forearm.visual(
        mesh_from_cadquery(_forearm_cover_shape(), "forearm_cover"),
        material="cover_black",
        name="forearm_cover",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.44, 0.08, 0.12)),
        mass=6.8,
        origin=Origin(xyz=(0.21, 0.0, 0.0)),
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        mesh_from_cadquery(_tool_flange_structure_shape(), "tool_flange_structure"),
        material="machined_cap",
        name="tool_flange_structure",
    )
    tool_flange.inertial = Inertial.from_geometry(
        Box((0.085, 0.145, 0.145)),
        mass=1.3,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.18, effort=220.0, velocity=0.9),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.45, upper=1.10, effort=140.0, velocity=1.2),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=tool_flange,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=50.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    tool_flange = object_model.get_part("tool_flange")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        pedestal,
        upper_link,
        elem_a="pedestal_structure",
        elem_b="upper_link_structure",
        reason="Shoulder trunnion is intentionally nested within the pedestal yoke volume at the revolute axis.",
    )
    ctx.allow_overlap(
        upper_link,
        forearm,
        elem_a="upper_link_structure",
        elem_b="forearm_structure",
        reason="Elbow pin housing intentionally nests the forearm trunnion inside the upper-link clevis cheeks.",
    )
    ctx.allow_overlap(
        forearm,
        tool_flange,
        elem_a="forearm_structure",
        elem_b="tool_flange_structure",
        reason="Wrist pilot hub intentionally seats inside the forearm bearing housing around the tool axis.",
    )

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

    ctx.expect_contact(pedestal, upper_link, name="shoulder_joint_is_grounded")
    ctx.expect_contact(upper_link, forearm, name="elbow_joint_is_grounded")
    ctx.expect_contact(forearm, tool_flange, name="wrist_joint_is_grounded")

    ctx.check(
        "joint_axes_match_prompt",
        shoulder.axis == (0.0, 1.0, 0.0)
        and elbow.axis == (0.0, 1.0, 0.0)
        and wrist.axis == (1.0, 0.0, 0.0),
        f"shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}",
    )
    ctx.check(
        "joint_limits_are_reasonable",
        shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and wrist.motion_limits is not None
        and shoulder.motion_limits.lower < 0.0 < shoulder.motion_limits.upper
        and elbow.motion_limits.lower < 0.0 < elbow.motion_limits.upper
        and wrist.motion_limits.lower < -2.0 < wrist.motion_limits.upper,
        "Expected real service-arm sweep at shoulder, elbow, and wrist.",
    )

    pedestal_aabb = ctx.part_world_aabb(pedestal)
    upper_aabb = ctx.part_world_aabb(upper_link)
    forearm_aabb = ctx.part_world_aabb(forearm)
    if pedestal_aabb and upper_aabb and forearm_aabb:
        pedestal_height = pedestal_aabb[1][2] - pedestal_aabb[0][2]
        upper_span = upper_aabb[1][0] - upper_aabb[0][0]
        forearm_span = forearm_aabb[1][0] - forearm_aabb[0][0]
        ctx.check(
            "service_arm_proportions",
            pedestal_height > 0.75 and upper_span > forearm_span > 0.30,
            f"pedestal_height={pedestal_height:.3f}, upper_span={upper_span:.3f}, forearm_span={forearm_span:.3f}",
        )

    with ctx.pose({shoulder: 0.75, elbow: -1.05, wrist: 1.20}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_reach_pose")

    with ctx.pose({shoulder: -0.30, elbow: 0.95, wrist: -1.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
