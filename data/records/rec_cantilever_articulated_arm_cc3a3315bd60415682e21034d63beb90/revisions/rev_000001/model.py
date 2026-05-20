from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_FOOT_X = 0.300
BASE_FOOT_Y = 0.240
BASE_FOOT_Z = 0.024

PEDESTAL_X = 0.190
PEDESTAL_Y = 0.150
PEDESTAL_Z = 0.098

SHOULDER_Z = 0.332
CHEEK_HEIGHT = 0.252
CHEEK_T = 0.028
SHOULDER_GAP = 0.074
CHEEK_Y = SHOULDER_GAP / 2.0 + CHEEK_T / 2.0
SHOULDER_HUB_LENGTH = SHOULDER_GAP

FIRST_LINK_LENGTH = 0.340
FIRST_LINK_WIDTH = 0.066
FIRST_LINK_HUB_RADIUS = 0.049
ELBOW_GAP = 0.070
ELBOW_CHEEK_T = 0.020
ELBOW_CHEEK_Y = ELBOW_GAP / 2.0 + ELBOW_CHEEK_T / 2.0
ELBOW_HUB_LENGTH = ELBOW_GAP

SECOND_LINK_LENGTH = 0.285
SECOND_LINK_WIDTH = 0.050
SECOND_LINK_HUB_RADIUS = 0.041

WRIST_FLANGE_RADIUS = 0.049
WRIST_BODY_RADIUS = 0.036
WRIST_LENGTH = 0.116


def _origin_xyz(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z))


def _origin_y_cylinder(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _origin_x_cylinder(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tending_arm")

    model.material("base_graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("cast_light", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("cast_mid", rgba=(0.64, 0.67, 0.71, 1.0))
    model.material("wrist_dark", rgba=(0.27, 0.29, 0.32, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((BASE_FOOT_X, BASE_FOOT_Y, BASE_FOOT_Z)),
        origin=_origin_xyz(0.0, 0.0, BASE_FOOT_Z / 2.0),
        material="base_graphite",
        name="foot",
    )
    base_frame.visual(
        Box((PEDESTAL_X, PEDESTAL_Y, PEDESTAL_Z)),
        origin=_origin_xyz(0.0, 0.0, BASE_FOOT_Z + PEDESTAL_Z / 2.0),
        material="base_graphite",
        name="pedestal",
    )
    base_frame.visual(
        Box((0.090, 0.006, 0.060)),
        origin=_origin_xyz(0.010, PEDESTAL_Y / 2.0 + 0.003, 0.082),
        material="cast_mid",
        name="left_inspection_cover",
    )
    base_frame.visual(
        Box((0.090, 0.006, 0.060)),
        origin=_origin_xyz(0.010, -PEDESTAL_Y / 2.0 - 0.003, 0.082),
        material="cast_mid",
        name="right_inspection_cover",
    )
    base_frame.visual(
        Box((0.074, 0.044, 0.150)),
        origin=_origin_xyz(-0.052, 0.046, 0.197),
        material="base_graphite",
        name="left_riser",
    )
    base_frame.visual(
        Box((0.074, 0.044, 0.150)),
        origin=_origin_xyz(-0.052, -0.046, 0.197),
        material="base_graphite",
        name="right_riser",
    )
    base_frame.visual(
        Box((0.036, 0.070, 0.126)),
        origin=_origin_xyz(-0.082, 0.0, 0.209),
        material="base_graphite",
        name="rear_spine",
    )
    base_frame.visual(
        Box((0.032, SHOULDER_GAP + 2.0 * CHEEK_T + 0.010, 0.026)),
        origin=_origin_xyz(-0.076, 0.0, 0.284),
        material="base_graphite",
        name="rear_bridge",
    )
    base_frame.visual(
        Box((0.094, CHEEK_T, CHEEK_HEIGHT)),
        origin=_origin_xyz(-0.030, CHEEK_Y, 0.246),
        material="base_graphite",
        name="shoulder_left_cheek",
    )
    base_frame.visual(
        Box((0.094, CHEEK_T, CHEEK_HEIGHT)),
        origin=_origin_xyz(-0.030, -CHEEK_Y, 0.246),
        material="base_graphite",
        name="shoulder_right_cheek",
    )
    base_frame.visual(
        Cylinder(radius=0.060, length=CHEEK_T),
        origin=_origin_y_cylinder(0.0, CHEEK_Y, SHOULDER_Z),
        material="cast_mid",
        name="shoulder_left_boss",
    )
    base_frame.visual(
        Cylinder(radius=0.060, length=CHEEK_T),
        origin=_origin_y_cylinder(0.0, -CHEEK_Y, SHOULDER_Z),
        material="cast_mid",
        name="shoulder_right_boss",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.30, 0.24, 0.38)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
    )

    first_link = model.part("first_link")
    first_link.visual(
        Cylinder(radius=FIRST_LINK_HUB_RADIUS, length=SHOULDER_HUB_LENGTH),
        origin=_origin_y_cylinder(0.0, 0.0, 0.0),
        material="cast_light",
        name="shoulder_hub",
    )
    first_link.visual(
        Box((0.240, FIRST_LINK_WIDTH, 0.086)),
        origin=_origin_xyz(0.160, 0.0, -0.010),
        material="cast_light",
        name="primary_beam",
    )
    first_link.visual(
        Box((0.120, 0.054, 0.040)),
        origin=_origin_xyz(0.105, 0.0, 0.040),
        material="cast_light",
        name="shoulder_hump",
    )
    first_link.visual(
        Box((0.110, 0.006, 0.060)),
        origin=_origin_xyz(0.155, FIRST_LINK_WIDTH / 2.0 + 0.0025, -0.004),
        material="cast_mid",
        name="left_link_cover",
    )
    first_link.visual(
        Box((0.110, 0.006, 0.060)),
        origin=_origin_xyz(0.155, -FIRST_LINK_WIDTH / 2.0 - 0.0025, -0.004),
        material="cast_mid",
        name="right_link_cover",
    )
    first_link.visual(
        Box((0.086, 0.016, 0.074)),
        origin=_origin_xyz(0.286, 0.040, -0.018),
        material="cast_light",
        name="elbow_left_web",
    )
    first_link.visual(
        Box((0.086, 0.016, 0.074)),
        origin=_origin_xyz(0.286, -0.040, -0.018),
        material="cast_light",
        name="elbow_right_web",
    )
    first_link.visual(
        Box((0.060, ELBOW_CHEEK_T, 0.118)),
        origin=_origin_xyz(0.314, ELBOW_CHEEK_Y, 0.0),
        material="cast_light",
        name="elbow_left_cheek",
    )
    first_link.visual(
        Box((0.060, ELBOW_CHEEK_T, 0.118)),
        origin=_origin_xyz(0.314, -ELBOW_CHEEK_Y, 0.0),
        material="cast_light",
        name="elbow_right_cheek",
    )
    first_link.visual(
        Cylinder(radius=0.043, length=ELBOW_CHEEK_T),
        origin=_origin_y_cylinder(FIRST_LINK_LENGTH, ELBOW_CHEEK_Y, 0.0),
        material="cast_mid",
        name="elbow_left_boss",
    )
    first_link.visual(
        Cylinder(radius=0.043, length=ELBOW_CHEEK_T),
        origin=_origin_y_cylinder(FIRST_LINK_LENGTH, -ELBOW_CHEEK_Y, 0.0),
        material="cast_mid",
        name="elbow_right_boss",
    )
    first_link.inertial = Inertial.from_geometry(
        Box((0.36, 0.10, 0.18)),
        mass=23.0,
        origin=Origin(xyz=(0.18, 0.0, -0.01)),
    )

    second_link = model.part("second_link")
    second_link.visual(
        Cylinder(radius=SECOND_LINK_HUB_RADIUS, length=ELBOW_HUB_LENGTH),
        origin=_origin_y_cylinder(0.0, 0.0, 0.0),
        material="cast_mid",
        name="elbow_hub",
    )
    second_link.visual(
        Box((0.200, SECOND_LINK_WIDTH, 0.068)),
        origin=_origin_xyz(0.140, 0.0, 0.0),
        material="cast_mid",
        name="secondary_beam",
    )
    second_link.visual(
        Box((0.120, 0.034, 0.036)),
        origin=_origin_xyz(0.130, 0.0, 0.028),
        material="cast_mid",
        name="upper_rib",
    )
    second_link.visual(
        Box((0.092, 0.006, 0.044)),
        origin=_origin_xyz(0.138, SECOND_LINK_WIDTH / 2.0 + 0.0025, 0.0),
        material="cast_light",
        name="left_second_cover",
    )
    second_link.visual(
        Box((0.092, 0.006, 0.044)),
        origin=_origin_xyz(0.138, -SECOND_LINK_WIDTH / 2.0 - 0.0025, 0.0),
        material="cast_light",
        name="right_second_cover",
    )
    second_link.visual(
        Cylinder(radius=0.039, length=0.060),
        origin=_origin_x_cylinder(0.255, 0.0, 0.0),
        material="cast_mid",
        name="wrist_collar",
    )
    second_link.visual(
        Cylinder(radius=0.048, length=0.008),
        origin=_origin_x_cylinder(0.272, 0.0, 0.0),
        material="cast_light",
        name="wrist_flange_ring",
    )
    second_link.inertial = Inertial.from_geometry(
        Box((0.30, 0.09, 0.12)),
        mass=14.0,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    wrist_nose = model.part("wrist_nose")
    wrist_nose.visual(
        Cylinder(radius=WRIST_FLANGE_RADIUS, length=0.012),
        origin=_origin_x_cylinder(0.006, 0.0, 0.0),
        material="wrist_dark",
        name="rear_flange",
    )
    wrist_nose.visual(
        Cylinder(radius=WRIST_BODY_RADIUS, length=0.078),
        origin=_origin_x_cylinder(0.051, 0.0, 0.0),
        material="wrist_dark",
        name="wrist_body",
    )
    wrist_nose.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=_origin_x_cylinder(0.095, 0.0, 0.0),
        material="cast_light",
        name="front_ring",
    )
    wrist_nose.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=_origin_x_cylinder(0.108, 0.0, 0.0),
        material="cast_light",
        name="tool_pilot",
    )
    wrist_nose.visual(
        Box((0.024, 0.016, 0.012)),
        origin=_origin_xyz(0.094, 0.0, 0.030),
        material="cast_light",
        name="clocking_key",
    )
    wrist_nose.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=WRIST_LENGTH),
        mass=4.8,
        origin=Origin(xyz=(WRIST_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=0.45, effort=420.0, velocity=1.3),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.55, upper=1.20, effort=260.0, velocity=1.6),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=wrist_nose,
        origin=Origin(xyz=(SECOND_LINK_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-3.10, upper=3.10, effort=110.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    wrist_nose = object_model.get_part("wrist_nose")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

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
        "shoulder and elbow cross-axes are parallel",
        tuple(round(v, 6) for v in shoulder.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 6) for v in elbow.axis) == (0.0, 1.0, 0.0),
        details=f"shoulder axis={shoulder.axis}, elbow axis={elbow.axis}",
    )
    ctx.check(
        "wrist rolls about tool axis",
        tuple(round(v, 6) for v in wrist.axis) == (1.0, 0.0, 0.0),
        details=f"wrist axis={wrist.axis}",
    )

    ctx.expect_contact(
        base_frame,
        first_link,
        elem_a="shoulder_left_cheek",
        elem_b="shoulder_hub",
        name="left shoulder trunnion cheek contacts hub",
    )
    ctx.expect_contact(
        base_frame,
        first_link,
        elem_a="shoulder_right_cheek",
        elem_b="shoulder_hub",
        name="right shoulder trunnion cheek contacts hub",
    )
    ctx.expect_contact(
        first_link,
        second_link,
        elem_a="elbow_left_cheek",
        elem_b="elbow_hub",
        name="left elbow trunnion cheek contacts hub",
    )
    ctx.expect_contact(
        first_link,
        second_link,
        elem_a="elbow_right_cheek",
        elem_b="elbow_hub",
        name="right elbow trunnion cheek contacts hub",
    )
    ctx.expect_contact(
        second_link,
        wrist_nose,
        elem_a="wrist_collar",
        elem_b="rear_flange",
        name="wrist rear flange seats on wrist collar",
    )

    with ctx.pose({shoulder: 0.30, elbow: -1.05, wrist: 1.40}):
        ctx.expect_contact(
            base_frame,
            first_link,
            elem_a="shoulder_left_cheek",
            elem_b="shoulder_hub",
            name="left shoulder cheek stays seated in service pose",
        )
        ctx.expect_contact(
            base_frame,
            first_link,
            elem_a="shoulder_right_cheek",
            elem_b="shoulder_hub",
            name="right shoulder cheek stays seated in service pose",
        )
        ctx.expect_contact(
            first_link,
            second_link,
            elem_a="elbow_left_cheek",
            elem_b="elbow_hub",
            name="left elbow cheek stays seated in service pose",
        )
        ctx.expect_contact(
            first_link,
            second_link,
            elem_a="elbow_right_cheek",
            elem_b="elbow_hub",
            name="right elbow cheek stays seated in service pose",
        )
        ctx.expect_contact(
            second_link,
            wrist_nose,
            elem_a="wrist_collar",
            elem_b="rear_flange",
            name="wrist rear flange stays seated in service pose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="service pose stays clear")

    with ctx.pose({shoulder: shoulder.motion_limits.upper, elbow: 0.0, wrist: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="raised pose stays clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
