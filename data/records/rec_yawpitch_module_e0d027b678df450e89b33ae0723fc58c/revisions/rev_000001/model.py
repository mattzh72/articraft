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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_DIAMETER = 0.140
BASE_RADIUS = BASE_DIAMETER / 2.0
BASE_FLANGE_THICKNESS = 0.012
BASE_BODY_RADIUS = 0.056
BASE_BODY_HEIGHT = 0.046
BASE_BEARING_RING_THICKNESS = 0.012
BASE_HEIGHT = BASE_FLANGE_THICKNESS + BASE_BODY_HEIGHT + BASE_BEARING_RING_THICKNESS
BASE_CENTER_BORE_RADIUS = 0.018

YAW_STAGE_DIAMETER = 0.104
YAW_STAGE_RADIUS = YAW_STAGE_DIAMETER / 2.0
YAW_STAGE_THICKNESS = 0.014
YAW_BEARING_RING_OUTER = 0.072
YAW_BEARING_RING_INNER = 0.044

FORK_ARM_CENTER_X = 0.048
FORK_ARM_THICKNESS = 0.010
FORK_ARM_DEPTH = 0.014
FORK_ARM_BASE_Z = 0.024
PITCH_AXIS_Z = 0.080
FORK_ARM_TOP_Z = 0.114

CRADLE_WIDTH = 0.060
CRADLE_DEPTH = 0.056
CRADLE_HEIGHT = 0.050
CRADLE_WALL = 0.0035
CRADLE_CENTER_Y = 0.018
CRADLE_CENTER_Z = 0.006
TRUNNION_BOSS_RADIUS = 0.007
TRUNNION_BOSS_LENGTH = 0.013
TRUNNION_WASHER_RADIUS = 0.011
TRUNNION_WASHER_THICKNESS = 0.003

YAW_LIMIT = 2.65
PITCH_LOWER = -0.80
PITCH_UPPER = 1.00


def _add_mesh_visual(part, shape: cq.Workplane, mesh_name: str, material: str, visual_name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def _make_base_rib(angle_deg: float) -> cq.Workplane:
    rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.042, BASE_FLANGE_THICKNESS),
                (0.067, BASE_FLANGE_THICKNESS),
                (0.058, BASE_FLANGE_THICKNESS + 0.030),
                (0.046, BASE_FLANGE_THICKNESS + 0.036),
                (0.042, BASE_FLANGE_THICKNESS + 0.020),
            ]
        )
        .close()
        .extrude(0.010)
        .translate((0.0, -0.005, 0.0))
    )
    return rib.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)


def _make_base_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_FLANGE_THICKNESS)
    body = (
        cq.Workplane("XY")
        .circle(BASE_BODY_RADIUS)
        .extrude(BASE_BODY_HEIGHT)
        .translate((0.0, 0.0, BASE_FLANGE_THICKNESS))
    )
    bearing_ring = (
        cq.Workplane("XY")
        .circle(0.061)
        .circle(0.039)
        .extrude(BASE_BEARING_RING_THICKNESS)
        .translate((0.0, 0.0, BASE_FLANGE_THICKNESS + BASE_BODY_HEIGHT))
    )
    top_cap = (
        cq.Workplane("XY")
        .circle(0.034)
        .extrude(0.004)
        .translate((0.0, 0.0, BASE_HEIGHT - 0.004))
    )
    lower_relief = (
        cq.Workplane("XY")
        .circle(BASE_BODY_RADIUS + 0.005)
        .circle(BASE_BODY_RADIUS - 0.005)
        .extrude(0.008)
        .translate((0.0, 0.0, BASE_FLANGE_THICKNESS + 0.008))
    )

    base = flange.union(body).union(bearing_ring).union(top_cap).union(lower_relief)
    for angle in (45.0, 135.0, 225.0, 315.0):
        base = base.union(_make_base_rib(angle))

    cable_bore = (
        cq.Workplane("XY")
        .circle(BASE_CENTER_BORE_RADIUS)
        .extrude(BASE_HEIGHT + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    service_pocket = (
        cq.Workplane("XZ")
        .rect(0.018, 0.016)
        .extrude(0.040)
        .translate((0.0, BASE_BODY_RADIUS - 0.018, BASE_FLANGE_THICKNESS + 0.022))
    )

    base = base.cut(cable_bore).cut(service_pocket)
    return base


def _make_fork_rib(side: float, y_center: float) -> cq.Workplane:
    sign = 1.0 if side >= 0.0 else -1.0
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (sign * 0.016, YAW_STAGE_THICKNESS),
                (sign * 0.028, YAW_STAGE_THICKNESS),
                (sign * 0.036, 0.032),
                (sign * (FORK_ARM_CENTER_X - 0.010), 0.054),
                (sign * (FORK_ARM_CENTER_X - 0.010), 0.074),
                (sign * 0.024, 0.060),
                (sign * 0.016, 0.040),
            ]
        )
        .close()
        .extrude(0.010)
        .translate((0.0, y_center - 0.005, 0.0))
    )


def _make_fork_shape() -> cq.Workplane:
    stage = cq.Workplane("XY").circle(YAW_STAGE_RADIUS).extrude(YAW_STAGE_THICKNESS)
    upper_bearing_ring = (
        cq.Workplane("XY")
        .circle(YAW_BEARING_RING_OUTER / 2.0)
        .circle(YAW_BEARING_RING_INNER / 2.0)
        .extrude(0.005)
        .translate((0.0, 0.0, YAW_STAGE_THICKNESS))
    )
    mast = (
        cq.Workplane("XY")
        .box(0.030, 0.022, 0.046, centered=(True, True, False))
        .translate((0.0, -0.030, YAW_STAGE_THICKNESS))
    )
    yoke_back = (
        cq.Workplane("XY")
        .box(2.0 * FORK_ARM_CENTER_X + FORK_ARM_THICKNESS, 0.010, 0.016, centered=(True, True, False))
        .translate((0.0, -0.024, 0.086))
    )
    fork = stage.union(upper_bearing_ring).union(mast).union(yoke_back)

    arm_height = FORK_ARM_TOP_Z - FORK_ARM_BASE_Z
    inner_pivot_ring_radius = 0.010
    pivot_hole_radius = 0.005
    for arm_center_x in (-FORK_ARM_CENTER_X, FORK_ARM_CENTER_X):
        arm = (
            cq.Workplane("XY")
            .box(FORK_ARM_THICKNESS, FORK_ARM_DEPTH, arm_height, centered=(True, True, False))
            .translate((arm_center_x, 0.0, FORK_ARM_BASE_Z))
        )
        shoulder = (
            cq.Workplane("XY")
            .box(0.018, 0.014, 0.022, centered=(True, True, False))
            .translate((arm_center_x * 0.78, -0.014, YAW_STAGE_THICKNESS))
        )
        inner_ring = (
            cq.Workplane("YZ")
            .circle(inner_pivot_ring_radius)
            .circle(pivot_hole_radius)
            .extrude(-0.002 if arm_center_x > 0.0 else 0.002)
            .translate((arm_center_x - (FORK_ARM_THICKNESS / 2.0), 0.0, PITCH_AXIS_Z))
        )
        outer_ring = (
            cq.Workplane("YZ")
            .circle(inner_pivot_ring_radius + 0.002)
            .circle(pivot_hole_radius)
            .extrude(0.002 if arm_center_x > 0.0 else -0.002)
            .translate((arm_center_x + (FORK_ARM_THICKNESS / 2.0), 0.0, PITCH_AXIS_Z))
        )
        fork = fork.union(arm).union(shoulder).union(inner_ring).union(outer_ring)

    for side in (-1.0, 1.0):
        fork = fork.union(_make_fork_rib(side, -0.024))

    center_bore = (
        cq.Workplane("XY")
        .circle(0.020)
        .extrude(YAW_STAGE_THICKNESS + 0.012)
        .translate((0.0, 0.0, -0.001))
    )
    open_window = (
        cq.Workplane("XY")
        .box(0.080, 0.028, 0.068, centered=(True, True, False))
        .translate((0.0, 0.018, 0.034))
    )
    fork = fork.cut(center_bore).cut(open_window)
    return fork


def _make_cradle_shape() -> cq.Workplane:
    body_width = 0.048
    body_depth = 0.044
    body_height = 0.036
    body_center_y = 0.032
    body_center_z = -0.004
    cheek_center_x = 0.033
    cheek_thickness = 0.004
    trunnion_radius = 0.010
    trunnion_stub_radius = 0.0045
    arm_inner_face_x = FORK_ARM_CENTER_X - (FORK_ARM_THICKNESS / 2.0)
    stub_length = arm_inner_face_x - (cheek_center_x + (cheek_thickness / 2.0))

    outer = (
        cq.Workplane("XY")
        .box(body_width, body_depth, body_height)
        .translate((0.0, body_center_y, body_center_z))
    )
    inner = (
        cq.Workplane("XY")
        .box(body_width - (2.0 * CRADLE_WALL), body_depth - (2.0 * CRADLE_WALL), body_height - (2.0 * CRADLE_WALL))
        .translate((0.0, body_center_y + 0.001, body_center_z))
    )
    shell = outer.cut(inner)

    front_opening = (
        cq.Workplane("XY")
        .box(body_width - 0.006, 0.012, body_height - 0.006)
        .translate((0.0, body_center_y + (body_depth / 2.0) - 0.006, body_center_z))
    )
    top_opening = (
        cq.Workplane("XY")
        .box(body_width - 0.008, body_depth - 0.010, 0.010)
        .translate((0.0, body_center_y, body_center_z + (body_height / 2.0) - 0.003))
    )
    shell = shell.cut(front_opening).cut(top_opening)

    bottom_rails = (
        cq.Workplane("XY")
        .box(0.034, 0.006, 0.006)
        .translate((0.012, 0.016, body_center_z - 0.014))
    )
    bottom_rails_mirror = bottom_rails.translate((-0.024, 0.0, 0.0))
    rear_stiffener = (
        cq.Workplane("XY")
        .box(0.052, 0.006, 0.008)
        .translate((0.0, 0.012, body_center_z))
    )
    shell = shell.union(bottom_rails).union(bottom_rails_mirror).union(rear_stiffener)

    cheek_profile = [
        (0.000, 0.000),
        (0.010, -0.010),
        (0.026, -0.018),
        (0.042, -0.018),
        (0.042, 0.016),
        (0.024, 0.020),
        (0.008, 0.014),
    ]
    right_cheek = (
        cq.Workplane("YZ")
        .polyline(cheek_profile)
        .close()
        .extrude(cheek_thickness)
        .translate((cheek_center_x - (cheek_thickness / 2.0), 0.0, 0.0))
    )
    left_cheek = (
        cq.Workplane("YZ")
        .polyline(cheek_profile)
        .close()
        .extrude(-cheek_thickness)
        .translate((-(cheek_center_x - (cheek_thickness / 2.0)), 0.0, 0.0))
    )
    right_trunnion = (
        cq.Workplane("YZ")
        .circle(trunnion_radius)
        .circle(trunnion_stub_radius)
        .extrude(stub_length)
        .translate((cheek_center_x + (cheek_thickness / 2.0), 0.0, 0.0))
    )
    left_trunnion = (
        cq.Workplane("YZ")
        .circle(trunnion_radius)
        .circle(trunnion_stub_radius)
        .extrude(-stub_length)
        .translate((-(cheek_center_x + (cheek_thickness / 2.0)), 0.0, 0.0))
    )

    shell = shell.union(right_cheek).union(left_cheek).union(right_trunnion).union(left_trunnion)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pan_tilt_module")

    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("alloy_gray", rgba=(0.70, 0.72, 0.75, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _make_base_shape(), "base_shell", "powder_black", "base_shell")
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    fork = model.part("fork")
    _add_mesh_visual(fork, _make_fork_shape(), "fork_shell", "graphite", "fork_shell")
    fork.inertial = Inertial.from_geometry(
        Box((0.126, 0.090, FORK_ARM_TOP_Z)),
        mass=1.10,
        origin=Origin(xyz=(0.0, -0.010, FORK_ARM_TOP_Z / 2.0)),
    )

    cradle = model.part("cradle")
    _add_mesh_visual(cradle, _make_cradle_shape(), "cradle_shell", "alloy_gray", "cradle_shell")
    cradle.inertial = Inertial.from_geometry(
        Box((0.093, 0.068, 0.064)),
        mass=0.58,
        origin=Origin(xyz=(0.0, CRADLE_CENTER_Y, CRADLE_CENTER_Z)),
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=2.2,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.0,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fork = object_model.get_part("fork")
    cradle = object_model.get_part("cradle")
    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        cradle,
        fork,
        reason=(
            "The pitch joint is modeled with captive cradle trunnions nested inside "
            "the fork bearing bosses, so the parent/child pair intentionally "
            "interpenetrates at the pivot."
        ),
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

    ctx.check(
        "yaw_axis_vertical",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        f"expected yaw axis (0,0,1), got {yaw_joint.axis}",
    )
    ctx.check(
        "pitch_axis_crosswise",
        tuple(pitch_joint.axis) == (1.0, 0.0, 0.0),
        f"expected pitch axis (1,0,0), got {pitch_joint.axis}",
    )
    ctx.check(
        "axes_distinct_and_stacked",
        tuple(yaw_joint.axis) != tuple(pitch_joint.axis) and pitch_joint.origin.xyz[2] > 0.050,
        (
            "expected separate lower yaw and upper pitch axes; "
            f"pitch origin={pitch_joint.origin.xyz}"
        ),
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0}):
        ctx.expect_contact(base, fork, contact_tol=0.001, name="fork_seated_on_base_ring")
        ctx.expect_overlap(fork, base, axes="xy", min_overlap=0.080, name="yaw_stage_centered_over_base")
        ctx.expect_contact(fork, cradle, contact_tol=0.001, name="cradle_supported_by_fork_trunnions")
        ctx.expect_gap(cradle, base, axis="z", min_gap=0.018, name="cradle_clear_of_base_at_rest")

    with ctx.pose({pitch_joint: PITCH_LOWER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_pitch_down_stop")
        ctx.expect_gap(cradle, base, axis="z", min_gap=0.018, name="cradle_clear_of_base_pitch_down")

    with ctx.pose({pitch_joint: PITCH_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_pitch_up_stop")
        ctx.expect_gap(cradle, base, axis="z", min_gap=0.010, name="cradle_clear_of_base_pitch_up")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
