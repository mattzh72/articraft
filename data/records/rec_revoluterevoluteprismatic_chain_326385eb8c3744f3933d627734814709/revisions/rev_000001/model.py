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


BASE_LENGTH = 0.220
BASE_WIDTH = 0.160
BASE_THICKNESS = 0.014

SHOULDER_AXIS_X = 0.085
SHOULDER_AXIS_Z = 0.095
SHOULDER_EAR_GAP = 0.034
SHOULDER_EAR_THICKNESS = 0.016
SHOULDER_EAR_OUTER_Y = SHOULDER_EAR_GAP / 2.0 + SHOULDER_EAR_THICKNESS

UPPER_LINK_LENGTH = 0.225
ELBOW_EAR_GAP = 0.028
ELBOW_EAR_THICKNESS = 0.011
ELBOW_EAR_OUTER_Y = ELBOW_EAR_GAP / 2.0 + ELBOW_EAR_THICKNESS

FOREARM_NOSE_X = 0.110
PROBE_TRAVEL = 0.075


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate(center)
    )


def _x_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _side_rib(
    points_xz: list[tuple[float, float]],
    *,
    y_center: float,
    width: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points_xz)
        .close()
        .extrude(width)
        .translate((0.0, y_center - width / 2.0, 0.0))
    )


def _shoulder_block_shape() -> cq.Workplane:
    plate = _box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS), (0.0, 0.0, BASE_THICKNESS / 2.0))
    pedestal = _box((0.082, 0.072, 0.060), (-0.030, 0.0, 0.044))
    crossbar = _box((0.028, 0.060, 0.020), (0.030, 0.0, 0.068))

    ear_left = _box(
        (0.018, SHOULDER_EAR_THICKNESS, 0.046),
        (SHOULDER_AXIS_X, 0.025, SHOULDER_AXIS_Z),
    )
    ear_right = _box(
        (0.018, SHOULDER_EAR_THICKNESS, 0.046),
        (SHOULDER_AXIS_X, -0.025, SHOULDER_AXIS_Z),
    )
    boss_left = _y_cylinder(0.011, SHOULDER_EAR_THICKNESS, (SHOULDER_AXIS_X, 0.039, SHOULDER_AXIS_Z))
    boss_right = _y_cylinder(0.011, SHOULDER_EAR_THICKNESS, (SHOULDER_AXIS_X, -0.039, SHOULDER_AXIS_Z))

    rib_left = _side_rib([(-0.050, BASE_THICKNESS), (-0.006, BASE_THICKNESS), (0.074, 0.074), (0.022, 0.074)], y_center=0.030, width=0.010)
    rib_right = _side_rib([(-0.050, BASE_THICKNESS), (-0.006, BASE_THICKNESS), (0.074, 0.074), (0.022, 0.074)], y_center=-0.030, width=0.010)

    body = plate.union(pedestal).union(crossbar).union(ear_left).union(ear_right).union(boss_left).union(boss_right).union(rib_left).union(rib_right)
    body = body.cut(_box((0.104, 0.042, 0.040), (0.056, 0.0, 0.084)))

    for x_pos in (-0.076, 0.076):
        for y_pos in (-0.054, 0.054):
            body = body.cut(cq.Workplane("XY").circle(0.008).extrude(BASE_THICKNESS + 0.006).translate((x_pos, y_pos, -0.003)))

    body = body.cut(_y_cylinder(0.0095, 0.108, (SHOULDER_AXIS_X, 0.0, SHOULDER_AXIS_Z)))
    return body


def _upper_link_shape() -> cq.Workplane:
    root_tongue = _box((0.016, 0.034, 0.034), (0.008, 0.0, 0.0))
    beam_shell = _box((0.146, 0.022, 0.032), (0.091, 0.0, 0.0))
    beam_pocket = _box((0.102, 0.010, 0.016), (0.095, 0.0, 0.0))
    top_rib = _box((0.134, 0.018, 0.006), (0.095, 0.0, 0.013))
    bottom_rib = _box((0.134, 0.018, 0.006), (0.095, 0.0, -0.013))
    distal_bridge = _box((0.018, 0.024, 0.018), (0.191, 0.0, 0.0))
    clevis_left = _box(
        (0.016, ELBOW_EAR_THICKNESS, 0.036),
        (UPPER_LINK_LENGTH - 0.008, 0.019, 0.0),
    )
    clevis_right = _box(
        (0.016, ELBOW_EAR_THICKNESS, 0.036),
        (UPPER_LINK_LENGTH - 0.008, -0.019, 0.0),
    )

    body = root_tongue.union(beam_shell).union(distal_bridge).union(clevis_left).union(clevis_right).union(top_rib).union(bottom_rib)
    body = body.cut(beam_pocket)
    body = body.cut(_box((0.022, 0.020, 0.028), (UPPER_LINK_LENGTH - 0.002, 0.0, 0.0)))
    body = body.cut(_y_cylinder(0.0082, 0.054, (UPPER_LINK_LENGTH, 0.0, 0.0)))
    return body


def _forearm_link_shape() -> cq.Workplane:
    root_tongue = _box((0.016, 0.028, 0.028), (0.008, 0.0, 0.0))
    beam = _box((0.090, 0.016, 0.024), (0.061, 0.0, 0.0))
    beam_pocket = _box((0.054, 0.008, 0.012), (0.063, 0.0, 0.0))

    guide_left = _box((0.132, 0.006, 0.028), (0.176, 0.017, 0.0))
    guide_right = _box((0.132, 0.006, 0.028), (0.176, -0.017, 0.0))
    guide_top = _box((0.132, 0.028, 0.006), (0.176, 0.0, 0.015))
    guide_bottom = _box((0.132, 0.028, 0.006), (0.176, 0.0, -0.015))
    guide_front_bridge = _box((0.012, 0.024, 0.032), (0.242, 0.0, 0.0))
    rear_stop = _box((0.008, 0.016, 0.016), (0.108, 0.0, 0.0))
    nose_opening = _x_cylinder(0.0060, 0.014, (0.248, 0.0, 0.0))

    body = root_tongue.union(beam).union(guide_left).union(guide_right).union(guide_top).union(guide_bottom).union(guide_front_bridge).union(rear_stop)
    body = body.cut(beam_pocket)
    body = body.cut(nose_opening)
    body = body.cut(_box((0.124, 0.024, 0.022), (0.176, 0.0, 0.0)))
    body = body.cut(_y_cylinder(0.0082, 0.050, (0.0, 0.0, 0.0)))
    return body


def _probe_stage_shape() -> cq.Workplane:
    collar = _box((0.008, 0.020, 0.020), (0.004, 0.0, 0.0))
    front_rod = _x_cylinder(0.0046, 0.146, (0.081, 0.0, 0.0))
    tip_stem = _x_cylinder(0.0038, 0.018, (0.163, 0.0, 0.0))
    tip_ball = cq.Workplane("XY").sphere(0.0052).translate((0.174, 0.0, 0.0))
    return collar.union(front_rod).union(tip_stem).union(tip_ball)


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    inertial_box: tuple[float, float, float],
    inertial_center: tuple[float, float, float],
) -> object:
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name="body")
    part.inertial = Inertial.from_geometry(
        Box(inertial_box),
        mass=max(0.15, inertial_box[0] * inertial_box[1] * inertial_box[2] * 2800.0),
        origin=Origin(xyz=inertial_center),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_probe_manipulator")

    model.material("base_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("anodized_aluminum", rgba=(0.69, 0.72, 0.75, 1.0))
    model.material("machined_forearm", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("probe_steel", rgba=(0.79, 0.81, 0.84, 1.0))

    shoulder_block = _add_mesh_part(
        model,
        name="shoulder_block",
        shape=_shoulder_block_shape(),
        mesh_name="shoulder_block",
        material="base_steel",
        inertial_box=(BASE_LENGTH, BASE_WIDTH, 0.143),
        inertial_center=(0.0, 0.0, 0.0715),
    )
    upper_link = _add_mesh_part(
        model,
        name="upper_link",
        shape=_upper_link_shape(),
        mesh_name="upper_link",
        material="anodized_aluminum",
        inertial_box=(0.255, 0.084, 0.058),
        inertial_center=(0.128, 0.0, 0.0),
    )
    forearm_link = _add_mesh_part(
        model,
        name="forearm_link",
        shape=_forearm_link_shape(),
        mesh_name="forearm_link",
        material="machined_forearm",
        inertial_box=(0.240, 0.074, 0.064),
        inertial_center=(0.120, 0.0, 0.0),
    )
    probe_stage = _add_mesh_part(
        model,
        name="probe_stage",
        shape=_probe_stage_shape(),
        mesh_name="probe_stage",
        material="probe_steel",
        inertial_box=(0.155, 0.032, 0.032),
        inertial_center=(0.020, 0.0, 0.0),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_block,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=1.10, effort=55.0, velocity=1.2),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm_link,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.45, upper=0.25, effort=40.0, velocity=1.6),
    )
    model.articulation(
        "probe_extend",
        ArticulationType.PRISMATIC,
        parent=forearm_link,
        child=probe_stage,
        origin=Origin(xyz=(FOREARM_NOSE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=PROBE_TRAVEL, effort=18.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder_block = object_model.get_part("shoulder_block")
    upper_link = object_model.get_part("upper_link")
    forearm_link = object_model.get_part("forearm_link")
    probe_stage = object_model.get_part("probe_stage")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    probe_extend = object_model.get_articulation("probe_extend")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        shoulder_block,
        upper_link,
        reason="Shoulder journal engagement is represented with shared hinge-volume occupancy at the clevis bore.",
    )
    ctx.allow_overlap(
        upper_link,
        forearm_link,
        reason="Elbow tongue and clevis are modeled with engaged hinge-journal volume rather than a separate pin part.",
    )
    ctx.allow_overlap(
        forearm_link,
        probe_stage,
        reason="The probe stage is represented as a nested carriage running inside the enclosed forearm guide sleeve.",
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

    ctx.check("shoulder_axis", shoulder_pitch.axis == (0.0, -1.0, 0.0), details=f"axis={shoulder_pitch.axis}")
    ctx.check("elbow_axis", elbow_pitch.axis == (0.0, -1.0, 0.0), details=f"axis={elbow_pitch.axis}")
    ctx.check("probe_axis", probe_extend.axis == (1.0, 0.0, 0.0), details=f"axis={probe_extend.axis}")

    ctx.expect_contact(upper_link, shoulder_block, name="shoulder_joint_supported")
    ctx.expect_contact(forearm_link, upper_link, name="elbow_joint_supported")
    ctx.expect_contact(probe_stage, forearm_link, name="probe_home_stop_contact")
    ctx.expect_overlap(probe_stage, forearm_link, axes="yz", min_overlap=0.014, name="probe_aligned_with_forearm_guide")

    with ctx.pose({shoulder_pitch: 0.82, elbow_pitch: -1.08}):
        ctx.expect_contact(forearm_link, upper_link, name="elbow_supported_in_fold_pose")

    with ctx.pose({probe_extend: PROBE_TRAVEL}):
        ctx.expect_overlap(probe_stage, forearm_link, axes="yz", min_overlap=0.012, name="probe_remains_guided_when_extended")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
