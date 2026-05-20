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


PLATE_W = 0.22
PLATE_H = 0.44
PLATE_T = 0.016

SHOULDER_Z = 0.06
SHOULDER_PIVOT_X = 0.084
SHOULDER_GAP = 0.058
SHOULDER_EAR_LEN = 0.022
SHOULDER_EAR_R = 0.040
SHOULDER_HUB_R = 0.030

UPPER_ARM_LEN = 0.32
UPPER_ARM_BODY_W = 0.072
UPPER_ARM_BODY_H = 0.120
ELBOW_GAP = 0.048
ELBOW_HUB_R = 0.026

FOREARM_LEN = 0.24
FOREARM_BODY_W = 0.060
FOREARM_BODY_H = 0.090
WRIST_GAP = 0.036
WRIST_HUB_R = 0.022


def _backplate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H, centered=(False, True, True))

    shoulder_block = (
        cq.Workplane("XY")
        .box(0.050, 0.088, 0.126, centered=(False, True, True))
        .translate((PLATE_T, 0.0, SHOULDER_Z))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(0.058, 0.050, 0.028, centered=(False, True, True))
        .translate((PLATE_T, 0.0, SHOULDER_Z + 0.068))
    )
    bottom_rib = (
        cq.Workplane("XY")
        .box(0.058, 0.050, 0.028, centered=(False, True, True))
        .translate((PLATE_T, 0.0, SHOULDER_Z - 0.068))
    )
    pivot_collar = (
        cq.Workplane("YZ")
        .circle(0.040)
        .extrude(0.018)
        .translate((SHOULDER_PIVOT_X - 0.018, 0.0, SHOULDER_Z))
    )
    collar_blend = (
        cq.Workplane("XY")
        .box(0.024, 0.064, 0.084, centered=(False, True, True))
        .translate((SHOULDER_PIVOT_X - 0.032, 0.0, SHOULDER_Z))
    )

    bolt_pattern = [(-0.075, 0.145), (0.075, 0.145), (-0.075, -0.145), (0.075, -0.145)]
    shape = plate.union(shoulder_block).union(top_rib).union(bottom_rib).union(collar_blend).union(pivot_collar)
    for y_pos, z_pos in bolt_pattern:
        shape = shape.union(
            cq.Workplane("XY")
            .box(0.006, 0.026, 0.026, centered=(False, True, True))
            .translate((PLATE_T - 0.003, y_pos, z_pos))
        )
    return shape


def _upper_arm_shape() -> cq.Workplane:
    root_hub = cq.Workplane("YZ").circle(0.034).extrude(0.020)
    root_web = (
        cq.Workplane("XY")
        .box(0.046, 0.064, 0.094, centered=(False, True, True))
        .translate((0.014, 0.0, 0.0))
    )
    main_beam = (
        cq.Workplane("XY")
        .box(0.214, UPPER_ARM_BODY_W, UPPER_ARM_BODY_H, centered=(False, True, True))
        .translate((0.052, 0.0, 0.0))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(0.160, 0.040, 0.024, centered=(False, True, True))
        .translate((0.080, 0.0, 0.050))
    )
    elbow_web = (
        cq.Workplane("XY")
        .box(0.044, 0.070, 0.094, centered=(False, True, True))
        .translate((UPPER_ARM_LEN - 0.056, 0.0, 0.0))
    )
    elbow_collar = (
        cq.Workplane("YZ")
        .circle(0.030)
        .extrude(0.020)
        .translate((UPPER_ARM_LEN - 0.020, 0.0, 0.0))
    )

    shape = root_hub.union(root_web).union(main_beam).union(top_rib).union(elbow_web).union(elbow_collar)
    return shape


def _forearm_shape() -> cq.Workplane:
    root_hub = cq.Workplane("YZ").circle(0.028).extrude(0.018)
    root_web = (
        cq.Workplane("XY")
        .box(0.040, 0.056, 0.082, centered=(False, True, True))
        .translate((0.012, 0.0, 0.0))
    )
    main_beam = (
        cq.Workplane("XY")
        .box(0.164, FOREARM_BODY_W, FOREARM_BODY_H, centered=(False, True, True))
        .translate((0.038, 0.0, 0.0))
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.118, 0.034, 0.022, centered=(False, True, True))
        .translate((0.064, 0.0, -0.034))
    )
    wrist_web = (
        cq.Workplane("XY")
        .box(0.034, 0.056, 0.078, centered=(False, True, True))
        .translate((FOREARM_LEN - 0.046, 0.0, 0.0))
    )
    wrist_collar = (
        cq.Workplane("YZ")
        .circle(0.024)
        .extrude(0.016)
        .translate((FOREARM_LEN - 0.016, 0.0, 0.0))
    )

    shape = root_hub.union(root_web).union(main_beam).union(lower_rib).union(wrist_web).union(wrist_collar)
    return shape


def _wrist_shape() -> cq.Workplane:
    root_hub = cq.Workplane("YZ").circle(0.024).extrude(0.012)
    wrist_lug = cq.Workplane("XY").box(0.020, 0.044, 0.050, centered=(False, True, True))
    neck = (
        cq.Workplane("YZ")
        .circle(0.022)
        .extrude(0.034)
        .translate((0.012, 0.0, 0.0))
    )
    flange = (
        cq.Workplane("YZ")
        .circle(0.046)
        .extrude(0.014)
        .translate((0.046, 0.0, 0.0))
    )
    face_pad = (
        cq.Workplane("YZ")
        .rect(0.050, 0.050)
        .extrude(0.008)
        .translate((0.058, 0.0, 0.0))
    )

    shape = root_hub.union(wrist_lug).union(neck).union(flange).union(face_pad)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_three_joint_arm")

    model.material("mount_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("cast_aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("dark_anodized", rgba=(0.27, 0.29, 0.32, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_backplate_shape(), "backplate"),
        material="mount_steel",
        name="backplate_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm"),
        material="cast_aluminum",
        name="upper_arm_body",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm"),
        material="cast_aluminum",
        name="forearm_body",
    )

    wrist = model.part("wrist")
    wrist.visual(
        mesh_from_cadquery(_wrist_shape(), "wrist"),
        material="dark_anodized",
        name="wrist_body",
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_PIVOT_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4, lower=-0.55, upper=1.15),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-0.20, upper=1.50),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.2, lower=-1.30, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist_joint = object_model.get_articulation("wrist_pitch")

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

    ctx.expect_contact(backplate, upper_arm, contact_tol=0.0015, name="shoulder_joint_is_mounted")
    ctx.expect_contact(upper_arm, forearm, contact_tol=0.0015, name="elbow_joint_is_mounted")
    ctx.expect_contact(forearm, wrist, contact_tol=0.0015, name="wrist_joint_is_mounted")

    def _center_z(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    rest_wrist_z = _center_z("wrist", "wrist_body")

    with ctx.pose({shoulder: 0.80}):
        shoulder_lift_z = _center_z("wrist", "wrist_body")
    ctx.check(
        "shoulder_positive_motion_lifts_the_chain",
        rest_wrist_z is not None and shoulder_lift_z is not None and shoulder_lift_z > rest_wrist_z + 0.18,
        details=f"rest_z={rest_wrist_z}, lifted_z={shoulder_lift_z}",
    )

    with ctx.pose({elbow: 1.00}):
        elbow_lift_z = _center_z("wrist", "wrist_body")
    ctx.check(
        "elbow_positive_motion_bends_upward",
        rest_wrist_z is not None and elbow_lift_z is not None and elbow_lift_z > rest_wrist_z + 0.08,
        details=f"rest_z={rest_wrist_z}, elbow_z={elbow_lift_z}",
    )

    with ctx.pose({wrist_joint: 0.85}):
        wrist_lift_z = _center_z("wrist", "wrist_body")
    ctx.check(
        "wrist_positive_motion_tilts_the_flange_upward",
        rest_wrist_z is not None and wrist_lift_z is not None and wrist_lift_z > rest_wrist_z + 0.01,
        details=f"rest_z={rest_wrist_z}, wrist_z={wrist_lift_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
