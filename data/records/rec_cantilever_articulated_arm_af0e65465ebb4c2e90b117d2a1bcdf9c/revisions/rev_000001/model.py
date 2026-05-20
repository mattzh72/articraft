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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BACKPLATE_THICKNESS = 0.018
BACKPLATE_WIDTH = 0.110
BACKPLATE_HEIGHT = 0.300
BACKPLATE_CENTER = (-0.015, -0.091, 0.0)

HUB_RADIUS = 0.018

UPPER_ARM_LENGTH = 0.240
FOREARM_LENGTH = 0.200

SHOULDER_LIMITS = (-0.75, 1.15)
ELBOW_LIMITS = (-1.35, 1.20)
WRIST_LIMITS = (-0.90, 0.90)


def _y_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_mounted_cantilever_arm")

    model.material("powder_coat_dark", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("graphite_face", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("fastener_steel", rgba=(0.63, 0.66, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        Box((BACKPLATE_THICKNESS, BACKPLATE_WIDTH, BACKPLATE_HEIGHT)),
        origin=Origin(xyz=BACKPLATE_CENTER),
        material="powder_coat_dark",
        name="backplate",
    )
    base.visual(
        Box((0.024, 0.030, 0.220)),
        origin=Origin(xyz=(-0.013, -0.060, 0.0)),
        material="powder_coat_dark",
        name="support_post",
    )
    base.visual(
        Box((0.028, 0.034, 0.070)),
        origin=Origin(xyz=(-0.011, -0.042, 0.0)),
        material="powder_coat_dark",
        name="shoulder_block",
    )
    base.visual(
        Box((0.040, 0.016, 0.060)),
        origin=Origin(xyz=(-0.010, -0.021, 0.0)),
        material="powder_coat_dark",
        name="shoulder_head",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=_y_axis_origin((0.0, -0.018, 0.0)),
        material="powder_coat_dark",
        name="shoulder_support",
    )
    base.visual(
        Box((0.022, 0.010, 0.040)),
        origin=Origin(xyz=(-0.030, -0.012, 0.0)),
        material="powder_coat_dark",
        name="shoulder_gusset",
    )
    base.visual(
        Box((0.012, 0.060, 0.090)),
        origin=Origin(xyz=(-0.018, -0.055, 0.050), rpy=(0.49, 0.0, 0.0)),
        material="powder_coat_dark",
        name="upper_rib",
    )
    base.visual(
        Box((0.012, 0.060, 0.090)),
        origin=Origin(xyz=(-0.018, -0.055, -0.050), rpy=(-0.49, 0.0, 0.0)),
        material="powder_coat_dark",
        name="lower_rib",
    )
    bolt_positions = (
        (-0.004, BACKPLATE_CENTER[1] - 0.030, -0.095),
        (-0.004, BACKPLATE_CENTER[1] + 0.030, -0.095),
        (-0.004, BACKPLATE_CENTER[1] - 0.030, 0.095),
        (-0.004, BACKPLATE_CENTER[1] + 0.030, 0.095),
    )
    for index, bolt_xyz in enumerate(bolt_positions, start=1):
        base.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=bolt_xyz, rpy=(0.0, pi / 2.0, 0.0)),
            material="fastener_steel",
            name=f"bolt_head_{index}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=HUB_RADIUS, length=0.020),
        origin=_y_axis_origin((0.018, -0.004, 0.0)),
        material="machined_aluminum",
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.060, 0.020, 0.040)),
        origin=Origin(xyz=(0.030, 0.004, 0.0)),
        material="machined_aluminum",
        name="shoulder_body",
    )
    upper_arm.visual(
        Box((0.150, 0.012, 0.014)),
        origin=Origin(xyz=(0.125, 0.004, 0.018)),
        material="machined_aluminum",
        name="upper_rail",
    )
    upper_arm.visual(
        Box((0.150, 0.012, 0.014)),
        origin=Origin(xyz=(0.125, 0.004, -0.018)),
        material="machined_aluminum",
        name="lower_rail",
    )
    upper_arm.visual(
        Box((0.160, 0.010, 0.036)),
        origin=Origin(xyz=(0.125, 0.000, 0.0)),
        material="machined_aluminum",
        name="mid_web",
    )
    upper_arm.visual(
        Box((0.040, 0.018, 0.044)),
        origin=Origin(xyz=(0.220, -0.004, 0.0)),
        material="machined_aluminum",
        name="elbow_block",
    )
    upper_arm.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=_y_axis_origin((UPPER_ARM_LENGTH, -0.018, 0.0)),
        material="machined_aluminum",
        name="elbow_support",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=HUB_RADIUS, length=0.020),
        origin=_y_axis_origin((0.018, -0.004, 0.0)),
        material="machined_aluminum",
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.042, 0.018, 0.038)),
        origin=Origin(xyz=(0.021, 0.004, 0.0)),
        material="machined_aluminum",
        name="elbow_body",
    )
    forearm.visual(
        Box((0.136, 0.012, 0.014)),
        origin=Origin(xyz=(0.101, 0.004, 0.016)),
        material="machined_aluminum",
        name="upper_rail",
    )
    forearm.visual(
        Box((0.136, 0.012, 0.014)),
        origin=Origin(xyz=(0.101, 0.004, -0.016)),
        material="machined_aluminum",
        name="lower_rail",
    )
    forearm.visual(
        Box((0.166, 0.010, 0.032)),
        origin=Origin(xyz=(0.101, 0.000, 0.0)),
        material="machined_aluminum",
        name="mid_web",
    )
    forearm.visual(
        Box((0.034, 0.018, 0.040)),
        origin=Origin(xyz=(0.183, -0.004, 0.0)),
        material="machined_aluminum",
        name="wrist_block",
    )
    forearm.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=_y_axis_origin((FOREARM_LENGTH, -0.018, 0.0)),
        material="machined_aluminum",
        name="wrist_support",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=_y_axis_origin((0.016, -0.004, 0.0)),
        material="graphite_face",
        name="wrist_hub",
    )
    wrist.visual(
        Box((0.030, 0.018, 0.036)),
        origin=Origin(xyz=(0.015, 0.004, 0.0)),
        material="graphite_face",
        name="wrist_mount",
    )
    wrist.visual(
        Box((0.040, 0.024, 0.040)),
        origin=Origin(xyz=(0.040, 0.000, 0.0)),
        material="graphite_face",
        name="face_mount",
    )
    wrist.visual(
        Box((0.012, 0.064, 0.054)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material="graphite_face",
        name="wrist_face",
    )
    wrist.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="fastener_steel",
        name="wrist_boss",
    )

    model.articulation(
        "base_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
        ),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=26.0,
            velocity=1.8,
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
        ),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=WRIST_LIMITS[0],
            upper=WRIST_LIMITS[1],
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple(0.5 * (aabb[0][i] + aabb[1][i]) for i in range(3))

    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    shoulder = object_model.get_articulation("base_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

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

    ctx.expect_gap(
        upper_arm,
        base,
        axis="y",
        min_gap=0.012,
        negative_elem="backplate",
        name="backplate stays clearly to one side of shoulder link",
    )

    elbow_rest = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: SHOULDER_LIMITS[1]}):
        elbow_lifted = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder positive motion lifts the elbow",
        elbow_rest is not None and elbow_lifted is not None and elbow_lifted[2] > elbow_rest[2] + 0.10,
        details=f"rest={elbow_rest}, lifted={elbow_lifted}",
    )

    wrist_rest = ctx.part_world_position(wrist)
    with ctx.pose({elbow: ELBOW_LIMITS[1]}):
        wrist_lifted = ctx.part_world_position(wrist)
    ctx.check(
        "elbow positive motion lifts the wrist stage",
        wrist_rest is not None and wrist_lifted is not None and wrist_lifted[2] > wrist_rest[2] + 0.08,
        details=f"rest={wrist_rest}, lifted={wrist_lifted}",
    )

    wrist_face_rest = _aabb_center(ctx.part_element_world_aabb(wrist, elem="wrist_face"))
    with ctx.pose({wrist_joint: WRIST_LIMITS[1]}):
        wrist_face_tipped = _aabb_center(ctx.part_element_world_aabb(wrist, elem="wrist_face"))
    ctx.check(
        "wrist positive motion tips the face upward",
        wrist_face_rest is not None and wrist_face_tipped is not None and wrist_face_tipped[2] > wrist_face_rest[2] + 0.02,
        details=f"rest={wrist_face_rest}, tipped={wrist_face_tipped}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
