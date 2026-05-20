from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


PLATE_THICKNESS = 0.014
PLATE_WIDTH = 0.170
PLATE_HEIGHT = 0.360

SHOULDER_AXIS_X = 0.078
SHOULDER_EAR_DEPTH = 0.026
SHOULDER_EAR_WIDTH = 0.074
SHOULDER_EAR_THICKNESS = 0.010
SHOULDER_GAP = 0.028
SHOULDER_HUB_RADIUS = 0.022
SHOULDER_BODY_END_X = 0.052
SHOULDER_BODY_WIDTH = 0.086
SHOULDER_BODY_HEIGHT = 0.088
SHOULDER_CONNECTOR_WIDTH = 0.050

UPPER_LINK_LENGTH = 0.400
UPPER_BEAM_START_X = 0.022
UPPER_BEAM_LENGTH = 0.300
UPPER_BEAM_WIDTH = 0.044
UPPER_BEAM_HEIGHT = 0.022
UPPER_ELBOW_BLOCK_LENGTH = 0.058
UPPER_ELBOW_BLOCK_WIDTH = 0.056
UPPER_ELBOW_BLOCK_HEIGHT = 0.032
UPPER_EAR_DEPTH = 0.026
UPPER_EAR_WIDTH = 0.052
UPPER_EAR_THICKNESS = 0.010
ELBOW_GAP = 0.026
UPPER_EAR_CONNECTOR_WIDTH = 0.040

DISTAL_LINK_LENGTH = 0.265
DISTAL_BEAM_START_X = 0.020
DISTAL_BEAM_LENGTH = 0.215
DISTAL_BEAM_WIDTH = 0.036
DISTAL_BEAM_HEIGHT = 0.020
DISTAL_HUB_RADIUS = 0.020
DISTAL_NOSE_RADIUS = 0.018
DISTAL_NOSE_LENGTH = 0.030


def _shoulder_ear_z() -> float:
    return SHOULDER_GAP / 2.0 + SHOULDER_EAR_THICKNESS / 2.0


def _elbow_ear_z() -> float:
    return ELBOW_GAP / 2.0 + UPPER_EAR_THICKNESS / 2.0


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_elbow_arm")

    model.material("powder_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("anodized_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("link_trim", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("fastener_dark", rgba=(0.30, 0.31, 0.33, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        material="powder_steel",
        name="wall_plate",
    )
    side_plate.visual(
        Box(
            (
                SHOULDER_BODY_END_X - PLATE_THICKNESS / 2.0,
                SHOULDER_BODY_WIDTH,
                SHOULDER_BODY_HEIGHT,
            )
        ),
        origin=Origin(xyz=((PLATE_THICKNESS / 2.0 + SHOULDER_BODY_END_X) / 2.0, 0.0, 0.0)),
        material="powder_steel",
        name="shoulder_block",
    )
    side_plate.visual(
        Box((SHOULDER_EAR_DEPTH, SHOULDER_EAR_WIDTH, SHOULDER_EAR_THICKNESS)),
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, _shoulder_ear_z())),
        material="powder_steel",
        name="upper_shoulder_ear",
    )
    side_plate.visual(
        Box((SHOULDER_EAR_DEPTH, SHOULDER_EAR_WIDTH, SHOULDER_EAR_THICKNESS)),
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, -_shoulder_ear_z())),
        material="powder_steel",
        name="lower_shoulder_ear",
    )
    side_plate.visual(
        Box(
            (
                SHOULDER_AXIS_X - SHOULDER_EAR_DEPTH / 2.0 - SHOULDER_BODY_END_X,
                SHOULDER_CONNECTOR_WIDTH,
                SHOULDER_EAR_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                (
                    SHOULDER_BODY_END_X
                    + SHOULDER_AXIS_X
                    - SHOULDER_EAR_DEPTH / 2.0
                )
                / 2.0,
                0.0,
                _shoulder_ear_z(),
            )
        ),
        material="powder_steel",
        name="upper_shoulder_bridge",
    )
    side_plate.visual(
        Box(
            (
                SHOULDER_AXIS_X - SHOULDER_EAR_DEPTH / 2.0 - SHOULDER_BODY_END_X,
                SHOULDER_CONNECTOR_WIDTH,
                SHOULDER_EAR_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                (
                    SHOULDER_BODY_END_X
                    + SHOULDER_AXIS_X
                    - SHOULDER_EAR_DEPTH / 2.0
                )
                / 2.0,
                0.0,
                -_shoulder_ear_z(),
            )
        ),
        material="powder_steel",
        name="lower_shoulder_bridge",
    )
    for y_pos, z_pos, name in (
        (-0.055, -0.120, "mount_bolt_ll"),
        (-0.055, 0.120, "mount_bolt_lu"),
        (0.055, -0.120, "mount_bolt_rl"),
        (0.055, 0.120, "mount_bolt_ru"),
    ):
        side_plate.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(-PLATE_THICKNESS / 2.0 + 0.003, y_pos, z_pos)),
            material="fastener_dark",
            name=name,
        )

    upper_elbow_block_center_x = UPPER_BEAM_START_X + UPPER_BEAM_LENGTH + UPPER_ELBOW_BLOCK_LENGTH / 2.0
    upper_ear_bridge_length = (
        UPPER_LINK_LENGTH - UPPER_EAR_DEPTH / 2.0 - (upper_elbow_block_center_x + UPPER_ELBOW_BLOCK_LENGTH / 2.0)
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=SHOULDER_HUB_RADIUS, length=SHOULDER_GAP),
        material="anodized_aluminum",
        name="shoulder_hub",
    )
    upper_link.visual(
        Box((UPPER_BEAM_LENGTH, UPPER_BEAM_WIDTH, UPPER_BEAM_HEIGHT)),
        origin=Origin(xyz=(UPPER_BEAM_START_X + UPPER_BEAM_LENGTH / 2.0, 0.0, 0.0)),
        material="anodized_aluminum",
        name="upper_beam",
    )
    upper_link.visual(
        Box((UPPER_ELBOW_BLOCK_LENGTH, UPPER_ELBOW_BLOCK_WIDTH, UPPER_ELBOW_BLOCK_HEIGHT)),
        origin=Origin(xyz=(upper_elbow_block_center_x, 0.0, 0.0)),
        material="anodized_aluminum",
        name="elbow_block",
    )
    upper_link.visual(
        Box((UPPER_EAR_DEPTH, UPPER_EAR_WIDTH, UPPER_EAR_THICKNESS)),
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, _elbow_ear_z())),
        material="anodized_aluminum",
        name="upper_elbow_ear",
    )
    upper_link.visual(
        Box((UPPER_EAR_DEPTH, UPPER_EAR_WIDTH, UPPER_EAR_THICKNESS)),
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, -_elbow_ear_z())),
        material="anodized_aluminum",
        name="lower_elbow_ear",
    )
    upper_link.visual(
        Box((upper_ear_bridge_length, UPPER_EAR_CONNECTOR_WIDTH, UPPER_EAR_THICKNESS)),
        origin=Origin(
            xyz=(
                (
                    upper_elbow_block_center_x
                    + UPPER_ELBOW_BLOCK_LENGTH / 2.0
                    + UPPER_LINK_LENGTH
                    - UPPER_EAR_DEPTH / 2.0
                )
                / 2.0,
                0.0,
                _elbow_ear_z(),
            )
        ),
        material="anodized_aluminum",
        name="upper_elbow_bridge",
    )
    upper_link.visual(
        Box((upper_ear_bridge_length, UPPER_EAR_CONNECTOR_WIDTH, UPPER_EAR_THICKNESS)),
        origin=Origin(
            xyz=(
                (
                    upper_elbow_block_center_x
                    + UPPER_ELBOW_BLOCK_LENGTH / 2.0
                    + UPPER_LINK_LENGTH
                    - UPPER_EAR_DEPTH / 2.0
                )
                / 2.0,
                0.0,
                -_elbow_ear_z(),
            )
        ),
        material="anodized_aluminum",
        name="lower_elbow_bridge",
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        Cylinder(radius=DISTAL_HUB_RADIUS, length=ELBOW_GAP),
        material="link_trim",
        name="elbow_hub",
    )
    distal_link.visual(
        Box((DISTAL_BEAM_LENGTH, DISTAL_BEAM_WIDTH, DISTAL_BEAM_HEIGHT)),
        origin=Origin(xyz=(DISTAL_BEAM_START_X + DISTAL_BEAM_LENGTH / 2.0, 0.0, 0.0)),
        material="link_trim",
        name="distal_beam",
    )
    distal_link.visual(
        Box((DISTAL_NOSE_LENGTH, 0.028, 0.028)),
        origin=Origin(xyz=(DISTAL_LINK_LENGTH - DISTAL_NOSE_LENGTH / 2.0, 0.0, 0.0)),
        material="link_trim",
        name="distal_tip",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=distal_link,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.8,
            lower=0.0,
            upper=1.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    upper_link = object_model.get_part("upper_link")
    distal_link = object_model.get_part("distal_link")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

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
        "shoulder joint axis is vertical",
        tuple(shoulder_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {tuple(shoulder_joint.axis)}",
    )
    ctx.check(
        "elbow joint axis is vertical",
        tuple(elbow_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {tuple(elbow_joint.axis)}",
    )
    ctx.expect_contact(
        upper_link,
        side_plate,
        name="upper link is physically seated in the shoulder clevis",
    )
    ctx.expect_contact(
        distal_link,
        upper_link,
        name="distal link is physically seated in the elbow clevis",
    )
    ctx.expect_origin_gap(
        upper_link,
        side_plate,
        axis="x",
        min_gap=SHOULDER_AXIS_X - 0.002,
        max_gap=SHOULDER_AXIS_X + 0.002,
        name="shoulder pivot projects forward from the wall plate",
    )

    with ctx.pose({shoulder_joint: 0.85}):
        ctx.expect_contact(
            upper_link,
            side_plate,
            name="shoulder seating stays engaged through rotation",
        )
        upper_center = _aabb_center(ctx.part_world_aabb(upper_link))
        ctx.check(
            "positive shoulder rotation swings the upper link toward +y",
            upper_center is not None and upper_center[1] > 0.08,
            details=f"upper link center in rotated pose was {upper_center}",
        )

    with ctx.pose({shoulder_joint: 0.35, elbow_joint: 1.20}):
        ctx.expect_contact(
            distal_link,
            upper_link,
            name="elbow seating stays engaged when bent",
        )
        distal_center = _aabb_center(ctx.part_world_aabb(distal_link))
        ctx.check(
            "positive elbow rotation bends the distal link outward toward +y",
            distal_center is not None and distal_center[1] > 0.18,
            details=f"distal link center in bent pose was {distal_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
