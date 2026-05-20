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


BARREL_RADIUS = 0.012
BARREL_LENGTH = 0.016
CHEEK_THICKNESS = 0.0045
CHEEK_LENGTH = 0.024
FORK_HEIGHT = 0.028
BEAM_WIDTH = BARREL_LENGTH
BEAM_HEIGHT = 0.014

SPINE_HEIGHT = 0.38
SPINE_LENGTH = 0.04
FOOT_LENGTH = 0.18
FOOT_WIDTH = 0.08
FOOT_THICKNESS = 0.012

LINK_1_LENGTH = 0.22
LINK_2_LENGTH = 0.19
LINK_3_LENGTH = 0.15


def _add_fork_cheeks(part, *, joint_x: float, prefix: str, material: str) -> None:
    cheek_y = BARREL_LENGTH / 2.0 + CHEEK_THICKNESS / 2.0
    for side, sign in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((CHEEK_LENGTH, CHEEK_THICKNESS, FORK_HEIGHT)),
            origin=Origin(xyz=(joint_x, sign * cheek_y, 0.0)),
            material=material,
            name=f"{prefix}_{side}_cheek",
        )


def _build_spine(model: ArticulatedObject, *, material: str) -> object:
    spine = model.part("spine")
    spine.visual(
        Box((SPINE_LENGTH, BEAM_WIDTH, SPINE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BARREL_RADIUS + SPINE_LENGTH / 2.0),
                0.0,
                -SPINE_HEIGHT / 2.0,
            )
        ),
        material=material,
        name="spine_column",
    )
    spine.visual(
        Box((FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS)),
        origin=Origin(
            xyz=(
                -(BARREL_RADIUS + FOOT_LENGTH * 0.45),
                0.0,
                -(SPINE_HEIGHT + FOOT_THICKNESS / 2.0),
            )
        ),
        material=material,
        name="base_foot",
    )
    spine.visual(
        Box((0.05, BEAM_WIDTH, 0.05)),
        origin=Origin(
            xyz=(
                -(BARREL_RADIUS + 0.025),
                0.0,
                -(SPINE_HEIGHT - 0.025),
            ),
            rpy=(0.0, -0.55, 0.0),
        ),
        material=material,
        name="rear_brace",
    )
    _add_fork_cheeks(spine, joint_x=0.0, prefix="shoulder", material=material)
    return spine


def _build_link(model: ArticulatedObject, name: str, length: float, *, material: str) -> object:
    link = model.part(name)
    beam_length = length - BARREL_RADIUS
    link.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name="prox_barrel",
    )
    link.visual(
        Box((beam_length, BEAM_WIDTH, BEAM_HEIGHT)),
        origin=Origin(xyz=(beam_length / 2.0, 0.0, 0.0)),
        material=material,
        name="beam",
    )
    _add_fork_cheeks(link, joint_x=length, prefix="distal", material=material)
    return link


def _build_platform_bracket(model: ArticulatedObject, *, material: str, bracket_material: str) -> object:
    bracket = model.part("platform_bracket")
    bracket.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name="prox_barrel",
    )
    bracket.visual(
        Box((0.052, BEAM_WIDTH, BEAM_HEIGHT)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=material,
        name="stem",
    )
    bracket.visual(
        Box((0.014, BEAM_WIDTH, 0.034)),
        origin=Origin(xyz=(0.047, 0.0, 0.017)),
        material=material,
        name="riser",
    )
    bracket.visual(
        Box((0.062, 0.038, 0.006)),
        origin=Origin(xyz=(0.075, 0.0, 0.034)),
        material=bracket_material,
        name="platform",
    )
    bracket.visual(
        Box((0.006, 0.038, 0.018)),
        origin=Origin(xyz=(0.103, 0.0, 0.028)),
        material=bracket_material,
        name="front_lip",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        bracket.visual(
            Box((0.03, 0.004, 0.018)),
            origin=Origin(xyz=(0.079, sign * 0.017, 0.028)),
            material=bracket_material,
            name=f"{side}_rail",
        )
    return bracket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_foldout_arm")

    model.material("powder_coat", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("bracket_plate", rgba=(0.62, 0.64, 0.66, 1.0))

    spine = _build_spine(model, material="powder_coat")
    link_1 = _build_link(model, "link_1", LINK_1_LENGTH, material="powder_coat")
    link_2 = _build_link(model, "link_2", LINK_2_LENGTH, material="powder_coat")
    link_3 = _build_link(model, "link_3", LINK_3_LENGTH, material="powder_coat")
    platform = _build_platform_bracket(
        model,
        material="powder_coat",
        bracket_material="bracket_plate",
    )

    joint_limits = MotionLimits(effort=18.0, velocity=1.5, lower=-1.65, upper=1.65)

    model.articulation(
        "spine_to_link_1",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "link_3_to_platform_bracket",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=platform,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=-1.8,
            upper=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    platform = object_model.get_part("platform_bracket")

    spine_to_link_1 = object_model.get_articulation("spine_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_platform = object_model.get_articulation("link_3_to_platform_bracket")

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

    for parent, child, label in (
        (spine, link_1, "shoulder_joint"),
        (link_1, link_2, "elbow_1_joint"),
        (link_2, link_3, "elbow_2_joint"),
        (link_3, platform, "wrist_joint"),
    ):
        ctx.expect_contact(child, parent, name=f"{label}_contact")

    joints = (
        spine_to_link_1,
        link_1_to_link_2,
        link_2_to_link_3,
        link_3_to_platform,
    )
    ctx.check(
        "parallel_revolute_axes",
        all(joint.axis == (0.0, -1.0, 0.0) for joint in joints),
        details=f"joint axes were {[joint.axis for joint in joints]}",
    )

    with ctx.pose(
        {
            spine_to_link_1: 0.55,
            link_1_to_link_2: 0.45,
            link_2_to_link_3: -0.35,
            link_3_to_platform: 0.3,
        }
    ):
        platform_pos = ctx.part_world_position(platform)
        deployed_ok = (
            platform_pos is not None
            and platform_pos[0] > 0.34
            and platform_pos[2] > 0.16
        )
        ctx.check(
            "platform_deploys_forward_and_up",
            deployed_ok,
            details=f"platform bracket origin in deployed pose: {platform_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
