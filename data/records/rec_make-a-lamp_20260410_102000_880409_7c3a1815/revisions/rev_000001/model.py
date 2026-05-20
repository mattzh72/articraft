from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_RADIUS = 0.095
BASE_THICKNESS = 0.018
BASE_FOOT_RADIUS = 0.074
BASE_FOOT_THICKNESS = 0.006
STEM_RADIUS = 0.011
STEM_HEIGHT = 0.300
SHOULDER_AXIS_Z = BASE_THICKNESS + STEM_HEIGHT + 0.026

HINGE_GAP = 0.022
HINGE_CHEEK_THICKNESS = 0.006
HINGE_SLEEVE_RADIUS = 0.011
HINGE_CHEEK_CENTER_Y = (HINGE_GAP * 0.5) + (HINGE_CHEEK_THICKNESS * 0.5)

ARM_WIDTH = 0.018
ARM_HEIGHT = 0.014
LOWER_ARM_LENGTH = 0.255
UPPER_ARM_LENGTH = 0.225

ARM_FORK_CHEEK_DEPTH = 0.020
ARM_FORK_CHEEK_HEIGHT = 0.028
ARM_FORK_BRIDGE_DEPTH = 0.008
ARM_FORK_BRIDGE_HEIGHT = 0.012
ARM_FORK_BRIDGE_OFFSET = 0.016

HEAD_NECK_LENGTH = 0.074
HEAD_NECK_RADIUS = 0.010
SHADE_PITCH = 2.18
SHADE_ORIGIN_X = 0.058
SHADE_ORIGIN_Z = -0.006


def _shade_mesh():
    outer_profile = [
        (0.015, 0.000),
        (0.022, 0.012),
        (0.037, 0.050),
        (0.056, 0.090),
        (0.072, 0.126),
    ]
    inner_profile = [
        (0.011, 0.000),
        (0.018, 0.012),
        (0.033, 0.050),
        (0.052, 0.090),
        (0.068, 0.124),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "shade_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_lamp")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    brass = model.material("brass", rgba=(0.68, 0.58, 0.31, 1.0))
    shade_white = model.material("shade_white", rgba=(0.93, 0.91, 0.86, 1.0))
    bulb_warm = model.material("bulb_warm", rgba=(0.98, 0.86, 0.63, 0.92))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=powder_black,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_THICKNESS * 0.5)),
        material=powder_black,
        name="foot_pad",
    )
    base.visual(
        Cylinder(radius=STEM_RADIUS, length=STEM_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + (STEM_HEIGHT * 0.5))),
        material=brass,
        name="stem",
    )
    base.visual(
        Box((0.020, HINGE_GAP + (2.0 * HINGE_CHEEK_THICKNESS), 0.014)),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z - 0.019)),
        material=brass,
        name="shoulder_bridge",
    )
    for index, y_pos in enumerate((-HINGE_CHEEK_CENTER_Y, HINGE_CHEEK_CENTER_Y)):
        base.visual(
            Box((0.018, HINGE_CHEEK_THICKNESS, 0.040)),
            origin=Origin(xyz=(0.0, y_pos, SHOULDER_AXIS_Z - 0.006)),
            material=brass,
            name=f"shoulder_cheek_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, SHOULDER_AXIS_Z + 0.03)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, (SHOULDER_AXIS_Z + 0.03) * 0.5)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=HINGE_SLEEVE_RADIUS, length=HINGE_GAP),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="shoulder_sleeve",
    )
    lower_arm.visual(
        Box((LOWER_ARM_LENGTH - 0.012, ARM_WIDTH, ARM_HEIGHT)),
        origin=Origin(xyz=((LOWER_ARM_LENGTH - 0.012) * 0.5, 0.0, 0.0)),
        material=brass,
        name="arm_beam",
    )
    lower_arm.visual(
        Box(
            (
                ARM_FORK_BRIDGE_DEPTH,
                HINGE_GAP + (2.0 * HINGE_CHEEK_THICKNESS),
                ARM_FORK_BRIDGE_HEIGHT,
            )
        ),
        origin=Origin(xyz=(LOWER_ARM_LENGTH - ARM_FORK_BRIDGE_OFFSET, 0.0, 0.0)),
        material=brass,
        name="elbow_bridge",
    )
    for index, y_pos in enumerate((-HINGE_CHEEK_CENTER_Y, HINGE_CHEEK_CENTER_Y)):
        lower_arm.visual(
            Box((ARM_FORK_CHEEK_DEPTH, HINGE_CHEEK_THICKNESS, ARM_FORK_CHEEK_HEIGHT)),
            origin=Origin(xyz=(LOWER_ARM_LENGTH - 0.010, y_pos, 0.0)),
            material=brass,
            name=f"elbow_cheek_{index}",
        )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH, 0.040, 0.030)),
        mass=0.45,
        origin=Origin(xyz=(LOWER_ARM_LENGTH * 0.5, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=HINGE_SLEEVE_RADIUS, length=HINGE_GAP),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="elbow_sleeve",
    )
    upper_arm.visual(
        Box((UPPER_ARM_LENGTH - 0.012, ARM_WIDTH, ARM_HEIGHT)),
        origin=Origin(xyz=((UPPER_ARM_LENGTH - 0.012) * 0.5, 0.0, 0.0)),
        material=brass,
        name="arm_beam",
    )
    upper_arm.visual(
        Box(
            (
                ARM_FORK_BRIDGE_DEPTH,
                HINGE_GAP + (2.0 * HINGE_CHEEK_THICKNESS),
                ARM_FORK_BRIDGE_HEIGHT,
            )
        ),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - ARM_FORK_BRIDGE_OFFSET, 0.0, 0.0)),
        material=brass,
        name="wrist_bridge",
    )
    for index, y_pos in enumerate((-HINGE_CHEEK_CENTER_Y, HINGE_CHEEK_CENTER_Y)):
        upper_arm.visual(
            Box((ARM_FORK_CHEEK_DEPTH, HINGE_CHEEK_THICKNESS, ARM_FORK_CHEEK_HEIGHT)),
            origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.010, y_pos, 0.0)),
            material=brass,
            name=f"wrist_cheek_{index}",
        )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.040, 0.030)),
        mass=0.38,
        origin=Origin(xyz=(UPPER_ARM_LENGTH * 0.5, 0.0, 0.0)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=HINGE_SLEEVE_RADIUS, length=HINGE_GAP),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="wrist_sleeve",
    )
    head.visual(
        Cylinder(radius=HEAD_NECK_RADIUS, length=HEAD_NECK_LENGTH),
        origin=Origin(
            xyz=(HEAD_NECK_LENGTH * 0.5, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="neck_tube",
    )
    head.visual(
        _shade_mesh(),
        origin=Origin(xyz=(SHADE_ORIGIN_X, 0.0, SHADE_ORIGIN_Z), rpy=(0.0, SHADE_PITCH, 0.0)),
        material=shade_white,
        name="shade_shell",
    )
    head.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(
            xyz=(SHADE_ORIGIN_X, 0.0, SHADE_ORIGIN_Z),
            rpy=(0.0, SHADE_PITCH, 0.0),
        ),
        material=powder_black,
        name="bulb_holder",
    )
    bulb_z_local = 0.038
    head.visual(
        Sphere(radius=0.016),
        origin=Origin(
            xyz=(
                SHADE_ORIGIN_X + (bulb_z_local * math.sin(SHADE_PITCH)),
                0.0,
                SHADE_ORIGIN_Z + (bulb_z_local * math.cos(SHADE_PITCH)),
            )
        ),
        material=bulb_warm,
        name="bulb_globe",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.12)),
        mass=0.35,
        origin=Origin(xyz=(0.09, 0.0, -0.03)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=18.0, velocity=1.3),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.15, upper=1.35, effort=12.0, velocity=1.7),
    )
    model.articulation(
        "upper_arm_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.85, effort=6.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head = object_model.get_part("head")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    wrist = object_model.get_articulation("upper_arm_to_head")

    ctx.expect_contact(base, lower_arm, name="shoulder hinge stays seated")
    ctx.expect_contact(lower_arm, upper_arm, name="elbow hinge stays seated")
    ctx.expect_contact(upper_arm, head, name="head hinge stays seated")
    ctx.expect_gap(
        head,
        base,
        axis="z",
        min_gap=0.12,
        negative_elem="base_disc",
        name="lamp head clears the base at rest",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({shoulder: 0.85, elbow: 0.60, wrist: 0.35}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.22,
            negative_elem="base_disc",
            name="raised lamp head stays well above the base",
        )
        raised_head_pos = ctx.part_world_position(head)

    ctx.check(
        "arm chain lifts the head upward",
        rest_head_pos is not None
        and raised_head_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.12,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
