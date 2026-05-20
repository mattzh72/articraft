import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tv_wall_bracket")

    # 1. Wall Mount
    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.01, 0.10, 0.25)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        name="base_plate",
    )
    wall_mount.visual(
        Box((0.05, 0.04, 0.01)),
        origin=Origin(xyz=(0.035, 0.0, 0.045)),
        name="top_bracket",
    )
    wall_mount.visual(
        Box((0.05, 0.04, 0.01)),
        origin=Origin(xyz=(0.035, 0.0, -0.045)),
        name="bottom_bracket",
    )

    # 2. Primary Arm
    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        Cylinder(radius=0.02, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="wall_barrel",
    )
    primary_arm.visual(
        Box((0.20, 0.03, 0.04)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        name="main_body",
    )
    primary_arm.visual(
        Box((0.06, 0.03, 0.01)),
        origin=Origin(xyz=(0.24, 0.0, 0.025)),
        name="sec_clevis_top",
    )
    primary_arm.visual(
        Box((0.06, 0.03, 0.01)),
        origin=Origin(xyz=(0.24, 0.0, -0.025)),
        name="sec_clevis_bottom",
    )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=primary_arm,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-2.0, upper=2.0),
    )

    # 3. Secondary Arm
    secondary_arm = model.part("secondary_arm")
    secondary_arm.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="primary_barrel",
    )
    secondary_arm.visual(
        Box((0.20, 0.02, 0.03)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        name="main_body",
    )
    secondary_arm.visual(
        Box((0.06, 0.02, 0.01)),
        origin=Origin(xyz=(0.24, 0.0, 0.02)),
        name="swivel_clevis_top",
    )
    secondary_arm.visual(
        Box((0.06, 0.02, 0.01)),
        origin=Origin(xyz=(0.24, 0.0, -0.02)),
        name="swivel_clevis_bottom",
    )

    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-2.0, upper=2.0),
    )

    # 4. Swivel Bracket
    swivel_bracket = model.part("swivel_bracket")
    swivel_bracket.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="swivel_barrel",
    )
    swivel_bracket.visual(
        Box((0.02, 0.03, 0.03)),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        name="body",
    )
    swivel_bracket.visual(
        Box((0.03, 0.01, 0.03)),
        origin=Origin(xyz=(0.03, 0.02, 0.0)),
        name="left_cheek",
    )
    swivel_bracket.visual(
        Box((0.03, 0.01, 0.03)),
        origin=Origin(xyz=(0.03, -0.02, 0.0)),
        name="right_cheek",
    )

    model.articulation(
        "secondary_to_swivel",
        ArticulationType.REVOLUTE,
        parent=secondary_arm,
        child=swivel_bracket,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-1.5, upper=1.5),
    )

    # 5. Mounting Frame
    mounting_frame = model.part("mounting_frame")
    mounting_frame.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="tilt_barrel",
    )
    mounting_frame.visual(
        Box((0.01, 0.02, 0.02)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        name="spacer",
    )
    mounting_frame.visual(
        Box((0.01, 0.20, 0.20)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        name="center_plate",
    )
    mounting_frame.visual(
        Box((0.01, 0.03, 0.40)),
        origin=Origin(xyz=(0.035, 0.08, 0.0)),
        name="left_rail",
    )
    mounting_frame.visual(
        Box((0.01, 0.03, 0.40)),
        origin=Origin(xyz=(0.035, -0.08, 0.0)),
        name="right_rail",
    )

    model.articulation(
        "swivel_to_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_bracket,
        child=mounting_frame,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.5, upper=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap("wall_mount", "primary_arm", reason="Primary arm barrel sits inside wall mount clevis.")
    ctx.allow_overlap("primary_arm", "secondary_arm", reason="Secondary arm barrel sits inside primary arm clevis.")
    ctx.allow_overlap("secondary_arm", "swivel_bracket", reason="Swivel barrel sits inside secondary arm clevis.")
    ctx.allow_overlap("swivel_bracket", "mounting_frame", reason="Tilt barrel sits inside swivel bracket cheeks.")

    return ctx.report()


object_model = build_object_model()