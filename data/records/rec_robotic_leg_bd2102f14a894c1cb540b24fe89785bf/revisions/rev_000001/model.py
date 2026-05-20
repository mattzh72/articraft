from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LATERAL_CYLINDER = (pi / 2.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="walking_machine_leg_module")

    dark_housing = Material("dark_anodized_housing", rgba=(0.08, 0.09, 0.10, 1.0))
    graphite = Material("graphite_composite", rgba=(0.18, 0.20, 0.22, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    machined = Material("brushed_steel", rgba=(0.68, 0.68, 0.62, 1.0))
    caution = Material("muted_safety_yellow", rgba=(0.78, 0.58, 0.12, 1.0))

    upper_housing = model.part("upper_housing")
    upper_housing.visual(
        Box((0.28, 0.22, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=dark_housing,
        name="hip_gearbox",
    )
    upper_housing.visual(
        Cylinder(radius=0.11, length=0.24),
        origin=Origin(rpy=LATERAL_CYLINDER),
        material=machined,
        name="hip_motor",
    )
    upper_housing.visual(
        Box((0.34, 0.26, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=dark_housing,
        name="mounting_flange",
    )
    upper_housing.visual(
        Box((0.06, 0.24, 0.10)),
        origin=Origin(xyz=(-0.16, 0.0, 0.18)),
        material=caution,
        name="service_cover",
    )

    thigh_link = model.part("thigh_link")
    thigh_link.visual(
        Cylinder(radius=0.045, length=0.30),
        origin=Origin(rpy=LATERAL_CYLINDER),
        material=machined,
        name="hip_shaft",
    )
    thigh_link.visual(
        Box((0.18, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.14)),
        material=graphite,
        name="hip_collar",
    )
    thigh_link.visual(
        Box((0.060, 0.050, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=graphite,
        name="hip_drive_web",
    )
    thigh_link.visual(
        Box((0.13, 0.10, 0.33)),
        origin=Origin(xyz=(0.0, 0.0, -0.305)),
        material=graphite,
        name="thigh_beam",
    )
    thigh_link.visual(
        Cylinder(radius=0.085, length=0.19),
        origin=Origin(xyz=(0.0, 0.0, -0.54), rpy=LATERAL_CYLINDER),
        material=dark_housing,
        name="knee_motor",
    )

    shank_link = model.part("shank_link")
    shank_link.visual(
        Cylinder(radius=0.040, length=0.26),
        origin=Origin(rpy=LATERAL_CYLINDER),
        material=machined,
        name="knee_shaft",
    )
    shank_link.visual(
        Box((0.13, 0.095, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=graphite,
        name="knee_collar",
    )
    shank_link.visual(
        Box((0.050, 0.045, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=graphite,
        name="knee_drive_web",
    )
    shank_link.visual(
        Box((0.11, 0.085, 0.31)),
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        material=graphite,
        name="shank_beam",
    )
    shank_link.visual(
        Cylinder(radius=0.070, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, -0.50), rpy=LATERAL_CYLINDER),
        material=dark_housing,
        name="ankle_motor",
    )

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.035, length=0.23),
        origin=Origin(rpy=LATERAL_CYLINDER),
        material=machined,
        name="ankle_shaft",
    )
    foot.visual(
        Box((0.14, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=dark_housing,
        name="ankle_block",
    )
    foot.visual(
        Box((0.045, 0.040, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=dark_housing,
        name="ankle_drive_web",
    )
    foot.visual(
        Box((0.34, 0.15, 0.06)),
        origin=Origin(xyz=(0.12, 0.0, -0.1575)),
        material=black_rubber,
        name="sole_pad",
    )
    foot.visual(
        Box((0.16, 0.13, 0.035)),
        origin=Origin(xyz=(0.22, 0.0, -0.115)),
        material=black_rubber,
        name="toe_lip",
    )

    model.articulation(
        "hip",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=3.0, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "knee",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.0, 0.0, -0.54)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=3.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "ankle",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.50)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=4.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper_housing = object_model.get_part("upper_housing")
    thigh_link = object_model.get_part("thigh_link")
    shank_link = object_model.get_part("shank_link")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip")
    knee = object_model.get_articulation("knee")
    ankle = object_model.get_articulation("ankle")

    ctx.allow_overlap(
        upper_housing,
        thigh_link,
        elem_a="hip_motor",
        elem_b="hip_shaft",
        reason="The hip shaft is intentionally captured through the compact hip actuator bore.",
    )
    ctx.expect_within(
        thigh_link,
        upper_housing,
        axes="xz",
        inner_elem="hip_shaft",
        outer_elem="hip_motor",
        name="hip shaft sits within hip motor envelope",
    )
    ctx.expect_overlap(
        thigh_link,
        upper_housing,
        axes="y",
        elem_a="hip_shaft",
        elem_b="hip_motor",
        min_overlap=0.18,
        name="hip shaft spans the lateral motor",
    )
    ctx.allow_overlap(
        upper_housing,
        thigh_link,
        elem_a="hip_motor",
        elem_b="hip_drive_web",
        reason="The compact drive web exits the hip actuator through the same covered hub opening.",
    )
    ctx.expect_overlap(
        thigh_link,
        upper_housing,
        axes="xz",
        elem_a="hip_drive_web",
        elem_b="hip_motor",
        min_overlap=0.04,
        name="hip drive web is local to hip motor hub",
    )

    ctx.allow_overlap(
        thigh_link,
        shank_link,
        elem_a="knee_motor",
        elem_b="knee_shaft",
        reason="The knee shaft is intentionally nested inside the knee motor hub.",
    )
    ctx.expect_within(
        shank_link,
        thigh_link,
        axes="xz",
        inner_elem="knee_shaft",
        outer_elem="knee_motor",
        name="knee shaft sits within knee motor envelope",
    )
    ctx.expect_overlap(
        shank_link,
        thigh_link,
        axes="y",
        elem_a="knee_shaft",
        elem_b="knee_motor",
        min_overlap=0.17,
        name="knee shaft spans the lateral motor",
    )
    ctx.allow_overlap(
        thigh_link,
        shank_link,
        elem_a="knee_motor",
        elem_b="knee_drive_web",
        reason="The knee drive web is the local output arm emerging from the motor hub.",
    )
    ctx.expect_overlap(
        shank_link,
        thigh_link,
        axes="xz",
        elem_a="knee_drive_web",
        elem_b="knee_motor",
        min_overlap=0.04,
        name="knee drive web is local to knee motor hub",
    )

    ctx.allow_overlap(
        shank_link,
        foot,
        elem_a="ankle_motor",
        elem_b="ankle_shaft",
        reason="The ankle shaft is intentionally captured through the ankle motor hub.",
    )
    ctx.expect_within(
        foot,
        shank_link,
        axes="xz",
        inner_elem="ankle_shaft",
        outer_elem="ankle_motor",
        name="ankle shaft sits within ankle motor envelope",
    )
    ctx.expect_overlap(
        foot,
        shank_link,
        axes="y",
        elem_a="ankle_shaft",
        elem_b="ankle_motor",
        min_overlap=0.16,
        name="ankle shaft spans the lateral motor",
    )
    ctx.allow_overlap(
        shank_link,
        foot,
        elem_a="ankle_motor",
        elem_b="ankle_drive_web",
        reason="The ankle drive web is the local output arm emerging from the motor hub.",
    )
    ctx.expect_overlap(
        foot,
        shank_link,
        axes="xz",
        elem_a="ankle_drive_web",
        elem_b="ankle_motor",
        min_overlap=0.04,
        name="ankle drive web is local to ankle motor hub",
    )

    for joint in (hip, knee, ankle):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a lateral revolute joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.upper > limits.lower,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    rest_foot_position = ctx.part_world_position(foot)
    with ctx.pose({hip: 0.35, knee: 0.55, ankle: -0.25}):
        posed_foot_position = ctx.part_world_position(foot)
    ctx.check(
        "leg articulation moves the distal foot",
        rest_foot_position is not None
        and posed_foot_position is not None
        and abs(posed_foot_position[0] - rest_foot_position[0]) > 0.18,
        details=f"rest={rest_foot_position}, posed={posed_foot_position}",
    )

    return ctx.report()


object_model = build_object_model()
