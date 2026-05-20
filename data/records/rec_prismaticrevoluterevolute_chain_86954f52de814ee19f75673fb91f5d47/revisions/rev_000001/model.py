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
    model = ArticulatedObject(name="slide_arm")

    base = model.part("base")
    base.visual(
        Box((1.0, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.15, 0.1, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="carriage_body",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.4, upper=0.4),
    )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.04, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="shoulder_hub",
    )
    shoulder.visual(
        Box((0.4, 0.05, 0.05)),
        origin=Origin(xyz=(0.2, 0.0, 0.025)),
        name="shoulder_arm",
    )

    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-2.0, upper=2.0),
    )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.04, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="forelink_hub",
    )
    forelink.visual(
        Box((0.4, 0.05, 0.05)),
        origin=Origin(xyz=(0.2, 0.0, 0.025)),
        name="forelink_arm",
    )

    model.articulation(
        "shoulder_to_forelink",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forelink,
        origin=Origin(xyz=(0.4, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-2.5, upper=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    shoulder = object_model.get_part("shoulder")
    forelink = object_model.get_part("forelink")

    ctx.expect_gap(carriage, base, axis="z", max_penetration=0.0, min_gap=0.0)
    ctx.expect_gap(shoulder, carriage, axis="z", max_penetration=0.0, min_gap=0.0)
    ctx.expect_gap(forelink, shoulder, axis="z", max_penetration=0.0, min_gap=0.0)

    return ctx.report()


object_model = build_object_model()