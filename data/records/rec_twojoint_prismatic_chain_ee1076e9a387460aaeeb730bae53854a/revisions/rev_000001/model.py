from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_two_stage_slide_chain")

    blackened = Material("blackened_steel", color=(0.08, 0.085, 0.09, 1.0))
    zinc = Material("zinc_plated_steel", color=(0.62, 0.64, 0.62, 1.0))
    polished = Material("polished_runner", color=(0.78, 0.79, 0.77, 1.0))
    graphite = Material("graphite_wear_pad", color=(0.025, 0.026, 0.028, 1.0))
    stop_rubber = Material("black_rubber_stop", color=(0.01, 0.01, 0.012, 1.0))

    guide = model.part("guide")
    guide.visual(
        Box((0.700, 0.085, 0.006)),
        origin=Origin(xyz=(0.350, 0.0, 0.003)),
        material=blackened,
        name="base_web",
    )
    for side, y in (("rail_pos", 0.0395), ("rail_neg", -0.0395)):
        guide.visual(
            Box((0.700, 0.006, 0.050)),
            origin=Origin(xyz=(0.350, y, 0.028)),
            material=blackened,
            name=f"{side}_wall",
        )
        guide.visual(
            Box((0.700, 0.014, 0.006)),
            origin=Origin(xyz=(0.350, 0.032 if y > 0 else -0.032, 0.052)),
            material=blackened,
            name=f"{side}_return_lip",
        )
        guide.visual(
            Box((0.690, 0.004, 0.004)),
            origin=Origin(xyz=(0.355, 0.0345 if y > 0 else -0.0345, 0.013)),
            material=graphite,
            name=f"{side}_wear_strip",
        )
    guide.visual(
        Box((0.012, 0.012, 0.028)),
        origin=Origin(xyz=(-0.006, 0.035, 0.022)),
        material=stop_rubber,
        name="rear_bumper_0",
    )
    guide.visual(
        Box((0.012, 0.012, 0.028)),
        origin=Origin(xyz=(-0.006, -0.035, 0.022)),
        material=stop_rubber,
        name="rear_bumper_1",
    )
    for i, x in enumerate((0.060, 0.640)):
        guide.visual(
            Cylinder(radius=0.008, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.0075)),
            material=graphite,
            name=f"mount_screw_{i}",
        )

    middle = model.part("middle_runner")
    middle.visual(
        Box((0.560, 0.052, 0.006)),
        origin=Origin(xyz=(0.280, 0.0, 0.012)),
        material=zinc,
        name="lower_web",
    )
    for side, y in (("rail_pos", 0.026), ("rail_neg", -0.026)):
        middle.visual(
            Box((0.560, 0.006, 0.030)),
            origin=Origin(xyz=(0.280, y, 0.030)),
            material=zinc,
            name=f"{side}_rib",
        )
        middle.visual(
            Box((0.560, 0.012, 0.005)),
            origin=Origin(xyz=(0.280, 0.020 if y > 0 else -0.020, 0.043)),
            material=zinc,
            name=f"{side}_inner_lip",
        )
        middle.visual(
            Box((0.540, 0.003, 0.003)),
            origin=Origin(xyz=(0.290, 0.016 if y > 0 else -0.016, 0.040)),
            material=graphite,
            name=f"{side}_inner_wear_strip",
        )
    middle.visual(
        Box((0.010, 0.040, 0.020)),
        origin=Origin(xyz=(-0.005, 0.0, 0.025)),
        material=stop_rubber,
        name="rear_stop",
    )

    inner = model.part("inner_runner")
    inner.visual(
        Box((0.480, 0.006, 0.030)),
        origin=Origin(xyz=(0.240, 0.0, 0.030)),
        material=polished,
        name="center_web",
    )
    inner.visual(
        Box((0.480, 0.026, 0.006)),
        origin=Origin(xyz=(0.240, 0.0, 0.018)),
        material=polished,
        name="lower_flange",
    )
    inner.visual(
        Box((0.480, 0.026, 0.006)),
        origin=Origin(xyz=(0.240, 0.0, 0.042)),
        material=polished,
        name="upper_flange",
    )
    inner.visual(
        Box((0.030, 0.020, 0.014)),
        origin=Origin(xyz=(0.495, 0.0, 0.030)),
        material=polished,
        name="front_pull_tab",
    )

    model.articulation(
        "guide_to_middle",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.280),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.280),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    guide_slide = object_model.get_articulation("guide_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        guide,
        axes="yz",
        margin=0.002,
        name="middle runner nests within the grounded guide section",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.002,
        name="inner runner nests within the middle runner",
    )
    ctx.expect_overlap(
        middle,
        guide,
        axes="x",
        min_overlap=0.50,
        name="retracted middle runner has long engagement in guide",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.45,
        name="retracted inner runner has long engagement in middle",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({guide_slide: 0.280, inner_slide: 0.280}):
        ctx.expect_within(
            middle,
            guide,
            axes="yz",
            margin=0.002,
            name="extended middle runner remains laterally captured",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.002,
            name="extended inner runner remains laterally captured",
        )
        ctx.expect_overlap(
            middle,
            guide,
            axes="x",
            min_overlap=0.35,
            name="extended middle runner remains inserted in guide",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.20,
            name="extended inner runner remains inserted in middle",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "serial prismatic chain extends along shared x axis",
        rest_middle is not None
        and rest_inner is not None
        and extended_middle is not None
        and extended_inner is not None
        and extended_middle[0] > rest_middle[0] + 0.25
        and extended_inner[0] > rest_inner[0] + 0.53,
        details=(
            f"rest_middle={rest_middle}, extended_middle={extended_middle}, "
            f"rest_inner={rest_inner}, extended_inner={extended_inner}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
