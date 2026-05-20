from __future__ import annotations

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
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    zinc = model.material("brushed_zinc", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_galvanized_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    bearing = model.material("polished_bearing_pads", rgba=(0.86, 0.87, 0.84, 1.0))
    stop = model.material("black_nylon_stops", rgba=(0.025, 0.025, 0.022, 1.0))

    outer = model.part("outer_body")
    outer.visual(
        Box((0.700, 0.140, 0.010)),
        origin=Origin(xyz=(0.325, 0.0, -0.042)),
        material=dark_steel,
        name="ground_plate",
    )
    outer.visual(
        Box((0.650, 0.114, 0.010)),
        origin=Origin(xyz=(0.325, 0.0, -0.033)),
        material=zinc,
        name="outer_bottom_web",
    )
    for suffix, y in (("pos", 0.053), ("neg", -0.053)):
        outer.visual(
            Box((0.650, 0.008, 0.060)),
            origin=Origin(xyz=(0.325, y, 0.0)),
            material=zinc,
            name=f"outer_side_wall_{suffix}",
        )
        outer.visual(
            Box((0.650, 0.020, 0.008)),
            origin=Origin(xyz=(0.325, 0.040 if y > 0 else -0.040, 0.027)),
            material=zinc,
            name=f"outer_return_lip_{suffix}",
        )

    for x in (0.055, 0.595):
        for y, suffix in ((0.063, "pos"), (-0.063, "neg")):
            outer.visual(
                Box((0.018, 0.014, 0.032)),
                origin=Origin(xyz=(x, y, -0.020)),
                material=stop,
                name=f"outer_stop_{int(round(x * 1000))}_{suffix}",
            )
    for x in (0.080, 0.570):
        for y in (-0.047, 0.047):
            outer.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(x, y, -0.034)),
                material=dark_steel,
                name=f"mount_screw_{int(round(x * 1000))}_{'pos' if y > 0 else 'neg'}",
            )

    middle = model.part("middle_section")
    middle.visual(
        Box((0.600, 0.078, 0.007)),
        origin=Origin(xyz=(0.325, 0.0, -0.020)),
        material=zinc,
        name="middle_bottom_web",
    )
    for suffix, y in (("pos", 0.036), ("neg", -0.036)):
        middle.visual(
            Box((0.600, 0.006, 0.037)),
            origin=Origin(xyz=(0.325, y, 0.0)),
            material=zinc,
            name=f"middle_side_wall_{suffix}",
        )
        middle.visual(
            Box((0.600, 0.014, 0.006)),
            origin=Origin(xyz=(0.325, 0.026 if y > 0 else -0.026, 0.017)),
            material=zinc,
            name=f"middle_return_lip_{suffix}",
        )

    for x in (0.095, 0.300, 0.520):
        for y, suffix in ((0.044, "pos"), (-0.044, "neg")):
            middle.visual(
                Box((0.045, 0.010, 0.012)),
                origin=Origin(xyz=(x, y, 0.001)),
                material=bearing,
                name=f"outer_bearing_{int(round(x * 1000))}_{suffix}",
            )
    for x in (0.050, 0.600):
        for y, suffix in ((0.044, "pos"), (-0.044, "neg")):
            middle.visual(
                Box((0.016, 0.010, 0.014)),
                origin=Origin(xyz=(x, y, -0.010)),
                material=stop,
                name=f"middle_stop_{int(round(x * 1000))}_{suffix}",
            )

    inner = model.part("inner_section")
    inner.visual(
        Box((0.560, 0.036, 0.022)),
        origin=Origin(xyz=(0.325, 0.0, 0.0)),
        material=zinc,
        name="inner_bar",
    )
    inner.visual(
        Box((0.520, 0.052, 0.006)),
        origin=Origin(xyz=(0.345, 0.0, 0.014)),
        material=zinc,
        name="inner_mount_flange",
    )
    for x in (0.120, 0.355, 0.565):
        for y, suffix in ((0.025, "pos"), (-0.025, "neg")):
            inner.visual(
                Box((0.040, 0.015, 0.010)),
                origin=Origin(xyz=(x, y, 0.0)),
                material=bearing,
                name=f"inner_bearing_{int(round(x * 1000))}_{suffix}",
            )
    for x in (0.095, 0.560):
        inner.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.018)),
            material=dark_steel,
            name=f"carriage_screw_{int(round(x * 1000))}",
        )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.340),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.340),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_body")
    middle = object_model.get_part("middle_section")
    inner = object_model.get_part("inner_section")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "middle slide is prismatic on x",
        outer_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={outer_slide.articulation_type}, axis={outer_slide.axis}",
    )
    ctx.check(
        "inner slide is serial prismatic on x",
        inner_slide.articulation_type == ArticulationType.PRISMATIC
        and inner_slide.parent == "middle_section"
        and tuple(inner_slide.axis) == (1.0, 0.0, 0.0),
        details=f"parent={inner_slide.parent}, type={inner_slide.articulation_type}, axis={inner_slide.axis}",
    )

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.001,
        name="middle section stays inside outer cross section",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.001,
        name="inner section stays inside middle cross section",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.55,
        name="middle section is retained when collapsed",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.50,
        name="inner section is retained when collapsed",
    )

    middle_rest = ctx.part_world_position(middle)
    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({outer_slide: 0.340, inner_slide: 0.340}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.001,
            name="extended middle remains aligned in outer",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.001,
            name="extended inner remains aligned in middle",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.24,
            name="extended middle keeps retained insertion",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.18,
            name="extended inner keeps retained insertion",
        )
        middle_extended = ctx.part_world_position(middle)
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "slides extend along one straight axis",
        middle_rest is not None
        and inner_rest is not None
        and middle_extended is not None
        and inner_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.30
        and inner_extended[0] > inner_rest[0] + 0.60
        and abs(middle_extended[1] - middle_rest[1]) < 1e-6
        and abs(inner_extended[1] - inner_rest[1]) < 1e-6
        and abs(middle_extended[2] - middle_rest[2]) < 1e-6
        and abs(inner_extended[2] - inner_rest[2]) < 1e-6,
        details=(
            f"middle_rest={middle_rest}, middle_extended={middle_extended}, "
            f"inner_rest={inner_rest}, inner_extended={inner_extended}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
