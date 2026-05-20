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
    model = ArticulatedObject(name="prismatic_revolute_prismatic_chain")

    black = model.material("black_anodized_aluminum", color=(0.03, 0.035, 0.04, 1.0))
    rail_steel = model.material("brushed_steel", color=(0.68, 0.70, 0.72, 1.0))
    carriage_orange = model.material("powder_coated_orange", color=(0.95, 0.34, 0.08, 1.0))
    guide_blue = model.material("blue_distal_guide", color=(0.10, 0.22, 0.62, 1.0))
    end_yellow = model.material("yellow_slider_member", color=(0.95, 0.74, 0.12, 1.0))
    rubber = model.material("dark_rubber_stops", color=(0.015, 0.015, 0.018, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((1.24, 0.26, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black,
        name="ground_plate",
    )
    for y, name in [(-0.065, "rail_0"), (0.065, "rail_1")]:
        base_rail.visual(
            Box((1.12, 0.030, 0.038)),
            # The rails are bolted into the plate with a slight authored embed.
            origin=Origin(xyz=(0.0, y, 0.068)),
            material=rail_steel,
            name=name,
        )
    for x, name in [(-0.585, "stop_0"), (0.585, "stop_1")]:
        base_rail.visual(
            Box((0.035, 0.23, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.090)),
            material=rubber,
            name=name,
        )

    pivot_bracket = model.part("pivot_bracket")
    pivot_bracket.visual(
        Box((0.205, 0.185, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=carriage_orange,
        name="slide_carriage",
    )
    pivot_bracket.visual(
        Cylinder(radius=0.046, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=carriage_orange,
        name="pivot_pedestal",
    )
    pivot_bracket.visual(
        Cylinder(radius=0.078, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.177)),
        material=rail_steel,
        name="pivot_cap",
    )
    for y, name in [(-0.066, "cheek_0"), (0.066, "cheek_1")]:
        pivot_bracket.visual(
            Box((0.086, 0.018, 0.128)),
            origin=Origin(xyz=(0.0, y, 0.109)),
            material=carriage_orange,
            name=name,
        )

    distal_guide = model.part("distal_guide")
    distal_guide.visual(
        Cylinder(radius=0.066, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=rail_steel,
        name="turntable_washer",
    )
    distal_guide.visual(
        Cylinder(radius=0.026, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=rail_steel,
        name="pivot_boss",
    )
    distal_guide.visual(
        Box((0.590, 0.082, 0.040)),
        origin=Origin(xyz=(0.330, 0.0, 0.040)),
        material=guide_blue,
        name="guide_spine",
    )
    for y, name in [(-0.030, "guide_rail_0"), (0.030, "guide_rail_1")]:
        distal_guide.visual(
            Box((0.560, 0.012, 0.021)),
            origin=Origin(xyz=(0.345, y, 0.0705)),
            material=rail_steel,
            name=name,
        )
    distal_guide.visual(
        Box((0.035, 0.092, 0.080)),
        origin=Origin(xyz=(0.635, 0.0, 0.060)),
        material=rubber,
        name="guide_stop",
    )

    end_member = model.part("end_member")
    end_member.visual(
        Box((0.145, 0.102, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=end_yellow,
        name="slider_saddle",
    )
    end_member.visual(
        Box((0.420, 0.045, 0.040)),
        origin=Origin(xyz=(0.210, 0.0, 0.060)),
        material=end_yellow,
        name="extension_bar",
    )
    end_member.visual(
        Box((0.050, 0.076, 0.082)),
        origin=Origin(xyz=(0.445, 0.0, 0.061)),
        material=end_yellow,
        name="end_pad",
    )

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=pivot_bracket,
        origin=Origin(xyz=(-0.300, 0.0, 0.087)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.600),
    )
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=pivot_bracket,
        child=distal_guide,
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-1.5708, upper=1.5708),
    )
    model.articulation(
        "distal_slide",
        ArticulationType.PRISMATIC,
        parent=distal_guide,
        child=end_member,
        origin=Origin(xyz=(0.205, 0.0, 0.081)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.280),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_slide = object_model.get_articulation("base_slide")
    pivot = object_model.get_articulation("pivot")
    distal_slide = object_model.get_articulation("distal_slide")
    base_rail = object_model.get_part("base_rail")
    pivot_bracket = object_model.get_part("pivot_bracket")
    distal_guide = object_model.get_part("distal_guide")
    end_member = object_model.get_part("end_member")

    ctx.check(
        "chain order is prismatic revolute prismatic",
        (
            base_slide.articulation_type == ArticulationType.PRISMATIC
            and pivot.articulation_type == ArticulationType.REVOLUTE
            and distal_slide.articulation_type == ArticulationType.PRISMATIC
        ),
        details=(
            f"types={base_slide.articulation_type}, "
            f"{pivot.articulation_type}, {distal_slide.articulation_type}"
        ),
    )
    ctx.expect_contact(
        base_rail,
        pivot_bracket,
        elem_a="rail_0",
        elem_b="slide_carriage",
        name="root carriage is seated on the grounded rail",
    )
    ctx.expect_contact(
        pivot_bracket,
        distal_guide,
        elem_a="pivot_cap",
        elem_b="turntable_washer",
        name="revolute stage bears on the pivot cap",
    )
    ctx.expect_contact(
        distal_guide,
        end_member,
        elem_a="guide_rail_0",
        elem_b="slider_saddle",
        name="distal slider rides on its local guide",
    )
    ctx.expect_overlap(
        distal_guide,
        end_member,
        axes="x",
        min_overlap=0.12,
        elem_a="guide_rail_0",
        elem_b="slider_saddle",
        name="distal saddle is retained on the guide at rest",
    )

    root_rest = ctx.part_world_position(pivot_bracket)
    distal_rest = ctx.part_world_position(end_member)
    with ctx.pose({base_slide: 0.600}):
        root_extended = ctx.part_world_position(pivot_bracket)
    with ctx.pose({distal_slide: 0.280}):
        distal_extended = ctx.part_world_position(end_member)
        ctx.expect_overlap(
            distal_guide,
            end_member,
            axes="x",
            min_overlap=0.07,
            elem_a="guide_rail_0",
            elem_b="slider_saddle",
            name="distal saddle remains captured at full travel",
        )

    ctx.check(
        "base slide translates the pivot bracket along +X",
        root_rest is not None
        and root_extended is not None
        and root_extended[0] > root_rest[0] + 0.55,
        details=f"rest={root_rest}, extended={root_extended}",
    )
    ctx.check(
        "distal slide translates the end member along its guide",
        distal_rest is not None
        and distal_extended is not None
        and distal_extended[0] > distal_rest[0] + 0.25,
        details=f"rest={distal_rest}, extended={distal_extended}",
    )

    return ctx.report()


object_model = build_object_model()
