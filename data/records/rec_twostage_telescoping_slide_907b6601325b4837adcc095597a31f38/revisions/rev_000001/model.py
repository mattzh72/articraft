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
    model = ArticulatedObject(name="telescoping_service_slide")

    sleeve_gray = model.material("sleeve_gray", rgba=(0.34, 0.37, 0.39, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    wear_pad = model.material("wear_pad", rgba=(0.06, 0.07, 0.08, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.38, 0.10, 1.0))
    brushed_edge = model.material("brushed_edge", rgba=(0.73, 0.75, 0.73, 1.0))

    outer = model.part("outer_sleeve")

    # The grounded base and welded cheek supports make the sleeve read as a
    # fixed service fixture rather than a loose tube.
    outer.visual(
        Box((0.90, 0.32, 0.025)),
        origin=Origin(xyz=(-0.35, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    for y, name in ((-0.095, "support_0"), (0.095, "support_1")):
        outer.visual(
            Box((0.66, 0.030, 0.102)),
            origin=Origin(xyz=(-0.35, y, 0.076)),
            material=sleeve_gray,
            name=name,
        )

    # Rectangular sleeve around the sliding stage.  The top is split into two
    # rails so the orange inner member is visible through the inspection slot.
    outer.visual(
        Box((0.70, 0.18, 0.014)),
        origin=Origin(xyz=(-0.35, 0.0, 0.132)),
        material=sleeve_gray,
        name="bottom_wall",
    )
    outer.visual(
        Box((0.70, 0.014, 0.110)),
        origin=Origin(xyz=(-0.35, -0.083, 0.180)),
        material=sleeve_gray,
        name="side_wall_0",
    )
    outer.visual(
        Box((0.70, 0.014, 0.110)),
        origin=Origin(xyz=(-0.35, 0.083, 0.180)),
        material=sleeve_gray,
        name="side_wall_1",
    )
    for y, name in ((-0.0625, "top_rail_0"), (0.0625, "top_rail_1")):
        outer.visual(
            Box((0.70, 0.055, 0.014)),
            origin=Origin(xyz=(-0.35, y, 0.228)),
            material=sleeve_gray,
            name=name,
        )

    # Short collars at both ends visibly capture the rectangular stage while
    # leaving a clear central opening for the prismatic travel.
    for x, stem in ((0.0, "front"), (-0.70, "rear")):
        outer.visual(
            Box((0.040, 0.19, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.128)),
            material=brushed_edge,
            name=f"{stem}_lower_lip",
        )
        outer.visual(
            Box((0.040, 0.058, 0.020)),
            origin=Origin(xyz=(x, -0.063, 0.231)),
            material=brushed_edge,
            name=f"{stem}_top_lip_0",
        )
        outer.visual(
            Box((0.040, 0.058, 0.020)),
            origin=Origin(xyz=(x, 0.063, 0.231)),
            material=brushed_edge,
            name=f"{stem}_top_lip_1",
        )
        outer.visual(
            Box((0.040, 0.020, 0.116)),
            origin=Origin(xyz=(x, -0.087, 0.180)),
            material=brushed_edge,
            name=f"{stem}_side_lip_0",
        )
        outer.visual(
            Box((0.040, 0.020, 0.116)),
            origin=Origin(xyz=(x, 0.087, 0.180)),
            material=brushed_edge,
            name=f"{stem}_side_lip_1",
        )

    # Low-friction wear strips touch the inner beam and make the support path
    # explicit without blocking the slide.
    outer.visual(
        Box((0.62, 0.026, 0.011)),
        origin=Origin(xyz=(-0.32, -0.045, 0.1445)),
        material=wear_pad,
        name="wear_pad_0",
    )
    outer.visual(
        Box((0.62, 0.026, 0.011)),
        origin=Origin(xyz=(-0.32, 0.045, 0.1445)),
        material=wear_pad,
        name="wear_pad_1",
    )

    for x, y, name in (
        (-0.69, -0.125, "bolt_0"),
        (-0.69, 0.125, "bolt_1"),
        (-0.35, -0.125, "bolt_2"),
        (-0.35, 0.125, "bolt_3"),
        (0.00, -0.125, "bolt_4"),
        (0.00, 0.125, "bolt_5"),
    ):
        outer.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(x, y, 0.030)),
            material=brushed_edge,
            name=name,
        )

    inner = model.part("inner_stage")
    inner.visual(
        Box((0.82, 0.118, 0.060)),
        # At q=0 the stage already projects from the sleeve mouth, but most of
        # its length remains hidden in the sleeve.  At full extension the rear
        # 0.10 m is still captured.
        origin=Origin(xyz=(-0.140, 0.0, 0.0)),
        material=safety_orange,
        name="stage_beam",
    )
    inner.visual(
        Box((0.26, 0.155, 0.020)),
        origin=Origin(xyz=(0.210, 0.0, 0.040)),
        material=safety_orange,
        name="front_table",
    )
    inner.visual(
        Box((0.040, 0.145, 0.072)),
        origin=Origin(xyz=(0.290, 0.0, 0.006)),
        material=brushed_edge,
        name="front_stop",
    )
    inner.visual(
        Box((0.030, 0.130, 0.066)),
        origin=Origin(xyz=(-0.560, 0.0, 0.003)),
        material=brushed_edge,
        name="rear_stop",
    )
    inner.visual(
        Box((0.50, 0.028, 0.008)),
        origin=Origin(xyz=(-0.140, 0.0, 0.035)),
        material=dark_steel,
        name="witness_stripe",
    )

    model.articulation(
        "sleeve_to_stage",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        # Joint frame is the sleeve mouth on the rectangular bore centerline.
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.45, effort=250.0, velocity=0.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_sleeve")
    inner = object_model.get_part("inner_stage")
    slide = object_model.get_articulation("sleeve_to_stage")

    ctx.expect_gap(
        inner,
        outer,
        axis="z",
        positive_elem="stage_beam",
        negative_elem="wear_pad_0",
        min_gap=0.0,
        max_gap=0.001,
        name="stage rests on wear pad",
    )
    ctx.expect_gap(
        outer,
        inner,
        axis="y",
        positive_elem="side_wall_1",
        negative_elem="stage_beam",
        min_gap=0.010,
        max_gap=0.025,
        name="positive side clearance",
    )
    ctx.expect_gap(
        inner,
        outer,
        axis="y",
        positive_elem="stage_beam",
        negative_elem="side_wall_0",
        min_gap=0.010,
        max_gap=0.025,
        name="negative side clearance",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="stage_beam",
        elem_b="bottom_wall",
        min_overlap=0.50,
        name="collapsed beam captured deeply",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({slide: 0.45}):
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            elem_a="stage_beam",
            elem_b="bottom_wall",
            min_overlap=0.09,
            name="extended beam remains captured",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="z",
            positive_elem="stage_beam",
            negative_elem="wear_pad_0",
            min_gap=0.0,
            max_gap=0.001,
            name="extended stage remains supported",
        )
        extended_pos = ctx.part_world_position(inner)

    ctx.check(
        "stage extends outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.40,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
