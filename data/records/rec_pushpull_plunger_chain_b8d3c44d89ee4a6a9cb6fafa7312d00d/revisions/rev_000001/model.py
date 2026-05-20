from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_pushrod")

    painted = model.material("painted_body", rgba=(0.12, 0.14, 0.15, 1.0))
    rail_wear = model.material("dark_wear_strip", rgba=(0.05, 0.055, 0.055, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_fastener", rgba=(0.025, 0.025, 0.025, 1.0))
    service_yellow = model.material("service_yellow", rgba=(0.95, 0.68, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.42, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=painted,
        name="base_slab",
    )
    body.visual(
        Box((0.31, 0.026, 0.052)),
        origin=Origin(xyz=(0.015, 0.062, 0.050)),
        material=painted,
        name="side_rail_0",
    )
    body.visual(
        Box((0.31, 0.026, 0.052)),
        origin=Origin(xyz=(0.015, -0.062, 0.050)),
        material=painted,
        name="side_rail_1",
    )
    for index, x in enumerate((-0.105, 0.105)):
        body.visual(
            Box((0.030, 0.145, 0.016)),
            origin=Origin(xyz=(x, 0.0, 0.083)),
            material=painted,
            name=f"top_bridge_{index}",
        )
    for index, y in enumerate((-0.043, 0.043)):
        body.visual(
            Box((0.245, 0.006, 0.006)),
            origin=Origin(xyz=(0.005, y, 0.0275)),
            material=rail_wear,
            name=f"wear_strip_{index}",
        )
    body.visual(
        Box((0.180, 0.028, 0.003)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0245)),
        material=rail_wear,
        name="center_bearing",
    )

    for index, (x, y) in enumerate(((-0.170, -0.070), (-0.170, 0.070), (0.170, -0.070), (0.170, 0.070))):
        body.visual(
            Cylinder(radius=0.009, length=0.007),
            origin=Origin(xyz=(x, y, 0.0275)),
            material=dark_steel,
            name=f"mount_bolt_{index}",
        )

    body.visual(
        Box((0.050, 0.020, 0.072)),
        origin=Origin(xyz=(0.180, 0.058, 0.060)),
        material=painted,
        name="hinge_cheek_0",
    )
    body.visual(
        Box((0.050, 0.020, 0.072)),
        origin=Origin(xyz=(0.180, -0.058, 0.060)),
        material=painted,
        name="hinge_cheek_1",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.138),
        origin=Origin(xyz=(0.180, 0.0, 0.075), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    for index, y in enumerate((-0.072, 0.072)):
        body.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.180, y, 0.075), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"pin_head_{index}",
        )

    core = model.part("core")
    core.visual(
        Box((0.200, 0.068, 0.028)),
        origin=Origin(),
        material=steel,
        name="core_slide",
    )
    core.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="front_rod",
    )
    core.visual(
        Box((0.018, 0.080, 0.030)),
        origin=Origin(xyz=(-0.108, 0.0, 0.001)),
        material=dark_steel,
        name="rear_stop",
    )
    core.visual(
        Box((0.040, 0.055, 0.006)),
        origin=Origin(xyz=(-0.010, 0.0, 0.017)),
        material=dark_steel,
        name="top_wiper",
    )

    output_plate = model.part("output_plate")
    output_plate.visual(
        Cylinder(radius=0.0135, length=0.074),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="plate_barrel",
    )
    output_plate.visual(
        Box((0.018, 0.078, 0.095)),
        origin=Origin(xyz=(0.006, 0.0, 0.058)),
        material=service_yellow,
        name="output_face",
    )
    output_plate.visual(
        Box((0.020, 0.050, 0.020)),
        origin=Origin(xyz=(-0.010, 0.0, -0.019)),
        material=service_yellow,
        name="drive_tab",
    )

    model.articulation(
        "body_to_core",
        ArticulationType.PRISMATIC,
        parent=body,
        child=core,
        origin=Origin(xyz=(-0.040, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.080),
    )
    model.articulation(
        "body_to_output_plate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=output_plate,
        origin=Origin(xyz=(0.180, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-0.35, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    core = object_model.get_part("core")
    output_plate = object_model.get_part("output_plate")
    slide = object_model.get_articulation("body_to_core")
    hinge = object_model.get_articulation("body_to_output_plate")

    ctx.check(
        "one prismatic core and one revolute plate",
        slide.articulation_type == ArticulationType.PRISMATIC
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"slide={slide.articulation_type}, hinge={hinge.articulation_type}",
    )

    ctx.allow_overlap(
        body,
        output_plate,
        elem_a="hinge_pin",
        elem_b="plate_barrel",
        reason="The dark hinge pin is intentionally captured inside the simplified solid plate barrel.",
    )
    ctx.expect_within(
        body,
        output_plate,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="plate_barrel",
        margin=0.001,
        name="hinge pin sits concentrically inside plate barrel",
    )
    ctx.expect_overlap(
        body,
        output_plate,
        axes="y",
        elem_a="hinge_pin",
        elem_b="plate_barrel",
        min_overlap=0.060,
        name="hinge pin spans through the plate barrel",
    )

    ctx.expect_gap(
        body,
        core,
        axis="y",
        positive_elem="side_rail_0",
        negative_elem="core_slide",
        min_gap=0.006,
        max_gap=0.025,
        name="core clears positive guide rail",
    )
    ctx.expect_gap(
        core,
        body,
        axis="y",
        positive_elem="core_slide",
        negative_elem="side_rail_1",
        min_gap=0.006,
        max_gap=0.025,
        name="core clears negative guide rail",
    )
    ctx.expect_gap(
        core,
        body,
        axis="z",
        positive_elem="core_slide",
        negative_elem="center_bearing",
        min_gap=0.0,
        max_gap=0.0005,
        name="core rests on the center bearing",
    )
    ctx.expect_overlap(
        core,
        body,
        axes="x",
        elem_a="core_slide",
        elem_b="side_rail_0",
        min_overlap=0.150,
        name="core remains well engaged in the guide at rest",
    )

    ctx.expect_gap(
        body,
        output_plate,
        axis="y",
        positive_elem="hinge_cheek_0",
        negative_elem="plate_barrel",
        min_gap=0.006,
        max_gap=0.020,
        name="plate barrel clears positive hinge cheek",
    )
    ctx.expect_gap(
        output_plate,
        body,
        axis="y",
        positive_elem="plate_barrel",
        negative_elem="hinge_cheek_1",
        min_gap=0.006,
        max_gap=0.020,
        name="plate barrel clears negative hinge cheek",
    )

    rest_core_position = ctx.part_world_position(core)
    with ctx.pose({slide: 0.080}):
        extended_core_position = ctx.part_world_position(core)
        ctx.expect_overlap(
            core,
            body,
            axes="x",
            elem_a="core_slide",
            elem_b="side_rail_0",
            min_overlap=0.150,
            name="extended core remains retained by the guide",
        )
        ctx.expect_gap(
            body,
            core,
            axis="z",
            positive_elem="hinge_pin",
            negative_elem="front_rod",
            min_gap=0.004,
            name="extended rod passes under the hinge pin",
        )
    ctx.check(
        "core travel is forward along the body",
        rest_core_position is not None
        and extended_core_position is not None
        and extended_core_position[0] > rest_core_position[0] + 0.070,
        details=f"rest={rest_core_position}, extended={extended_core_position}",
    )

    rest_plate_bounds = ctx.part_element_world_aabb(output_plate, elem="output_face")
    with ctx.pose({hinge: 0.65}):
        lifted_plate_bounds = ctx.part_element_world_aabb(output_plate, elem="output_face")
    if rest_plate_bounds is not None and lifted_plate_bounds is not None:
        rest_center_x = (rest_plate_bounds[0][0] + rest_plate_bounds[1][0]) * 0.5
        lifted_center_x = (lifted_plate_bounds[0][0] + lifted_plate_bounds[1][0]) * 0.5
    else:
        rest_center_x = None
        lifted_center_x = None
    ctx.check(
        "output plate swings about the hinge",
        rest_center_x is not None and lifted_center_x is not None and lifted_center_x > rest_center_x + 0.020,
        details=f"rest_x={rest_center_x}, lifted_x={lifted_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
