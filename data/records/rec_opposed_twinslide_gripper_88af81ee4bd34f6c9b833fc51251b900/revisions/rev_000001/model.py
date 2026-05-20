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
    model = ArticulatedObject(name="heavy_block_gripper")

    cast_iron = model.material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    oiled_steel = model.material("oiled_slide_steel", rgba=(0.34, 0.37, 0.39, 1.0))
    blue_steel = model.material("blued_carriage_steel", rgba=(0.13, 0.18, 0.24, 1.0))
    black_tool = model.material("blackened_tool_steel", rgba=(0.015, 0.015, 0.014, 1.0))
    bolt_dark = model.material("dark_socket_bolts", rgba=(0.02, 0.02, 0.018, 1.0))
    brass = model.material("brass_hydraulic_plug", rgba=(0.85, 0.58, 0.25, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.56, 0.28, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=cast_iron,
        name="base_block",
    )
    housing.visual(
        Box((0.66, 0.080, 0.025)),
        origin=Origin(xyz=(0.0, -0.160, 0.0125)),
        material=cast_iron,
        name="front_foot",
    )
    housing.visual(
        Box((0.66, 0.080, 0.025)),
        origin=Origin(xyz=(0.0, 0.160, 0.0125)),
        material=cast_iron,
        name="rear_foot",
    )
    housing.visual(
        Box((0.50, 0.18, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=oiled_steel,
        name="guide_bed",
    )
    housing.visual(
        Box((0.50, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, 0.119, 0.1075)),
        material=oiled_steel,
        name="rear_guide_lip",
    )
    housing.visual(
        Box((0.50, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.119, 0.092)),
        material=oiled_steel,
        name="front_low_lip",
    )
    housing.visual(
        Box((0.025, 0.22, 0.065)),
        origin=Origin(xyz=(-0.265, 0.0, 0.1125)),
        material=cast_iron,
        name="end_stop_0",
    )
    housing.visual(
        Box((0.025, 0.22, 0.065)),
        origin=Origin(xyz=(0.265, 0.0, 0.1125)),
        material=cast_iron,
        name="end_stop_1",
    )
    housing.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=brass,
        name="center_plug",
    )
    for i, x in enumerate((-0.235, 0.235)):
        for j, y in enumerate((-0.160, 0.160)):
            housing.visual(
                Cylinder(radius=0.014, length=0.008),
                origin=Origin(xyz=(x, y, 0.029)),
                material=bolt_dark,
                name=f"foot_bolt_{i}_{j}",
            )

    carriage_origins = (-0.160, 0.160)
    inward_signs = (1.0, -1.0)
    carriage_parts = []
    for index, (x_origin, inward) in enumerate(zip(carriage_origins, inward_signs)):
        carriage = model.part(f"jaw_carriage_{index}")
        carriage_parts.append(carriage)
        carriage.visual(
            Box((0.130, 0.200, 0.070)),
            origin=Origin(xyz=(0.0, 0.0, 0.035)),
            material=blue_steel,
            name="body",
        )
        carriage.visual(
            Box((0.150, 0.025, 0.025)),
            origin=Origin(xyz=(0.0, -0.0875, 0.0125)),
            material=oiled_steel,
            name="front_slide_shoe",
        )
        carriage.visual(
            Box((0.150, 0.025, 0.025)),
            origin=Origin(xyz=(0.0, 0.0875, 0.0125)),
            material=oiled_steel,
            name="rear_slide_shoe",
        )
        carriage.visual(
            Box((0.110, 0.160, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.079)),
            material=oiled_steel,
            name="cap_plate",
        )
        carriage.visual(
            Box((0.060, 0.045, 0.080)),
            origin=Origin(xyz=(inward * 0.045, -0.118, 0.020)),
            material=black_tool,
            name="finger_root",
        )
        carriage.visual(
            Box((0.055, 0.050, 0.095)),
            origin=Origin(xyz=(inward * 0.080, -0.155, 0.005)),
            material=black_tool,
            name="finger_step",
        )
        carriage.visual(
            Box((0.035, 0.045, 0.120)),
            origin=Origin(xyz=(inward * 0.115, -0.185, -0.025)),
            material=black_tool,
            name="finger_tip",
        )
        for i, bx in enumerate((-0.037, 0.037)):
            for j, by in enumerate((-0.052, 0.052)):
                carriage.visual(
                    Cylinder(radius=0.007, length=0.006),
                    origin=Origin(xyz=(bx, by, 0.091)),
                    material=bolt_dark,
                    name=f"cap_bolt_{i}_{j}",
                )

        model.articulation(
            f"housing_to_carriage_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=carriage,
            origin=Origin(xyz=(x_origin, 0.0, 0.125)),
            axis=(inward, 0.0, 0.0),
            motion_limits=MotionLimits(effort=900.0, velocity=0.12, lower=0.0, upper=0.020),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    carriage_0 = object_model.get_part("jaw_carriage_0")
    carriage_1 = object_model.get_part("jaw_carriage_1")
    slide_0 = object_model.get_articulation("housing_to_carriage_0")
    slide_1 = object_model.get_articulation("housing_to_carriage_1")

    ctx.check(
        "mirrored carriages use two separate prismatic slides",
        slide_0 is not slide_1
        and slide_0.articulation_type == ArticulationType.PRISMATIC
        and slide_1.articulation_type == ArticulationType.PRISMATIC
        and abs(abs(slide_0.axis[0]) - 1.0) < 1e-6
        and abs(abs(slide_1.axis[0]) - 1.0) < 1e-6
        and slide_0.axis[0] * slide_1.axis[0] < 0.0,
        details=f"axes={slide_0.axis}, {slide_1.axis}",
    )

    body_size = getattr(carriage_0.get_visual("body").geometry, "size", None)
    tip_size = getattr(carriage_0.get_visual("finger_tip").geometry, "size", None)
    ctx.check(
        "carriage body is much larger than supported finger",
        body_size is not None
        and tip_size is not None
        and body_size[0] * body_size[1] * body_size[2]
        > 6.0 * tip_size[0] * tip_size[1] * tip_size[2],
        details=f"body_size={body_size}, tip_size={tip_size}",
    )

    for carriage in (carriage_0, carriage_1):
        ctx.expect_gap(
            carriage,
            housing,
            axis="z",
            positive_elem="body",
            negative_elem="guide_bed",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{carriage.name} body sits on the grounded guide bed",
        )
        ctx.expect_overlap(
            carriage,
            housing,
            axes="xy",
            elem_a="body",
            elem_b="guide_bed",
            min_overlap=0.12,
            name=f"{carriage.name} has broad footprint on guide bed",
        )

    ctx.expect_gap(
        carriage_1,
        carriage_0,
        axis="x",
        positive_elem="finger_tip",
        negative_elem="finger_tip",
        min_gap=0.045,
        max_gap=0.065,
        name="rest pose leaves an open gripping gap",
    )

    rest_0 = ctx.part_world_position(carriage_0)
    rest_1 = ctx.part_world_position(carriage_1)
    with ctx.pose({slide_0: 0.020, slide_1: 0.020}):
        closed_0 = ctx.part_world_position(carriage_0)
        closed_1 = ctx.part_world_position(carriage_1)
        ctx.expect_gap(
            carriage_1,
            carriage_0,
            axis="x",
            positive_elem="finger_tip",
            negative_elem="finger_tip",
            min_gap=0.005,
            max_gap=0.025,
            name="closed pose brings stepped fingers together without collision",
        )

    ctx.check(
        "positive slide motion drives both jaw carriages inward",
        rest_0 is not None
        and rest_1 is not None
        and closed_0 is not None
        and closed_1 is not None
        and closed_0[0] > rest_0[0] + 0.015
        and closed_1[0] < rest_1[0] - 0.015,
        details=f"rest={rest_0}, {rest_1}; closed={closed_0}, {closed_1}",
    )

    return ctx.report()


object_model = build_object_model()
