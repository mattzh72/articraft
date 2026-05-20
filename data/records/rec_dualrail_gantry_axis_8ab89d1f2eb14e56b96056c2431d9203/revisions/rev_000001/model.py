from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_gantry_axis")

    rail_steel = model.material("ground_rail_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_steel = model.material("black_oxide_fasteners", rgba=(0.015, 0.014, 0.013, 1.0))
    black_anodized = model.material("black_anodized_aluminum", rgba=(0.025, 0.027, 0.030, 1.0))
    slot_shadow = model.material("slot_shadow", rgba=(0.004, 0.004, 0.005, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", rgba=(0.035, 0.145, 0.30, 1.0))
    block_gray = model.material("linear_block_hardcoat", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("black_rubber_wipers", rgba=(0.006, 0.006, 0.006, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.36, 0.58, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=black_anodized,
        name="base_plate",
    )
    base.visual(
        Box((1.30, 0.040, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=black_anodized,
        name="center_stiffener",
    )

    rail_y_positions = (-0.20, 0.20)
    for rail_index, y in enumerate(rail_y_positions):
        base.visual(
            Box((1.25, 0.082, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.065)),
            material=black_anodized,
            name=f"extrusion_{rail_index}",
        )
        # Narrow dark slot strips make the support read as a machined extrusion
        # rather than a plain rectangular bar.
        for side_index, side in enumerate((-1.0, 1.0)):
            base.visual(
                Box((1.20, 0.004, 0.014)),
                origin=Origin(xyz=(0.0, y + side * 0.042, 0.065)),
                material=slot_shadow,
                name=f"extrusion_{rail_index}_side_slot_{side_index}",
            )
            base.visual(
                Box((1.20, 0.004, 0.010)),
                origin=Origin(xyz=(0.0, y + side * 0.030, 0.105)),
                material=slot_shadow,
                name=f"extrusion_{rail_index}_top_slot_{side_index}",
            )

        base.visual(
            Box((1.16, 0.030, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.120)),
            material=rail_steel,
            name=f"rail_{rail_index}",
        )
        base.visual(
            Box((1.16, 0.016, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.138)),
            material=rail_steel,
            name=f"rail_{rail_index}_crown",
        )
        for bolt_index, x in enumerate((-0.48, -0.32, -0.16, 0.16, 0.32, 0.48)):
            base.visual(
                Cylinder(radius=0.007, length=0.004),
                origin=Origin(xyz=(x, y, 0.143)),
                material=dark_steel,
                name=f"rail_{rail_index}_bolt_{bolt_index}",
            )
        for stop_index, x in enumerate((-0.555, 0.555)):
            base.visual(
                Box((0.030, 0.058, 0.040)),
                origin=Origin(xyz=(x, y, 0.155)),
                material=dark_steel,
                name=f"end_stop_{rail_index}_{stop_index}",
            )

    for end_index, x in enumerate((-0.64, 0.64)):
        base.visual(
            Box((0.036, 0.54, 0.115)),
            origin=Origin(xyz=(x, 0.0, 0.0825)),
            material=black_anodized,
            name=f"end_plate_{end_index}",
        )
        base.visual(
            Box((0.050, 0.44, 0.035)),
            origin=Origin(xyz=(x * 0.94, 0.0, 0.1325)),
            material=black_anodized,
            name=f"cross_tie_{end_index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.245, 0.500, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        material=carriage_blue,
        name="bridge_plate",
    )
    carriage.visual(
        Box((0.140, 0.220, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=carriage_blue,
        name="tooling_boss",
    )
    carriage.visual(
        Box((0.020, 0.420, 0.060)),
        origin=Origin(xyz=(-0.095, 0.0, 0.218)),
        material=carriage_blue,
        name="web_rib_0",
    )
    carriage.visual(
        Box((0.020, 0.420, 0.060)),
        origin=Origin(xyz=(0.095, 0.0, 0.218)),
        material=carriage_blue,
        name="web_rib_1",
    )

    block_x_positions = (-0.070, 0.070)
    for rail_index, y in enumerate(rail_y_positions):
        for block_index, x in enumerate(block_x_positions):
            guide_index = rail_index * 2 + block_index
            carriage.visual(
                Box((0.100, 0.078, 0.040)),
                origin=Origin(xyz=(x, y, 0.163)),
                material=block_gray,
                name=f"guide_block_{guide_index}_body",
            )
            for cheek_index, side in enumerate((-1.0, 1.0)):
                carriage.visual(
                    Box((0.100, 0.012, 0.050)),
                    origin=Origin(xyz=(x, y + side * 0.034, 0.141)),
                    material=block_gray,
                    name=f"guide_block_{guide_index}_cheek_{cheek_index}",
                )
            for wiper_index, side in enumerate((-1.0, 1.0)):
                carriage.visual(
                    Box((0.010, 0.084, 0.032)),
                    origin=Origin(xyz=(x + side * 0.055, y, 0.158)),
                    material=rubber,
                    name=f"guide_block_{guide_index}_wiper_{wiper_index}",
                )
            carriage.visual(
                Box((0.076, 0.012, 0.006)),
                origin=Origin(xyz=(x, y, 0.144)),
                material=rail_steel,
                name=f"guide_block_{guide_index}_bearing_strip",
            )
            for bolt_index, bx in enumerate((-0.026, 0.026)):
                carriage.visual(
                    Cylinder(radius=0.006, length=0.005),
                    origin=Origin(xyz=(x + bx, y - 0.020, 0.1845)),
                    material=dark_steel,
                    name=f"guide_block_{guide_index}_bolt_{bolt_index}",
                )
                carriage.visual(
                    Cylinder(radius=0.006, length=0.005),
                    origin=Origin(xyz=(x + bx, y + 0.020, 0.1845)),
                    material=dark_steel,
                    name=f"guide_block_{guide_index}_bolt_{bolt_index + 2}",
                )

    for bolt_index, (x, y) in enumerate(
        (
            (-0.070, -0.075),
            (0.070, -0.075),
            (-0.070, 0.075),
            (0.070, 0.075),
            (-0.070, -0.175),
            (0.070, -0.175),
            (-0.070, 0.175),
            (0.070, 0.175),
        )
    ):
        carriage.visual(
            Cylinder(radius=0.0075, length=0.005),
            origin=Origin(xyz=(x, y, 0.2165)),
            material=dark_steel,
            name=f"bridge_bolt_{bolt_index}",
        )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.75, lower=-0.40, upper=0.40),
        motion_properties=MotionProperties(damping=8.0, friction=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("carriage_slide")

    ctx.expect_overlap(
        carriage,
        base,
        axes="y",
        elem_a="guide_block_0_body",
        elem_b="rail_0",
        min_overlap=0.025,
        name="guide block spans first rail",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="y",
        elem_a="guide_block_2_body",
        elem_b="rail_1",
        min_overlap=0.025,
        name="guide block spans second rail",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="guide_block_0_body",
        negative_elem="rail_0_crown",
        min_gap=0.002,
        max_gap=0.012,
        name="first rail has bearing clearance",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="guide_block_2_body",
        negative_elem="rail_1_crown",
        min_gap=0.002,
        max_gap=0.012,
        name="second rail has bearing clearance",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.40}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="guide_block_1_body",
            elem_b="rail_0",
            min_overlap=0.080,
            name="extended carriage remains on first rail",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="guide_block_3_body",
            elem_b="rail_1",
            min_overlap=0.080,
            name="extended carriage remains on second rail",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along rail axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
