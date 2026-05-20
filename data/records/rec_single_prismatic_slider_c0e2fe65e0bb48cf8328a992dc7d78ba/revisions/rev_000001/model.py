from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_rail_shuttle")

    rail_metal = Material("powder_coated_rail", color=(0.32, 0.34, 0.35, 1.0))
    worn_steel = Material("bright_wear_surfaces", color=(0.64, 0.67, 0.68, 1.0))
    shuttle_blue = Material("anodized_shuttle", color=(0.08, 0.20, 0.42, 1.0))
    front_orange = Material("front_plate_orange", color=(0.95, 0.35, 0.07, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((0.80, 0.22, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=rail_metal,
        name="base_bed",
    )
    rail.visual(
        Box((0.80, 0.025, 0.085)),
        origin=Origin(xyz=(0.0, 0.0975, 0.0725)),
        material=rail_metal,
        name="guide_wall_0",
    )
    rail.visual(
        Box((0.80, 0.025, 0.085)),
        origin=Origin(xyz=(0.0, -0.0975, 0.0725)),
        material=rail_metal,
        name="guide_wall_1",
    )
    rail.visual(
        Box((0.80, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.070, 0.125)),
        material=rail_metal,
        name="top_lip_0",
    )
    rail.visual(
        Box((0.80, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.070, 0.125)),
        material=rail_metal,
        name="top_lip_1",
    )
    rail.visual(
        Box((0.030, 0.22, 0.135)),
        origin=Origin(xyz=(-0.415, 0.0, 0.0675)),
        material=rail_metal,
        name="rear_stop",
    )
    rail.visual(
        Box((0.74, 0.135, 0.006)),
        origin=Origin(xyz=(0.015, 0.0, 0.033)),
        material=worn_steel,
        name="slide_wear_strip",
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((0.340, 0.105, 0.075)),
        # The child frame is at the front opening of the rail.  The moving block
        # extends back into the guide so it remains captured at full travel.
        origin=Origin(xyz=(-0.170, 0.0, -0.0215)),
        material=shuttle_blue,
        name="slider_body",
    )
    shuttle.visual(
        Box((0.305, 0.018, 0.012)),
        origin=Origin(xyz=(-0.170, 0.056, -0.010)),
        material=worn_steel,
        name="side_bearing_0",
    )
    shuttle.visual(
        Box((0.305, 0.018, 0.012)),
        origin=Origin(xyz=(-0.170, -0.056, -0.010)),
        material=worn_steel,
        name="side_bearing_1",
    )
    shuttle.visual(
        Box((0.030, 0.180, 0.180)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=front_orange,
        name="front_plate",
    )

    model.articulation(
        "rail_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=shuttle,
        origin=Origin(xyz=(0.400, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    shuttle = object_model.get_part("shuttle")
    slide = object_model.get_articulation("rail_to_shuttle")

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            shuttle,
            rail,
            elem_a="slider_body",
            elem_b="slide_wear_strip",
            name="shuttle rides on rail bed at rest",
        )
        ctx.expect_gap(
            shuttle,
            rail,
            axis="x",
            positive_elem="front_plate",
            negative_elem="base_bed",
            max_gap=0.001,
            max_penetration=0.0,
            name="front plate starts seated at rail face",
        )
        ctx.expect_overlap(
            shuttle,
            rail,
            axes="x",
            elem_a="slider_body",
            elem_b="base_bed",
            min_overlap=0.330,
            name="rest pose has long guide insertion",
        )

    rest_pos = ctx.part_world_position(shuttle)
    with ctx.pose({slide: 0.220}):
        ctx.expect_contact(
            shuttle,
            rail,
            elem_a="slider_body",
            elem_b="slide_wear_strip",
            name="shuttle still bears on rail when extended",
        )
        ctx.expect_overlap(
            shuttle,
            rail,
            axes="x",
            elem_a="slider_body",
            elem_b="base_bed",
            min_overlap=0.100,
            name="extended shuttle remains guided in rail",
        )
        extended_pos = ctx.part_world_position(shuttle)

    ctx.check(
        "prismatic joint moves shuttle along rail axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
