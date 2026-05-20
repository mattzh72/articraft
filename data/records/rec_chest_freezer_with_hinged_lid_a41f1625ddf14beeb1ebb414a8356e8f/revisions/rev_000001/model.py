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
    model = ArticulatedObject(name="glass_top_display_chest_freezer")

    white_enamel = model.material("white_enamel", color=(0.92, 0.94, 0.93, 1.0))
    blue_trim = model.material("blue_trim", color=(0.06, 0.22, 0.58, 1.0))
    aluminum = model.material("brushed_aluminum", color=(0.70, 0.72, 0.70, 1.0))
    dark_rubber = model.material("dark_rubber", color=(0.02, 0.025, 0.03, 1.0))
    interior = model.material("pale_liner", color=(0.82, 0.94, 0.98, 1.0))
    glass = model.material("green_tinted_glass", color=(0.62, 0.92, 1.0, 0.36))
    black = model.material("black_vent_shadow", color=(0.01, 0.012, 0.014, 1.0))
    red_pack = model.material("red_pack", color=(0.78, 0.08, 0.06, 1.0))
    yellow_pack = model.material("yellow_pack", color=(0.98, 0.78, 0.08, 1.0))
    green_pack = model.material("green_pack", color=(0.10, 0.55, 0.22, 1.0))

    body = model.part("body")

    # Wide insulated merchandising tub: thick base, four walls, and a visible
    # light liner so the chest reads as hollow through the glass.
    body.visual(
        Box((2.20, 0.90, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=white_enamel,
        name="base_plinth",
    )
    body.visual(
        Box((2.20, 0.08, 0.74)),
        origin=Origin(xyz=(0.0, -0.41, 0.49)),
        material=white_enamel,
        name="front_wall",
    )
    body.visual(
        Box((2.20, 0.08, 0.74)),
        origin=Origin(xyz=(0.0, 0.41, 0.49)),
        material=white_enamel,
        name="rear_wall",
    )
    body.visual(
        Box((0.08, 0.74, 0.74)),
        origin=Origin(xyz=(-1.06, 0.0, 0.49)),
        material=white_enamel,
        name="end_wall_0",
    )
    body.visual(
        Box((0.08, 0.74, 0.74)),
        origin=Origin(xyz=(1.06, 0.0, 0.49)),
        material=white_enamel,
        name="end_wall_1",
    )
    body.visual(
        Box((2.00, 0.70, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=interior,
        name="liner_floor",
    )
    body.visual(
        Box((2.00, 0.012, 0.68)),
        origin=Origin(xyz=(0.0, -0.356, 0.49)),
        material=interior,
        name="front_liner",
    )
    body.visual(
        Box((2.00, 0.012, 0.68)),
        origin=Origin(xyz=(0.0, 0.356, 0.49)),
        material=interior,
        name="rear_liner",
    )
    body.visual(
        Box((0.012, 0.70, 0.68)),
        origin=Origin(xyz=(-1.006, 0.0, 0.49)),
        material=interior,
        name="end_liner_0",
    )
    body.visual(
        Box((0.012, 0.70, 0.68)),
        origin=Origin(xyz=(1.006, 0.0, 0.49)),
        material=interior,
        name="end_liner_1",
    )

    # Static merchandise blocks sit on the liner floor, visible through the lid.
    for i, (x, y, mat) in enumerate(
        [
            (-0.72, -0.18, red_pack),
            (-0.36, -0.18, yellow_pack),
            (0.00, -0.18, green_pack),
            (0.36, -0.18, red_pack),
            (0.72, -0.18, yellow_pack),
            (-0.54, 0.16, green_pack),
            (-0.18, 0.16, red_pack),
            (0.18, 0.16, yellow_pack),
            (0.54, 0.16, green_pack),
        ]
    ):
        body.visual(
            Box((0.28, 0.18, 0.060)),
            origin=Origin(xyz=(x, y, 0.180)),
            material=mat,
            name=f"product_{i}",
        )

    # Blue merchandising band, toe vent slots, feet, and a robust top rim.
    body.visual(
        Box((2.12, 0.010, 0.16)),
        origin=Origin(xyz=(0.0, -0.454, 0.46)),
        material=blue_trim,
        name="front_brand_band",
    )
    for i, x in enumerate([-0.28, -0.20, -0.12, -0.04, 0.04, 0.12, 0.20, 0.28]):
        body.visual(
            Box((0.030, 0.012, 0.115)),
            origin=Origin(xyz=(x, -0.452, 0.250)),
            material=black,
            name=f"vent_slot_{i}",
        )
    for i, (x, y) in enumerate([(-0.95, -0.32), (0.95, -0.32), (-0.95, 0.32), (0.95, 0.32)]):
        body.visual(
            Cylinder(radius=0.045, length=0.060),
            origin=Origin(xyz=(x, y, -0.030)),
            material=dark_rubber,
            name=f"foot_{i}",
        )

    body.visual(
        Box((2.26, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, -0.445, 0.885)),
        material=blue_trim,
        name="front_top_rim",
    )
    body.visual(
        Box((2.26, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, 0.445, 0.885)),
        material=blue_trim,
        name="rear_top_rim",
    )
    body.visual(
        Box((0.070, 0.88, 0.070)),
        origin=Origin(xyz=(-1.095, 0.0, 0.885)),
        material=blue_trim,
        name="end_top_rim_0",
    )
    body.visual(
        Box((0.070, 0.88, 0.070)),
        origin=Origin(xyz=(1.095, 0.0, 0.885)),
        material=blue_trim,
        name="end_top_rim_1",
    )
    body.visual(
        Box((2.08, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, -0.405, 0.923)),
        material=aluminum,
        name="front_guide_rail",
    )
    body.visual(
        Box((2.08, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.405, 0.923)),
        material=aluminum,
        name="rear_guide_rail",
    )

    # Fixed lower glass leaf on the right half of the freezer top.  The movable
    # panel rides above it so the two leaves can overlap without colliding.
    body.visual(
        Box((1.06, 0.66, 0.012)),
        origin=Origin(xyz=(0.53, 0.0, 0.936)),
        material=glass,
        name="fixed_glass",
    )
    body.visual(
        Box((1.10, 0.040, 0.030)),
        origin=Origin(xyz=(0.53, -0.370, 0.935)),
        material=aluminum,
        name="fixed_front_frame",
    )
    body.visual(
        Box((1.10, 0.040, 0.030)),
        origin=Origin(xyz=(0.53, 0.370, 0.935)),
        material=aluminum,
        name="fixed_rear_frame",
    )
    body.visual(
        Box((0.040, 0.72, 0.030)),
        origin=Origin(xyz=(0.00, 0.0, 0.935)),
        material=aluminum,
        name="fixed_center_frame",
    )
    body.visual(
        Box((0.040, 0.72, 0.030)),
        origin=Origin(xyz=(1.06, 0.0, 0.935)),
        material=aluminum,
        name="fixed_end_frame",
    )

    sliding_lid = model.part("sliding_lid")
    sliding_lid.visual(
        Box((1.08, 0.71, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=glass,
        name="glass_pane",
    )
    sliding_lid.visual(
        Box((1.10, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, -0.370, 0.000)),
        material=aluminum,
        name="front_frame",
    )
    sliding_lid.visual(
        Box((1.10, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, 0.370, 0.000)),
        material=aluminum,
        name="rear_frame",
    )
    sliding_lid.visual(
        Box((0.045, 0.74, 0.030)),
        origin=Origin(xyz=(-0.550, 0.0, 0.000)),
        material=aluminum,
        name="side_frame_0",
    )
    sliding_lid.visual(
        Box((0.045, 0.74, 0.030)),
        origin=Origin(xyz=(0.550, 0.0, 0.000)),
        material=aluminum,
        name="side_frame_1",
    )
    sliding_lid.visual(
        Box((1.08, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.405, -0.025)),
        material=dark_rubber,
        name="front_runner",
    )
    sliding_lid.visual(
        Box((1.08, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.405, -0.025)),
        material=dark_rubber,
        name="rear_runner",
    )
    sliding_lid.visual(
        Box((0.22, 0.035, 0.035)),
        origin=Origin(xyz=(-0.28, -0.365, 0.030)),
        material=dark_rubber,
        name="pull_handle",
    )

    model.articulation(
        "body_to_sliding_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=sliding_lid,
        origin=Origin(xyz=(-0.50, 0.0, 0.970)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.45, lower=0.0, upper=0.98),
        motion_properties=MotionProperties(damping=2.0, friction=0.6),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("sliding_lid")
    slide = object_model.get_articulation("body_to_sliding_lid")

    ctx.check("wide_merchandising_body", body is not None, "Expected a body root part.")
    ctx.check("sliding_lid_present", lid is not None, "Expected a sliding glass lid part.")
    ctx.check(
        "prismatic_lid_slide",
        slide is not None and slide.articulation_type == ArticulationType.PRISMATIC,
        "The glass lid must use a prismatic guide-rail slide.",
    )
    if body is None or lid is None or slide is None:
        return ctx.report()

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        min_pt, max_pt = body_aabb
        ctx.check(
            "body_is_wide",
            float(max_pt[0] - min_pt[0]) > 2.0 and float(max_pt[1] - min_pt[1]) > 0.8,
            details=f"aabb={body_aabb}",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="front_runner",
        negative_elem="front_guide_rail",
        max_penetration=0.0005,
        max_gap=0.006,
        name="front runner sits on guide rail",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="front_runner",
        elem_b="front_guide_rail",
        min_overlap=0.020,
        name="front runner remains on guide rail",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="glass_pane",
        negative_elem="fixed_glass",
        min_gap=0.020,
        name="sliding glass rides above fixed glass",
    )

    rest_pos = ctx.part_world_position(lid)
    with ctx.pose({slide: 0.98}):
        extended_pos = ctx.part_world_position(lid)
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="rear_runner",
            elem_b="rear_guide_rail",
            min_overlap=0.020,
            name="extended rear runner stays captured on rail",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="glass_pane",
            negative_elem="fixed_glass",
            min_gap=0.020,
            name="extended glass clears fixed glass",
        )
    ctx.check(
        "lid_slides_lengthwise",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.90,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
