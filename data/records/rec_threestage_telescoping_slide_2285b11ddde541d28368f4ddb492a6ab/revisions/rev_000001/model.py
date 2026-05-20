from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_extension_rail")

    zinc = model.material("brushed_zinc", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_zinc = model.material("dark_zinc", rgba=(0.30, 0.32, 0.33, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.04, 0.045, 0.05, 1.0))
    polymer = model.material("black_polymer_wear", rgba=(0.015, 0.014, 0.012, 1.0))
    stop_gold = model.material("yellow_stop_tabs", rgba=(0.90, 0.66, 0.22, 1.0))
    carriage_paint = model.material("carriage_paint", rgba=(0.12, 0.17, 0.22, 1.0))

    def box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    outer_length = 1.20
    middle_length = 1.05
    inner_length = 0.90
    middle_rest_inset = 0.10
    inner_rest_inset = 0.10

    outer = model.part("outer_member")
    box(outer, "bottom_web", (outer_length, 0.120, 0.008), (outer_length / 2.0, 0.0, 0.004), zinc)
    box(outer, "side_wall_0", (outer_length, 0.008, 0.050), (outer_length / 2.0, -0.056, 0.029), zinc)
    box(outer, "side_wall_1", (outer_length, 0.008, 0.050), (outer_length / 2.0, 0.056, 0.029), zinc)
    box(outer, "rolled_lip_0", (outer_length, 0.020, 0.006), (outer_length / 2.0, -0.042, 0.051), zinc)
    box(outer, "rolled_lip_1", (outer_length, 0.020, 0.006), (outer_length / 2.0, 0.042, 0.051), zinc)
    box(outer, "rear_mount_plate", (0.070, 0.138, 0.010), (0.035, 0.0, -0.005), dark_zinc)
    box(outer, "front_mount_plate", (0.070, 0.138, 0.010), (outer_length - 0.035, 0.0, -0.005), dark_zinc)
    for idx, x in enumerate((0.13, outer_length - 0.13)):
        box(outer, f"mount_slot_{idx}", (0.050, 0.018, 0.001), (x, 0.0, 0.0075), black_oxide)
    for idx, y in enumerate((-0.042, 0.042)):
        box(outer, f"front_stop_{idx}", (0.018, 0.012, 0.024), (outer_length - 0.030, y, 0.036), stop_gold)
        wear_y = -0.034 if y < 0.0 else 0.034
        box(outer, f"lip_wear_{idx}", (0.160, 0.007, 0.003), (outer_length - 0.240, wear_y, 0.0465), polymer)
        box(outer, f"rear_wear_{idx}", (0.130, 0.007, 0.003), (0.220, wear_y, 0.0465), polymer)

    middle = model.part("middle_rail")
    box(middle, "middle_web", (middle_length, 0.064, 0.006), (middle_length / 2.0, 0.0, 0.013), dark_zinc)
    box(middle, "middle_wall_0", (middle_length, 0.006, 0.024), (middle_length / 2.0, -0.033, 0.028), dark_zinc)
    box(middle, "middle_wall_1", (middle_length, 0.006, 0.024), (middle_length / 2.0, 0.033, 0.028), dark_zinc)
    box(middle, "middle_lip_0", (middle_length, 0.013, 0.005), (middle_length / 2.0, -0.0265, 0.0425), dark_zinc)
    box(middle, "middle_lip_1", (middle_length, 0.013, 0.005), (middle_length / 2.0, 0.0265, 0.0425), dark_zinc)
    for idx, y in enumerate((-0.026, 0.026)):
        wear_y = -0.0245 if y < 0.0 else 0.0245
        box(middle, f"inner_wear_{idx}", (0.150, 0.005, 0.003), (0.180, wear_y, 0.039), polymer)
        box(middle, f"front_wear_{idx}", (0.150, 0.005, 0.003), (middle_length - 0.180, wear_y, 0.039), polymer)
        box(middle, f"middle_stop_{idx}", (0.020, 0.008, 0.018), (middle_length - 0.040, y, 0.031), stop_gold)
        box(middle, f"rear_keeper_{idx}", (0.020, 0.008, 0.016), (0.040, y, 0.030), stop_gold)

    inner = model.part("inner_rail")
    box(inner, "inner_lower_web", (inner_length, 0.040, 0.006), (inner_length / 2.0, 0.0, 0.024), black_oxide)
    box(inner, "inner_rib_0", (inner_length, 0.005, 0.012), (inner_length / 2.0, -0.0195, 0.0325), black_oxide)
    box(inner, "inner_rib_1", (inner_length, 0.005, 0.012), (inner_length / 2.0, 0.0195, 0.0325), black_oxide)
    box(inner, "inner_top_face", (inner_length, 0.032, 0.004), (inner_length / 2.0, 0.0, 0.039), black_oxide)
    for idx, x in enumerate((0.080, inner_length - 0.180)):
        box(inner, f"shoe_pad_{idx}", (0.060, 0.030, 0.002), (x, 0.0, 0.042), polymer)
    for idx, y in enumerate((-0.017, 0.017)):
        box(inner, f"nose_stop_{idx}", (0.018, 0.006, 0.014), (inner_length - 0.030, y, 0.032), stop_gold)

    carriage = model.part("top_carriage")
    box(carriage, "neck_post", (0.045, 0.020, 0.019), (-0.055, 0.0, 0.0505), carriage_paint)
    box(carriage, "top_plate", (0.200, 0.080, 0.012), (-0.020, 0.0, 0.066), carriage_paint)
    box(carriage, "front_lug", (0.035, 0.050, 0.018), (0.055, 0.0, 0.081), carriage_paint)
    for idx, y in enumerate((-0.025, 0.025)):
        box(carriage, f"bolt_pad_{idx}", (0.030, 0.018, 0.002), (-0.035, y, 0.073), black_oxide)

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(middle_rest_inset, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.55, effort=140.0, velocity=0.35),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(inner_rest_inset, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.45, effort=100.0, velocity=0.35),
    )
    model.articulation(
        "inner_to_carriage",
        ArticulationType.FIXED,
        parent=inner,
        child=carriage,
        origin=Origin(xyz=(inner_length, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_member")
    middle = object_model.get_part("middle_rail")
    inner = object_model.get_part("inner_rail")
    carriage = object_model.get_part("top_carriage")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(middle, outer, axes="yz", margin=0.001, name="middle rail is nested in outer channel")
    ctx.expect_within(inner, middle, axes="yz", margin=0.001, name="inner rail is nested in middle channel")
    ctx.expect_overlap(middle, outer, axes="x", min_overlap=0.90, name="retracted middle retains long overlap")
    ctx.expect_overlap(inner, middle, axes="x", min_overlap=0.78, name="retracted inner retains long overlap")
    ctx.expect_contact(middle, outer, contact_tol=0.001, name="retracted middle bears on outer wear strips")
    ctx.expect_contact(inner, middle, contact_tol=0.001, name="retracted inner bears on middle wear strips")
    ctx.expect_contact(carriage, inner, elem_a="neck_post", elem_b="inner_top_face", contact_tol=0.001, name="carriage post is seated on inner rail")

    rest_middle = ctx.part_world_position(middle)
    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({outer_slide: 0.55, inner_slide: 0.45}):
        ctx.expect_within(middle, outer, axes="yz", margin=0.001, name="extended middle stays inside outer channel")
        ctx.expect_within(inner, middle, axes="yz", margin=0.001, name="extended inner stays inside middle channel")
        ctx.expect_overlap(middle, outer, axes="x", min_overlap=0.50, name="extended middle keeps believable engagement")
        ctx.expect_overlap(inner, middle, axes="x", min_overlap=0.45, name="extended inner keeps believable engagement")
        ctx.expect_contact(middle, outer, contact_tol=0.001, name="extended middle remains carried by outer channel")
        ctx.expect_contact(inner, middle, contact_tol=0.001, name="extended inner remains carried by middle channel")
        extended_middle = ctx.part_world_position(middle)
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "middle stage translates forward",
        rest_middle is not None and extended_middle is not None and extended_middle[0] > rest_middle[0] + 0.50,
        details=f"rest={rest_middle}, extended={extended_middle}",
    )
    ctx.check(
        "inner stage translates forward serially",
        rest_inner is not None and extended_inner is not None and extended_inner[0] > rest_inner[0] + 0.90,
        details=f"rest={rest_inner}, extended={extended_inner}",
    )

    return ctx.report()


object_model = build_object_model()
