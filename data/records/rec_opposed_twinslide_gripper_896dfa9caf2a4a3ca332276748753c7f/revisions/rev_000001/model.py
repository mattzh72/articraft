from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="opposed_twin_slide_gripper")

    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.06, 0.07, 0.08, 1.0))
    blue_anodized = Material("blue_anodized_carriage", rgba=(0.02, 0.18, 0.42, 1.0))
    steel = Material("polished_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    bolt_black = Material("black_oxide_bolts", rgba=(0.015, 0.015, 0.018, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.200, 0.120, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_anodized,
        name="main_housing",
    )
    body.visual(
        Box((0.155, 0.012, 0.120)),
        origin=Origin(xyz=(0.0, -0.066, 0.060)),
        material=dark_anodized,
        name="rear_mounting_plate",
    )
    body.visual(
        Box((0.020, 0.104, 0.046)),
        origin=Origin(xyz=(-0.105, 0.0, 0.046)),
        material=dark_anodized,
        name="rail_support_0",
    )
    body.visual(
        Box((0.020, 0.104, 0.046)),
        origin=Origin(xyz=(0.105, 0.0, 0.046)),
        material=dark_anodized,
        name="rail_support_1",
    )
    for y, rail_name in ((-0.035, "guide_rail_0"), (0.035, "guide_rail_1")):
        body.visual(
            Cylinder(radius=0.006, length=0.230),
            origin=Origin(xyz=(0.0, y, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=rail_name,
        )
    for ix, x in enumerate((-0.052, 0.052)):
        for iz, z in enumerate((0.035, 0.085)):
            body.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(x, -0.073, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=bolt_black,
                name=f"mount_bolt_{ix}_{iz}",
            )

    left_jaw = model.part("jaw_0")
    _add_jaw_visuals(left_jaw, side=1.0, carriage_material=blue_anodized, pad_material=black_rubber, bolt_material=bolt_black)

    right_jaw = model.part("jaw_1")
    _add_jaw_visuals(right_jaw, side=-1.0, carriage_material=blue_anodized, pad_material=black_rubber, bolt_material=bolt_black)

    travel = 0.040
    model.articulation(
        "body_to_jaw_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=travel),
    )
    model.articulation(
        "body_to_jaw_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=travel),
    )
    return model


def _add_jaw_visuals(part, *, side: float, carriage_material: Material, pad_material: Material, bolt_material: Material) -> None:
    """Add one mirrored sliding jaw. side=+1 points the jaw nose toward +X."""
    part.visual(
        Box((0.038, 0.096, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=carriage_material,
        name="carriage_bridge",
    )
    part.visual(
        Box((0.038, 0.050, 0.012)),
        origin=Origin(xyz=(side * 0.014, 0.060, 0.070)),
        material=carriage_material,
        name="front_neck",
    )
    for iy, rail_y in enumerate((-0.035, 0.035)):
        for io, offset in enumerate((-0.00875, 0.00875)):
            part.visual(
                Box((0.038, 0.0055, 0.020)),
                origin=Origin(xyz=(0.0, rail_y + offset, 0.056)),
                material=carriage_material,
                name=f"rail_cheek_{iy}_{io}",
            )
    part.visual(
        Box((0.020, 0.024, 0.064)),
        origin=Origin(xyz=(side * 0.018, 0.078, 0.040)),
        material=carriage_material,
        name="jaw_upright",
    )
    part.visual(
        Box((0.014, 0.030, 0.052)),
        origin=Origin(xyz=(side * 0.025, 0.082, 0.034)),
        material=pad_material,
        name="grip_pad",
    )
    for iz, z in enumerate((0.044, 0.062)):
        part.visual(
            Cylinder(radius=0.0035, length=0.003),
            origin=Origin(xyz=(side * 0.026, 0.066, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_material,
            name=f"jaw_bolt_{iz}",
        )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_jaw = object_model.get_part("jaw_0")
    right_jaw = object_model.get_part("jaw_1")
    left_slide = object_model.get_articulation("body_to_jaw_0")
    right_slide = object_model.get_articulation("body_to_jaw_1")

    ctx.check(
        "opposed slides have 40 mm travel",
        abs(left_slide.motion_limits.upper - 0.040) < 1e-6
        and abs(right_slide.motion_limits.upper - 0.040) < 1e-6
        and left_slide.motion_limits.lower == 0.0
        and right_slide.motion_limits.lower == 0.0,
        details=f"limits={left_slide.motion_limits}, {right_slide.motion_limits}",
    )
    ctx.check(
        "slides share one closing axis",
        tuple(left_slide.axis) == (1.0, 0.0, 0.0) and tuple(right_slide.axis) == (-1.0, 0.0, 0.0),
        details=f"axes={left_slide.axis}, {right_slide.axis}",
    )

    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        min_gap=0.080,
        max_gap=0.092,
        positive_elem="grip_pad",
        negative_elem="grip_pad",
        name="open jaws leave a wide central grip gap",
    )

    rest_left = ctx.part_world_position(left_jaw)
    rest_right = ctx.part_world_position(right_jaw)
    with ctx.pose({left_slide: 0.040, right_slide: 0.040}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.004,
            max_gap=0.008,
            positive_elem="grip_pad",
            negative_elem="grip_pad",
            name="closed jaws nearly meet at the center",
        )
        closed_left = ctx.part_world_position(left_jaw)
        closed_right = ctx.part_world_position(right_jaw)

    ctx.check(
        "left jaw translates toward center",
        rest_left is not None and closed_left is not None and closed_left[0] > rest_left[0] + 0.039,
        details=f"rest={rest_left}, closed={closed_left}",
    )
    ctx.check(
        "right jaw translates toward center",
        rest_right is not None and closed_right is not None and closed_right[0] < rest_right[0] - 0.039,
        details=f"rest={rest_right}, closed={closed_right}",
    )

    return ctx.report()


object_model = build_object_model()
