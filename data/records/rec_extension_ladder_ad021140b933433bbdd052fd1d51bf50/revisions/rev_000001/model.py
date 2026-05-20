from __future__ import annotations

import math

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
    model = ArticulatedObject(name="loft_access_ladder")

    aluminium = model.material("satin_aluminium", rgba=(0.76, 0.78, 0.76, 1.0))
    darker_aluminium = model.material("brushed_shadow_aluminium", rgba=(0.48, 0.50, 0.50, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    pin_steel = model.material("dark_hinge_pin", rgba=(0.12, 0.13, 0.14, 1.0))
    warning_red = model.material("red_release_catch", rgba=(0.70, 0.04, 0.02, 1.0))

    rung_rot = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    lower = model.part("lower_section")
    # Flat stabilising base and four soft feet.
    lower.visual(
        Box((0.70, 0.32, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=darker_aluminium,
        name="flat_base",
    )
    for i, x in enumerate((-0.28, 0.28)):
        for j, y in enumerate((-0.115, 0.115)):
            lower.visual(
                Box((0.105, 0.065, 0.026)),
                origin=Origin(xyz=(x, y, 0.013)),
                material=rubber,
                name=f"rubber_foot_{i}_{j}",
            )

    # Two narrow aluminium lower rails, bolted to the base and tied by round rungs.
    for i, x in enumerate((-0.235, 0.235)):
        lower.visual(
            Box((0.045, 0.045, 1.36)),
            origin=Origin(xyz=(x, 0.0, 0.765)),
            material=aluminium,
            name=f"lower_rail_{i}",
        )
        lower.visual(
            Box((0.080, 0.070, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.071)),
            material=darker_aluminium,
            name=f"rail_shoe_{i}",
        )
        lower.visual(
            Box((0.060, 0.046, 0.120)),
            origin=Origin(xyz=(x, -0.020, 1.445)),
            material=darker_aluminium,
            name=f"hinge_plate_{i}",
        )

    for idx, z in enumerate((0.30, 0.56, 0.82, 1.08, 1.32)):
        lower.visual(
            Cylinder(radius=0.018, length=0.49),
            origin=Origin(xyz=(0.0, -0.006, z), rpy=rung_rot.rpy),
            material=aluminium,
            name=f"lower_rung_{idx}",
        )

    # Split lower hinge knuckles leave the centre space for the folding upper knuckle.
    for i, x in enumerate((-0.165, 0.165)):
        lower.visual(
            Cylinder(radius=0.025, length=0.190),
            origin=Origin(xyz=(x, -0.030, 1.480), rpy=rung_rot.rpy),
            material=pin_steel,
            name=f"lower_hinge_knuckle_{i}",
        )

    upper = model.part("upper_carriage")
    # The upper carriage frame is located at the midpoint hinge and initially points upward.
    upper.visual(
        Cylinder(radius=0.022, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=rung_rot.rpy),
        material=pin_steel,
        name="upper_hinge_knuckle",
    )
    upper.visual(
        Box((0.120, 0.038, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=darker_aluminium,
        name="hinge_web",
    )
    upper.visual(
        Box((0.545, 0.038, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=darker_aluminium,
        name="hinge_crossbar",
    )

    for i, x in enumerate((-0.245, 0.245)):
        upper.visual(
            Box((0.040, 0.045, 1.05)),
            origin=Origin(xyz=(x, -0.002, 0.625)),
            material=aluminium,
            name=f"guide_rail_{i}",
        )
        # Small lips make the fixed upper rails read as slide guides.
        upper.visual(
            Box((0.027, 0.018, 0.96)),
            origin=Origin(xyz=(x * 1.10, 0.0295, 0.650)),
            material=darker_aluminium,
            name=f"guide_lip_{i}",
        )

    for idx, z in enumerate((0.27, 0.53, 0.79, 1.05)):
        upper.visual(
            Cylinder(radius=0.017, length=0.51),
            origin=Origin(xyz=(0.0, -0.027, z), rpy=rung_rot.rpy),
            material=aluminium,
            name=f"upper_rung_{idx}",
        )

    upper.visual(
        Box((0.075, 0.028, 0.045)),
        origin=Origin(xyz=(0.0, -0.026, 0.135)),
        material=warning_red,
        name="release_catch",
    )

    fly = model.part("fly_section")
    # Inboard sliding fly section: clear of the outer guide rails, but overlapping
    # them along length so it remains visibly retained at full extension.
    for i, x in enumerate((-0.207, 0.207)):
        fly.visual(
            Box((0.036, 0.038, 1.380)),
            origin=Origin(xyz=(x, 0.0395, 0.690)),
            material=aluminium,
            name=f"fly_rail_{i}",
        )
        fly.visual(
            Box((0.060, 0.046, 0.050)),
            origin=Origin(xyz=(x * 0.87, 0.0395, 0.035)),
            material=darker_aluminium,
            name=f"slide_stop_{i}",
        )

    for idx, z in enumerate((0.22, 0.48, 0.74, 1.00, 1.26)):
        fly.visual(
            Cylinder(radius=0.0155, length=0.452),
            origin=Origin(xyz=(0.0, 0.0395, z), rpy=rung_rot.rpy),
            material=aluminium,
            name=f"fly_rung_{idx}",
        )

    fly.visual(
        Box((0.452, 0.046, 0.035)),
        origin=Origin(xyz=(0.0, 0.0395, 1.390)),
        material=darker_aluminium,
        name="top_cap",
    )
    fly.visual(
        Box((0.452, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0395, 0.035)),
        material=darker_aluminium,
        name="bottom_tie",
    )

    model.articulation(
        "mid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, -0.030, 1.480)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=upper,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_section")
    upper = object_model.get_part("upper_carriage")
    fly = object_model.get_part("fly_section")
    hinge = object_model.get_articulation("mid_hinge")
    slide = object_model.get_articulation("fly_slide")

    ctx.expect_overlap(
        fly,
        upper,
        axes="z",
        elem_a="fly_rail_0",
        elem_b="guide_rail_0",
        min_overlap=0.90,
        name="retracted fly rail remains captured by guide",
    )

    rest_aabb = ctx.part_world_aabb(fly)
    rest_top = rest_aabb[1][2] if rest_aabb is not None else None
    with ctx.pose({slide: 0.55}):
        ctx.expect_overlap(
            fly,
            upper,
            axes="z",
            elem_a="fly_rail_0",
            elem_b="guide_rail_0",
            min_overlap=0.42,
            name="extended fly rail keeps retained insertion",
        )
        extended_aabb = ctx.part_world_aabb(fly)
        extended_top = extended_aabb[1][2] if extended_aabb is not None else None

    ctx.check(
        "prismatic joint extends the fly section upward",
        rest_top is not None and extended_top is not None and extended_top > rest_top + 0.50,
        details=f"rest_top={rest_top}, extended_top={extended_top}",
    )

    upright_aabb = ctx.part_world_aabb(upper)
    upright_center_y = (
        (upright_aabb[0][1] + upright_aabb[1][1]) / 2.0 if upright_aabb is not None else None
    )
    with ctx.pose({hinge: 1.25}):
        folded_aabb = ctx.part_world_aabb(upper)
        folded_center_y = (
            (folded_aabb[0][1] + folded_aabb[1][1]) / 2.0 if folded_aabb is not None else None
        )
    ctx.check(
        "revolute midpoint hinge folds the upper carriage forward",
        upright_center_y is not None
        and folded_center_y is not None
        and folded_center_y < upright_center_y - 0.45,
        details=f"upright_y={upright_center_y}, folded_y={folded_center_y}",
    )

    ctx.expect_overlap(
        lower,
        upper,
        axes="x",
        elem_a="lower_hinge_knuckle_0",
        elem_b="upper_hinge_knuckle",
        min_overlap=0.0,
        name="split hinge barrels share a common width line",
    )

    return ctx.report()


object_model = build_object_model()
