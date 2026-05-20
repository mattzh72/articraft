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


TUBE = Material("brushed_aluminum", color=(0.72, 0.74, 0.72, 1.0))
WIRE = Material("white_coated_wire", color=(0.92, 0.94, 0.91, 1.0))
PLASTIC = Material("white_plastic_bracket", color=(0.86, 0.88, 0.84, 1.0))
RUBBER = Material("dark_rubber", color=(0.04, 0.04, 0.035, 1.0))


def _rod(part, name: str, axis: str, center, length: float, radius: float, material):
    if axis == "x":
        rpy = (0.0, pi / 2.0, 0.0)
    elif axis == "y":
        rpy = (-pi / 2.0, 0.0, 0.0)
    elif axis == "z":
        rpy = (0.0, 0.0, 0.0)
    else:
        raise ValueError(f"Unsupported rod axis {axis!r}")
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _box(part, name: str, center, size, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    top_z = 0.86
    half_width = 0.31
    half_depth = 0.36

    central = model.part("central_frame")
    _rod(central, "side_rod_pos", "y", (half_width, 0.0, top_z), 0.74, 0.012, TUBE)
    _rod(central, "side_rod_neg", "y", (-half_width, 0.0, top_z), 0.74, 0.012, TUBE)
    _rod(central, "front_rod", "x", (0.0, half_depth, top_z), 0.64, 0.012, TUBE)
    _rod(central, "rear_rod", "x", (0.0, -half_depth, top_z), 0.64, 0.012, TUBE)

    for i, y in enumerate((-0.30, -0.18, -0.06, 0.06, 0.18, 0.30)):
        _rod(central, f"drying_rail_{i}", "x", (0.0, y, top_z), 0.60, 0.0055, WIRE)

    # Small brackets keep the lower folding frame close to the rear top rail.
    _rod(central, "lower_hinge_pin", "x", (0.0, -0.395, top_z - 0.055), 0.58, 0.008, TUBE)
    _box(central, "top_bracket_0", (0.245, -0.385, top_z - 0.032), (0.075, 0.045, 0.075), PLASTIC)
    _box(central, "top_bracket_1", (-0.245, -0.385, top_z - 0.032), (0.075, 0.045, 0.075), PLASTIC)

    def make_wing(name: str, sign: float):
        wing = model.part(name)
        _rod(wing, "hinge_barrel_0", "y", (0.0, -0.24, 0.0), 0.07, 0.016, PLASTIC)
        _rod(wing, "hinge_barrel_1", "y", (0.0, 0.24, 0.0), 0.07, 0.016, PLASTIC)
        _rod(wing, "inner_rod", "y", (sign * 0.026, 0.0, 0.0), 0.74, 0.010, TUBE)
        _rod(wing, "outer_rod", "y", (sign * 0.42, 0.0, 0.0), 0.74, 0.010, TUBE)
        _rod(wing, "front_rod", "x", (sign * 0.22, half_depth, 0.0), 0.41, 0.010, TUBE)
        _rod(wing, "rear_rod", "x", (sign * 0.22, -half_depth, 0.0), 0.41, 0.010, TUBE)
        for i, y in enumerate((-0.30, -0.18, -0.06, 0.06, 0.18, 0.30)):
            _rod(wing, f"drying_rail_{i}", "x", (sign * 0.223, y, 0.0), 0.398, 0.005, WIRE)
        # Lug blocks visibly tie the hinge barrels to the welded wing frame.
        _box(wing, "hinge_lug_0", (sign * 0.022, -0.24, -0.015), (0.032, 0.055, 0.006), PLASTIC)
        _box(wing, "hinge_lug_1", (sign * 0.022, 0.24, -0.015), (0.032, 0.055, 0.006), PLASTIC)
        return wing

    wing_0 = make_wing("wing_0", 1.0)
    wing_1 = make_wing("wing_1", -1.0)

    lower = model.part("lower_support")
    _rod(lower, "hinge_barrel_0", "x", (-0.14, 0.0, 0.0), 0.12, 0.016, PLASTIC)
    _rod(lower, "hinge_barrel_1", "x", (0.14, 0.0, 0.0), 0.12, 0.016, PLASTIC)
    _box(lower, "hinge_lug_0", (-0.14, 0.0, -0.042), (0.10, 0.035, 0.060), PLASTIC)
    _box(lower, "hinge_lug_1", (0.14, 0.0, -0.042), (0.10, 0.035, 0.060), PLASTIC)
    _rod(lower, "top_rail", "x", (0.0, 0.0, -0.078), 0.56, 0.010, TUBE)
    _rod(lower, "side_rod_0", "z", (-0.265, 0.0, -0.245), 0.32, 0.010, TUBE)
    _rod(lower, "side_rod_1", "z", (0.265, 0.0, -0.245), 0.32, 0.010, TUBE)
    _rod(lower, "bottom_rail", "x", (0.0, 0.0, -0.405), 0.56, 0.010, TUBE)
    for i, z in enumerate((-0.165, -0.245, -0.325)):
        _rod(lower, f"hanging_rail_{i}", "x", (0.0, 0.0, z), 0.53, 0.0055, WIRE)
    # The short folding stays are deliberately close to the top bracket, not long leg braces.
    _rod(lower, "short_fold_member_0", "z", (-0.235, 0.0, -0.075), 0.105, 0.006, WIRE)
    _rod(lower, "short_fold_member_1", "z", (0.235, 0.0, -0.075), 0.105, 0.006, WIRE)
    _box(lower, "rubber_foot_0", (-0.265, 0.0, -0.420), (0.055, 0.032, 0.018), RUBBER)
    _box(lower, "rubber_foot_1", (0.265, 0.0, -0.420), (0.055, 0.032, 0.018), RUBBER)

    model.articulation(
        "central_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_0,
        origin=Origin(xyz=(half_width, 0.0, top_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "central_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_1,
        origin=Origin(xyz=(-half_width, 0.0, top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "central_to_lower_support",
        ArticulationType.REVOLUTE,
        parent=central,
        child=lower,
        origin=Origin(xyz=(0.0, -0.395, top_z - 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    lower = object_model.get_part("lower_support")
    wing_0_joint = object_model.get_articulation("central_to_wing_0")
    wing_1_joint = object_model.get_articulation("central_to_wing_1")
    lower_joint = object_model.get_articulation("central_to_lower_support")

    for wing, side_elem in ((wing_0, "side_rod_pos"), (wing_1, "side_rod_neg")):
        for barrel in ("hinge_barrel_0", "hinge_barrel_1"):
            ctx.allow_overlap(
                central,
                wing,
                elem_a=side_elem,
                elem_b=barrel,
                reason="The wing hinge barrel is modeled as a solid sleeve around the central side rod/pin.",
            )
            ctx.expect_overlap(
                central,
                wing,
                axes="y",
                elem_a=side_elem,
                elem_b=barrel,
                min_overlap=0.055,
                name=f"{wing.name}_{barrel}_has_pin_engagement",
            )
            ctx.expect_within(
                central,
                wing,
                axes="xz",
                inner_elem=side_elem,
                outer_elem=barrel,
                margin=0.002,
                name=f"{wing.name}_{barrel}_wraps_pin_cross_section",
            )

    for barrel in ("hinge_barrel_0", "hinge_barrel_1"):
        ctx.allow_overlap(
            central,
            lower,
            elem_a="lower_hinge_pin",
            elem_b=barrel,
            reason="The lower support hinge barrel is modeled as a solid sleeve around the top bracket pin.",
        )
        ctx.expect_overlap(
            central,
            lower,
            axes="x",
            elem_a="lower_hinge_pin",
            elem_b=barrel,
            min_overlap=0.10,
            name=f"lower_{barrel}_has_pin_engagement",
        )
        ctx.expect_within(
            central,
            lower,
            axes="yz",
            inner_elem="lower_hinge_pin",
            outer_elem=barrel,
            margin=0.002,
            name=f"lower_{barrel}_wraps_pin_cross_section",
        )

    with ctx.pose({wing_0_joint: 0.0, wing_1_joint: 0.0, lower_joint: 0.0}):
        ctx.expect_overlap(wing_0, central, axes="y", min_overlap=0.60, elem_a="outer_rod", elem_b="side_rod_pos")
        ctx.expect_overlap(wing_1, central, axes="y", min_overlap=0.60, elem_a="outer_rod", elem_b="side_rod_neg")
        rest_wing_0_outer = ctx.part_element_world_aabb(wing_0, elem="outer_rod")
        rest_wing_1_outer = ctx.part_element_world_aabb(wing_1, elem="outer_rod")
        rest_lower_bottom = ctx.part_element_world_aabb(lower, elem="bottom_rail")
        short_member = ctx.part_element_world_aabb(lower, elem="short_fold_member_0")
        top_bracket = ctx.part_element_world_aabb(central, elem="top_bracket_0")

    with ctx.pose({wing_0_joint: 1.20}):
        raised = ctx.part_element_world_aabb(wing_0, elem="outer_rod")
        ctx.check(
            "wing_0_folds_upward",
            rest_wing_0_outer is not None and raised is not None and raised[1][2] > rest_wing_0_outer[1][2] + 0.25,
            details=f"rest={rest_wing_0_outer}, raised={raised}",
        )

    with ctx.pose({wing_1_joint: 1.20}):
        raised = ctx.part_element_world_aabb(wing_1, elem="outer_rod")
        ctx.check(
            "wing_1_folds_upward",
            rest_wing_1_outer is not None and raised is not None and raised[1][2] > rest_wing_1_outer[1][2] + 0.25,
            details=f"rest={rest_wing_1_outer}, raised={raised}",
        )

    with ctx.pose({lower_joint: 1.35}):
        folded = ctx.part_element_world_aabb(lower, elem="bottom_rail")
        ctx.check(
            "lower_support_folds_toward_top",
            rest_lower_bottom is not None and folded is not None and folded[1][2] > rest_lower_bottom[1][2] + 0.25,
            details=f"rest={rest_lower_bottom}, folded={folded}",
        )

    ctx.check(
        "short_folding_member_near_top_bracket",
        short_member is not None
        and top_bracket is not None
        and (short_member[1][2] - short_member[0][2]) < 0.13
        and abs(short_member[1][2] - top_bracket[0][2]) < 0.04,
        details=f"short_member={short_member}, top_bracket={top_bracket}",
    )

    return ctx.report()


object_model = build_object_model()
