from __future__ import annotations

import math

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


def _add_box_tube(
    part,
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
    origin_xyz,
    material,
    *,
    front_name: str,
    back_name: str,
    side_0_name: str,
    side_1_name: str,
):
    """Add four slightly interlocked rectangular walls for a hollow square tube."""
    x, y, z = origin_xyz
    wall_x = (outer_x - inner_x) * 0.5
    wall_y = (outer_y - inner_y) * 0.5
    corner_key = 0.002
    part.visual(
        Box((outer_x, wall_y, height)),
        origin=Origin(xyz=(x, y + inner_y * 0.5 + wall_y * 0.5, z)),
        material=material,
        name=front_name,
    )
    part.visual(
        Box((outer_x, wall_y, height)),
        origin=Origin(xyz=(x, y - inner_y * 0.5 - wall_y * 0.5, z)),
        material=material,
        name=back_name,
    )
    part.visual(
        Box((wall_x, inner_y + corner_key, height)),
        origin=Origin(xyz=(x + inner_x * 0.5 + wall_x * 0.5, y, z)),
        material=material,
        name=side_0_name,
    )
    part.visual(
        Box((wall_x, inner_y + corner_key, height)),
        origin=Origin(xyz=(x - inner_x * 0.5 - wall_x * 0.5, y, z)),
        material=material,
        name=side_1_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_telescoping_mast")

    painted_steel = Material("painted_steel", color=(0.12, 0.13, 0.14, 1.0))
    dark_steel = Material("dark_steel", color=(0.03, 0.035, 0.04, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.55, 0.58, 0.56, 1.0))
    slide_anodized = Material("slide_anodized", color=(0.34, 0.37, 0.38, 1.0))
    bearing_plastic = Material("bearing_plastic", color=(0.86, 0.84, 0.76, 1.0))
    cartridge_blue = Material("cartridge_blue", color=(0.08, 0.18, 0.32, 1.0))
    warning_red = Material("warning_red", color=(0.9, 0.08, 0.03, 1.0))

    side_support = model.part("side_support")
    side_support.visual(
        Box((0.430, 0.300, 0.050)),
        origin=Origin(xyz=(0.015, 0.0, 0.025)),
        material=dark_steel,
        name="floor_foot",
    )
    side_support.visual(
        Box((0.045, 0.400, 1.350)),
        origin=Origin(xyz=(-0.195, 0.0, 0.675)),
        material=painted_steel,
        name="side_wall_plate",
    )
    side_support.visual(
        Box((0.295, 0.205, 0.070)),
        origin=Origin(xyz=(-0.045, 0.0, 0.115)),
        material=painted_steel,
        name="lower_socket_arm",
    )
    _add_box_tube(
        side_support,
        0.130,
        0.130,
        0.100,
        0.100,
        0.780,
        (0.080, 0.0, 0.510),
        dark_steel,
        front_name="fixed_front_wall",
        back_name="fixed_back_wall",
        side_0_name="fixed_side_0",
        side_1_name="fixed_side_1",
    )
    _add_box_tube(
        side_support,
        0.176,
        0.176,
        0.128,
        0.128,
        0.060,
        (0.080, 0.0, 0.620),
        brushed_steel,
        front_name="clamp_front_wall",
        back_name="clamp_back_wall",
        side_0_name="clamp_side_0",
        side_1_name="clamp_side_1",
    )
    for y, visual_name in ((0.082, "upper_arm_0"), (-0.082, "upper_arm_1")):
        side_support.visual(
            Box((0.285, 0.055, 0.055)),
            origin=Origin(xyz=(-0.055, y, 0.620)),
            material=painted_steel,
            name=visual_name,
        )
    for y, z, visual_name in (
        (-0.135, 0.235, "bolt_0"),
        (0.135, 0.235, "bolt_1"),
        (-0.135, 1.095, "bolt_2"),
        (0.135, 1.095, "bolt_3"),
    ):
        side_support.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(-0.166, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=visual_name,
        )

    lower_member = model.part("lower_member")
    _add_box_tube(
        lower_member,
        0.084,
        0.084,
        0.056,
        0.056,
        0.750,
        (0.0, 0.0, -0.075),
        slide_anodized,
        front_name="lower_front_wall",
        back_name="lower_back_wall",
        side_0_name="lower_side_0",
        side_1_name="lower_side_1",
    )
    _add_box_tube(
        lower_member,
        0.110,
        0.110,
        0.082,
        0.082,
        0.055,
        (0.0, 0.0, 0.285),
        brushed_steel,
        front_name="collar_front_wall",
        back_name="collar_back_wall",
        side_0_name="collar_side_0",
        side_1_name="collar_side_1",
    )
    for z, suffix in ((-0.430, "low"), (-0.340, "high")):
        lower_member.visual(
            Box((0.010, 0.038, 0.050)),
            origin=Origin(xyz=(0.045, 0.0, z)),
            material=bearing_plastic,
            name=f"lower_guide_x_pos_{suffix}",
        )
        lower_member.visual(
            Box((0.010, 0.038, 0.050)),
            origin=Origin(xyz=(-0.045, 0.0, z)),
            material=bearing_plastic,
            name=f"lower_guide_x_neg_{suffix}",
        )
        lower_member.visual(
            Box((0.038, 0.010, 0.050)),
            origin=Origin(xyz=(0.0, 0.045, z)),
            material=bearing_plastic,
            name=f"lower_guide_y_pos_{suffix}",
        )
        lower_member.visual(
            Box((0.038, 0.010, 0.050)),
            origin=Origin(xyz=(0.0, -0.045, z)),
            material=bearing_plastic,
            name=f"lower_guide_y_neg_{suffix}",
        )

    upper_member = model.part("upper_member")
    upper_member.visual(
        Box((0.046, 0.046, 0.620)),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=brushed_steel,
        name="upper_mast",
    )
    for z, suffix in ((-0.400, "low"), (-0.300, "high")):
        upper_member.visual(
            Box((0.006, 0.026, 0.050)),
            origin=Origin(xyz=(0.025, 0.0, z)),
            material=bearing_plastic,
            name=f"upper_guide_x_pos_{suffix}",
        )
        upper_member.visual(
            Box((0.006, 0.026, 0.050)),
            origin=Origin(xyz=(-0.025, 0.0, z)),
            material=bearing_plastic,
            name=f"upper_guide_x_neg_{suffix}",
        )
        upper_member.visual(
            Box((0.026, 0.006, 0.050)),
            origin=Origin(xyz=(0.0, 0.025, z)),
            material=bearing_plastic,
            name=f"upper_guide_y_pos_{suffix}",
        )
        upper_member.visual(
            Box((0.026, 0.006, 0.050)),
            origin=Origin(xyz=(0.0, -0.025, z)),
            material=bearing_plastic,
            name=f"upper_guide_y_neg_{suffix}",
        )
    upper_member.visual(
        Cylinder(radius=0.038, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.2225)),
        material=dark_steel,
        name="bearing_boss",
    )

    rotary_cartridge = model.part("rotary_cartridge")
    rotary_cartridge.visual(
        Cylinder(radius=0.067, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="bearing_disk",
    )
    rotary_cartridge.visual(
        Cylinder(radius=0.052, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=cartridge_blue,
        name="cartridge_drum",
    )
    rotary_cartridge.visual(
        Box((0.160, 0.076, 0.062)),
        origin=Origin(xyz=(0.090, 0.0, 0.112)),
        material=cartridge_blue,
        name="cartridge_head",
    )
    rotary_cartridge.visual(
        Box((0.014, 0.080, 0.066)),
        origin=Origin(xyz=(0.177, 0.0, 0.112)),
        material=dark_steel,
        name="front_cap",
    )
    rotary_cartridge.visual(
        Box((0.055, 0.008, 0.020)),
        origin=Origin(xyz=(0.048, -0.040, 0.146)),
        material=warning_red,
        name="index_mark",
    )

    model.articulation(
        "side_to_lower",
        ArticulationType.PRISMATIC,
        parent=side_support,
        child=lower_member,
        origin=Origin(xyz=(0.080, 0.0, 0.880)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.22, lower=0.0, upper=0.320),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_member,
        child=upper_member,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.260),
    )
    model.articulation(
        "upper_to_cartridge",
        ArticulationType.REVOLUTE,
        parent=upper_member,
        child=rotary_cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_support = object_model.get_part("side_support")
    lower_member = object_model.get_part("lower_member")
    upper_member = object_model.get_part("upper_member")
    rotary_cartridge = object_model.get_part("rotary_cartridge")
    side_to_lower = object_model.get_articulation("side_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    upper_to_cartridge = object_model.get_articulation("upper_to_cartridge")

    ctx.check(
        "serial vertical prismatic slides",
        side_to_lower.articulation_type == ArticulationType.PRISMATIC
        and lower_to_upper.articulation_type == ArticulationType.PRISMATIC
        and side_to_lower.axis == (0.0, 0.0, 1.0)
        and lower_to_upper.axis == (0.0, 0.0, 1.0),
        details=f"joints={side_to_lower.articulation_type}, {lower_to_upper.articulation_type}",
    )
    ctx.check(
        "top cartridge vertical revolute",
        upper_to_cartridge.articulation_type == ArticulationType.REVOLUTE
        and upper_to_cartridge.axis == (0.0, 0.0, 1.0),
        details=f"type={upper_to_cartridge.articulation_type}, axis={upper_to_cartridge.axis}",
    )

    ctx.expect_gap(
        side_support,
        lower_member,
        axis="x",
        min_gap=0.006,
        max_gap=0.010,
        positive_elem="fixed_side_0",
        negative_elem="lower_side_0",
        name="lower tube clears fixed sleeve side wall",
    )
    ctx.expect_gap(
        side_support,
        lower_member,
        axis="y",
        min_gap=0.006,
        max_gap=0.010,
        positive_elem="fixed_front_wall",
        negative_elem="lower_front_wall",
        name="lower tube clears fixed sleeve front wall",
    )
    ctx.expect_overlap(
        lower_member,
        side_support,
        axes="z",
        elem_a="lower_front_wall",
        elem_b="fixed_front_wall",
        min_overlap=0.40,
        name="lower tube retains collapsed insertion",
    )
    ctx.expect_gap(
        lower_member,
        upper_member,
        axis="x",
        min_gap=0.004,
        max_gap=0.007,
        positive_elem="lower_side_0",
        negative_elem="upper_mast",
        name="upper mast clears lower tube side wall",
    )
    ctx.expect_gap(
        lower_member,
        upper_member,
        axis="y",
        min_gap=0.004,
        max_gap=0.007,
        positive_elem="lower_front_wall",
        negative_elem="upper_mast",
        name="upper mast clears lower tube front wall",
    )
    ctx.expect_overlap(
        upper_member,
        lower_member,
        axes="z",
        elem_a="upper_mast",
        elem_b="lower_front_wall",
        min_overlap=0.40,
        name="upper mast retains collapsed insertion",
    )
    ctx.expect_contact(
        rotary_cartridge,
        upper_member,
        elem_a="bearing_disk",
        elem_b="bearing_boss",
        contact_tol=0.001,
        name="rotary bearing sits on upper boss",
    )

    lower_rest = ctx.part_world_position(lower_member)
    upper_rest = ctx.part_world_position(upper_member)
    head_rest_aabb = ctx.part_element_world_aabb(rotary_cartridge, elem="cartridge_head")
    with ctx.pose({side_to_lower: 0.320, lower_to_upper: 0.260}):
        ctx.expect_overlap(
            lower_member,
            side_support,
            axes="z",
            elem_a="lower_front_wall",
            elem_b="fixed_front_wall",
            min_overlap=0.14,
            name="lower tube remains retained at full lift",
        )
        ctx.expect_overlap(
            upper_member,
            lower_member,
            axes="z",
            elem_a="upper_mast",
            elem_b="lower_front_wall",
            min_overlap=0.14,
            name="upper mast remains retained at full lift",
        )
        lower_lifted = ctx.part_world_position(lower_member)
        upper_lifted = ctx.part_world_position(upper_member)

    ctx.check(
        "slides lift upward",
        lower_rest is not None
        and upper_rest is not None
        and lower_lifted is not None
        and upper_lifted is not None
        and lower_lifted[2] > lower_rest[2] + 0.30
        and upper_lifted[2] > upper_rest[2] + 0.55,
        details=f"lower={lower_rest}->{lower_lifted}, upper={upper_rest}->{upper_lifted}",
    )

    with ctx.pose({upper_to_cartridge: math.pi / 2.0}):
        head_rotated_aabb = ctx.part_element_world_aabb(rotary_cartridge, elem="cartridge_head")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    head_rest = _aabb_center(head_rest_aabb)
    head_rotated = _aabb_center(head_rotated_aabb)
    ctx.check(
        "cartridge head yaws around vertical axis",
        head_rest is not None
        and head_rotated is not None
        and head_rest[0] > 0.14
        and head_rotated[1] > 0.06
        and abs(head_rotated[0] - 0.08) < 0.03,
        details=f"head_rest={head_rest}, head_rotated={head_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
