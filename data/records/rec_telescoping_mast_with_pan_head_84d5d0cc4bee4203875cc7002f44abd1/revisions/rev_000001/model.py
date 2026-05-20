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


def _square_tube(
    part,
    prefix: str,
    *,
    side: float,
    wall: float,
    length: float,
    z_center: float,
    material,
    front_name: str | None = None,
    back_name: str | None = None,
    side_name: str | None = None,
    opposite_name: str | None = None,
) -> None:
    """Build a connected square tube from four overlapping wall bars."""
    half = side / 2.0
    wall_half = wall / 2.0
    offset = half - wall_half

    part.visual(
        Box((side, wall, length)),
        origin=Origin(xyz=(0.0, offset, z_center)),
        material=material,
        name=front_name or f"{prefix}_front_wall",
    )
    part.visual(
        Box((side, wall, length)),
        origin=Origin(xyz=(0.0, -offset, z_center)),
        material=material,
        name=back_name or f"{prefix}_back_wall",
    )
    part.visual(
        Box((wall, side, length)),
        origin=Origin(xyz=(offset, 0.0, z_center)),
        material=material,
        name=side_name or f"{prefix}_side_wall",
    )
    part.visual(
        Box((wall, side, length)),
        origin=Origin(xyz=(-offset, 0.0, z_center)),
        material=material,
        name=opposite_name or f"{prefix}_opposite_wall",
    )


def _square_ring(part, prefix: str, *, outer: float, wall: float, height: float, z_center: float, material) -> None:
    """A square collar/cap ring with a clear central opening."""
    half = outer / 2.0
    wall_half = wall / 2.0
    offset = half - wall_half

    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, offset, z_center)),
        material=material,
        name=f"{prefix}_front",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, -offset, z_center)),
        material=material,
        name=f"{prefix}_back",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(offset, 0.0, z_center)),
        material=material,
        name=f"{prefix}_side",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(-offset, 0.0, z_center)),
        material=material,
        name=f"{prefix}_opposite",
    )


def _clamp_handle(part, prefix: str, *, outer_half: float, z_center: float, screw_length: float, material_screw, material_grip) -> None:
    """Static clamp screw and flip handle mounted on the positive-X collar side."""
    screw_center_x = outer_half + screw_length / 2.0
    knob_center_x = outer_half + screw_length + 0.014
    part.visual(
        Cylinder(radius=0.007, length=screw_length),
        origin=Origin(xyz=(screw_center_x, 0.0, z_center), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material_screw,
        name=f"{prefix}_screw",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.028),
        origin=Origin(xyz=(knob_center_x, 0.0, z_center), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material_grip,
        name=f"{prefix}_round_knob",
    )
    part.visual(
        Box((0.020, 0.105, 0.024)),
        origin=Origin(xyz=(knob_center_x + 0.018, 0.0, z_center + 0.006)),
        material=material_grip,
        name=f"{prefix}_flip_handle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_survey_mast")

    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    galvanized = model.material("galvanized_aluminum", rgba=(0.63, 0.66, 0.67, 1.0))
    bright_aluminum = model.material("brushed_inner_aluminum", rgba=(0.80, 0.82, 0.80, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
    acetal = model.material("white_acetal_guide_pads", rgba=(0.92, 0.90, 0.82, 1.0))
    brass = model.material("brass_clamp_screws", rgba=(0.88, 0.62, 0.24, 1.0))
    head_dark = model.material("matte_pan_head_plate", rgba=(0.03, 0.035, 0.04, 1.0))
    slot_black = model.material("black_recesses", rgba=(0.0, 0.0, 0.0, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.42, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="weighted_foot_plate",
    )
    base.visual(
        Box((0.27, 0.23, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=dark_steel,
        name="raised_socket_plinth",
    )
    for ix, x in enumerate((-0.235, 0.235)):
        for iy, y in enumerate((-0.145, 0.145)):
            base.visual(
                Box((0.115, 0.075, 0.018)),
                origin=Origin(xyz=(x, y, -0.006)),
                material=black_rubber,
                name=f"rubber_foot_{ix}_{iy}",
            )

    _square_tube(
        base,
        "sleeve",
        side=0.160,
        wall=0.018,
        length=0.590,
        z_center=0.385,
        material=dark_steel,
        front_name="sleeve_front_wall",
        back_name="sleeve_back_wall",
        side_name="sleeve_side_wall",
        opposite_name="sleeve_opposite_wall",
    )
    _square_ring(base, "base_top_collar", outer=0.210, wall=0.035, height=0.052, z_center=0.684, material=dark_steel)
    _square_ring(base, "base_lower_collar", outer=0.190, wall=0.028, height=0.045, z_center=0.125, material=dark_steel)
    _clamp_handle(
        base,
        "base_clamp",
        outer_half=0.105,
        z_center=0.684,
        screw_length=0.060,
        material_screw=brass,
        material_grip=black_rubber,
    )

    lower = model.part("lower_section")
    _square_tube(
        lower,
        "lower",
        side=0.105,
        wall=0.012,
        length=0.950,
        z_center=-0.025,
        material=galvanized,
        front_name="lower_front_wall",
        back_name="lower_back_wall",
        side_name="lower_side_wall",
        opposite_name="lower_opposite_wall",
    )
    _square_ring(lower, "lower_top_collar", outer=0.135, wall=0.024, height=0.160, z_center=0.430, material=galvanized)
    lower.visual(
        Box((0.125, 0.125, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.496)),
        material=galvanized,
        name="lower_bottom_stop_plate",
    )
    # Low-friction guide shoes touch the inside of the fixed sleeve, grounding the
    # telescoping section without making the aluminum tube scrape the steel walls.
    lower.visual(
        Box((0.045, 0.0095, 0.075)),
        origin=Origin(xyz=(0.0, 0.05725, -0.420)),
        material=acetal,
        name="lower_front_pad",
    )
    lower.visual(
        Box((0.045, 0.0095, 0.075)),
        origin=Origin(xyz=(0.0, -0.05725, -0.420)),
        material=acetal,
        name="lower_back_pad",
    )
    lower.visual(
        Box((0.0095, 0.045, 0.075)),
        origin=Origin(xyz=(0.05725, 0.0, -0.420)),
        material=acetal,
        name="lower_side_pad",
    )
    lower.visual(
        Box((0.0095, 0.045, 0.075)),
        origin=Origin(xyz=(-0.05725, 0.0, -0.420)),
        material=acetal,
        name="lower_opposite_pad",
    )
    _clamp_handle(
        lower,
        "lower_clamp",
        outer_half=0.0675,
        z_center=0.430,
        screw_length=0.052,
        material_screw=brass,
        material_grip=black_rubber,
    )

    upper = model.part("upper_section")
    _square_tube(
        upper,
        "upper",
        side=0.075,
        wall=0.010,
        length=0.850,
        z_center=-0.045,
        material=bright_aluminum,
        front_name="upper_front_wall",
        back_name="upper_back_wall",
        side_name="upper_side_wall",
        opposite_name="upper_opposite_wall",
    )
    upper.visual(
        Box((0.095, 0.095, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.371)),
        material=bright_aluminum,
        name="upper_cap_plate",
    )
    upper.visual(
        Box((0.060, 0.003, 0.070)),
        origin=Origin(xyz=(0.0, 0.0390, -0.400)),
        material=acetal,
        name="upper_front_pad",
    )
    upper.visual(
        Box((0.060, 0.003, 0.070)),
        origin=Origin(xyz=(0.0, -0.0390, -0.400)),
        material=acetal,
        name="upper_back_pad",
    )
    upper.visual(
        Box((0.003, 0.060, 0.070)),
        origin=Origin(xyz=(0.0390, 0.0, -0.400)),
        material=acetal,
        name="upper_side_pad",
    )
    upper.visual(
        Box((0.003, 0.060, 0.070)),
        origin=Origin(xyz=(-0.0390, 0.0, -0.400)),
        material=acetal,
        name="upper_opposite_pad",
    )

    head = model.part("pan_head")
    head.visual(
        Cylinder(radius=0.050, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=head_dark,
        name="turntable_bearing",
    )
    head.visual(
        Cylinder(radius=0.038, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=head_dark,
        name="pan_neck",
    )
    head.visual(
        Box((0.280, 0.220, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=head_dark,
        name="flat_mounting_plate",
    )
    head.visual(
        Box((0.165, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.055, 0.099)),
        material=slot_black,
        name="front_mount_slot",
    )
    head.visual(
        Box((0.165, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.055, 0.099)),
        material=slot_black,
        name="rear_mount_slot",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.22, lower=0.0, upper=0.350),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.20, lower=0.0, upper=0.320),
    )
    model.articulation(
        "upper_to_head",
        ArticulationType.CONTINUOUS,
        parent=upper,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_section")
    upper = object_model.get_part("upper_section")
    head = object_model.get_part("pan_head")
    lower_slide = object_model.get_articulation("base_to_lower")
    upper_slide = object_model.get_articulation("lower_to_upper")
    pan_axis = object_model.get_articulation("upper_to_head")

    # The visible guide shoes are deliberate sliding contacts; they keep the
    # nested square sections mechanically captured without broad interpenetration.
    ctx.expect_contact(
        lower,
        base,
        elem_a="lower_front_pad",
        elem_b="sleeve_front_wall",
        name="lower section front guide pad rides the base sleeve",
    )
    ctx.expect_contact(
        upper,
        lower,
        elem_a="upper_front_pad",
        elem_b="lower_front_wall",
        name="upper section front guide pad rides the lower tube",
    )
    ctx.expect_gap(
        base,
        lower,
        axis="y",
        positive_elem="sleeve_front_wall",
        negative_elem="lower_front_wall",
        min_gap=0.008,
        max_gap=0.012,
        name="lower tube clears the fixed sleeve front wall",
    )
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="lower_front_wall",
        negative_elem="upper_front_wall",
        min_gap=0.002,
        max_gap=0.005,
        name="upper tube clears the lower tube front wall",
    )
    ctx.expect_gap(
        head,
        upper,
        axis="z",
        positive_elem="turntable_bearing",
        negative_elem="upper_cap_plate",
        min_gap=0.0,
        max_gap=0.001,
        name="pan bearing is seated on the upper cap plate",
    )

    ctx.expect_overlap(
        lower,
        base,
        axes="z",
        elem_a="lower_front_wall",
        elem_b="sleeve_front_wall",
        min_overlap=0.45,
        name="retracted lower section remains deeply inserted in base sleeve",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="z",
        elem_a="upper_front_wall",
        elem_b="lower_front_wall",
        min_overlap=0.45,
        name="retracted upper section remains deeply inserted in lower section",
    )

    rest_lower_pos = ctx.part_world_position(lower)
    rest_upper_pos = ctx.part_world_position(upper)
    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({lower_slide: 0.350, upper_slide: 0.320, pan_axis: math.pi / 2.0}):
        extended_lower_pos = ctx.part_world_position(lower)
        extended_upper_pos = ctx.part_world_position(upper)
        extended_head_pos = ctx.part_world_position(head)
        ctx.expect_contact(
            lower,
            base,
            elem_a="lower_front_pad",
            elem_b="sleeve_front_wall",
            name="extended lower guide pad still rides inside base sleeve",
        )
        ctx.expect_contact(
            upper,
            lower,
            elem_a="upper_front_pad",
            elem_b="lower_front_wall",
            name="extended upper guide pad still rides inside lower tube",
        )
        ctx.expect_overlap(
            lower,
            base,
            axes="z",
            elem_a="lower_front_wall",
            elem_b="sleeve_front_wall",
            min_overlap=0.145,
            name="extended lower section retains insertion in base sleeve",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="z",
            elem_a="upper_front_wall",
            elem_b="lower_front_wall",
            min_overlap=0.170,
            name="extended upper section retains insertion in lower section",
        )
        ctx.expect_gap(
            head,
            upper,
            axis="z",
            positive_elem="turntable_bearing",
            negative_elem="upper_cap_plate",
            min_gap=0.0,
            max_gap=0.001,
            name="pan head stays seated while rotated",
        )

    ctx.check(
        "lower section extends upward on its prismatic joint",
        rest_lower_pos is not None and extended_lower_pos is not None and extended_lower_pos[2] > rest_lower_pos[2] + 0.34,
        details=f"rest={rest_lower_pos}, extended={extended_lower_pos}",
    )
    ctx.check(
        "upper section stacks additional extension above the lower section",
        rest_upper_pos is not None and extended_upper_pos is not None and extended_upper_pos[2] > rest_upper_pos[2] + 0.66,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )
    ctx.check(
        "pan head rises with the deployed mast",
        rest_head_pos is not None and extended_head_pos is not None and extended_head_pos[2] > rest_head_pos[2] + 0.66,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    return ctx.report()


object_model = build_object_model()
