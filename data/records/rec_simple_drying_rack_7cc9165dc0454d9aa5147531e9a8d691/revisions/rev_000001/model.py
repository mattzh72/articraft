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


def _rod_x(part, name: str, *, center, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _rod_y(part, name: str, *, center, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _rod_z(part, name: str, *, center, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    hinge_grey = model.material("hinge_grey", rgba=(0.42, 0.44, 0.44, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.022, 0.024, 1.0))

    rack_z = 0.90
    half_x = 0.55
    half_y = 0.23
    main_r = 0.0085
    rail_r = 0.0052

    central = model.part("central_frame")
    _rod_x(central, "side_rail_0", center=(0.0, -half_y, rack_z), length=2.0 * half_x, radius=main_r, material=steel)
    _rod_x(central, "side_rail_1", center=(0.0, half_y, rack_z), length=2.0 * half_x, radius=main_r, material=steel)
    _rod_y(central, "end_rail_0", center=(-half_x, 0.0, rack_z), length=2.0 * half_y, radius=main_r, material=steel)
    _rod_y(central, "end_rail_1", center=(half_x, 0.0, rack_z), length=2.0 * half_y, radius=main_r, material=steel)
    for i, y in enumerate((-0.155, -0.078, 0.0, 0.078, 0.155)):
        _rod_x(central, f"hanging_rail_{i}", center=(0.0, y, rack_z), length=2.0 * half_x, radius=rail_r, material=steel)

    # Two fixed hinge lugs under the center rack visually explain the lower folding frame.
    for i, x in enumerate((-0.43, 0.43)):
        central.visual(
            Box((0.070, 0.055, 0.032)),
            origin=Origin(xyz=(x, -0.178, rack_z - 0.012)),
            material=hinge_grey,
            name=f"lower_hinge_lug_{i}",
        )

    wing_span = 1.06
    wing_depth = 0.34
    # The wing hinge tube is a close sleeved fit around the central side rail.
    # A fractional millimeter of intentional interpenetration keeps the hinge
    # visually connected in the folded pose while remaining hidden in the tubes.
    wing_y_offset = 0.0157
    wing_r = 0.0075
    wing_rail_r = 0.0048

    for idx, sign in enumerate((1.0, -1.0)):
        wing = model.part(f"wing_{idx}")
        local_y = sign * wing_y_offset
        _rod_x(wing, "hinge_tube", center=(0.0, local_y, 0.0), length=wing_span, radius=wing_r, material=hinge_grey)
        _rod_x(wing, "outer_rail", center=(0.0, local_y, -wing_depth), length=wing_span, radius=wing_r, material=steel)
        _rod_z(wing, "end_stile_0", center=(-wing_span / 2.0, local_y, -wing_depth / 2.0), length=wing_depth, radius=wing_r, material=steel)
        _rod_z(wing, "end_stile_1", center=(wing_span / 2.0, local_y, -wing_depth / 2.0), length=wing_depth, radius=wing_r, material=steel)
        for rail_i, z in enumerate((-0.075, -0.145, -0.215, -0.285)):
            _rod_x(wing, f"hanging_rail_{rail_i}", center=(0.0, local_y, z), length=wing_span, radius=wing_rail_r, material=steel)
        for cap_i, x in enumerate((-wing_span / 2.0, wing_span / 2.0)):
            wing.visual(
                Box((0.028, 0.020, 0.020)),
                origin=Origin(xyz=(x, local_y, -wing_depth - 0.004)),
                material=black,
                name=f"rubber_tip_{cap_i}",
            )

        model.articulation(
            f"central_to_wing_{idx}",
            ArticulationType.REVOLUTE,
            parent=central,
            child=wing,
            origin=Origin(xyz=(0.0, sign * half_y, rack_z)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=7.0, velocity=2.5, lower=0.0, upper=1.48),
        )

    support = model.part("lower_support")
    support_width = 0.90
    support_depth = 0.38
    support_r = 0.007
    support_rail_r = 0.0046
    _rod_x(support, "hinge_tube", center=(0.0, 0.0, 0.0), length=support_width, radius=support_r, material=hinge_grey)
    _rod_x(support, "outer_rail", center=(0.0, support_depth, 0.0), length=support_width, radius=support_r, material=steel)
    _rod_y(support, "side_stile_0", center=(-support_width / 2.0, support_depth / 2.0, 0.0), length=support_depth, radius=support_r, material=steel)
    _rod_y(support, "side_stile_1", center=(support_width / 2.0, support_depth / 2.0, 0.0), length=support_depth, radius=support_r, material=steel)
    for i, y in enumerate((0.080, 0.160, 0.240, 0.320)):
        _rod_x(support, f"hanging_rail_{i}", center=(0.0, y, 0.0), length=support_width, radius=support_rail_r, material=steel)
    for i, x in enumerate((-support_width / 2.0, support_width / 2.0)):
        support.visual(
            Box((0.034, 0.026, 0.020)),
            origin=Origin(xyz=(x, support_depth + 0.004, -0.004)),
            material=black,
            name=f"foot_pad_{i}",
        )

    model.articulation(
        "central_to_lower_support",
        ArticulationType.REVOLUTE,
        parent=central,
        child=support,
        origin=Origin(xyz=(0.0, -0.178, rack_z - 0.035)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    support = object_model.get_part("lower_support")
    wing_0_joint = object_model.get_articulation("central_to_wing_0")
    wing_1_joint = object_model.get_articulation("central_to_wing_1")
    support_joint = object_model.get_articulation("central_to_lower_support")

    ctx.allow_overlap(
        central,
        wing_0,
        elem_a="side_rail_1",
        elem_b="hinge_tube",
        reason="The wing hinge tube is modeled as a captured sleeve around the central side rail.",
    )
    ctx.allow_overlap(
        central,
        wing_1,
        elem_a="side_rail_0",
        elem_b="hinge_tube",
        reason="The wing hinge tube is modeled as a captured sleeve around the central side rail.",
    )

    # In the folded rest state the moving frames sit close to, but not embedded in,
    # the fixed central rack.
    ctx.expect_gap(
        wing_0,
        central,
        axis="y",
        positive_elem="hinge_tube",
        negative_elem="side_rail_1",
        max_penetration=0.001,
        max_gap=0.003,
        name="folded upper wing stays close",
    )
    ctx.expect_gap(
        central,
        wing_1,
        axis="y",
        positive_elem="side_rail_0",
        negative_elem="hinge_tube",
        max_penetration=0.001,
        max_gap=0.003,
        name="folded lower wing stays close",
    )
    ctx.expect_gap(
        central,
        support,
        axis="z",
        min_gap=0.0,
        max_gap=0.040,
        name="folded lower support tucks below frame",
    )

    rest_wing_0 = ctx.part_world_aabb(wing_0)
    rest_wing_1 = ctx.part_world_aabb(wing_1)
    rest_support = ctx.part_world_aabb(support)
    with ctx.pose({wing_0_joint: 1.48, wing_1_joint: 1.48, support_joint: 1.15}):
        open_wing_0 = ctx.part_world_aabb(wing_0)
        open_wing_1 = ctx.part_world_aabb(wing_1)
        open_support = ctx.part_world_aabb(support)

    ctx.check(
        "wing_0 folds outward",
        rest_wing_0 is not None and open_wing_0 is not None and open_wing_0[1][1] > rest_wing_0[1][1] + 0.25,
        details=f"rest={rest_wing_0}, open={open_wing_0}",
    )
    ctx.check(
        "wing_1 folds outward",
        rest_wing_1 is not None and open_wing_1 is not None and open_wing_1[0][1] < rest_wing_1[0][1] - 0.25,
        details=f"rest={rest_wing_1}, open={open_wing_1}",
    )
    ctx.check(
        "lower support folds downward",
        rest_support is not None and open_support is not None and open_support[0][2] < rest_support[0][2] - 0.25,
        details=f"rest={rest_support}, open={open_support}",
    )

    return ctx.report()


object_model = build_object_model()
