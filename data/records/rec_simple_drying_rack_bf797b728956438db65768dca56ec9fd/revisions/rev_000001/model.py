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


def _cylinder_x(part, name, *, radius, length, xyz, material):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_y(part, name, *, radius, length, xyz, material):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _sloped_yz_cylinder(part, name, *, radius, start, end, material):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # All sloped members here live in a Y/Z plane.  Rotate local +Z into the
    # desired line direction; cylinders are symmetric, so no yaw is needed.
    roll = math.atan2(-dy, dz)
    midpoint = (
        (start[0] + end[0]) / 2.0,
        (start[1] + end[1]) / 2.0,
        (start[2] + end[2]) / 2.0,
    )
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=midpoint, rpy=(roll, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    satin_metal = model.material("satin_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    rail_metal = model.material("pale_steel_rails", rgba=(0.88, 0.90, 0.88, 1.0))
    white_plastic = model.material("white_hinge_plastic", rgba=(0.94, 0.94, 0.90, 1.0))
    dark_feet = model.material("dark_rubber_feet", rgba=(0.04, 0.045, 0.045, 1.0))

    length = 1.10
    central_width = 0.42
    wing_depth = 0.36
    rack_height = 0.90
    rail_radius = 0.0125
    rod_radius = 0.0055
    bracket_radius = 0.010
    knuckle_radius = 0.017

    central = model.part("central_frame")

    # Obvious central modules: two long side rails plus the molded top brackets
    # that tie them together and carry the drying rods.
    central.visual(
        Cylinder(radius=rail_radius, length=length),
        origin=Origin(xyz=(0.0, central_width / 2.0, rack_height), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="left_rail",
    )
    central.visual(
        Cylinder(radius=rail_radius, length=length),
        origin=Origin(xyz=(0.0, -central_width / 2.0, rack_height), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="right_rail",
    )
    for x, name in ((-length / 2.0, "top_bracket_0"), (length / 2.0, "top_bracket_1")):
        _cylinder_y(
            central,
            name,
            radius=bracket_radius,
            length=central_width + 2.0 * rail_radius,
            xyz=(x, 0.0, rack_height),
            material=white_plastic,
        )
        central.visual(
            Box((0.050, 0.055, 0.030)),
            origin=Origin(xyz=(x, central_width / 2.0, rack_height - 0.003)),
            material=white_plastic,
            name=f"left_corner_cap_{0 if x < 0 else 1}",
        )
        central.visual(
            Box((0.050, 0.055, 0.030)),
            origin=Origin(xyz=(x, -central_width / 2.0, rack_height - 0.003)),
            material=white_plastic,
            name=f"right_corner_cap_{0 if x < 0 else 1}",
        )

    for index, y in enumerate((-0.145, -0.0725, 0.0, 0.0725, 0.145)):
        _cylinder_x(
            central,
            f"center_hanging_rail_{index}",
            radius=rod_radius,
            length=length,
            xyz=(0.0, y, rack_height + 0.001),
            material=rail_metal,
        )

    support_hinge_z = rack_height - 0.060
    central.visual(
        Cylinder(radius=0.014, length=1.00),
        origin=Origin(xyz=(0.0, 0.0, support_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="support_pin",
    )
    for index, x in enumerate((-0.48, 0.48)):
        central.visual(
            Box((0.026, 0.024, 0.052)),
            origin=Origin(xyz=(x, 0.0, support_hinge_z + 0.026)),
            material=white_plastic,
            name=f"support_yoke_{index}",
        )
        central.visual(
            Box((0.080, 0.026, 0.050)),
            origin=Origin(xyz=(x + (0.035 if x < 0 else -0.035), 0.0, support_hinge_z + 0.048)),
            material=white_plastic,
            name=f"support_bridge_{index}",
        )

    def add_wing(name: str, *, side: float):
        wing = model.part(name)
        outward = side
        inner_y = outward * 0.045
        outer_y = outward * wing_depth

        # The three hinge knuckles intentionally wrap around the central side
        # rail.  The actual drying frame is offset outward, leaving the brackets
        # visually readable rather than one fused rod.
        for index, x in enumerate((-0.34, 0.0, 0.34)):
            _cylinder_x(
                wing,
                f"hinge_knuckle_{index}",
                radius=knuckle_radius,
                length=0.155,
                xyz=(x, 0.0, 0.0),
                material=white_plastic,
            )
            wing.visual(
                Box((0.150, 0.031, 0.014)),
                origin=Origin(xyz=(x, outward * 0.030, 0.0)),
                material=white_plastic,
                name=f"hinge_web_{index}",
            )

        _cylinder_x(
            wing,
            "inner_rail",
            radius=0.011,
            length=length,
            xyz=(0.0, inner_y, 0.0),
            material=satin_metal,
        )
        _cylinder_x(
            wing,
            "outer_rail",
            radius=0.011,
            length=length,
            xyz=(0.0, outer_y, 0.0),
            material=satin_metal,
        )
        for x, rail_name in ((-length / 2.0, "end_bracket_0"), (length / 2.0, "end_bracket_1")):
            _cylinder_y(
                wing,
                rail_name,
                radius=0.0095,
                length=wing_depth - 0.045,
                xyz=(x, outward * (0.045 + wing_depth) / 2.0, 0.0),
                material=white_plastic,
            )

        for index, y_abs in enumerate((0.105, 0.165, 0.225, 0.285, 0.335)):
            _cylinder_x(
                wing,
                f"wing_hanging_rail_{index}",
                radius=rod_radius,
                length=length,
                xyz=(0.0, outward * y_abs, 0.0),
                material=rail_metal,
            )
        return wing

    wing_0 = add_wing("wing_0", side=1.0)
    wing_1 = add_wing("wing_1", side=-1.0)

    support = model.part("support_frame")
    for index, x in enumerate((-0.30, 0.30)):
        _cylinder_x(
            support,
            f"hinge_knuckle_{index}",
            radius=knuckle_radius,
            length=0.165,
            xyz=(x, 0.0, 0.0),
            material=white_plastic,
        )
        support.visual(
            Box((0.160, 0.018, 0.032)),
            origin=Origin(xyz=(x, 0.0, -0.032)),
            material=white_plastic,
            name=f"hinge_drop_{index}",
        )

    _cylinder_x(
        support,
        "top_rail",
        radius=0.010,
        length=0.98,
        xyz=(0.0, 0.0, -0.055),
        material=satin_metal,
    )
    _cylinder_x(
        support,
        "lower_rail",
        radius=0.012,
        length=0.98,
        xyz=(0.0, -0.30, -0.70),
        material=satin_metal,
    )
    for x, name in ((-0.49, "side_leg_0"), (0.49, "side_leg_1")):
        _sloped_yz_cylinder(
            support,
            name,
            radius=0.010,
            start=(x, 0.0, -0.055),
            end=(x, -0.30, -0.70),
            material=satin_metal,
        )
    for index, t in enumerate((0.26, 0.46, 0.66, 0.84)):
        y = -0.30 * t
        z = -0.055 - 0.645 * t
        _cylinder_x(
            support,
            f"lower_hanging_rail_{index}",
            radius=rod_radius,
            length=0.98,
            xyz=(0.0, y, z),
            material=rail_metal,
        )
    for x, name in ((-0.52, "foot_0"), (0.52, "foot_1")):
        support.visual(
            Box((0.070, 0.045, 0.028)),
            origin=Origin(xyz=(x, -0.30, -0.715)),
            material=dark_feet,
            name=name,
        )

    model.articulation(
        "central_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_0,
        origin=Origin(xyz=(0.0, central_width / 2.0, rack_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "central_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_1,
        origin=Origin(xyz=(0.0, -central_width / 2.0, rack_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "central_to_support",
        ArticulationType.REVOLUTE,
        parent=central,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, support_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-1.10, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    support = object_model.get_part("support_frame")
    wing_0_joint = object_model.get_articulation("central_to_wing_0")
    wing_1_joint = object_model.get_articulation("central_to_wing_1")
    support_joint = object_model.get_articulation("central_to_support")

    for index in range(3):
        knuckle = f"hinge_knuckle_{index}"
        ctx.allow_overlap(
            central,
            wing_0,
            elem_a="left_rail",
            elem_b=knuckle,
            reason="The side-wing hinge knuckle is intentionally captured around the central side rail.",
        )
        ctx.expect_within(
            central,
            wing_0,
            axes="yz",
            inner_elem="left_rail",
            outer_elem=knuckle,
            margin=0.002,
            name=f"wing 0 hinge knuckle {index} wraps the left rail",
        )
        ctx.expect_overlap(
            central,
            wing_0,
            axes="x",
            elem_a="left_rail",
            elem_b=knuckle,
            min_overlap=0.10,
            name=f"wing 0 hinge knuckle {index} has retained pin length",
        )

        ctx.allow_overlap(
            central,
            wing_1,
            elem_a="right_rail",
            elem_b=knuckle,
            reason="The side-wing hinge knuckle is intentionally captured around the central side rail.",
        )
        ctx.expect_within(
            central,
            wing_1,
            axes="yz",
            inner_elem="right_rail",
            outer_elem=knuckle,
            margin=0.002,
            name=f"wing 1 hinge knuckle {index} wraps the right rail",
        )
        ctx.expect_overlap(
            central,
            wing_1,
            axes="x",
            elem_a="right_rail",
            elem_b=knuckle,
            min_overlap=0.10,
            name=f"wing 1 hinge knuckle {index} has retained pin length",
        )

    for index in range(2):
        knuckle = f"hinge_knuckle_{index}"
        ctx.allow_overlap(
            central,
            support,
            elem_a="support_pin",
            elem_b=knuckle,
            reason="The lower frame hinge knuckle is intentionally captured around the support pin.",
        )
        ctx.expect_within(
            central,
            support,
            axes="yz",
            inner_elem="support_pin",
            outer_elem=knuckle,
            margin=0.003,
            name=f"support hinge knuckle {index} wraps the support pin",
        )
        ctx.expect_overlap(
            central,
            support,
            axes="x",
            elem_a="support_pin",
            elem_b=knuckle,
            min_overlap=0.10,
            name=f"support hinge knuckle {index} has retained pin length",
        )

    wing_0_rest = ctx.part_world_aabb(wing_0)
    wing_1_rest = ctx.part_world_aabb(wing_1)
    with ctx.pose({wing_0_joint: 1.20, wing_1_joint: 1.20}):
        wing_0_folded = ctx.part_world_aabb(wing_0)
        wing_1_folded = ctx.part_world_aabb(wing_1)
    ctx.check(
        "wing frames fold upward",
        wing_0_rest is not None
        and wing_1_rest is not None
        and wing_0_folded is not None
        and wing_1_folded is not None
        and wing_0_folded[1][2] > wing_0_rest[1][2] + 0.20
        and wing_1_folded[1][2] > wing_1_rest[1][2] + 0.20,
        details=f"rest={wing_0_rest},{wing_1_rest} folded={wing_0_folded},{wing_1_folded}",
    )

    support_rest = ctx.part_world_aabb(support)
    with ctx.pose({support_joint: -1.00}):
        support_folded = ctx.part_world_aabb(support)
    ctx.check(
        "lower support frame folds upward",
        support_rest is not None
        and support_folded is not None
        and support_folded[0][2] > support_rest[0][2] + 0.25,
        details=f"rest={support_rest} folded={support_folded}",
    )

    return ctx.report()


object_model = build_object_model()
