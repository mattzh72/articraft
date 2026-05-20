from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _rod_origin(center: tuple[float, float, float], axis: str) -> Origin:
    """URDF cylinders are local-Z aligned; rotate them onto a world/local axis."""
    if axis == "x":
        return Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0))
    if axis == "y":
        return Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0))
    if axis == "z":
        return Origin(xyz=center)
    raise ValueError(axis)


def _add_rod(part, name: str, center, axis: str, length: float, radius: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_rod_origin(center, axis),
        material=material,
        name=name,
    )


def _add_flat_link(part, name: str, x: float, dy: float, dz: float, material):
    """A slim flat strap lying in the local YZ plane."""
    length = math.hypot(dy, dz)
    angle = math.atan2(dz, dy)
    part.visual(
        Box((0.012, length, 0.006)),
        origin=Origin(xyz=(x, dy / 2.0, dz / 2.0), rpy=(angle, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_drying_rack")

    coated_steel = model.material("white_powder_coated_steel", color=(0.92, 0.94, 0.92, 1.0))
    rail_steel = model.material("slightly_warm_white_rails", color=(0.96, 0.97, 0.94, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", color=(0.55, 0.58, 0.60, 1.0))
    rubber = model.material("soft_grey_rubber", color=(0.10, 0.11, 0.12, 1.0))
    stop_plastic = model.material("pale_safety_stops", color=(0.78, 0.82, 0.84, 1.0))

    # The root is a compact tabletop center rack.  It is intentionally shallow
    # so the two wing frames provide most of the drying area when deployed.
    center = model.part("center_frame")
    top_z = 0.40
    half_len = 0.29
    half_width = 0.13

    _add_rod(center, "side_rail_0", (0.0, half_width, top_z), "x", 0.58, 0.008, coated_steel)
    _add_rod(center, "side_rail_1", (0.0, -half_width, top_z), "x", 0.58, 0.008, coated_steel)
    _add_rod(center, "end_rail_0", (half_len, 0.0, top_z), "y", 0.26, 0.008, coated_steel)
    _add_rod(center, "end_rail_1", (-half_len, 0.0, top_z), "y", 0.26, 0.008, coated_steel)

    for i, y in enumerate((-0.078, -0.026, 0.026, 0.078)):
        _add_rod(center, f"center_hanging_rail_{i}", (0.0, y, top_z + 0.004), "x", 0.58, 0.0045, rail_steel)

    # Low integrated feet keep it usable on a desk without turning it into a
    # full-size floor rack.
    for ix, x in enumerate((-0.235, 0.235)):
        for iy, y in enumerate((-half_width, half_width)):
            _add_rod(center, f"short_leg_{ix}_{iy}", (x, y, 0.225), "z", 0.35, 0.0065, coated_steel)
            center.visual(
                Box((0.080, 0.034, 0.018)),
                origin=Origin(xyz=(x, y, 0.041)),
                material=rubber,
                name=f"rubber_foot_{ix}_{iy}",
            )

    # Continuous hinge reference rods and small stop pads mark the wing hinge
    # lines while leaving clearance for the wing knuckles.
    for side, y in enumerate((0.142, -0.142)):
        _add_rod(center, f"hinge_pin_{side}", (0.0, y, top_z), "x", 0.54, 0.005, hinge_metal)
        for j, x in enumerate((-0.215, 0.215)):
            center.visual(
                Box((0.050, 0.018, 0.014)),
                origin=Origin(xyz=(x, y, top_z - 0.014)),
                material=stop_plastic,
                name=f"wing_stop_{side}_{j}",
            )

    for side, y in enumerate((0.088, -0.088)):
        _add_rod(center, f"support_pivot_pin_{side}", (0.0, y, top_z - 0.080), "x", 0.42, 0.0045, hinge_metal)
        for j, x in enumerate((-0.235, 0.235)):
            center.visual(
                Box((0.020, 0.056, 0.026)),
                origin=Origin(xyz=(math.copysign(0.205, x), (y + math.copysign(half_width, y)) / 2.0, top_z - 0.080)),
                material=hinge_metal,
                name=f"support_pivot_web_{side}_{j}",
            )
            _add_rod(
                center,
                f"support_drop_bracket_{side}_{j}",
                (math.copysign(0.205, x), math.copysign(half_width, y), top_z - 0.040),
                "z",
                0.080,
                0.004,
                hinge_metal,
            )

    # Wing frames: both child frames are located exactly on their hinge lines.
    # At q=0 they deploy horizontally; positive motion raises them toward a
    # near-vertical stow position.
    for idx, sign in enumerate((1.0, -1.0)):
        wing = model.part(f"wing_{idx}")
        dir_y = sign
        y_inner = 0.030 * dir_y
        y_outer = 0.292 * dir_y
        y_mid = 0.161 * dir_y
        y_knuckle = 0.0

        _add_rod(wing, "inner_side_rail", (0.0, y_inner, 0.0), "x", 0.50, 0.006, coated_steel)
        _add_rod(wing, "outer_side_rail", (0.0, y_outer, 0.0), "x", 0.50, 0.006, coated_steel)
        _add_rod(wing, "edge_rail_0", (-0.25, y_mid, 0.0), "y", 0.262, 0.006, coated_steel)
        _add_rod(wing, "edge_rail_1", (0.25, y_mid, 0.0), "y", 0.262, 0.006, coated_steel)

        for j, y_abs in enumerate((0.082, 0.134, 0.186, 0.238)):
            _add_rod(
                wing,
                f"hanging_rail_{j}",
                (0.0, y_abs * dir_y, 0.004),
                "x",
                0.50,
                0.0042,
                rail_steel,
            )

        # Alternating knuckles sit just outside the root hinge pin with a small
        # visible air gap; they make the pivot line legible without requiring an
        # interpenetrating hinge proxy.
        for j, x in enumerate((-0.125, 0.125)):
            _add_rod(wing, f"wing_knuckle_{j}", (x, y_knuckle, 0.0), "x", 0.080, 0.005, hinge_metal)
            wing.visual(
                Box((0.060, 0.030, 0.006)),
                origin=Origin(xyz=(x, 0.010 * dir_y, -0.004)),
                material=hinge_metal,
                name=f"hinge_leaf_{j}",
            )

        model.articulation(
            f"center_to_wing_{idx}",
            ArticulationType.REVOLUTE,
            parent=center,
            child=wing,
            origin=Origin(xyz=(0.0, 0.145 * dir_y, top_z)),
            axis=(dir_y, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.35),
            motion_properties=MotionProperties(damping=0.18, friction=0.04),
        )

    # Hinged support links swing down as the wings stow.  They are separate
    # articulated parts, not decorative fixed struts, so the rack reads as a
    # functional foldable tabletop appliance.
    for idx, sign in enumerate((1.0, -1.0)):
        link = model.part(f"support_link_{idx}")
        dir_y = sign
        _add_rod(link, "pivot_sleeve", (0.0, 0.0, 0.0), "x", 0.54, 0.005, hinge_metal)
        _add_flat_link(link, "strap_0", -0.255, 0.155 * dir_y, 0.074, hinge_metal)
        _add_flat_link(link, "strap_1", 0.255, 0.155 * dir_y, 0.074, hinge_metal)
        _add_rod(link, "tip_saddle", (0.0, 0.155 * dir_y, 0.074), "x", 0.54, 0.004, stop_plastic)

        model.articulation(
            f"center_to_support_link_{idx}",
            ArticulationType.REVOLUTE,
            parent=center,
            child=link,
            origin=Origin(xyz=(0.0, 0.088 * dir_y, top_z - 0.080)),
            axis=(dir_y, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=-1.0, upper=0.0),
            motion_properties=MotionProperties(damping=0.12, friction=0.06),
            mimic=Mimic(joint=f"center_to_wing_{idx}", multiplier=-0.70, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    center = object_model.get_part("center_frame")
    link_0 = object_model.get_part("support_link_0")
    link_1 = object_model.get_part("support_link_1")
    hinge_0 = object_model.get_articulation("center_to_wing_0")
    hinge_1 = object_model.get_articulation("center_to_wing_1")

    ctx.allow_overlap(
        center,
        wing_0,
        elem_a="hinge_pin_0",
        elem_b="wing_knuckle_0",
        reason="The wing knuckle is intentionally captured around the hinge pin for a real folding pivot.",
    )
    ctx.allow_overlap(
        center,
        wing_0,
        elem_a="hinge_pin_0",
        elem_b="wing_knuckle_1",
        reason="The second wing knuckle shares the same captured hinge pin.",
    )
    ctx.allow_overlap(
        center,
        wing_1,
        elem_a="hinge_pin_1",
        elem_b="wing_knuckle_0",
        reason="The wing knuckle is intentionally captured around the hinge pin for a real folding pivot.",
    )
    ctx.allow_overlap(
        center,
        wing_1,
        elem_a="hinge_pin_1",
        elem_b="wing_knuckle_1",
        reason="The second wing knuckle shares the same captured hinge pin.",
    )
    ctx.allow_overlap(
        center,
        link_0,
        elem_a="support_pivot_pin_0",
        elem_b="pivot_sleeve",
        reason="The support link sleeve rotates around a small captured pivot pin.",
    )
    ctx.allow_overlap(
        center,
        link_0,
        elem_a="support_pivot_web_0_0",
        elem_b="pivot_sleeve",
        reason="The compact yoke web is simplified as a close capture around the support sleeve.",
    )
    ctx.allow_overlap(
        center,
        link_0,
        elem_a="support_pivot_web_0_1",
        elem_b="pivot_sleeve",
        reason="The second compact yoke web shares the same close support-sleeve capture.",
    )
    ctx.allow_overlap(
        center,
        link_1,
        elem_a="support_pivot_pin_1",
        elem_b="pivot_sleeve",
        reason="The support link sleeve rotates around a small captured pivot pin.",
    )
    ctx.allow_overlap(
        center,
        link_1,
        elem_a="support_pivot_web_1_0",
        elem_b="pivot_sleeve",
        reason="The compact yoke web is simplified as a close capture around the support sleeve.",
    )
    ctx.allow_overlap(
        center,
        link_1,
        elem_a="support_pivot_web_1_1",
        elem_b="pivot_sleeve",
        reason="The second compact yoke web shares the same close support-sleeve capture.",
    )

    ctx.expect_gap(
        wing_0,
        center,
        axis="y",
        min_gap=0.002,
        positive_elem="inner_side_rail",
        negative_elem="hinge_pin_0",
        name="deployed wing 0 clears center hinge pin",
    )
    ctx.expect_overlap(
        center,
        wing_0,
        axes="x",
        elem_a="hinge_pin_0",
        elem_b="wing_knuckle_0",
        min_overlap=0.070,
        name="wing 0 knuckle remains captured on hinge pin",
    )
    ctx.expect_gap(
        center,
        wing_1,
        axis="y",
        min_gap=0.002,
        positive_elem="hinge_pin_1",
        negative_elem="inner_side_rail",
        name="deployed wing 1 clears center hinge pin",
    )
    ctx.expect_overlap(
        center,
        wing_1,
        axes="x",
        elem_a="hinge_pin_1",
        elem_b="wing_knuckle_0",
        min_overlap=0.070,
        name="wing 1 knuckle remains captured on hinge pin",
    )
    ctx.expect_overlap(
        center,
        link_0,
        axes="x",
        elem_a="support_pivot_pin_0",
        elem_b="pivot_sleeve",
        min_overlap=0.36,
        name="support link 0 remains captured on pivot pin",
    )
    ctx.expect_overlap(
        center,
        link_0,
        axes="x",
        elem_a="support_pivot_web_0_0",
        elem_b="pivot_sleeve",
        min_overlap=0.015,
        name="support link 0 yoke web captures sleeve",
    )
    ctx.expect_overlap(
        center,
        link_1,
        axes="x",
        elem_a="support_pivot_pin_1",
        elem_b="pivot_sleeve",
        min_overlap=0.36,
        name="support link 1 remains captured on pivot pin",
    )
    ctx.expect_overlap(
        center,
        link_1,
        axes="x",
        elem_a="support_pivot_web_1_0",
        elem_b="pivot_sleeve",
        min_overlap=0.015,
        name="support link 1 yoke web captures sleeve",
    )
    ctx.expect_overlap(
        wing_0,
        link_0,
        axes="x",
        elem_a="hanging_rail_2",
        elem_b="tip_saddle",
        min_overlap=0.30,
        name="support link 0 spans under drying rails",
    )
    ctx.expect_overlap(
        wing_1,
        link_1,
        axes="x",
        elem_a="hanging_rail_2",
        elem_b="tip_saddle",
        min_overlap=0.30,
        name="support link 1 spans under drying rails",
    )

    deployed_aabb_0 = ctx.part_world_aabb(wing_0)
    deployed_aabb_1 = ctx.part_world_aabb(wing_1)
    deployed_width = None
    if deployed_aabb_0 is not None and deployed_aabb_1 is not None:
        deployed_width = deployed_aabb_0[1][1] - deployed_aabb_1[0][1]

    with ctx.pose({hinge_0: 1.35, hinge_1: 1.35}):
        stowed_aabb_0 = ctx.part_world_aabb(wing_0)
        stowed_aabb_1 = ctx.part_world_aabb(wing_1)
        if stowed_aabb_0 is not None and deployed_aabb_0 is not None:
            ctx.check(
                "wing 0 stows upward on hinge line",
                stowed_aabb_0[1][2] > deployed_aabb_0[1][2] + 0.20,
                details=f"deployed={deployed_aabb_0}, stowed={stowed_aabb_0}",
            )
        if stowed_aabb_1 is not None and deployed_aabb_1 is not None:
            ctx.check(
                "wing 1 stows upward on hinge line",
                stowed_aabb_1[1][2] > deployed_aabb_1[1][2] + 0.20,
                details=f"deployed={deployed_aabb_1}, stowed={stowed_aabb_1}",
            )
        if deployed_width is not None and stowed_aabb_0 is not None and stowed_aabb_1 is not None:
            stowed_width = stowed_aabb_0[1][1] - stowed_aabb_1[0][1]
            ctx.check(
                "folded wings reduce rack footprint",
                stowed_width < deployed_width - 0.30,
                details=f"deployed_width={deployed_width}, stowed_width={stowed_width}",
            )

    return ctx.report()


object_model = build_object_model()
