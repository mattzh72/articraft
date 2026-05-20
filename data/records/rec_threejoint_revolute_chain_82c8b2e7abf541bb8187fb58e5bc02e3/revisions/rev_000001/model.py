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


HINGE_RADIUS = 0.035
CENTER_BARREL_LENGTH = 0.056
SIDE_BARREL_LENGTH = 0.024
SIDE_BARREL_Y = 0.040
CAP_LENGTH = 0.008
LINK_0_LENGTH = 0.38
LINK_1_LENGTH = 0.30
END_LINK_LENGTH = 0.18
HINGE_AXIS = (0.0, -1.0, 0.0)


def _cylinder_y(part, radius, length, xyz, material, name):
    """Add a cylinder whose axis is parallel to the model Y axis."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _side_barrels(part, x, z, index, metal, cap_material):
    for suffix, y in (("pos", SIDE_BARREL_Y), ("neg", -SIDE_BARREL_Y)):
        _cylinder_y(
            part,
            HINGE_RADIUS,
            SIDE_BARREL_LENGTH,
            (x, y, z),
            metal,
            f"hinge_{index}_side_{suffix}",
        )
        cap_y = y + math.copysign((SIDE_BARREL_LENGTH + CAP_LENGTH) / 2.0 - 0.001, y)
        _cylinder_y(
            part,
            HINGE_RADIUS * 0.78,
            CAP_LENGTH,
            (x, cap_y, z),
            cap_material,
            f"hinge_{index}_cap_{suffix}",
        )


def _center_barrel(part, index, metal):
    _cylinder_y(
        part,
        HINGE_RADIUS,
        CENTER_BARREL_LENGTH,
        (0.0, 0.0, 0.0),
        metal,
        f"hinge_{index}_center",
    )


def _add_link_with_distal_fork(part, length, proximal_index, distal_index, body, metal, cap):
    """Planar link with a central proximal knuckle and an offset fork at the distal end."""
    _center_barrel(part, proximal_index, metal)

    # The rectangular beam sits below the pin axis; compact riser blocks carry the
    # beam up to the hinge knuckles, making the offset hinge-block construction visible.
    _box(part, (0.095, 0.052, 0.058), (0.042, 0.0, -0.050), body, "proximal_block")
    rail_start = 0.064
    rail_end = length - 0.086
    _box(
        part,
        (rail_end - rail_start, 0.046, 0.034),
        ((rail_start + rail_end) / 2.0, 0.0, -0.070),
        body,
        "lower_rail",
    )
    _box(part, (0.060, 0.112, 0.036), (length - 0.095, 0.0, -0.070), body, "fork_bridge")

    for suffix, y in (("pos", SIDE_BARREL_Y), ("neg", -SIDE_BARREL_Y)):
        _box(
            part,
            (0.080, SIDE_BARREL_LENGTH, 0.076),
            (length - 0.035, y, -0.034),
            body,
            f"fork_cheek_{suffix}",
        )

    _side_barrels(part, length, 0.0, distal_index, metal, cap)


def _add_end_link(part, length, proximal_index, body, metal):
    _center_barrel(part, proximal_index, metal)
    _box(part, (0.095, 0.052, 0.058), (0.042, 0.0, -0.050), body, "proximal_block")
    _box(part, (length - 0.060, 0.044, 0.034), ((length + 0.060) / 2.0, 0.0, -0.070), body, "short_rail")
    _box(part, (0.050, 0.070, 0.040), (length, 0.0, -0.070), body, "tip_pad")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_serial_hinge_chain")

    dark_steel = Material("dark_burnished_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    pin_steel = Material("polished_pin_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    base_paint = Material("matte_graphite_base", rgba=(0.06, 0.07, 0.08, 1.0))
    link_blue = Material("anodized_blue_link", rgba=(0.10, 0.28, 0.72, 1.0))
    link_orange = Material("anodized_orange_link", rgba=(0.90, 0.38, 0.08, 1.0))
    link_green = Material("anodized_green_end", rgba=(0.15, 0.62, 0.34, 1.0))

    base = model.part("base")
    _box(base, (0.30, 0.22, 0.030), (-0.105, 0.0, -0.225), base_paint, "floor_plate")
    _box(base, (0.060, 0.080, 0.185), (-0.070, 0.0, -0.120), base_paint, "pedestal")
    _box(base, (0.050, 0.116, 0.062), (-0.075, 0.0, -0.020), base_paint, "offset_head")
    for suffix, y in (("pos", SIDE_BARREL_Y), ("neg", -SIDE_BARREL_Y)):
        _box(
            base,
            (0.078, SIDE_BARREL_LENGTH, 0.074),
            (-0.035, y, -0.032),
            base_paint,
            f"base_cheek_{suffix}",
        )
    _side_barrels(base, 0.0, 0.0, 0, dark_steel, pin_steel)

    link_0 = model.part("link_0")
    _add_link_with_distal_fork(link_0, LINK_0_LENGTH, 0, 1, link_blue, dark_steel, pin_steel)

    link_1 = model.part("link_1")
    _add_link_with_distal_fork(link_1, LINK_1_LENGTH, 1, 2, link_orange, dark_steel, pin_steel)

    end_link = model.part("end_link")
    _add_end_link(end_link, END_LINK_LENGTH, 2, link_green, dark_steel)

    limits = MotionLimits(effort=18.0, velocity=2.5, lower=-0.95, upper=1.35)
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(),
        axis=HINGE_AXIS,
        motion_limits=limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_0_LENGTH, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=limits,
    )
    model.articulation(
        "link_1_to_end_link",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_link,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_link = object_model.get_part("end_link")
    j0 = object_model.get_articulation("base_to_link_0")
    j1 = object_model.get_articulation("link_0_to_link_1")
    j2 = object_model.get_articulation("link_1_to_end_link")

    joints = (j0, j1, j2)
    ctx.check(
        "three revolute hinge joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "all hinge axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == HINGE_AXIS for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_origin_gap(
        link_1,
        link_0,
        axis="x",
        min_gap=LINK_0_LENGTH - 0.002,
        max_gap=LINK_0_LENGTH + 0.002,
        name="first intermediate link reaches second hinge",
    )
    ctx.expect_origin_gap(
        end_link,
        link_1,
        axis="x",
        min_gap=LINK_1_LENGTH - 0.002,
        max_gap=LINK_1_LENGTH + 0.002,
        name="second intermediate link reaches end hinge",
    )

    # The fork-side barrels and center barrel at each hinge are interleaved at
    # face contact, giving a continuous support path without occupying the same volume.
    hinge_pairs = (
        (base, link_0, "hinge_0_side_pos", "hinge_0_side_neg", "hinge_0_center"),
        (link_0, link_1, "hinge_1_side_pos", "hinge_1_side_neg", "hinge_1_center"),
        (link_1, end_link, "hinge_2_side_pos", "hinge_2_side_neg", "hinge_2_center"),
    )
    for parent, child, pos_side, neg_side, center in hinge_pairs:
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            positive_elem=pos_side,
            negative_elem=center,
            min_gap=0.0,
            max_gap=0.001,
            name=f"{center} positive-side fork clearance",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            positive_elem=center,
            negative_elem=neg_side,
            min_gap=0.0,
            max_gap=0.001,
            name=f"{center} negative-side fork clearance",
        )

    rest_link_1 = ctx.part_world_position(link_1)
    with ctx.pose({j0: 0.65}):
        raised_link_1 = ctx.part_world_position(link_1)
    ctx.check(
        "first joint swings chain in the hinge plane",
        rest_link_1 is not None
        and raised_link_1 is not None
        and abs(raised_link_1[1] - rest_link_1[1]) < 0.001
        and raised_link_1[2] > rest_link_1[2] + 0.18,
        details=f"rest={rest_link_1}, raised={raised_link_1}",
    )

    rest_end = ctx.part_world_position(end_link)
    with ctx.pose({j1: 0.65}):
        raised_end = ctx.part_world_position(end_link)
    ctx.check(
        "second joint swings downstream links in the same plane",
        rest_end is not None
        and raised_end is not None
        and abs(raised_end[1] - rest_end[1]) < 0.001
        and raised_end[2] > rest_end[2] + 0.14,
        details=f"rest={rest_end}, raised={raised_end}",
    )

    rest_tip = ctx.part_element_world_aabb(end_link, elem="tip_pad")
    with ctx.pose({j2: 0.70}):
        raised_tip = ctx.part_element_world_aabb(end_link, elem="tip_pad")

    def _center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.check(
        "third joint swings the short end link",
        _center_z(rest_tip) is not None
        and _center_z(raised_tip) is not None
        and _center_z(raised_tip) > _center_z(rest_tip) + 0.09,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
