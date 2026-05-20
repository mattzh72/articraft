from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


ALONG_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
ALONG_Y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _origin_xyz_rpy(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _add_channel_rail(
    part,
    *,
    prefix: str,
    x: float,
    y: float,
    z: float,
    length: float,
    width: float,
    depth: float,
    web: float,
    flange: float,
    material,
) -> None:
    """Build an aluminium stile as a light channel/H-section instead of a solid bar."""

    part.visual(
        Box((web, depth, length)),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=f"{prefix}_web",
    )
    part.visual(
        Box((width, flange, length)),
        origin=Origin(xyz=(x, y - depth / 2.0 + flange / 2.0, z)),
        material=material,
        name=f"{prefix}_rear_flange",
    )
    part.visual(
        Box((width, flange, length)),
        origin=Origin(xyz=(x, y + depth / 2.0 - flange / 2.0, z)),
        material=material,
        name=f"{prefix}_front_flange",
    )


def _add_rung(part, *, name: str, z: float, length: float, radius: float, y: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin_xyz_rpy(0.0, y, z, ALONG_X.rpy),
        material=material,
        name=name,
    )


def _add_guide_block(
    part,
    *,
    prefix: str,
    fly_x: float,
    base_x: float,
    fly_y: float,
    z: float,
    material,
) -> None:
    """A fixed U-channel guide block with clearance around one sliding fly rail."""

    rail_width = 0.055
    rail_depth = 0.040
    clearance = 0.0
    plate = 0.012
    guide_height = 0.140
    side_depth = 0.090

    side_offset = rail_width / 2.0 + clearance + plate / 2.0
    side_a = fly_x - side_offset
    side_b = fly_x + side_offset
    back_y = fly_y + rail_depth / 2.0 + clearance + plate / 2.0
    side_y = fly_y + 0.010
    back_span = rail_width + 2.0 * clearance + 2.0 * plate

    part.visual(
        Box((plate, side_depth, guide_height)),
        origin=Origin(xyz=(side_a, side_y, z)),
        material=material,
        name=f"{prefix}_outer_cheek" if fly_x < 0.0 else f"{prefix}_inner_cheek",
    )
    part.visual(
        Box((plate, side_depth, guide_height)),
        origin=Origin(xyz=(side_b, side_y, z)),
        material=material,
        name=f"{prefix}_inner_cheek" if fly_x < 0.0 else f"{prefix}_outer_cheek",
    )
    part.visual(
        Box((back_span, plate, guide_height)),
        origin=Origin(xyz=(fly_x, back_y, z)),
        material=material,
        name=f"{prefix}_back_plate",
    )

    # Mount strap reaches from the base stile front face to the outside cheek,
    # stopping just short of the sliding fly rail clearance envelope.
    if fly_x < 0.0:
        strap_min_x = base_x - 0.035
        strap_max_x = side_a + plate / 2.0
    else:
        strap_min_x = side_b - plate / 2.0
        strap_max_x = base_x + 0.035
    strap_x = (strap_min_x + strap_max_x) / 2.0
    strap_w = strap_max_x - strap_min_x
    strap_y_min = 0.018
    strap_y_max = back_y + plate / 2.0
    part.visual(
        Box((strap_w, strap_y_max - strap_y_min, guide_height)),
        origin=Origin(xyz=(strap_x, (strap_y_min + strap_y_max) / 2.0, z)),
        material=material,
        name=f"{prefix}_mount_strap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_hook_extension_ladder")

    aluminium = model.material("brushed_aluminium", rgba=(0.78, 0.80, 0.78, 1.0))
    darker_aluminium = model.material("darker_aluminium", rgba=(0.55, 0.58, 0.57, 1.0))
    guide_gray = model.material("anodized_guide_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    hook_orange = model.material("safety_orange_hook", rgba=(0.95, 0.38, 0.06, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base_length = 3.20
    base_rail_x = 0.30
    for side, x in (("left", -base_rail_x), ("right", base_rail_x)):
        _add_channel_rail(
            base,
            prefix=f"{side}_base_rail",
            x=x,
            y=0.0,
            z=base_length / 2.0,
            length=base_length,
            width=0.070,
            depth=0.052,
            web=0.014,
            flange=0.012,
            material=aluminium,
        )
        base.visual(
            Box((0.13, 0.090, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0225)),
            material=rubber,
            name=f"{side}_foot",
        )

    for i, z in enumerate((0.34, 0.69, 1.04, 1.39, 1.74, 2.09, 2.44, 2.79)):
        _add_rung(
            base,
            name=f"base_rung_{i}",
            z=z,
            length=0.59,
            radius=0.018,
            y=0.0,
            material=darker_aluminium,
        )

    fly_y = 0.080
    fly_rail_x = 0.24
    for level, z in (("lower", 2.50), ("upper", 3.15)):
        _add_guide_block(
            base,
            prefix=f"left_{level}_guide",
            fly_x=-fly_rail_x,
            base_x=-base_rail_x,
            fly_y=fly_y,
            z=z,
            material=guide_gray,
        )
        _add_guide_block(
            base,
            prefix=f"right_{level}_guide",
            fly_x=fly_rail_x,
            base_x=base_rail_x,
            fly_y=fly_y,
            z=z,
            material=guide_gray,
        )

    fly = model.part("fly_section")
    fly_length = 3.20
    for side, x in (("left", -fly_rail_x), ("right", fly_rail_x)):
        _add_channel_rail(
            fly,
            prefix=f"{side}_fly_rail",
            x=x,
            y=0.0,
            z=fly_length / 2.0,
            length=fly_length,
            width=0.055,
            depth=0.040,
            web=0.012,
            flange=0.010,
            material=aluminium,
        )

    for i, z in enumerate((0.35, 0.70, 1.05, 1.40, 1.75, 2.10, 2.45, 2.80)):
        _add_rung(
            fly,
            name=f"fly_rung_{i}",
            z=z,
            length=0.49,
            radius=0.016,
            y=0.0,
            material=darker_aluminium,
        )

    fly.visual(
        Cylinder(radius=0.017, length=0.56),
        origin=_origin_xyz_rpy(0.0, 0.030, 3.13, ALONG_X.rpy),
        material=darker_aluminium,
        name="top_pivot_tube",
    )
    fly.visual(
        Box((0.56, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, -0.018, 3.105)),
        material=aluminium,
        name="top_tie_plate",
    )

    hook = model.part("roof_hook")
    hook.visual(
        Cylinder(radius=0.012, length=0.82),
        origin=ALONG_X,
        material=hook_orange,
        name="pivot_shaft",
    )
    for side, x in (("left", -0.315), ("right", 0.315)):
        hook.visual(
            Cylinder(radius=0.014, length=0.56),
            origin=Origin(xyz=(x, 0.0, 0.28)),
            material=hook_orange,
            name=f"{side}_hook_arm",
        )
        hook.visual(
            Cylinder(radius=0.014, length=0.26),
            origin=_origin_xyz_rpy(x, 0.13, 0.56, ALONG_Y.rpy),
            material=hook_orange,
            name=f"{side}_downturn",
        )
        hook.visual(
            Sphere(radius=0.026),
            origin=Origin(xyz=(x, 0.265, 0.56)),
            material=rubber,
            name=f"{side}_rubber_tip",
        )
    for side, x in (("left", -0.385), ("right", 0.385)):
        hook.visual(
            Cylinder(radius=0.026, length=0.020),
            origin=_origin_xyz_rpy(x, 0.0, 0.0, ALONG_X.rpy),
            material=hook_orange,
            name=f"{side}_shaft_collar",
        )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, fly_y, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=1.25),
    )

    model.articulation(
        "fly_to_roof_hook",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=hook,
        origin=Origin(xyz=(0.0, 0.030, 3.13)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fly = object_model.get_part("fly_section")
    hook = object_model.get_part("roof_hook")
    slide = object_model.get_articulation("base_to_fly")
    hook_joint = object_model.get_articulation("fly_to_roof_hook")

    ctx.allow_overlap(
        fly,
        hook,
        elem_a="top_pivot_tube",
        elem_b="pivot_shaft",
        reason="The roof-hook pivot shaft is intentionally captured inside the fly-section top tube.",
    )
    ctx.expect_overlap(
        fly,
        hook,
        axes="x",
        elem_a="top_pivot_tube",
        elem_b="pivot_shaft",
        min_overlap=0.50,
        name="roof hook pivot shaft retained in tube",
    )

    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=0.10,
        name="fly rail passes through upper guide",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({slide: 1.25}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=0.10,
            name="extended fly remains captured by guide",
        )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 1.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    folded_aabb = ctx.part_world_aabb(hook)
    with ctx.pose({hook_joint: math.pi / 2.0}):
        deployed_aabb = ctx.part_world_aabb(hook)
    ctx.check(
        "roof hook deploys perpendicular to rails",
        folded_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[1][1] > folded_aabb[1][1] + 0.25
        and deployed_aabb[1][2] < folded_aabb[1][2] - 0.45,
        details=f"folded={folded_aabb}, deployed={deployed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
