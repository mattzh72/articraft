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

LOWER_RAIL_CENTER_X = 0.212
LOWER_RAIL_WIDTH = 0.072
LOWER_RAIL_DEPTH = 0.028
LOWER_RAIL_LENGTH = 3.65
LOWER_RUNG_RADIUS = 0.017
LOWER_RUNG_Y = -0.005
LOWER_TOP_BRIDGE_WIDTH = 0.390

FLY_RAIL_CENTER_X = 0.184
FLY_RAIL_WIDTH = 0.058
FLY_RAIL_DEPTH = 0.024
FLY_RAIL_LENGTH = 3.10
FLY_RAIL_Y = 0.035
FLY_RUNG_RADIUS = 0.014
FLY_GUIDE_X = 0.214
FLY_GUIDE_Y = 0.022
FLY_GUIDE_DEPTH = 0.016
FLY_GUIDE_WIDTH = 0.036
FLY_GUIDE_HEIGHT = 0.140

FLY_REST_Z = 1.15
FLY_TRAVEL = 1.25
CADDY_HINGE_Y = 0.052
CADDY_HINGE_Z = 3.05
CADDY_OPEN_ANGLE = math.radians(85.0)


def _add_rung(
    part,
    *,
    z: float,
    y: float,
    length: float,
    radius: float,
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.74, 0.77, 0.80, 1.0))
    rung_aluminum = model.material("rung_aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    rubber = model.material("rubber", rgba=(0.17, 0.17, 0.18, 1.0))
    caddy_plastic = model.material("caddy_plastic", rgba=(0.92, 0.58, 0.14, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.28, 0.30, 0.33, 1.0))

    lower = model.part("lower_section")
    for rail_index, x_pos in enumerate((-LOWER_RAIL_CENTER_X, LOWER_RAIL_CENTER_X)):
        lower.visual(
            Box((LOWER_RAIL_WIDTH, LOWER_RAIL_DEPTH, LOWER_RAIL_LENGTH)),
            origin=Origin(xyz=(x_pos, 0.0, LOWER_RAIL_LENGTH * 0.5)),
            material=aluminum,
            name=f"lower_rail_{rail_index}",
        )

    lower_rung_length = 2.0 * (LOWER_RAIL_CENTER_X - (LOWER_RAIL_WIDTH * 0.5))
    for rung_index in range(12):
        _add_rung(
            lower,
            z=0.34 + rung_index * 0.265,
            y=LOWER_RUNG_Y,
            length=lower_rung_length,
            radius=LOWER_RUNG_RADIUS,
            material="rung_aluminum",
            name=f"lower_rung_{rung_index}",
        )

    for foot_index, x_pos in enumerate((-LOWER_RAIL_CENTER_X, LOWER_RAIL_CENTER_X)):
        lower.visual(
            Box((0.088, 0.050, 0.032)),
            origin=Origin(xyz=(x_pos, 0.006, 0.016)),
            material=rubber,
            name=f"foot_{foot_index}",
        )

    lower.visual(
        Box((LOWER_TOP_BRIDGE_WIDTH, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, -0.004, 3.62)),
        material=aluminum,
        name="top_bridge",
    )

    fly = model.part("fly_section")
    for rail_index, x_pos in enumerate((-FLY_RAIL_CENTER_X, FLY_RAIL_CENTER_X)):
        fly.visual(
            Box((FLY_RAIL_WIDTH, FLY_RAIL_DEPTH, FLY_RAIL_LENGTH)),
            origin=Origin(xyz=(x_pos, FLY_RAIL_Y, FLY_RAIL_LENGTH * 0.5)),
            material=aluminum,
            name=f"fly_rail_{rail_index}",
        )

    fly_rung_length = 2.0 * (FLY_RAIL_CENTER_X - (FLY_RAIL_WIDTH * 0.5))
    for rung_index in range(11):
        _add_rung(
            fly,
            z=0.30 + rung_index * 0.250,
            y=FLY_RAIL_Y,
            length=fly_rung_length,
            radius=FLY_RUNG_RADIUS,
            material="rung_aluminum",
            name=f"fly_rung_{rung_index}",
        )

    fly.visual(
        Box((fly_rung_length, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, 0.034, 3.055)),
        material=aluminum,
        name="upper_cap",
    )
    for mount_index, x_pos in enumerate((-0.112, 0.112)):
        fly.visual(
            Box((0.048, 0.024, 0.040)),
            origin=Origin(xyz=(x_pos, 0.039, 3.022)),
            material=dark_hardware,
            name=f"hinge_mount_{mount_index}",
        )
    for guide_row, z_pos in enumerate((0.62, 2.16)):
        for side_index, x_pos in enumerate((-FLY_GUIDE_X, FLY_GUIDE_X)):
            fly.visual(
                Box((FLY_GUIDE_WIDTH, FLY_GUIDE_DEPTH, FLY_GUIDE_HEIGHT)),
                origin=Origin(xyz=(x_pos, FLY_GUIDE_Y, z_pos)),
                material=dark_hardware,
                name=f"guide_shoe_{guide_row}_{side_index}",
            )

    caddy = model.part("tool_caddy")
    caddy.visual(
        Box((0.285, 0.010, 0.200)),
        origin=Origin(xyz=(0.0, 0.056, -0.115)),
        material=caddy_plastic,
        name="caddy_panel",
    )
    caddy.visual(
        Box((0.250, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.048, -0.009)),
        material=caddy_plastic,
        name="top_flange",
    )
    for hinge_index, x_pos in enumerate((-0.112, 0.112)):
        caddy.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(xyz=(x_pos, 0.041, -0.009), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_hardware,
            name=f"hinge_barrel_{hinge_index}",
        )
    for wall_index, x_pos in enumerate((-0.138, 0.138)):
        caddy.visual(
            Box((0.010, 0.050, 0.180)),
            origin=Origin(xyz=(x_pos, 0.084, -0.110)),
            material=caddy_plastic,
            name=f"side_wall_{wall_index}",
        )
    caddy.visual(
        Box((0.285, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.084, -0.210)),
        material=caddy_plastic,
        name="front_lip",
    )
    caddy.visual(
        Box((0.010, 0.045, 0.135)),
        origin=Origin(xyz=(0.0, 0.081, -0.138)),
        material=caddy_plastic,
        name="center_divider",
    )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, FLY_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=FLY_TRAVEL,
        ),
    )
    model.articulation(
        "caddy_hinge",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=caddy,
        origin=Origin(xyz=(0.0, CADDY_HINGE_Y - FLY_RAIL_Y, CADDY_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=CADDY_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_section")
    fly = object_model.get_part("fly_section")
    caddy = object_model.get_part("tool_caddy")
    fly_slide = object_model.get_articulation("fly_slide")
    caddy_hinge = object_model.get_articulation("caddy_hinge")

    ctx.expect_gap(
        fly,
        lower,
        axis="y",
        min_gap=0.006,
        max_gap=0.030,
        positive_elem="fly_rail_0",
        negative_elem="lower_rail_0",
        name="fly rails run just ahead of the lower rails",
    )
    ctx.expect_overlap(
        fly,
        lower,
        axes="z",
        min_overlap=2.40,
        elem_a="fly_rail_0",
        elem_b="lower_rail_0",
        name="collapsed fly section remains deeply engaged in the ladder",
    )
    ctx.expect_contact(
        caddy,
        fly,
        elem_a="hinge_barrel_0",
        elem_b="hinge_mount_0",
        name="closed caddy hangs from the hinge knuckle",
    )

    rest_fly_pos = ctx.part_world_position(fly)
    rest_caddy_aabb = ctx.part_world_aabb(caddy)

    with ctx.pose({fly_slide: FLY_TRAVEL}):
        ctx.expect_overlap(
            fly,
            lower,
            axes="z",
            min_overlap=1.20,
            elem_a="fly_rail_0",
            elem_b="lower_rail_0",
            name="extended fly section retains insertion in the lower rails",
        )
        extended_fly_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        rest_fly_pos is not None
        and extended_fly_pos is not None
        and extended_fly_pos[2] > rest_fly_pos[2] + 1.20,
        details=f"rest={rest_fly_pos}, extended={extended_fly_pos}",
    )

    with ctx.pose({caddy_hinge: CADDY_OPEN_ANGLE}):
        open_caddy_aabb = ctx.part_world_aabb(caddy)

    caddy_swings_out = (
        rest_caddy_aabb is not None
        and open_caddy_aabb is not None
        and open_caddy_aabb[1][1] > rest_caddy_aabb[1][1] + 0.10
        and open_caddy_aabb[1][2] > rest_caddy_aabb[1][2] + 0.035
    )
    ctx.check(
        "tool caddy folds up into a usable tray position",
        caddy_swings_out,
        details=f"rest={rest_caddy_aabb}, open={open_caddy_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
