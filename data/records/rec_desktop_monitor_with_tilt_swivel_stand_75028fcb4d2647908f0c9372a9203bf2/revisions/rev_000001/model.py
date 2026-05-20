from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel(width: float, height: float, depth: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, height, radius),
            depth,
        ),
        name,
    )


def _rounded_column(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, depth, radius),
            height,
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_monitor")

    shell_black = model.material("shell_black", rgba=(0.15, 0.16, 0.18, 1.0))
    matte_graphite = model.material("matte_graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.63, 0.65, 0.68, 1.0))
    screen_black = model.material("screen_black", rgba=(0.04, 0.05, 0.06, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.236, 0.178, 0.040),
                0.012,
            ),
            "pedestal_base",
        ),
        material=matte_graphite,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=hinge_dark,
        name="swivel_platter",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=hinge_dark,
        name="swivel_hub",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=hinge_dark,
        name="stand_collar",
    )
    stand.visual(
        Box((0.044, 0.006, 0.162)),
        origin=Origin(xyz=(0.0, -0.011, 0.081)),
        material=matte_graphite,
        name="front_wall",
    )
    for index, x_pos in enumerate((-0.019, 0.019)):
        stand.visual(
            Box((0.006, 0.028, 0.162)),
            origin=Origin(xyz=(x_pos, 0.0, 0.081)),
            material=matte_graphite,
            name=f"rail_{index}",
        )
    stand.visual(
        Box((0.020, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.009, 0.020)),
        material=matte_graphite,
        name="lower_spine",
    )
    stand.visual(
        Cylinder(radius=0.0036, length=0.032),
        origin=Origin(xyz=(-0.0145, 0.017, 0.052)),
        material=hinge_dark,
        name="door_knuckle_lower",
    )
    stand.visual(
        Cylinder(radius=0.0036, length=0.032),
        origin=Origin(xyz=(-0.0145, 0.017, 0.138)),
        material=hinge_dark,
        name="door_knuckle_upper",
    )

    mast = model.part("mast")
    mast.visual(
        _rounded_column(0.026, 0.012, 0.320, 0.0045, "monitor_mast"),
        origin=Origin(xyz=(0.0, 0.002, 0.000)),
        material=satin_silver,
        name="mast_column",
    )
    mast.visual(
        Box((0.052, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.133)),
        material=hinge_dark,
        name="head_block",
    )
    mast.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(
            xyz=(-0.023, 0.0, 0.145),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_dark,
        name="tilt_knuckle_0",
    )
    mast.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(
            xyz=(0.023, 0.0, 0.145),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_dark,
        name="tilt_knuckle_1",
    )

    display = model.part("display")
    display.visual(
        Cylinder(radius=0.0048, length=0.024),
        origin=Origin(
            xyz=(0.0, -0.012, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_dark,
        name="tilt_barrel",
    )
    display.visual(
        Box((0.034, 0.028, 0.064)),
        origin=Origin(xyz=(0.0, -0.018, 0.032)),
        material=hinge_dark,
        name="mount_arm",
    )
    display.visual(
        Box((0.082, 0.010, 0.084)),
        origin=Origin(xyz=(0.0, 0.012, 0.064)),
        material=hinge_dark,
        name="vesa_plate",
    )
    display.visual(
        _rounded_panel(0.538, 0.318, 0.050, 0.020, "display_shell"),
        origin=Origin(
            xyz=(0.0, -0.019, 0.177),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shell_black,
        name="shell",
    )
    display.visual(
        _rounded_panel(0.192, 0.126, 0.018, 0.012, "display_rear_pod"),
        origin=Origin(
            xyz=(0.0, 0.010, 0.152),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=matte_graphite,
        name="rear_pod",
    )
    display.visual(
        Box((0.492, 0.010, 0.276)),
        origin=Origin(xyz=(0.0, -0.039, 0.180)),
        material=screen_black,
        name="screen_glass",
    )
    display.visual(
        Box((0.036, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.039, 0.051)),
        material=hinge_dark,
        name="status_bar",
    )

    cable_door = model.part("cable_door")
    cable_door.visual(
        Cylinder(radius=0.0032, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=hinge_dark,
        name="door_barrel",
    )
    cable_door.visual(
        Box((0.010, 0.0035, 0.110)),
        origin=Origin(xyz=(0.005, 0.0, 0.095)),
        material=hinge_dark,
        name="door_leaf",
    )
    cable_door.visual(
        _rounded_panel(0.029, 0.110, 0.004, 0.004, "cable_door_panel"),
        origin=Origin(
            xyz=(0.015, 0.0, 0.095),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=matte_graphite,
        name="door_flap",
    )

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.5,
        ),
    )
    model.articulation(
        "stand_to_mast",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.14,
            lower=0.0,
            upper=0.100,
        ),
    )
    model.articulation(
        "mast_to_display",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.55,
        ),
    )
    model.articulation(
        "stand_to_cable_door",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=cable_door,
        origin=Origin(xyz=(-0.0145, 0.017, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.8,
            lower=0.0,
            upper=1.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stand = object_model.get_part("stand")
    mast = object_model.get_part("mast")
    display = object_model.get_part("display")
    cable_door = object_model.get_part("cable_door")

    swivel = object_model.get_articulation("base_to_stand")
    slide = object_model.get_articulation("stand_to_mast")
    tilt = object_model.get_articulation("mast_to_display")
    door_hinge = object_model.get_articulation("stand_to_cable_door")

    slide_limits = slide.motion_limits
    tilt_limits = tilt.motion_limits
    door_limits = door_hinge.motion_limits

    ctx.allow_overlap(
        stand,
        mast,
        elem_a="stand_collar",
        elem_b="mast_column",
        reason="The telescoping monitor mast is intentionally represented as sliding through a simplified solid collar that stands in for a hollow sleeve.",
    )

    ctx.expect_overlap(
        mast,
        stand,
        axes="z",
        elem_a="mast_column",
        elem_b="rail_0",
        min_overlap=0.12,
        name="collapsed mast remains deeply inserted",
    )

    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(mast)
        with ctx.pose({slide: slide_limits.upper}):
            ctx.expect_overlap(
                mast,
                stand,
                axes="z",
                elem_a="mast_column",
                elem_b="rail_0",
                min_overlap=0.058,
                name="raised mast retains insertion in the sleeve",
            )
            extended_pos = ctx.part_world_position(mast)
        ctx.check(
            "mast extends upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.09,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    rest_screen = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_glass"))
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_screen = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_glass"))
    ctx.check(
        "display swivels around the stand axis",
        rest_screen is not None
        and turned_screen is not None
        and abs(turned_screen[0]) > abs(rest_screen[0]) + 0.015
        and abs(turned_screen[1]) < abs(rest_screen[1]),
        details=f"rest={rest_screen}, turned={turned_screen}",
    )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.upper}):
            back_tilted = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_glass"))
        with ctx.pose({tilt: tilt_limits.lower}):
            forward_tilted = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_glass"))
        ctx.check(
            "screen tilts backward and forward",
            rest_screen is not None
            and back_tilted is not None
            and forward_tilted is not None
            and back_tilted[1] > rest_screen[1] + 0.04
            and forward_tilted[1] < rest_screen[1] - 0.02,
            details=f"rest={rest_screen}, back={back_tilted}, forward={forward_tilted}",
        )

    if door_limits is not None and door_limits.upper is not None:
        closed_door = _aabb_center(ctx.part_element_world_aabb(cable_door, elem="door_flap"))
        with ctx.pose({door_hinge: door_limits.upper}):
            open_door = _aabb_center(ctx.part_element_world_aabb(cable_door, elem="door_flap"))
        ctx.check(
            "cable door swings outward from the rear spine",
            closed_door is not None
            and open_door is not None
            and open_door[1] > closed_door[1] + 0.01
            and abs(open_door[0] - closed_door[0]) > 0.015,
            details=f"closed={closed_door}, open={open_door}",
        )

    return ctx.report()


object_model = build_object_model()
