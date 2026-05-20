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


WIDTH = 0.76
DEPTH = 0.62
HEIGHT = 1.78
FRONT_Y = -DEPTH / 2.0
TRIM_Y = FRONT_Y - 0.017
HINGE_Y = FRONT_Y - 0.055

DOOR_WIDTH = 0.66
DOOR_HEIGHT = 0.58
DOOR_THICKNESS = 0.040
DOOR_BOTTOM_CLEARANCE = 0.018
LOWER_HINGE_Z = 0.430
UPPER_HINGE_Z = 1.100
DRAWER_CENTER_Z = 0.225


def _add_cavity(housing, *, name: str, z_mid: float) -> None:
    """Dark, connected oven cavity surfaces behind a front opening."""

    cavity_mat = "cavity_black"
    inner_w = 0.585
    inner_d = 0.520
    inner_h = 0.500
    y_mid = -0.020

    housing.visual(
        Box((inner_w, 0.030, inner_h - 0.070)),
        origin=Origin(xyz=(0.0, 0.255, z_mid)),
        material=cavity_mat,
        name=f"{name}_cavity_back",
    )
    housing.visual(
        Box((0.020, inner_d, inner_h)),
        origin=Origin(xyz=(-inner_w / 2.0, y_mid, z_mid)),
        material=cavity_mat,
        name=f"{name}_cavity_side_0",
    )
    housing.visual(
        Box((0.020, inner_d, inner_h)),
        origin=Origin(xyz=(inner_w / 2.0, y_mid, z_mid)),
        material=cavity_mat,
        name=f"{name}_cavity_side_1",
    )
    housing.visual(
        Box((inner_w, inner_d, 0.020)),
        origin=Origin(xyz=(0.0, y_mid, z_mid - inner_h / 2.0)),
        material=cavity_mat,
        name=f"{name}_cavity_floor",
    )
    housing.visual(
        Box((inner_w, inner_d, 0.020)),
        origin=Origin(xyz=(0.0, y_mid, z_mid + inner_h / 2.0)),
        material=cavity_mat,
        name=f"{name}_cavity_ceiling",
    )

    # Two shiny rack rails span between the side walls so they are visibly
    # supported rather than floating rods.
    for rail_i, rail_z in enumerate((z_mid - 0.115, z_mid + 0.065)):
        housing.visual(
            Cylinder(radius=0.006, length=inner_w + 0.012),
            origin=Origin(xyz=(0.0, -0.145, rail_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_metal",
            name=f"{name}_rack_rail_{rail_i}",
        )


def _add_hinge_knuckles(housing, *, name: str, z: float) -> None:
    """Fixed side knuckles for a segmented bottom oven hinge."""

    for i, x in enumerate((-0.255, 0.255)):
        bracket_x = x + (0.025 if x > 0.0 else -0.025)
        housing.visual(
            Box((0.100, 0.025, 0.018)),
            origin=Origin(xyz=(bracket_x, HINGE_Y - 0.0015, z - 0.009)),
            material="brushed_metal",
            name=f"{name}_hinge_bracket_{i}",
        )
        housing.visual(
            Cylinder(radius=0.014, length=0.100),
            origin=Origin(xyz=(x, HINGE_Y - 0.018, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_steel",
            name=f"{name}_hinge_knuckle_{i}",
        )


def _add_door_visuals(door, *, prefix: str) -> None:
    """Metal framed drop-down oven door with glass pane and bar handle."""

    frame_mat = "brushed_metal"
    glass_mat = "smoked_glass"
    gasket_mat = "rubber_black"
    handle_mat = "dark_steel"

    frame_y = -0.018
    left_x = -DOOR_WIDTH / 2.0 + 0.026
    right_x = DOOR_WIDTH / 2.0 - 0.026
    vertical_z = DOOR_BOTTOM_CLEARANCE + DOOR_HEIGHT / 2.0

    door.visual(
        Box((0.052, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(left_x, frame_y, vertical_z)),
        material=frame_mat,
        name=f"{prefix}_side_0",
    )
    door.visual(
        Box((0.052, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(right_x, frame_y, vertical_z)),
        material=frame_mat,
        name=f"{prefix}_side_1",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, 0.060)),
        origin=Origin(xyz=(0.0, frame_y, DOOR_BOTTOM_CLEARANCE + 0.030)),
        material=frame_mat,
        name=f"{prefix}_bottom_rail",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, 0.055)),
        origin=Origin(xyz=(0.0, frame_y, DOOR_BOTTOM_CLEARANCE + DOOR_HEIGHT - 0.0275)),
        material=frame_mat,
        name=f"{prefix}_top_rail",
    )

    door.visual(
        Box((DOOR_WIDTH - 0.105, 0.014, DOOR_HEIGHT - 0.165)),
        origin=Origin(xyz=(0.0, -0.043, DOOR_BOTTOM_CLEARANCE + 0.300)),
        material=glass_mat,
        name=f"{prefix}_glass",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.075, 0.010, DOOR_HEIGHT - 0.120)),
        origin=Origin(xyz=(0.0, -0.036, DOOR_BOTTOM_CLEARANCE + 0.300)),
        material=gasket_mat,
        name=f"{prefix}_glass_gasket",
    )

    handle_z = DOOR_BOTTOM_CLEARANCE + DOOR_HEIGHT - 0.105
    door.visual(
        Cylinder(radius=0.017, length=0.510),
        origin=Origin(xyz=(0.0, -0.118, handle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_mat,
        name=f"{prefix}_handle_bar",
    )
    for i, x in enumerate((-0.215, 0.215)):
        door.visual(
            Cylinder(radius=0.010, length=0.082),
            origin=Origin(xyz=(x, -0.078, handle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=handle_mat,
            name=f"{prefix}_handle_post_{i}",
        )

    # The central rotating barrel is attached to the lower rail; fixed side
    # knuckles live on the housing and occupy separate x-spans.
    door.visual(
        Cylinder(radius=DOOR_BOTTOM_CLEARANCE, length=0.410),
        origin=Origin(xyz=(0.0, frame_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_mat,
        name=f"{prefix}_hinge_barrel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_wall_oven")

    model.material("brushed_metal", rgba=(0.70, 0.70, 0.68, 1.0))
    model.material("dark_steel", rgba=(0.08, 0.08, 0.085, 1.0))
    model.material("cavity_black", rgba=(0.010, 0.010, 0.012, 1.0))
    model.material("rubber_black", rgba=(0.0, 0.0, 0.0, 1.0))
    model.material("smoked_glass", rgba=(0.03, 0.06, 0.075, 0.46))
    model.material("display_glass", rgba=(0.0, 0.015, 0.030, 1.0))

    housing = model.part("housing")

    # Tall outer carcass, built from connected walls and front rails so the two
    # oven cavities and lower drawer bay read as actual openings rather than a
    # solid block.
    housing.visual(
        Box((0.055, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + 0.0275, 0.0, HEIGHT / 2.0)),
        material="brushed_metal",
        name="side_wall_0",
    )
    housing.visual(
        Box((0.055, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - 0.0275, 0.0, HEIGHT / 2.0)),
        material="brushed_metal",
        name="side_wall_1",
    )
    housing.visual(
        Box((WIDTH, DEPTH, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - 0.030)),
        material="brushed_metal",
        name="top_panel",
    )
    housing.visual(
        Box((WIDTH, DEPTH, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material="brushed_metal",
        name="bottom_plinth",
    )
    housing.visual(
        Box((WIDTH, 0.045, HEIGHT)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - 0.0225, HEIGHT / 2.0)),
        material="brushed_metal",
        name="rear_panel",
    )
    for rail_name, rail_z, rail_h in (
        ("drawer_sill", 0.090, 0.020),
        ("lower_sill_rail", 0.405, 0.050),
        ("middle_sill_rail", 1.075, 0.050),
        ("control_rail", 1.685, 0.100),
    ):
        housing.visual(
            Box((WIDTH, 0.055, rail_h)),
            origin=Origin(xyz=(0.0, TRIM_Y, rail_z)),
            material="brushed_metal",
            name=rail_name,
        )

    # A simple dark control display in the top band; it is not a moving control.
    housing.visual(
        Box((0.260, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, TRIM_Y - 0.030, 1.705)),
        material="display_glass",
        name="clock_display",
    )

    _add_cavity(housing, name="lower", z_mid=0.735)
    _add_cavity(housing, name="upper", z_mid=1.405)
    _add_hinge_knuckles(housing, name="lower", z=LOWER_HINGE_Z)
    _add_hinge_knuckles(housing, name="upper", z=UPPER_HINGE_Z)

    # Stationary drawer guide rails mounted to the side walls of the base bay.
    for i, x in enumerate((-0.315, 0.315)):
        housing.visual(
            Box((0.030, 0.365, 0.035)),
            origin=Origin(xyz=(x, -0.120, DRAWER_CENTER_Z)),
            material="dark_steel",
            name=f"drawer_guide_{i}",
        )

    lower_door = model.part("lower_door")
    _add_door_visuals(lower_door, prefix="lower")
    upper_door = model.part("upper_door")
    _add_door_visuals(upper_door, prefix="upper")

    drawer = model.part("storage_drawer")
    drawer.visual(
        Box((0.655, 0.040, 0.240)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material="brushed_metal",
        name="drawer_front",
    )
    drawer.visual(
        Box((0.600, 0.340, 0.140)),
        origin=Origin(xyz=(0.0, 0.170, 0.020)),
        material="dark_steel",
        name="drawer_tray",
    )
    drawer.visual(
        Cylinder(radius=0.014, length=0.430),
        origin=Origin(xyz=(0.0, -0.105, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="drawer_handle_bar",
    )
    for i, x in enumerate((-0.175, 0.175)):
        drawer.visual(
            Cylinder(radius=0.008, length=0.070),
            origin=Origin(xyz=(x, -0.072, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="dark_steel",
            name=f"drawer_handle_post_{i}",
        )

    model.articulation(
        "lower_door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lower_door,
        origin=Origin(xyz=(0.0, HINGE_Y, LOWER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=1.62),
    )
    model.articulation(
        "upper_door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=upper_door,
        origin=Origin(xyz=(0.0, HINGE_Y, UPPER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=1.62),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(0.0, HINGE_Y, DRAWER_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=0.320),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lower_door = object_model.get_part("lower_door")
    upper_door = object_model.get_part("upper_door")
    drawer = object_model.get_part("storage_drawer")
    lower_hinge = object_model.get_articulation("lower_door_hinge")
    upper_hinge = object_model.get_articulation("upper_door_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.expect_overlap(
        lower_door,
        housing,
        axes="x",
        min_overlap=0.60,
        name="lower oven door spans the frame width",
    )
    ctx.expect_overlap(
        upper_door,
        housing,
        axes="x",
        min_overlap=0.60,
        name="upper oven door spans the frame width",
    )
    ctx.expect_origin_gap(
        upper_door,
        lower_door,
        axis="z",
        min_gap=0.64,
        max_gap=0.70,
        name="oven doors are vertically stacked",
    )
    ctx.expect_origin_gap(
        lower_door,
        drawer,
        axis="z",
        min_gap=0.18,
        max_gap=0.24,
        name="storage drawer sits below the lower oven",
    )

    upper_closed = ctx.part_world_aabb(upper_door)
    with ctx.pose({upper_hinge: 1.20}):
        upper_open = ctx.part_world_aabb(upper_door)
    ctx.check(
        "upper door drops forward on bottom hinge",
        upper_closed is not None
        and upper_open is not None
        and upper_open[0][1] < upper_closed[0][1] - 0.35
        and upper_open[1][2] < upper_closed[1][2] - 0.25,
        details=f"closed={upper_closed}, open={upper_open}",
    )

    lower_closed = ctx.part_world_aabb(lower_door)
    with ctx.pose({lower_hinge: 1.20}):
        lower_open = ctx.part_world_aabb(lower_door)
    ctx.check(
        "lower door drops forward on bottom hinge",
        lower_closed is not None
        and lower_open is not None
        and lower_open[0][1] < lower_closed[0][1] - 0.35
        and lower_open[1][2] < lower_closed[1][2] - 0.25,
        details=f"closed={lower_closed}, open={lower_open}",
    )

    drawer_closed = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_slide: 0.30}):
        drawer_open = ctx.part_world_aabb(drawer)
    ctx.check(
        "storage drawer slides out from base",
        drawer_closed is not None
        and drawer_open is not None
        and drawer_open[0][1] < drawer_closed[0][1] - 0.25,
        details=f"closed={drawer_closed}, open={drawer_open}",
    )

    return ctx.report()


object_model = build_object_model()
