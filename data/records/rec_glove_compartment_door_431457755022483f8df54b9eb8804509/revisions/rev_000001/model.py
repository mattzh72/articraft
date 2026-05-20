from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PANEL_WIDTH = 0.72
PANEL_HEIGHT = 0.36
PANEL_THICKNESS = 0.04
OPENING_WIDTH = 0.60
OPENING_HEIGHT = 0.24
OPENING_CENTER_Z = 0.12
BIN_DEPTH = 0.305
BIN_WALL = 0.015
HINGE_X = 0.017
HINGE_Z = 0.0
DOOR_WIDTH = 0.585
DOOR_HEIGHT = 0.225
DOOR_THICKNESS = 0.024
DOOR_CENTER_Z = 0.012 + DOOR_HEIGHT / 2.0
DOOR_FRONT_X = DOOR_THICKNESS / 2.0


def _dashboard_shell() -> cq.Workplane:
    """Continuous dashboard fascia with a through opening and a hollow bin."""
    fascia = cq.Workplane("XY").box(PANEL_THICKNESS, PANEL_WIDTH, PANEL_HEIGHT).translate(
        (-PANEL_THICKNESS / 2.0, 0.0, OPENING_CENTER_Z)
    )
    opening_cut = cq.Workplane("XY").box(
        PANEL_THICKNESS + 0.012, OPENING_WIDTH, OPENING_HEIGHT
    ).translate((-PANEL_THICKNESS / 2.0, 0.0, OPENING_CENTER_Z))
    frame = fascia.cut(opening_cut)

    bin_center_x = -PANEL_THICKNESS - BIN_DEPTH / 2.0 + 0.002
    bin_inner_width = OPENING_WIDTH - 2.0 * BIN_WALL
    bin_inner_height = OPENING_HEIGHT - 2.0 * BIN_WALL
    bin_length = BIN_DEPTH + 0.004
    lower_z = OPENING_CENTER_Z - OPENING_HEIGHT / 2.0
    upper_z = OPENING_CENTER_Z + OPENING_HEIGHT / 2.0

    bottom = cq.Workplane("XY").box(bin_length, bin_inner_width, BIN_WALL).translate(
        (bin_center_x, 0.0, lower_z - BIN_WALL / 2.0)
    )
    top = cq.Workplane("XY").box(bin_length, bin_inner_width, BIN_WALL).translate(
        (bin_center_x, 0.0, upper_z + BIN_WALL / 2.0)
    )
    side_y = bin_inner_width / 2.0 + BIN_WALL / 2.0
    side_height = bin_inner_height + 2.0 * BIN_WALL
    side_0 = cq.Workplane("XY").box(bin_length, BIN_WALL, side_height).translate(
        (bin_center_x, -side_y, OPENING_CENTER_Z)
    )
    side_1 = cq.Workplane("XY").box(bin_length, BIN_WALL, side_height).translate(
        (bin_center_x, side_y, OPENING_CENTER_Z)
    )
    back = cq.Workplane("XY").box(BIN_WALL, bin_inner_width + 2.0 * BIN_WALL, side_height).translate(
        (-PANEL_THICKNESS - BIN_DEPTH, 0.0, OPENING_CENTER_Z)
    )

    return frame.union(bottom).union(top).union(side_0).union(side_1).union(back)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dashboard_glove_compartment")

    dashboard_mat = model.material("soft_black_dashboard", rgba=(0.055, 0.058, 0.060, 1.0))
    bin_mat = model.material("dark_bin_felt", rgba=(0.012, 0.012, 0.013, 1.0))
    door_mat = model.material("charcoal_door_panel", rgba=(0.075, 0.079, 0.083, 1.0))
    rubber_mat = model.material("black_rubber_seam", rgba=(0.006, 0.006, 0.006, 1.0))
    button_mat = model.material("satin_latch_button", rgba=(0.020, 0.021, 0.022, 1.0))
    hinge_mat = model.material("dark_hinge_pin", rgba=(0.018, 0.018, 0.017, 1.0))

    dashboard = model.part("dashboard")
    dashboard.visual(
        mesh_from_cadquery(_dashboard_shell(), "dashboard_shell", tolerance=0.001),
        material=dashboard_mat,
        name="dashboard_shell",
    )
    dashboard.visual(
        Box((BIN_DEPTH - 0.035, OPENING_WIDTH - 0.065, 0.004)),
        origin=Origin(xyz=(-PANEL_THICKNESS - BIN_DEPTH / 2.0 + 0.020, 0.0, 0.002)),
        material=bin_mat,
        name="bin_felt_liner",
    )
    dashboard.visual(
        Cylinder(radius=0.0045, length=0.630),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="hinge_pin",
    )

    parent_knuckles = (
        (-0.285, 0.060),
        (0.0, 0.100),
        (0.285, 0.060),
    )
    for index, (y_pos, length) in enumerate(parent_knuckles):
        dashboard.visual(
            Cylinder(radius=0.009, length=length),
            origin=Origin(xyz=(HINGE_X, y_pos, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"fixed_hinge_barrel_{index}",
        )
        dashboard.visual(
            Box((0.028, length, 0.012)),
            origin=Origin(xyz=(0.003, y_pos, 0.000)),
            material=hinge_mat,
            name=f"fixed_hinge_leaf_{index}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_rounded_box((DOOR_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT), 0.004), "door_panel"),
        origin=Origin(xyz=(0.0, 0.0, DOOR_CENTER_Z)),
        material=door_mat,
        name="door_panel",
    )

    # A thin black perimeter break makes the panel read as a separate glove-box door.
    trim_x = DOOR_FRONT_X + 0.0018
    trim_y = DOOR_WIDTH / 2.0 - 0.020
    trim_z_top = DOOR_CENTER_Z + DOOR_HEIGHT / 2.0 - 0.017
    trim_z_bottom = DOOR_CENTER_Z - DOOR_HEIGHT / 2.0 + 0.017
    door.visual(
        Box((0.004, DOOR_WIDTH - 0.040, 0.005)),
        origin=Origin(xyz=(trim_x, 0.0, trim_z_top)),
        material=rubber_mat,
        name="upper_seam",
    )
    door.visual(
        Box((0.004, DOOR_WIDTH - 0.040, 0.005)),
        origin=Origin(xyz=(trim_x, 0.0, trim_z_bottom)),
        material=rubber_mat,
        name="lower_seam",
    )
    for side, y_pos in enumerate((-trim_y, trim_y)):
        door.visual(
            Box((0.004, 0.005, DOOR_HEIGHT - 0.035)),
            origin=Origin(xyz=(trim_x, y_pos, DOOR_CENTER_Z)),
            material=rubber_mat,
            name=f"side_seam_{side}",
        )

    for index, y_pos in enumerate((-0.160, 0.160)):
        door.visual(
            Cylinder(radius=0.009, length=0.130),
            origin=Origin(xyz=(0.0, y_pos, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"door_hinge_barrel_{index}",
        )
        door.visual(
            Box((0.018, 0.130, 0.022)),
            origin=Origin(xyz=(0.002, y_pos, 0.014)),
            material=hinge_mat,
            name=f"door_hinge_leaf_{index}",
        )

    # Four strips form a small latch bezel without filling the button opening.
    latch_z = 0.171
    bezel_x = DOOR_FRONT_X + 0.0017
    door.visual(
        Box((0.004, 0.100, 0.006)),
        origin=Origin(xyz=(bezel_x, 0.0, latch_z + 0.027)),
        material=rubber_mat,
        name="latch_bezel_top",
    )
    door.visual(
        Box((0.004, 0.100, 0.006)),
        origin=Origin(xyz=(bezel_x, 0.0, latch_z - 0.027)),
        material=rubber_mat,
        name="latch_bezel_bottom",
    )
    for side, y_pos in enumerate((-0.052, 0.052)):
        door.visual(
            Box((0.004, 0.006, 0.054)),
            origin=Origin(xyz=(bezel_x, y_pos, latch_z)),
            material=rubber_mat,
            name=f"latch_bezel_side_{side}",
        )

    model.articulation(
        "dashboard_to_door",
        ArticulationType.REVOLUTE,
        parent=dashboard,
        child=door,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.15),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_cadquery(_rounded_box((0.014, 0.070, 0.030), 0.003), "button_cap"),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=button_mat,
        name="button_cap",
    )
    latch_button.visual(
        Box((0.018, 0.034, 0.018)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material=button_mat,
        name="button_stem",
    )

    model.articulation(
        "door_to_latch",
        ArticulationType.PRISMATIC,
        parent=door,
        child=latch_button,
        origin=Origin(xyz=(DOOR_FRONT_X + 0.003, 0.0, latch_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.12, lower=0.0, upper=0.010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dashboard = object_model.get_part("dashboard")
    door = object_model.get_part("door")
    latch_button = object_model.get_part("latch_button")
    door_hinge = object_model.get_articulation("dashboard_to_door")
    latch_slide = object_model.get_articulation("door_to_latch")

    ctx.allow_overlap(
        door,
        latch_button,
        elem_a="door_panel",
        elem_b="button_stem",
        reason="The latch stem is intentionally seated inside the solid-proxy door pocket.",
    )
    for barrel_name in ("door_hinge_barrel_0", "door_hinge_barrel_1"):
        ctx.allow_overlap(
            dashboard,
            door,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            reason="The continuous hinge pin is intentionally captured through the rotating door barrel.",
        )
        ctx.expect_within(
            dashboard,
            door,
            axes="xz",
            margin=0.001,
            inner_elem="hinge_pin",
            outer_elem=barrel_name,
            name=f"hinge pin centered in {barrel_name}",
        )
        ctx.expect_overlap(
            dashboard,
            door,
            axes="y",
            min_overlap=0.10,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            name=f"hinge pin retained through {barrel_name}",
        )

    with ctx.pose({door_hinge: 0.0, latch_slide: 0.0}):
        ctx.expect_gap(
            door,
            dashboard,
            axis="x",
            min_gap=0.002,
            max_gap=0.030,
            positive_elem="door_panel",
            negative_elem="dashboard_shell",
            name="closed door sits just proud of the dashboard opening",
        )
        ctx.expect_overlap(
            door,
            dashboard,
            axes="yz",
            min_overlap=0.20,
            elem_a="door_panel",
            elem_b="dashboard_shell",
            name="door spans the glove-box opening",
        )
        ctx.expect_within(
            latch_button,
            door,
            axes="yz",
            margin=0.010,
            inner_elem="button_stem",
            outer_elem="door_panel",
            name="latch stem is centered in the door pocket",
        )
        ctx.expect_gap(
            latch_button,
            door,
            axis="x",
            max_penetration=0.014,
            positive_elem="button_stem",
            negative_elem="door_panel",
            name="latch stem has only local seated insertion",
        )

    closed_aabb = None
    open_aabb = None
    with ctx.pose({door_hinge: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.expect_gap(
            door,
            dashboard,
            axis="x",
            min_gap=0.020,
            positive_elem="door_panel",
            negative_elem="dashboard_shell",
            name="opened door swings outward from the dashboard",
        )

    ctx.check(
        "door rotates downward about the lower hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.12
        and open_aabb[1][2] < closed_aabb[1][2] - 0.06,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    rest_button = None
    pressed_button = None
    with ctx.pose({latch_slide: 0.0}):
        rest_button = ctx.part_world_position(latch_button)
    with ctx.pose({latch_slide: 0.010}):
        pressed_button = ctx.part_world_position(latch_button)
    ctx.check(
        "button plunges inward into the door",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[0] < rest_button[0] - 0.008,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
