from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DESK_WIDTH = 1.20
DESK_DEPTH = 0.60
DESK_HEIGHT = 0.75
TOP_THICKNESS = 0.03

PEDESTAL_WIDTH = 0.42
PEDESTAL_INNER_X = -0.189
PEDESTAL_CENTER_X = -0.39
RIGHT_PANEL_X = 0.585
FRAME_FRONT_Y = -DESK_DEPTH / 2.0

DRAWER_SPECS = (
    {
        "name": "drawer_0",
        "joint": "desk_to_drawer_0",
        "center_z": 0.640,
        "front_height": 0.118,
        "box_height": 0.086,
        "runner_outer": "runner_0_outer",
    },
    {
        "name": "drawer_1",
        "joint": "desk_to_drawer_1",
        "center_z": 0.462,
        "front_height": 0.176,
        "box_height": 0.142,
        "runner_outer": "runner_1_outer",
    },
    {
        "name": "drawer_2",
        "joint": "desk_to_drawer_2",
        "center_z": 0.212,
        "front_height": 0.286,
        "box_height": 0.250,
        "runner_outer": "runner_2_outer",
    },
)


def _add_drawer_part(
    model: ArticulatedObject,
    *,
    name: str,
    center_z: float,
    front_height: float,
    box_height: float,
    wood: str,
    carcass: str,
    steel: str,
) -> None:
    drawer = model.part(name)

    front_width = 0.394
    front_thickness = 0.018
    box_width = 0.322
    box_depth = 0.470
    wall = 0.012
    bottom_thickness = 0.012
    handle_width = 0.110
    handle_depth = 0.014
    handle_height = 0.010
    runner_thickness = 0.008
    runner_length = 0.340
    runner_height = 0.014

    drawer.visual(
        Box((front_width, front_thickness, front_height)),
        origin=Origin(xyz=(0.0, -front_thickness / 2.0, 0.0)),
        material=wood,
        name="front",
    )
    drawer.visual(
        Box((handle_width, handle_depth, handle_height)),
        origin=Origin(xyz=(0.0, -front_thickness - handle_depth / 2.0, 0.0)),
        material=steel,
        name="pull",
    )

    side_x = box_width / 2.0 - wall / 2.0
    box_center_y = box_depth / 2.0
    bottom_z = -box_height / 2.0 + bottom_thickness / 2.0

    drawer.visual(
        Box((wall, box_depth, box_height)),
        origin=Origin(xyz=(-side_x, box_center_y, 0.0)),
        material=carcass,
        name="side_outer",
    )
    drawer.visual(
        Box((wall, box_depth, box_height)),
        origin=Origin(xyz=(side_x, box_center_y, 0.0)),
        material=carcass,
        name="side_inner",
    )
    drawer.visual(
        Box((box_width, wall, box_height)),
        origin=Origin(xyz=(0.0, box_depth - wall / 2.0, 0.0)),
        material=carcass,
        name="back",
    )
    drawer.visual(
        Box((box_width - 2.0 * wall, box_depth - wall, bottom_thickness)),
        origin=Origin(xyz=(0.0, (box_depth - wall) / 2.0, bottom_z)),
        material=carcass,
        name="bottom",
    )

    runner_x = box_width / 2.0 + runner_thickness / 2.0
    runner_y = 0.020 + runner_length / 2.0
    runner_z = 0.0
    drawer.visual(
        Box((runner_thickness, runner_length, runner_height)),
        origin=Origin(xyz=(-runner_x, runner_y, runner_z)),
        material=steel,
        name="runner_outer",
    )
    drawer.visual(
        Box((runner_thickness, runner_length, runner_height)),
        origin=Origin(xyz=(runner_x, runner_y, runner_z)),
        material=steel,
        name="runner_inner",
    )

    model.articulation(
        f"desk_to_{name}",
        ArticulationType.PRISMATIC,
        parent="desk",
        child=drawer,
        origin=Origin(xyz=(PEDESTAL_CENTER_X, FRAME_FRONT_Y, center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.28,
            lower=0.0,
            upper=0.24,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="computer_desk")

    top_material = model.material("top_material", rgba=(0.58, 0.42, 0.28, 1.0))
    carcass_material = model.material("carcass_material", rgba=(0.90, 0.90, 0.88, 1.0))
    steel_material = model.material("steel_material", rgba=(0.34, 0.36, 0.39, 1.0))
    tray_material = model.material("tray_material", rgba=(0.36, 0.26, 0.18, 1.0))

    desk = model.part("desk")

    desk.visual(
        Box((DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS / 2.0)),
        material=top_material,
        name="top",
    )
    desk.visual(
        Box((0.42, 0.57, 0.05)),
        origin=Origin(xyz=(PEDESTAL_CENTER_X, 0.0, 0.025)),
        material=carcass_material,
        name="plinth",
    )
    desk.visual(
        Box((0.03, 0.57, 0.67)),
        origin=Origin(xyz=(-0.585, 0.0, 0.385)),
        material=carcass_material,
        name="pedestal_side",
    )
    desk.visual(
        Box((0.018, 0.55, 0.67)),
        origin=Origin(xyz=(PEDESTAL_INNER_X, 0.0, 0.385)),
        material=carcass_material,
        name="pedestal_inner",
    )
    desk.visual(
        Box((0.372, 0.018, 0.67)),
        origin=Origin(xyz=(PEDESTAL_CENTER_X + 0.006, 0.282, 0.385)),
        material=carcass_material,
        name="pedestal_back",
    )
    desk.visual(
        Box((0.372, 0.55, 0.018)),
        origin=Origin(xyz=(PEDESTAL_CENTER_X + 0.006, 0.0, 0.362)),
        material=carcass_material,
        name="divider_0",
    )
    desk.visual(
        Box((0.372, 0.55, 0.018)),
        origin=Origin(xyz=(PEDESTAL_CENTER_X + 0.006, 0.0, 0.560)),
        material=carcass_material,
        name="divider_1",
    )
    desk.visual(
        Box((0.372, 0.55, 0.018)),
        origin=Origin(xyz=(PEDESTAL_CENTER_X + 0.006, 0.0, 0.059)),
        material=carcass_material,
        name="pedestal_floor",
    )
    desk.visual(
        Box((0.03, 0.57, 0.72)),
        origin=Origin(xyz=(RIGHT_PANEL_X, 0.0, 0.36)),
        material=carcass_material,
        name="support_panel",
    )
    desk.visual(
        Box((0.75, 0.018, 0.32)),
        origin=Origin(xyz=(0.195, 0.276, 0.36)),
        material=carcass_material,
        name="modesty_panel",
    )

    for index, spec in enumerate(DRAWER_SPECS):
        desk.visual(
            Box((0.012, 0.42, 0.018)),
            origin=Origin(xyz=(-0.564, -0.075, spec["center_z"])),
            material=steel_material,
            name=f"runner_{index}_outer",
        )
        desk.visual(
            Box((0.012, 0.42, 0.018)),
            origin=Origin(xyz=(-0.204, -0.075, spec["center_z"])),
            material=steel_material,
            name=f"runner_{index}_inner",
        )

    desk.visual(
        Box((0.012, 0.36, 0.05)),
        origin=Origin(xyz=(-0.174, -0.10, 0.695)),
        material=steel_material,
        name="tray_guide_0",
    )
    desk.visual(
        Box((0.012, 0.36, 0.05)),
        origin=Origin(xyz=(0.564, -0.10, 0.695)),
        material=steel_material,
        name="tray_guide_1",
    )

    for spec in DRAWER_SPECS:
        _add_drawer_part(
            model,
            name=spec["name"],
            center_z=spec["center_z"],
            front_height=spec["front_height"],
            box_height=spec["box_height"],
            wood=top_material.name,
            carcass=carcass_material.name,
            steel=steel_material.name,
        )

    keyboard_tray = model.part("keyboard_tray")
    keyboard_tray.visual(
        Box((0.69, 0.30, 0.018)),
        origin=Origin(xyz=(0.0, 0.15, 0.0)),
        material=tray_material,
        name="tray",
    )
    keyboard_tray.visual(
        Box((0.64, 0.020, 0.045)),
        origin=Origin(xyz=(0.0, -0.010, 0.004)),
        material=tray_material,
        name="lip",
    )
    keyboard_tray.visual(
        Box((0.018, 0.30, 0.018)),
        origin=Origin(xyz=(-0.354, 0.15, -0.018)),
        material=steel_material,
        name="slide_0",
    )
    keyboard_tray.visual(
        Box((0.018, 0.30, 0.018)),
        origin=Origin(xyz=(0.354, 0.15, -0.018)),
        material=steel_material,
        name="slide_1",
    )

    model.articulation(
        "desk_to_keyboard_tray",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=keyboard_tray,
        origin=Origin(xyz=(0.195, -0.270, 0.690)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.25,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk = object_model.get_part("desk")

    for index, spec in enumerate(DRAWER_SPECS):
        drawer = object_model.get_part(spec["name"])
        joint = object_model.get_articulation(spec["joint"])
        limits = joint.motion_limits
        upper = 0.0 if limits is None or limits.upper is None else limits.upper

        ctx.expect_gap(
            desk,
            drawer,
            axis="y",
            negative_elem="front",
            max_gap=0.002,
            max_penetration=0.0,
            name=f"{spec['name']} front sits flush with the desk face",
        )
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="runner_outer",
            elem_b=f"runner_{index}_outer",
            min_overlap=0.18,
            name=f"{spec['name']} stays engaged on its runners when closed",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: upper}):
            open_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                desk,
                axes="y",
                elem_a="runner_outer",
                elem_b=f"runner_{index}_outer",
                min_overlap=0.05,
                name=f"{spec['name']} keeps retained runner overlap when open",
            )

        ctx.check(
            f"{spec['name']} extends forward",
            rest_pos is not None and open_pos is not None and open_pos[1] < rest_pos[1] - 0.16,
            details=f"rest={rest_pos}, open={open_pos}",
        )

    keyboard_tray = object_model.get_part("keyboard_tray")
    tray_joint = object_model.get_articulation("desk_to_keyboard_tray")
    tray_limits = tray_joint.motion_limits
    tray_upper = 0.0 if tray_limits is None or tray_limits.upper is None else tray_limits.upper

    ctx.expect_overlap(
        keyboard_tray,
        desk,
        axes="y",
        elem_a="slide_0",
        elem_b="tray_guide_0",
        min_overlap=0.16,
        name="keyboard tray starts captured by the short side guides",
    )

    tray_rest = ctx.part_world_position(keyboard_tray)
    with ctx.pose({tray_joint: tray_upper}):
        tray_open = ctx.part_world_position(keyboard_tray)
        ctx.expect_overlap(
            keyboard_tray,
            desk,
            axes="y",
            elem_a="slide_0",
            elem_b="tray_guide_0",
            min_overlap=0.08,
            name="keyboard tray keeps engagement at full extension",
        )

    ctx.check(
        "keyboard tray extends forward",
        tray_rest is not None and tray_open is not None and tray_open[1] < tray_rest[1] - 0.10,
        details=f"rest={tray_rest}, open={tray_open}",
    )

    return ctx.report()


object_model = build_object_model()
