from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


DESK_WIDTH = 1.68
DESK_DEPTH = 0.92
DESK_HEIGHT = 0.78
TOP_THICKNESS = 0.05
TOP_Z = DESK_HEIGHT - TOP_THICKNESS / 2.0
TOP_UNDERSIDE_Z = DESK_HEIGHT - TOP_THICKNESS

SIDE_OVERHANG = 0.06
PEDESTAL_WIDTH = 0.42
PEDESTAL_DEPTH = 0.62
PEDESTAL_HEIGHT = TOP_UNDERSIDE_Z + 0.001
PEDESTAL_CENTER_X = DESK_WIDTH / 2.0 - SIDE_OVERHANG - PEDESTAL_WIDTH / 2.0

WALL_THICKNESS = 0.018
TOP_PANEL_THICKNESS = 0.018
SHELF_THICKNESS = 0.018
PLINTH_HEIGHT = 0.05
PLINTH_INSET = 0.012

DRAWER_PROUD = 0.003
DRAWER_FRONT_THICKNESS = 0.022
DRAWER_BODY_DEPTH = 0.47
DRAWER_OVERALL_DEPTH = DRAWER_FRONT_THICKNESS + DRAWER_BODY_DEPTH
DRAWER_BOX_WIDTH = 0.36
DRAWER_FRONT_WIDTH = 0.378
DRAWER_SIDE_THICKNESS = 0.014
DRAWER_BACK_THICKNESS = 0.014
DRAWER_BOTTOM_THICKNESS = 0.008
DRAWER_TRAVEL = 0.28

GUIDE_RAIL_WIDTH = 0.012
GUIDE_RAIL_HEIGHT = 0.018
GUIDE_RAIL_LENGTH = 0.46
GUIDE_RAIL_X_OFFSET = DRAWER_BOX_WIDTH / 2.0 + GUIDE_RAIL_WIDTH / 2.0

LOWER_BAY_Z = PLINTH_HEIGHT
LOWER_BAY_HEIGHT = 0.314
MIDDLE_BAY_Z = LOWER_BAY_Z + LOWER_BAY_HEIGHT + SHELF_THICKNESS
MIDDLE_BAY_HEIGHT = 0.156
UPPER_BAY_Z = MIDDLE_BAY_Z + MIDDLE_BAY_HEIGHT + SHELF_THICKNESS
UPPER_BAY_HEIGHT = 0.156

DRAWER_LAYOUT = (
    {
        "part": "left_upper_drawer",
        "joint": "body_to_left_upper_drawer",
        "x_center": -PEDESTAL_CENTER_X,
        "front_sign": 1.0,
        "center_z": UPPER_BAY_Z + UPPER_BAY_HEIGHT / 2.0,
        "front_height": 0.150,
        "box_height": 0.120,
        "body_rail": "left_upper_left_rail",
        "drawer_side": "left_side",
    },
    {
        "part": "left_middle_drawer",
        "joint": "body_to_left_middle_drawer",
        "x_center": -PEDESTAL_CENTER_X,
        "front_sign": 1.0,
        "center_z": MIDDLE_BAY_Z + MIDDLE_BAY_HEIGHT / 2.0,
        "front_height": 0.150,
        "box_height": 0.120,
        "body_rail": "left_middle_left_rail",
        "drawer_side": "left_side",
    },
    {
        "part": "left_lower_drawer",
        "joint": "body_to_left_lower_drawer",
        "x_center": -PEDESTAL_CENTER_X,
        "front_sign": 1.0,
        "center_z": LOWER_BAY_Z + LOWER_BAY_HEIGHT / 2.0,
        "front_height": 0.304,
        "box_height": 0.260,
        "body_rail": "left_lower_left_rail",
        "drawer_side": "left_side",
    },
    {
        "part": "right_upper_drawer",
        "joint": "body_to_right_upper_drawer",
        "x_center": PEDESTAL_CENTER_X,
        "front_sign": -1.0,
        "center_z": UPPER_BAY_Z + UPPER_BAY_HEIGHT / 2.0,
        "front_height": 0.150,
        "box_height": 0.120,
        "body_rail": "right_upper_right_rail",
        "drawer_side": "right_side",
    },
    {
        "part": "right_middle_drawer",
        "joint": "body_to_right_middle_drawer",
        "x_center": PEDESTAL_CENTER_X,
        "front_sign": -1.0,
        "center_z": MIDDLE_BAY_Z + MIDDLE_BAY_HEIGHT / 2.0,
        "front_height": 0.150,
        "box_height": 0.120,
        "body_rail": "right_middle_right_rail",
        "drawer_side": "right_side",
    },
    {
        "part": "right_lower_drawer",
        "joint": "body_to_right_lower_drawer",
        "x_center": PEDESTAL_CENTER_X,
        "front_sign": -1.0,
        "center_z": LOWER_BAY_Z + LOWER_BAY_HEIGHT / 2.0,
        "front_height": 0.304,
        "box_height": 0.260,
        "body_rail": "right_lower_right_rail",
        "drawer_side": "right_side",
    },
)


def _drawer_center_y(front_sign: float) -> float:
    return front_sign * (PEDESTAL_DEPTH / 2.0 + DRAWER_PROUD - DRAWER_OVERALL_DEPTH / 2.0)


def _drawer_box_center_y(front_sign: float) -> float:
    return _drawer_center_y(front_sign) - front_sign * (DRAWER_FRONT_THICKNESS / 2.0)


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str,
    rpy: tuple[float, float, float],
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_pedestal(
    body,
    *,
    prefix: str,
    x_center: float,
    front_sign: float,
    wood,
    trim,
    rail_material,
) -> None:
    side_z = (PLINTH_HEIGHT + PEDESTAL_HEIGHT) / 2.0
    side_height = PEDESTAL_HEIGHT - PLINTH_HEIGHT
    side_x = x_center + (PEDESTAL_WIDTH / 2.0 - WALL_THICKNESS / 2.0)
    back_y = -front_sign * (PEDESTAL_DEPTH / 2.0 - WALL_THICKNESS / 2.0)

    _add_box(
        body,
        size=(PEDESTAL_WIDTH - 2.0 * PLINTH_INSET, PEDESTAL_DEPTH - 2.0 * PLINTH_INSET, PLINTH_HEIGHT),
        xyz=(x_center, 0.0, PLINTH_HEIGHT / 2.0),
        material=trim,
        name=f"{prefix}_plinth",
    )
    _add_box(
        body,
        size=(WALL_THICKNESS, PEDESTAL_DEPTH, side_height),
        xyz=(side_x, 0.0, side_z),
        material=wood,
        name=f"{prefix}_right_side_panel",
    )
    _add_box(
        body,
        size=(WALL_THICKNESS, PEDESTAL_DEPTH, side_height),
        xyz=(2.0 * x_center - side_x, 0.0, side_z),
        material=wood,
        name=f"{prefix}_left_side_panel",
    )
    _add_box(
        body,
        size=(PEDESTAL_WIDTH - 2.0 * WALL_THICKNESS + 0.001, WALL_THICKNESS, side_height),
        xyz=(x_center, back_y, side_z),
        material=wood,
        name=f"{prefix}_back_panel",
    )
    _add_box(
        body,
        size=(PEDESTAL_WIDTH - 2.0 * WALL_THICKNESS + 0.001, PEDESTAL_DEPTH, TOP_PANEL_THICKNESS),
        xyz=(x_center, 0.0, PEDESTAL_HEIGHT - TOP_PANEL_THICKNESS / 2.0),
        material=wood,
        name=f"{prefix}_top_panel",
    )

    rear_limit = -front_sign * (PEDESTAL_DEPTH / 2.0 - WALL_THICKNESS)
    front_limit = front_sign * (PEDESTAL_DEPTH / 2.0 - 0.03)
    shelf_depth = abs(front_limit - rear_limit)
    shelf_y = (front_limit + rear_limit) / 2.0

    for shelf_name, shelf_z in (
        ("lower_shelf", LOWER_BAY_Z + LOWER_BAY_HEIGHT + SHELF_THICKNESS / 2.0),
        ("upper_shelf", MIDDLE_BAY_Z + MIDDLE_BAY_HEIGHT + SHELF_THICKNESS / 2.0),
    ):
        _add_box(
            body,
            size=(PEDESTAL_WIDTH - 2.0 * WALL_THICKNESS + 0.001, shelf_depth, SHELF_THICKNESS),
            xyz=(x_center, shelf_y, shelf_z),
            material=wood,
            name=f"{prefix}_{shelf_name}",
        )

    rail_specs = (
        ("upper", UPPER_BAY_Z + UPPER_BAY_HEIGHT / 2.0),
        ("middle", MIDDLE_BAY_Z + MIDDLE_BAY_HEIGHT / 2.0),
        ("lower", LOWER_BAY_Z + LOWER_BAY_HEIGHT / 2.0),
    )
    rail_y = _drawer_box_center_y(front_sign)
    for level_name, rail_z in rail_specs:
        _add_box(
            body,
            size=(GUIDE_RAIL_WIDTH, GUIDE_RAIL_LENGTH, GUIDE_RAIL_HEIGHT),
            xyz=(x_center - GUIDE_RAIL_X_OFFSET, rail_y, rail_z),
            material=rail_material,
            name=f"{prefix}_{level_name}_left_rail",
        )
        _add_box(
            body,
            size=(GUIDE_RAIL_WIDTH, GUIDE_RAIL_LENGTH, GUIDE_RAIL_HEIGHT),
            xyz=(x_center + GUIDE_RAIL_X_OFFSET, rail_y, rail_z),
            material=rail_material,
            name=f"{prefix}_{level_name}_right_rail",
        )


def _add_drawer(
    model: ArticulatedObject,
    *,
    part_name: str,
    joint_name: str,
    x_center: float,
    front_sign: float,
    center_z: float,
    front_height: float,
    box_height: float,
    wood,
    trim,
    brass,
) -> None:
    drawer = model.part(part_name)
    front_center_y = front_sign * (DRAWER_OVERALL_DEPTH / 2.0 - DRAWER_FRONT_THICKNESS / 2.0)
    box_center_y = -front_sign * (DRAWER_FRONT_THICKNESS / 2.0)
    back_center_y = -front_sign * (DRAWER_OVERALL_DEPTH / 2.0 - DRAWER_BACK_THICKNESS / 2.0)
    side_x = DRAWER_BOX_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0
    bottom_z = -box_height / 2.0 + DRAWER_BOTTOM_THICKNESS / 2.0

    _add_box(
        drawer,
        size=(DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, front_height),
        xyz=(0.0, front_center_y, 0.0),
        material=wood,
        name="front_panel",
    )
    _add_box(
        drawer,
        size=(DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, box_height),
        xyz=(-side_x, box_center_y, 0.0),
        material=trim,
        name="left_side",
    )
    _add_box(
        drawer,
        size=(DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, box_height),
        xyz=(side_x, box_center_y, 0.0),
        material=trim,
        name="right_side",
    )
    _add_box(
        drawer,
        size=(DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS, DRAWER_BACK_THICKNESS, box_height),
        xyz=(0.0, back_center_y, 0.0),
        material=trim,
        name="back_panel",
    )
    _add_box(
        drawer,
        size=(DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH - DRAWER_BACK_THICKNESS, DRAWER_BOTTOM_THICKNESS),
        xyz=(0.0, box_center_y + front_sign * DRAWER_BACK_THICKNESS / 2.0, bottom_z),
        material=trim,
        name="bottom_panel",
    )

    pull_z = 0.0 if front_height < 0.2 else front_height * 0.10
    boss_y = front_sign * (DRAWER_OVERALL_DEPTH / 2.0 + 0.007)
    bar_y = front_sign * (DRAWER_OVERALL_DEPTH / 2.0 + 0.016)
    boss_offset_x = 0.05
    bar_length = 0.11 if front_height < 0.2 else 0.13

    _add_cylinder(
        drawer,
        radius=0.005,
        length=0.016,
        xyz=(-boss_offset_x, boss_y, pull_z),
        material=brass,
        name="left_pull_post",
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    _add_cylinder(
        drawer,
        radius=0.005,
        length=0.016,
        xyz=(boss_offset_x, boss_y, pull_z),
        material=brass,
        name="right_pull_post",
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    _add_cylinder(
        drawer,
        radius=0.0055,
        length=bar_length,
        xyz=(0.0, bar_y, pull_z),
        material=brass,
        name="pull_bar",
        rpy=(0.0, pi / 2.0, 0.0),
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent="desk_body",
        child=drawer,
        origin=Origin(xyz=(x_center, _drawer_center_y(front_sign), center_z)),
        axis=(0.0, front_sign, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_partners_desk")

    wood = model.material("mahogany", rgba=(0.35, 0.17, 0.09, 1.0))
    trim = model.material("mahogany_dark", rgba=(0.22, 0.10, 0.05, 1.0))
    leather = model.material("green_leather", rgba=(0.13, 0.22, 0.13, 1.0))
    brass = model.material("antique_brass", rgba=(0.74, 0.60, 0.30, 1.0))
    rail_material = model.material("runner_metal", rgba=(0.44, 0.44, 0.47, 1.0))

    body = model.part("desk_body")
    _add_box(
        body,
        size=(DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS),
        xyz=(0.0, 0.0, TOP_Z),
        material=wood,
        name="top_slab",
    )
    _add_box(
        body,
        size=(1.18, 0.56, 0.004),
        xyz=(0.0, 0.0, DESK_HEIGHT - 0.002),
        material=leather,
        name="writing_inlay",
    )
    apron_z = TOP_UNDERSIDE_Z - 0.055
    _add_box(
        body,
        size=(0.72, 0.03, 0.111),
        xyz=(0.0, DESK_DEPTH / 2.0 - 0.035, apron_z),
        material=trim,
        name="front_apron",
    )
    _add_box(
        body,
        size=(0.72, 0.03, 0.111),
        xyz=(0.0, -DESK_DEPTH / 2.0 + 0.035, apron_z),
        material=trim,
        name="rear_apron",
    )

    _add_pedestal(
        body,
        prefix="left",
        x_center=-PEDESTAL_CENTER_X,
        front_sign=1.0,
        wood=wood,
        trim=trim,
        rail_material=rail_material,
    )
    _add_pedestal(
        body,
        prefix="right",
        x_center=PEDESTAL_CENTER_X,
        front_sign=-1.0,
        wood=wood,
        trim=trim,
        rail_material=rail_material,
    )

    for spec in DRAWER_LAYOUT:
        _add_drawer(
            model,
            part_name=spec["part"],
            joint_name=spec["joint"],
            x_center=spec["x_center"],
            front_sign=spec["front_sign"],
            center_z=spec["center_z"],
            front_height=spec["front_height"],
            box_height=spec["box_height"],
            wood=wood,
            trim=trim,
            brass=brass,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("desk_body")

    drawer_checks = []
    for spec in DRAWER_LAYOUT:
        drawer = object_model.get_part(spec["part"])
        joint = object_model.get_articulation(spec["joint"])
        drawer_side = drawer.get_visual(spec["drawer_side"])
        guide_rail = body.get_visual(spec["body_rail"])
        drawer_checks.append((spec, drawer, joint, drawer_side, guide_rail))

    for spec, drawer, joint, drawer_side, guide_rail in drawer_checks:
        ctx.expect_contact(
            drawer,
            body,
            elem_a=drawer_side,
            elem_b=guide_rail,
            name=f"{drawer.name} is seated on its guide rail when closed",
        )

        rest_pos = ctx.part_world_position(drawer)
        upper = 0.0 if joint.motion_limits is None or joint.motion_limits.upper is None else joint.motion_limits.upper

        with ctx.pose({joint: upper}):
            ctx.expect_contact(
                drawer,
                body,
                elem_a=drawer_side,
                elem_b=guide_rail,
                name=f"{drawer.name} stays guided at full extension",
            )
            extended_pos = ctx.part_world_position(drawer)

        moved_outward = (
            rest_pos is not None
            and extended_pos is not None
            and (
                extended_pos[1] > rest_pos[1] + 0.20
                if spec["front_sign"] > 0.0
                else extended_pos[1] < rest_pos[1] - 0.20
            )
        )
        ctx.check(
            f"{drawer.name} opens away from its pedestal face",
            moved_outward,
            details=f"rest={rest_pos}, extended={extended_pos}, sign={spec['front_sign']}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
