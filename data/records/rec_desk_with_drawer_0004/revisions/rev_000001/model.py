from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

HERE = Path(__file__).resolve().parent
ASSETS = AssetContext.from_script(__file__)

DESK_WIDTH = 1.60
DESK_DEPTH = 0.76
TOP_THICKNESS = 0.04
DESK_HEIGHT = 0.76

PEDESTAL_WIDTH = 0.38
PEDESTAL_DEPTH = 0.66
PEDESTAL_HEIGHT = DESK_HEIGHT - TOP_THICKNESS
KNEE_SPACE_WIDTH = 0.76
PEDESTAL_OFFSET_X = KNEE_SPACE_WIDTH * 0.5 + PEDESTAL_WIDTH * 0.5

SIDE_THICKNESS = 0.018
BACK_THICKNESS = 0.014
CAP_THICKNESS = 0.034
PLINTH_HEIGHT = 0.055
SEPARATOR_THICKNESS = 0.014
MODESTY_PANEL_THICKNESS = 0.018
MODESTY_PANEL_WIDTH = KNEE_SPACE_WIDTH - 0.04
MODESTY_PANEL_HEIGHT = 0.50
MODESTY_PANEL_CENTER_Y = -PEDESTAL_DEPTH * 0.5 + MODESTY_PANEL_THICKNESS * 0.5
MODESTY_PANEL_CENTER_Z = DESK_HEIGHT - TOP_THICKNESS - MODESTY_PANEL_HEIGHT * 0.5

DRAWER_FACE_WIDTH = 0.328
DRAWER_FACE_THICKNESS = 0.018
DRAWER_BODY_WIDTH = 0.308
DRAWER_BODY_DEPTH = 0.50
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BACK_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.012
DRAWER_RAIL_THICKNESS = 0.008
DRAWER_RAIL_HEIGHT = 0.018
DRAWER_RAIL_LENGTH = 0.47
DRAWER_TRAVEL = 0.28

PEDESTAL_RAIL_THICKNESS = 0.010
PEDESTAL_RAIL_HEIGHT = 0.018
PEDESTAL_RAIL_LENGTH = 0.47
PEDESTAL_INNER_HALF_WIDTH = PEDESTAL_WIDTH * 0.5 - SIDE_THICKNESS
PEDESTAL_RAIL_X = PEDESTAL_INNER_HALF_WIDTH - PEDESTAL_RAIL_THICKNESS * 0.5
DRAWER_RAIL_X = PEDESTAL_RAIL_X - PEDESTAL_RAIL_THICKNESS * 0.5 - DRAWER_RAIL_THICKNESS * 0.5
PEDESTAL_RAIL_Y = 0.075
DRAWER_RAIL_Y = -0.255

DRAWER_LAYOUTS = (
    ("top", 0.540, 0.120),
    ("middle", 0.336, 0.180),
    ("file", 0.072, 0.240),
)


def _face_height(bay_height: float) -> float:
    return bay_height - 0.012


def _body_height(bay_height: float) -> float:
    return max(bay_height - 0.034, 0.07)


def _drawer_center_z(bay_bottom: float, bay_height: float) -> float:
    return bay_bottom + bay_height * 0.5


def _add_pedestal(model: ArticulatedObject, name: str, shell_material, rail_material):
    pedestal = model.part(name)

    side_height = PEDESTAL_HEIGHT - PLINTH_HEIGHT
    side_center_z = PLINTH_HEIGHT + side_height * 0.5
    inner_width = PEDESTAL_WIDTH - 2.0 * SIDE_THICKNESS
    inner_depth = PEDESTAL_DEPTH - BACK_THICKNESS
    inner_center_y = BACK_THICKNESS * 0.5

    pedestal.visual(
        Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=shell_material,
        name="plinth",
    )
    pedestal.visual(
        Box((SIDE_THICKNESS, PEDESTAL_DEPTH, side_height)),
        origin=Origin(xyz=(-PEDESTAL_WIDTH * 0.5 + SIDE_THICKNESS * 0.5, 0.0, side_center_z)),
        material=shell_material,
        name="outer_side",
    )
    pedestal.visual(
        Box((SIDE_THICKNESS, PEDESTAL_DEPTH, side_height)),
        origin=Origin(xyz=(PEDESTAL_WIDTH * 0.5 - SIDE_THICKNESS * 0.5, 0.0, side_center_z)),
        material=shell_material,
        name="inner_side",
    )
    pedestal.visual(
        Box((inner_width, BACK_THICKNESS, side_height)),
        origin=Origin(
            xyz=(0.0, -PEDESTAL_DEPTH * 0.5 + BACK_THICKNESS * 0.5, side_center_z)
        ),
        material=shell_material,
        name="back_panel",
    )
    pedestal.visual(
        Box((inner_width, inner_depth, CAP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, inner_center_y, PEDESTAL_HEIGHT - CAP_THICKNESS * 0.5)
        ),
        material=shell_material,
        name="cap",
    )

    separator_zs = (
        DRAWER_LAYOUTS[1][1] - SEPARATOR_THICKNESS * 0.5,
        DRAWER_LAYOUTS[0][1] - SEPARATOR_THICKNESS * 0.5,
    )
    for index, z_center in enumerate(separator_zs, start=1):
        pedestal.visual(
            Box((inner_width, inner_depth, SEPARATOR_THICKNESS)),
            origin=Origin(xyz=(0.0, inner_center_y, z_center)),
            material=shell_material,
            name=f"separator_{index}",
        )

    for drawer_name, bay_bottom, bay_height in DRAWER_LAYOUTS:
        z_center = _drawer_center_z(bay_bottom, bay_height)
        pedestal.visual(
            Box((PEDESTAL_RAIL_THICKNESS, PEDESTAL_RAIL_LENGTH, PEDESTAL_RAIL_HEIGHT)),
            origin=Origin(xyz=(-PEDESTAL_RAIL_X, PEDESTAL_RAIL_Y, z_center)),
            material=rail_material,
            name=f"{drawer_name}_left_rail",
        )
        pedestal.visual(
            Box((PEDESTAL_RAIL_THICKNESS, PEDESTAL_RAIL_LENGTH, PEDESTAL_RAIL_HEIGHT)),
            origin=Origin(xyz=(PEDESTAL_RAIL_X, PEDESTAL_RAIL_Y, z_center)),
            material=rail_material,
            name=f"{drawer_name}_right_rail",
        )

    pedestal.inertial = Inertial.from_geometry(
        Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, PEDESTAL_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT * 0.5)),
    )
    return pedestal


def _add_drawer(
    model: ArticulatedObject,
    name: str,
    bay_height: float,
    drawer_material,
    rail_material,
    hardware_material,
):
    drawer = model.part(name)
    face_height = _face_height(bay_height)
    body_height = _body_height(bay_height)
    handle_width = 0.16 if bay_height < 0.20 else 0.18
    body_side_z = -DRAWER_BOTTOM_THICKNESS * 0.5
    body_bottom_z = -body_height * 0.5 + DRAWER_BOTTOM_THICKNESS * 0.5
    side_x = DRAWER_BODY_WIDTH * 0.5 - DRAWER_SIDE_THICKNESS * 0.5
    inner_body_width = DRAWER_BODY_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS
    bottom_depth = DRAWER_BODY_DEPTH - DRAWER_BACK_THICKNESS
    bottom_center_y = -bottom_depth * 0.5
    back_center_y = -DRAWER_BODY_DEPTH + DRAWER_BACK_THICKNESS * 0.5

    drawer.visual(
        Box((DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS, face_height)),
        origin=Origin(xyz=(0.0, DRAWER_FACE_THICKNESS * 0.5, 0.0)),
        material=drawer_material,
        name="front",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, body_height)),
        origin=Origin(xyz=(-side_x, -DRAWER_BODY_DEPTH * 0.5, body_side_z)),
        material=drawer_material,
        name="left_side",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, body_height)),
        origin=Origin(xyz=(side_x, -DRAWER_BODY_DEPTH * 0.5, body_side_z)),
        material=drawer_material,
        name="right_side",
    )
    drawer.visual(
        Box((inner_body_width, DRAWER_BACK_THICKNESS, body_height)),
        origin=Origin(xyz=(0.0, back_center_y, body_side_z)),
        material=drawer_material,
        name="back",
    )
    drawer.visual(
        Box((inner_body_width, bottom_depth, DRAWER_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, bottom_center_y, body_bottom_z)),
        material=drawer_material,
        name="bottom",
    )
    drawer.visual(
        Box((DRAWER_RAIL_THICKNESS, DRAWER_RAIL_LENGTH, DRAWER_RAIL_HEIGHT)),
        origin=Origin(xyz=(-DRAWER_RAIL_X, DRAWER_RAIL_Y, 0.0)),
        material=rail_material,
        name="left_rail",
    )
    drawer.visual(
        Box((DRAWER_RAIL_THICKNESS, DRAWER_RAIL_LENGTH, DRAWER_RAIL_HEIGHT)),
        origin=Origin(xyz=(DRAWER_RAIL_X, DRAWER_RAIL_Y, 0.0)),
        material=rail_material,
        name="right_rail",
    )
    drawer.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(-handle_width * 0.36, DRAWER_FACE_THICKNESS + 0.006, 0.0)),
        material=hardware_material,
        name="left_pull_post",
    )
    drawer.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(handle_width * 0.36, DRAWER_FACE_THICKNESS + 0.006, 0.0)),
        material=hardware_material,
        name="right_pull_post",
    )
    drawer.visual(
        Box((handle_width, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, DRAWER_FACE_THICKNESS + 0.015, 0.0)),
        material=hardware_material,
        name="pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FACE_WIDTH, DRAWER_BODY_DEPTH + DRAWER_FACE_THICKNESS, face_height)),
        mass=3.0 if bay_height >= 0.20 else 2.0,
        origin=Origin(
            xyz=(0.0, -(DRAWER_BODY_DEPTH - DRAWER_FACE_THICKNESS) * 0.5, 0.0)
        ),
    )
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_desk", assets=ASSETS)

    walnut = model.material("walnut", rgba=(0.39, 0.25, 0.15, 1.0))
    painted_wood = model.material("painted_wood", rgba=(0.84, 0.82, 0.77, 1.0))
    drawer_paint = model.material("drawer_paint", rgba=(0.86, 0.84, 0.79, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.20, 0.19, 0.18, 1.0))

    desk_top = model.part("desk_top")
    desk_top.visual(
        Box((DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS * 0.5)),
        material=walnut,
        name="desktop",
    )
    desk_top.inertial = Inertial.from_geometry(
        Box((DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS)),
        mass=20.0,
        origin=Origin(xyz=(0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS * 0.5)),
    )

    left_pedestal = _add_pedestal(model, "left_pedestal", painted_wood, steel)
    right_pedestal = _add_pedestal(model, "right_pedestal", painted_wood, steel)
    modesty_panel = model.part("modesty_panel")
    modesty_panel.visual(
        Box((MODESTY_PANEL_WIDTH, MODESTY_PANEL_THICKNESS, MODESTY_PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, MODESTY_PANEL_CENTER_Y, MODESTY_PANEL_CENTER_Z)),
        material=painted_wood,
        name="panel",
    )
    modesty_panel.inertial = Inertial.from_geometry(
        Box((MODESTY_PANEL_WIDTH, MODESTY_PANEL_THICKNESS, MODESTY_PANEL_HEIGHT)),
        mass=7.0,
        origin=Origin(xyz=(0.0, MODESTY_PANEL_CENTER_Y, MODESTY_PANEL_CENTER_Z)),
    )

    model.articulation(
        "desk_to_left_pedestal",
        ArticulationType.FIXED,
        parent=desk_top,
        child=left_pedestal,
        origin=Origin(xyz=(-PEDESTAL_OFFSET_X, 0.0, 0.0)),
    )
    model.articulation(
        "desk_to_right_pedestal",
        ArticulationType.FIXED,
        parent=desk_top,
        child=right_pedestal,
        origin=Origin(xyz=(PEDESTAL_OFFSET_X, 0.0, 0.0)),
    )
    model.articulation(
        "desk_to_modesty_panel",
        ArticulationType.FIXED,
        parent=desk_top,
        child=modesty_panel,
        origin=Origin(),
    )

    for side, pedestal in (("left", left_pedestal), ("right", right_pedestal)):
        for drawer_name, bay_bottom, bay_height in DRAWER_LAYOUTS:
            drawer = _add_drawer(
                model,
                f"{side}_{drawer_name}_drawer",
                bay_height,
                drawer_paint,
                steel,
                dark_hardware,
            )
            model.articulation(
                f"{side}_pedestal_to_{drawer_name}_drawer",
                ArticulationType.PRISMATIC,
                parent=pedestal,
                child=drawer,
                origin=Origin(
                    xyz=(0.0, PEDESTAL_DEPTH * 0.5, _drawer_center_z(bay_bottom, bay_height))
                ),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=45.0,
                    velocity=0.35,
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    desk_top = object_model.get_part("desk_top")
    left_pedestal = object_model.get_part("left_pedestal")
    right_pedestal = object_model.get_part("right_pedestal")
    modesty_panel = object_model.get_part("modesty_panel")
    desktop = desk_top.get_visual("desktop")
    left_cap = left_pedestal.get_visual("cap")
    right_cap = right_pedestal.get_visual("cap")
    panel = modesty_panel.get_visual("panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=8)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_within(left_pedestal, desk_top, axes="xy", inner_elem=left_cap, outer_elem=desktop)
    ctx.expect_within(right_pedestal, desk_top, axes="xy", inner_elem=right_cap, outer_elem=desktop)
    ctx.expect_overlap(left_pedestal, desk_top, axes="xy", elem_a=left_cap, elem_b=desktop, min_overlap=0.20)
    ctx.expect_overlap(right_pedestal, desk_top, axes="xy", elem_a=right_cap, elem_b=desktop, min_overlap=0.20)
    ctx.expect_within(modesty_panel, desk_top, axes="xy", inner_elem=panel, outer_elem=desktop)
    ctx.expect_gap(
        desk_top,
        modesty_panel,
        axis="z",
        positive_elem=desktop,
        negative_elem=panel,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        desk_top,
        left_pedestal,
        axis="z",
        positive_elem=desktop,
        negative_elem=left_cap,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        desk_top,
        right_pedestal,
        axis="z",
        positive_elem=desktop,
        negative_elem=right_cap,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        right_pedestal,
        left_pedestal,
        axis="x",
        positive_elem=right_cap,
        negative_elem=left_cap,
        min_gap=KNEE_SPACE_WIDTH,
    )
    ctx.expect_gap(
        modesty_panel,
        left_pedestal,
        axis="x",
        positive_elem=panel,
        negative_elem=left_cap,
        min_gap=0.015,
    )
    ctx.expect_gap(
        right_pedestal,
        modesty_panel,
        axis="x",
        positive_elem=right_cap,
        negative_elem=panel,
        min_gap=0.015,
    )

    for side, pedestal in (("left", left_pedestal), ("right", right_pedestal)):
        for drawer_name, _, bay_height in DRAWER_LAYOUTS:
            drawer = object_model.get_part(f"{side}_{drawer_name}_drawer")
            articulation = object_model.get_articulation(
                f"{side}_pedestal_to_{drawer_name}_drawer"
            )
            drawer_front = drawer.get_visual("front")
            drawer_left_rail = drawer.get_visual("left_rail")
            drawer_right_rail = drawer.get_visual("right_rail")
            pedestal_left_rail = pedestal.get_visual(f"{drawer_name}_left_rail")
            pedestal_right_rail = pedestal.get_visual(f"{drawer_name}_right_rail")
            limits = articulation.motion_limits

            ctx.check(
                f"{articulation.name}_axis",
                articulation.axis == (0.0, 1.0, 0.0),
                details=f"expected slide axis (0, 1, 0), got {articulation.axis}",
            )
            ctx.check(
                f"{articulation.name}_travel_limits",
                limits is not None
                and limits.lower == 0.0
                and limits.upper == DRAWER_TRAVEL,
                details="drawer travel should run from 0.0 m closed to 0.28 m open",
            )

            ctx.expect_gap(
                drawer,
                pedestal,
                axis="y",
                positive_elem=drawer_front,
                max_gap=0.001,
                max_penetration=0.0,
            )
            ctx.expect_within(drawer, pedestal, axes="xz", margin=0.0)
            ctx.expect_overlap(drawer, pedestal, axes="xz", min_overlap=0.03)
            ctx.expect_contact(drawer, pedestal, elem_a=drawer_left_rail, elem_b=pedestal_left_rail)
            ctx.expect_contact(drawer, pedestal, elem_a=drawer_right_rail, elem_b=pedestal_right_rail)

            if limits is not None and limits.lower is not None and limits.upper is not None:
                with ctx.pose({articulation: limits.lower}):
                    ctx.fail_if_parts_overlap_in_current_pose(
                        name=f"{articulation.name}_lower_no_overlap"
                    )
                    ctx.fail_if_isolated_parts(
                        name=f"{articulation.name}_lower_no_floating"
                    )

                with ctx.pose({articulation: limits.upper}):
                    ctx.fail_if_parts_overlap_in_current_pose(
                        name=f"{articulation.name}_upper_no_overlap"
                    )
                    ctx.fail_if_isolated_parts(
                        name=f"{articulation.name}_upper_no_floating"
                    )

            with ctx.pose({articulation: DRAWER_TRAVEL * 0.8}):
                ctx.expect_gap(
                    drawer,
                    pedestal,
                    axis="y",
                    positive_elem=drawer_front,
                    min_gap=0.20,
                )
                ctx.expect_within(drawer, pedestal, axes="xz", margin=0.0)
                ctx.expect_overlap(
                    drawer,
                    pedestal,
                    axes="xz",
                    min_overlap=0.03 if bay_height < 0.20 else 0.04,
                )
                ctx.expect_contact(
                    drawer,
                    pedestal,
                    elem_a=drawer_left_rail,
                    elem_b=pedestal_left_rail,
                )
                ctx.expect_contact(
                    drawer,
                    pedestal,
                    elem_a=drawer_right_rail,
                    elem_b=pedestal_right_rail,
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
