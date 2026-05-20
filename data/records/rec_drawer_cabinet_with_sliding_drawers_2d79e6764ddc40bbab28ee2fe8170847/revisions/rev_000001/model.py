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


CABINET_WIDTH = 0.36
CABINET_DEPTH = 0.58
CABINET_HEIGHT = 0.86
PANEL_T = 0.018

FRONT_MARGIN_BOTTOM = 0.065
FRONT_MARGIN_TOP = 0.030
FRONT_GAP = 0.012
DRAWER_FRONT_H = (
    CABINET_HEIGHT - FRONT_MARGIN_BOTTOM - FRONT_MARGIN_TOP - 2.0 * FRONT_GAP
) / 3.0
DRAWER_FRONT_W = CABINET_WIDTH - 2.0 * PANEL_T - 0.008
DRAWER_FRONT_T = 0.024
DRAWER_DEPTH = 0.505
DRAWER_TRAY_W = 0.284
DRAWER_TRAY_H = 0.145
DRAWER_TRAVEL = 0.380


def _drawer_centers() -> list[tuple[str, float]]:
    lower = FRONT_MARGIN_BOTTOM + DRAWER_FRONT_H / 2.0
    middle = lower + DRAWER_FRONT_H + FRONT_GAP
    upper = middle + DRAWER_FRONT_H + FRONT_GAP
    return [("lower", lower), ("middle", middle), ("upper", upper)]


def _add_cabinet_body(model: ArticulatedObject):
    cabinet = model.part("cabinet")

    # Structural carcass: a narrow, under-counter cabinet shell with a real open
    # front, side walls, top/bottom panels, and a back panel.
    cabinet.visual(
        Box((CABINET_DEPTH, PANEL_T, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(-CABINET_DEPTH / 2.0, CABINET_WIDTH / 2.0 - PANEL_T / 2.0, CABINET_HEIGHT / 2.0)
        ),
        material="painted_white",
        name="side_panel_0",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, PANEL_T, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(-CABINET_DEPTH / 2.0, -CABINET_WIDTH / 2.0 + PANEL_T / 2.0, CABINET_HEIGHT / 2.0)
        ),
        material="painted_white",
        name="side_panel_1",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, PANEL_T)),
        origin=Origin(xyz=(-CABINET_DEPTH / 2.0, 0.0, PANEL_T / 2.0)),
        material="painted_white",
        name="bottom_panel",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, PANEL_T)),
        origin=Origin(xyz=(-CABINET_DEPTH / 2.0, 0.0, CABINET_HEIGHT - PANEL_T / 2.0)),
        material="painted_white",
        name="top_panel",
    )
    cabinet.visual(
        Box((PANEL_T, CABINET_WIDTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_DEPTH + PANEL_T / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material="painted_white",
        name="back_panel",
    )

    # Front face-frame strips keep the open carcass visually connected while
    # leaving clear, separated drawer reveals.
    front_strip_x = -0.010
    cabinet.visual(
        Box((0.020, PANEL_T, CABINET_HEIGHT)),
        origin=Origin(xyz=(front_strip_x, CABINET_WIDTH / 2.0 - PANEL_T / 2.0, CABINET_HEIGHT / 2.0)),
        material="painted_white",
        name="front_stile_0",
    )
    cabinet.visual(
        Box((0.020, PANEL_T, CABINET_HEIGHT)),
        origin=Origin(xyz=(front_strip_x, -CABINET_WIDTH / 2.0 + PANEL_T / 2.0, CABINET_HEIGHT / 2.0)),
        material="painted_white",
        name="front_stile_1",
    )
    for z in (
        FRONT_MARGIN_BOTTOM - 0.009,
        FRONT_MARGIN_BOTTOM + DRAWER_FRONT_H + FRONT_GAP / 2.0,
        FRONT_MARGIN_BOTTOM + 2.0 * DRAWER_FRONT_H + 1.5 * FRONT_GAP,
        CABINET_HEIGHT - FRONT_MARGIN_TOP + 0.009,
    ):
        cabinet.visual(
            Box((0.020, CABINET_WIDTH, 0.010)),
            origin=Origin(xyz=(front_strip_x, 0.0, z)),
            material="painted_white",
            name=f"front_reveal_{z:.3f}",
        )

    # Recessed black toe-kick, typical of an under-counter base unit.
    cabinet.visual(
        Box((0.028, CABINET_WIDTH - 0.055, 0.055)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0275)),
        material="shadow_black",
        name="toe_kick",
    )

    # Stationary halves of the soft-close guide rails, mounted to the inner side
    # walls.  Small damper capsules and end bumpers communicate the soft-close
    # mechanism instead of using plain rails.
    rail_len = DRAWER_DEPTH
    fixed_rail_y = CABINET_WIDTH / 2.0 - PANEL_T - 0.005
    for level, zc in _drawer_centers():
        rail_z = zc - 0.028
        for idx, sign in enumerate((1.0, -1.0)):
            y = sign * fixed_rail_y
            cabinet.visual(
                Box((rail_len, 0.010, 0.026)),
                origin=Origin(xyz=(-rail_len / 2.0, y, rail_z)),
                material="rail_steel",
                name=f"{level}_fixed_rail_{idx}",
            )
            cabinet.visual(
                Box((0.022, 0.012, 0.032)),
                origin=Origin(xyz=(-0.065, y, rail_z)),
                material="soft_close_black",
                name=f"{level}_rail_bumper_{idx}",
            )
            cabinet.visual(
                Cylinder(radius=0.0045, length=0.155),
                origin=Origin(
                    xyz=(-0.390, sign * (fixed_rail_y + 0.0005), rail_z + 0.009),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material="damper_grey",
                name=f"{level}_damper_{idx}",
            )

    return cabinet


def _add_drawer(model: ArticulatedObject, cabinet, level: str, z_center: float):
    drawer = model.part(f"{level}_drawer")

    front_min_x = 0.002
    front_center_x = front_min_x + DRAWER_FRONT_T / 2.0
    slot_w = 0.220
    slot_h = 0.035
    slot_z = DRAWER_FRONT_H / 2.0 - 0.045
    top_h = DRAWER_FRONT_H / 2.0 - (slot_z + slot_h / 2.0)
    bottom_h = (slot_z - slot_h / 2.0) + DRAWER_FRONT_H / 2.0
    cap_w = (DRAWER_FRONT_W - slot_w) / 2.0

    # The drawer front is built as connected rails around a real horizontal
    # recessed pull opening rather than a flat painted rectangle.
    drawer.visual(
        Box((DRAWER_FRONT_T, DRAWER_FRONT_W, bottom_h)),
        origin=Origin(
            xyz=(
                front_center_x,
                0.0,
                -DRAWER_FRONT_H / 2.0 + bottom_h / 2.0,
            )
        ),
        material="warm_white_lacquer",
        name="front_lower_panel",
    )
    drawer.visual(
        Box((DRAWER_FRONT_T, DRAWER_FRONT_W, top_h + 0.001)),
        origin=Origin(
            xyz=(
                front_center_x,
                0.0,
                DRAWER_FRONT_H / 2.0 - top_h / 2.0,
            )
        ),
        material="warm_white_lacquer",
        name="front_top_rail",
    )
    for idx, sign in enumerate((1.0, -1.0)):
        drawer.visual(
            Box((DRAWER_FRONT_T, cap_w + 0.001, slot_h + 0.002)),
            origin=Origin(
                xyz=(
                    front_center_x,
                    sign * (slot_w / 2.0 + cap_w / 2.0),
                    slot_z,
                )
            ),
            material="warm_white_lacquer",
            name=f"front_side_rail_{idx}",
        )

    # Recess pocket and integral bar lip inside the pull cutout.
    drawer.visual(
        Box((0.007, slot_w, slot_h)),
        origin=Origin(xyz=(-0.0015, 0.0, slot_z)),
        material="recess_shadow",
        name="handle_recess",
    )
    drawer.visual(
        Box((0.012, slot_w, 0.010)),
        origin=Origin(xyz=(0.014, 0.0, slot_z + slot_h / 2.0 - 0.005)),
        material="brushed_aluminum",
        name="handle_bar",
    )

    # Open-topped drawer tray, with the same depth on all three drawers.
    wall_t = 0.012
    bottom_t = 0.012
    tray_side_z = -0.020
    tray_bottom_z = tray_side_z - DRAWER_TRAY_H / 2.0 + bottom_t / 2.0
    rail_y = DRAWER_TRAY_W / 2.0 + 0.005
    drawer.visual(
        Box((DRAWER_DEPTH, DRAWER_TRAY_W, bottom_t)),
        origin=Origin(xyz=(-DRAWER_DEPTH / 2.0, 0.0, tray_bottom_z)),
        material="drawer_grey",
        name="drawer_floor",
    )
    for idx, sign in enumerate((1.0, -1.0)):
        drawer.visual(
            Box((DRAWER_DEPTH, wall_t, DRAWER_TRAY_H)),
            origin=Origin(
                xyz=(-DRAWER_DEPTH / 2.0, sign * (DRAWER_TRAY_W / 2.0 - wall_t / 2.0), tray_side_z)
            ),
            material="drawer_grey",
            name=f"drawer_side_{idx}",
        )
        drawer.visual(
            Box((DRAWER_DEPTH, 0.010, 0.022)),
            origin=Origin(xyz=(-DRAWER_DEPTH / 2.0, sign * rail_y, tray_side_z - 0.008)),
            material="rail_steel",
            name=f"moving_rail_{idx}",
        )
    drawer.visual(
        Box((0.014, DRAWER_TRAY_W, DRAWER_TRAY_H)),
        origin=Origin(xyz=(-DRAWER_DEPTH + 0.007, 0.0, tray_side_z)),
        material="drawer_grey",
        name="drawer_back",
    )
    drawer.visual(
        Box((0.020, DRAWER_TRAY_W, DRAWER_TRAY_H * 0.55)),
        origin=Origin(xyz=(-0.008, 0.0, tray_side_z - 0.018)),
        material="drawer_grey",
        name="front_inner_wall",
    )

    model.articulation(
        f"cabinet_to_{level}_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, z_center)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DRAWER_TRAVEL, effort=55.0, velocity=0.32),
    )

    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_under_counter_drawer_unit")
    model.material("painted_white", rgba=(0.92, 0.92, 0.88, 1.0))
    model.material("warm_white_lacquer", rgba=(0.96, 0.95, 0.90, 1.0))
    model.material("drawer_grey", rgba=(0.64, 0.65, 0.64, 1.0))
    model.material("rail_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.76, 1.0))
    model.material("soft_close_black", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("shadow_black", rgba=(0.015, 0.014, 0.012, 1.0))
    model.material("recess_shadow", rgba=(0.04, 0.04, 0.035, 1.0))
    model.material("damper_grey", rgba=(0.46, 0.48, 0.50, 1.0))

    cabinet = _add_cabinet_body(model)
    for level, z_center in _drawer_centers():
        _add_drawer(model, cabinet, level, z_center)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")

    centers = _drawer_centers()
    ctx.check("three equal-depth drawers", len(centers) == 3)

    closed_positions = {}
    for level, _ in centers:
        drawer = object_model.get_part(f"{level}_drawer")
        joint = object_model.get_articulation(f"cabinet_to_{level}_drawer")
        limits = joint.motion_limits
        ctx.check(
            f"{level} drawer has soft-close prismatic travel",
            limits is not None
            and abs((limits.upper or 0.0) - DRAWER_TRAVEL) < 1e-6
            and abs((limits.lower or 0.0)) < 1e-6,
        )

        # Closed fronts sit just proud of the cabinet face-frame and remain in
        # the cabinet opening.
        ctx.expect_gap(
            drawer,
            cabinet,
            axis="x",
            positive_elem="front_lower_panel",
            negative_elem="front_stile_0",
            min_gap=0.001,
            max_gap=0.004,
            name=f"{level} front is proud of frame",
        )
        ctx.expect_within(
            drawer,
            cabinet,
            axes="y",
            inner_elem="front_lower_panel",
            outer_elem="top_panel",
            margin=0.0,
            name=f"{level} front fits cabinet width",
        )

        # The side-mounted guide rails are adjacent but not fused; at full
        # extension they still retain a meaningful insertion length in the case.
        for idx in (0, 1):
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="x",
                elem_a=f"moving_rail_{idx}",
                elem_b=f"{level}_fixed_rail_{idx}",
                min_overlap=0.45,
                name=f"{level} rail {idx} retained when closed",
            )

        closed_positions[level] = ctx.part_world_position(drawer)
        with ctx.pose({joint: DRAWER_TRAVEL}):
            open_position = ctx.part_world_position(drawer)
            ctx.check(
                f"{level} drawer extends outward",
                closed_positions[level] is not None
                and open_position is not None
                and open_position[0] > closed_positions[level][0] + DRAWER_TRAVEL * 0.95,
                details=f"closed={closed_positions[level]}, open={open_position}",
            )
            for idx in (0, 1):
                ctx.expect_overlap(
                    drawer,
                    cabinet,
                    axes="x",
                    elem_a=f"moving_rail_{idx}",
                    elem_b=f"{level}_fixed_rail_{idx}",
                    min_overlap=0.10,
                    name=f"{level} rail {idx} retained when extended",
                )

    return ctx.report()


object_model = build_object_model()
