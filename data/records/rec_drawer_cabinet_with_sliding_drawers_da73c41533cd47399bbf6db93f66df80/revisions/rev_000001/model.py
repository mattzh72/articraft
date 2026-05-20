from __future__ import annotations

from math import pi

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


CABINET_WIDTH = 0.22
CABINET_DEPTH = 0.18
CABINET_HEIGHT = 0.43

SIDE_WALL = 0.010
BACK_WALL = 0.006
BOTTOM_PANEL = 0.012
DIVIDER_T = 0.006

TRAY_HEIGHT = 0.065
TRAY_FLOOR = 0.008
TRAY_FELT_T = 0.002

DRAWER_COUNT = 5
DRAWER_STACK_TOP = CABINET_HEIGHT - TRAY_HEIGHT - TRAY_FLOOR
DRAWER_SLOT_H = (DRAWER_STACK_TOP - BOTTOM_PANEL - (DRAWER_COUNT - 1) * DIVIDER_T) / DRAWER_COUNT

DRAWER_FRONT_T = 0.012
DRAWER_FRONT_W = CABINET_WIDTH - 2.0 * SIDE_WALL - 0.006
DRAWER_FRONT_H = DRAWER_SLOT_H - 0.004
DRAWER_BOX_W = CABINET_WIDTH - 2.0 * SIDE_WALL - 0.014
DRAWER_BOX_H = DRAWER_SLOT_H - 0.012
DRAWER_BOX_D = 0.155
DRAWER_SIDE_T = 0.006
DRAWER_BOTTOM_T = 0.004
DRAWER_BACK_T = 0.005
DRAWER_TRAVEL = 0.075
DRAWER_LINER_T = 0.0015

GUIDE_LENGTH = 0.090
GUIDE_WIDTH = 0.013
GUIDE_HEIGHT = 0.005
GUIDE_CENTER_X = 0.025

KNOB_RADIUS = 0.0075
KNOB_BASE_RADIUS = 0.010
KNOB_LENGTH = 0.010
KNOB_BASE_LENGTH = 0.0025

LID_THICKNESS = 0.012
HINGE_RADIUS = 0.005
LID_FRONT_OVERHANG = 0.006
LID_SIDE_OVERHANG = 0.002
LID_REAR_SETBACK = 0.004
LID_OPEN_LIMIT = 1.18


def _box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(xyz)


def _y_cylinder(radius: float, length: float, xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((xyz[0], xyz[1] - length / 2.0, xyz[2]))


def _drawer_slot_bottom(index: int) -> float:
    return BOTTOM_PANEL + index * (DRAWER_SLOT_H + DIVIDER_T)


def _drawer_center_z(index: int) -> float:
    return _drawer_slot_bottom(index) + DRAWER_SLOT_H / 2.0


def _drawer_shape() -> cq.Workplane:
    front_panel = _box((DRAWER_FRONT_T, DRAWER_FRONT_W, DRAWER_FRONT_H), (DRAWER_FRONT_T / 2.0, 0.0, 0.0))
    shell_left = _box(
        (DRAWER_BOX_D, DRAWER_SIDE_T, DRAWER_BOX_H),
        (-DRAWER_BOX_D / 2.0, -(DRAWER_BOX_W - DRAWER_SIDE_T) / 2.0, 0.0),
    )
    shell_right = _box(
        (DRAWER_BOX_D, DRAWER_SIDE_T, DRAWER_BOX_H),
        (-DRAWER_BOX_D / 2.0, (DRAWER_BOX_W - DRAWER_SIDE_T) / 2.0, 0.0),
    )
    shell_back = _box(
        (DRAWER_BACK_T, DRAWER_BOX_W - 2.0 * DRAWER_SIDE_T, DRAWER_BOX_H),
        (-(DRAWER_BOX_D - DRAWER_BACK_T / 2.0), 0.0, 0.0),
    )
    shell_bottom = _box(
        (DRAWER_BOX_D, DRAWER_BOX_W - 2.0 * DRAWER_SIDE_T, DRAWER_BOTTOM_T),
        (-DRAWER_BOX_D / 2.0, 0.0, -DRAWER_BOX_H / 2.0 + DRAWER_BOTTOM_T / 2.0),
    )

    drawer = front_panel.union(shell_left).union(shell_right).union(shell_back).union(shell_bottom)
    return drawer


def _lid_shape() -> cq.Workplane:
    lid_width = CABINET_WIDTH + 2.0 * LID_SIDE_OVERHANG
    lid_depth = CABINET_DEPTH + LID_FRONT_OVERHANG - LID_REAR_SETBACK
    panel_center_x = LID_REAR_SETBACK + lid_depth / 2.0
    panel_center_z = LID_THICKNESS / 2.0 - HINGE_RADIUS

    panel = _box((lid_depth, lid_width, LID_THICKNESS), (panel_center_x, 0.0, panel_center_z))
    hinge_leaf = _box((0.012, CABINET_WIDTH - 0.020, 0.004), (0.006, 0.0, -0.002))
    hinge_barrel = _y_cylinder(HINGE_RADIUS, CABINET_WIDTH - 0.020, (0.0, 0.0, 0.0))
    return panel.union(hinge_leaf).union(hinge_barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jewelry_drawer_cabinet")

    model.material("walnut", rgba=(0.42, 0.28, 0.18, 1.0))
    model.material("walnut_dark", rgba=(0.34, 0.22, 0.14, 1.0))
    model.material("felt_burgundy", rgba=(0.44, 0.10, 0.16, 1.0))
    model.material("antique_brass", rgba=(0.78, 0.67, 0.39, 1.0))

    body = model.part("body")
    inner_width = CABINET_WIDTH - 2.0 * SIDE_WALL
    shelf_depth = CABINET_DEPTH - BACK_WALL

    body.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, BOTTOM_PANEL)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_PANEL / 2.0)),
        material="walnut",
        name="bottom_panel",
    )
    for side_index, y_pos in enumerate((-(CABINET_WIDTH - SIDE_WALL) / 2.0, (CABINET_WIDTH - SIDE_WALL) / 2.0)):
        body.visual(
            Box((CABINET_DEPTH, SIDE_WALL, CABINET_HEIGHT)),
            origin=Origin(xyz=(0.0, y_pos, CABINET_HEIGHT / 2.0)),
            material="walnut",
            name=f"side_wall_{side_index}",
        )
    body.visual(
        Box((BACK_WALL, inner_width, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_DEPTH / 2.0 + BACK_WALL / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material="walnut",
        name="back_wall",
    )
    body.visual(
        Box((shelf_depth, inner_width, TRAY_FLOOR)),
        origin=Origin(xyz=(BACK_WALL / 2.0, 0.0, DRAWER_STACK_TOP + TRAY_FLOOR / 2.0)),
        material="walnut",
        name="tray_floor",
    )
    body.visual(
        Box((SIDE_WALL, inner_width, TRAY_HEIGHT)),
        origin=Origin(xyz=(CABINET_DEPTH / 2.0 - SIDE_WALL / 2.0, 0.0, CABINET_HEIGHT - TRAY_HEIGHT / 2.0)),
        material="walnut",
        name="tray_lip",
    )
    for divider_index in range(DRAWER_COUNT - 1):
        divider_center_z = _drawer_slot_bottom(divider_index) + DRAWER_SLOT_H + DIVIDER_T / 2.0
        body.visual(
            Box((shelf_depth, inner_width, DIVIDER_T)),
            origin=Origin(xyz=(BACK_WALL / 2.0, 0.0, divider_center_z)),
            material="walnut",
            name=f"divider_{divider_index}",
        )
    guide_y = inner_width / 2.0 - GUIDE_WIDTH / 2.0
    for slot_index in range(DRAWER_COUNT):
        guide_center_z = _drawer_slot_bottom(slot_index) + GUIDE_HEIGHT / 2.0
        for side_index, y_sign in enumerate((-1.0, 1.0)):
            body.visual(
                Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
                origin=Origin(xyz=(GUIDE_CENTER_X, y_sign * guide_y, guide_center_z)),
                material="walnut_dark",
                name=f"guide_{slot_index}_{side_index}",
            )
    body.visual(
        Box((CABINET_DEPTH - BACK_WALL - SIDE_WALL - 0.014, CABINET_WIDTH - 2.0 * SIDE_WALL - 0.014, TRAY_FELT_T)),
        origin=Origin(
            xyz=(
                (CABINET_DEPTH / 2.0 - SIDE_WALL + (-CABINET_DEPTH / 2.0 + BACK_WALL)) / 2.0,
                0.0,
                CABINET_HEIGHT - TRAY_HEIGHT + TRAY_FELT_T / 2.0,
            )
        ),
        material="felt_burgundy",
        name="tray_liner",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shape(), "lid_shell"), material="walnut_dark", name="lid_shell")

    for drawer_index in range(DRAWER_COUNT):
        drawer = model.part(f"drawer_{drawer_index}")
        drawer.visual(
            mesh_from_cadquery(_drawer_shape(), f"drawer_shell_{drawer_index}"),
            material="walnut_dark",
            name="drawer_shell",
        )
        drawer.visual(
            Cylinder(radius=KNOB_BASE_RADIUS, length=KNOB_BASE_LENGTH),
            origin=Origin(
                xyz=(DRAWER_FRONT_T + KNOB_BASE_LENGTH / 2.0 - 0.0005, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="antique_brass",
            name="knob_base",
        )
        drawer.visual(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
            origin=Origin(
                xyz=(DRAWER_FRONT_T + KNOB_LENGTH / 2.0 + 0.0008, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="antique_brass",
            name="knob",
        )
        drawer.visual(
            Box((DRAWER_BOX_D - 0.014, DRAWER_BOX_W - 2.0 * DRAWER_SIDE_T - 0.008, DRAWER_LINER_T)),
            origin=Origin(
                xyz=(
                    -DRAWER_BOX_D / 2.0,
                    0.0,
                    -DRAWER_BOX_H / 2.0 + DRAWER_BOTTOM_T + DRAWER_LINER_T / 2.0,
                )
            ),
            material="felt_burgundy",
            name="drawer_liner",
        )

        model.articulation(
            f"drawer_slide_{drawer_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(CABINET_DEPTH / 2.0 - DRAWER_FRONT_T, 0.0, _drawer_center_z(drawer_index) - 0.001)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=DRAWER_TRAVEL, effort=30.0, velocity=0.20),
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-CABINET_DEPTH / 2.0 - 0.002, 0.0, CABINET_HEIGHT + HINGE_RADIUS)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_OPEN_LIMIT, effort=8.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    body_shell_aabb = ctx.part_world_aabb(body)
    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    for drawer_index in range(DRAWER_COUNT):
        drawer = object_model.get_part(f"drawer_{drawer_index}")
        slide = object_model.get_articulation(f"drawer_slide_{drawer_index}")
        drawer_shell_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_shell")

        ctx.check(
            f"drawer_{drawer_index} front sits flush with the carcass",
            body_shell_aabb is not None
            and drawer_shell_aabb is not None
            and abs(drawer_shell_aabb[1][0] - body_shell_aabb[1][0]) <= 0.0015,
            details=f"body={body_shell_aabb}, drawer={drawer_shell_aabb}",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            inner_elem="drawer_shell",
            margin=0.0,
            name=f"drawer_{drawer_index} stays centered in its slot",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: DRAWER_TRAVEL}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="drawer_shell",
                min_overlap=0.085,
                name=f"drawer_{drawer_index} remains captured on its guides",
            )
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="drawer_shell",
                margin=0.0,
                name=f"drawer_{drawer_index} stays aligned when extended",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer_{drawer_index} extends forward",
            rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.05,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        min_overlap=0.17,
        name="lid covers the tray opening when closed",
    )

    ctx.check(
        "lid seats on the cabinet top",
        body_shell_aabb is not None
        and closed_lid_aabb is not None
        and abs(closed_lid_aabb[0][2] - body_shell_aabb[1][2]) <= 0.0015,
        details=f"body={body_shell_aabb}, lid={closed_lid_aabb}",
    )

    with ctx.pose({lid_hinge: LID_OPEN_LIMIT}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    ctx.check(
        "lid opens upward above the tray",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and body_shell_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10
        and open_lid_aabb[1][2] > body_shell_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}, body={body_shell_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
