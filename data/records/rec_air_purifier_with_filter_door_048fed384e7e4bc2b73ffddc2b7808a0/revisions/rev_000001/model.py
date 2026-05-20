from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.24
BODY_D = 0.18
BODY_H = 0.42
WALL = 0.0045

DOOR_W = 0.212
DOOR_H = 0.332
DOOR_T = 0.010
DOOR_Z = -0.005

FILTER_W = 0.188
FILTER_H = 0.156
FILTER_D = 0.104
FILTER_Z = -0.082
FILTER_TRAVEL = 0.076

HANDLE_RECESS_W = 0.190
HANDLE_RECESS_D = 0.062
HANDLE_RECESS_H = 0.034
HANDLE_BAR_W = 0.148
HANDLE_BAR_D = 0.032
HANDLE_BAR_T = 0.012
HANDLE_DROP = 0.016
HANDLE_PIVOT_SPAN = 0.174
HANDLE_PIVOT_LEN = 0.016
HANDLE_PIVOT_R = 0.005


def _build_shell_shape() -> cq.Workplane:
    def panel(size_x: float, size_y: float, size_z: float, xyz: tuple[float, float, float]) -> cq.Workplane:
        return cq.Workplane("XY").box(size_x, size_y, size_z).translate(xyz)

    front_y = BODY_D / 2.0 - WALL / 2.0
    back_y = -BODY_D / 2.0 + WALL / 2.0
    side_x = BODY_W / 2.0 - WALL / 2.0
    bottom_t = 0.012
    top_t = 0.008
    bottom_z = -BODY_H / 2.0 + bottom_t / 2.0
    top_z = BODY_H / 2.0 - top_t / 2.0

    open_w = DOOR_W - 0.016
    open_h = DOOR_H - 0.016
    side_frame_w = (BODY_W - open_w) / 2.0
    top_frame_h = BODY_H / 2.0 - (DOOR_Z + open_h / 2.0)
    bottom_frame_h = (DOOR_Z - open_h / 2.0) + BODY_H / 2.0

    shell = panel(BODY_W, WALL, BODY_H, (0.0, back_y, 0.0))
    shell = shell.union(panel(WALL, BODY_D, BODY_H, (-side_x, 0.0, 0.0)))
    shell = shell.union(panel(WALL, BODY_D, BODY_H, (side_x, 0.0, 0.0)))
    shell = shell.union(panel(BODY_W, BODY_D, bottom_t, (0.0, 0.0, bottom_z)))

    top_panel = panel(BODY_W, BODY_D, top_t, (0.0, 0.0, top_z))
    handle_pocket = panel(HANDLE_RECESS_W, HANDLE_RECESS_D, HANDLE_RECESS_H, (0.0, 0.0, BODY_H / 2.0 - HANDLE_RECESS_H / 2.0 + 0.002))
    shell = shell.union(top_panel.cut(handle_pocket))

    shell = shell.union(panel(side_frame_w, WALL, open_h, (-(open_w / 2.0 + side_frame_w / 2.0), front_y, DOOR_Z)))
    shell = shell.union(panel(side_frame_w, WALL, open_h, (open_w / 2.0 + side_frame_w / 2.0, front_y, DOOR_Z)))
    shell = shell.union(panel(BODY_W, WALL, top_frame_h, (0.0, front_y, (BODY_H / 2.0 + DOOR_Z + open_h / 2.0) / 2.0)))
    shell = shell.union(panel(BODY_W, WALL, bottom_frame_h, (0.0, front_y, (-BODY_H / 2.0 + DOOR_Z - open_h / 2.0) / 2.0)))

    guide_rail = panel(0.024, 0.096, 0.010, (0.0, 0.0, 0.0))
    rail_y = 0.018
    rail_z = FILTER_Z - FILTER_H / 2.0 + 0.006
    shell = shell.union(guide_rail.translate((0.106, rail_y, rail_z)))
    shell = shell.union(guide_rail.translate((-0.106, rail_y, rail_z)))

    return shell.combine()


def _build_door_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H, centered=(False, True, True))

    inset = cq.Workplane("XY").box(DOOR_W - 0.030, 0.0035, DOOR_H - 0.060, centered=(False, True, True))
    inset = inset.translate((0.015, DOOR_T / 2.0 - 0.00175, 0.0))
    door = panel.cut(inset)

    finger_pull = cq.Workplane("XY").box(0.016, DOOR_T * 0.75, 0.074)
    finger_pull = finger_pull.translate((DOOR_W - 0.009, DOOR_T * 0.12, 0.0))
    door = door.cut(finger_pull)

    hinge_barrel = cq.Workplane("XY").circle(0.0045).extrude(0.060)
    upper_barrel = hinge_barrel.translate((0.0, DOOR_T / 2.0 + 0.003, 0.094))
    lower_barrel = hinge_barrel.translate((0.0, DOOR_T / 2.0 + 0.003, -0.154))
    return door.union(upper_barrel).union(lower_barrel).combine()


def _build_filter_shape() -> cq.Workplane:
    front_frame = cq.Workplane("XY").box(FILTER_W, 0.012, FILTER_H)
    media = cq.Workplane("XY").box(FILTER_W - 0.018, FILTER_D, FILTER_H - 0.020)
    media = media.translate((0.0, -FILTER_D / 2.0 - 0.004, 0.0))

    pull_tab = cq.Workplane("XY").box(0.060, 0.012, 0.024)
    pull_tab = pull_tab.translate((0.0, 0.011, 0.0))

    return front_frame.union(media).union(pull_tab).combine()


def _build_handle_shape() -> cq.Workplane:
    grip = cq.Workplane("XY").box(HANDLE_BAR_W, HANDLE_BAR_D, HANDLE_BAR_T)
    grip = grip.translate((0.0, 0.0, -HANDLE_DROP))

    end_lobe = cq.Workplane("XY").box(0.032, HANDLE_BAR_D + 0.006, 0.020)
    left_lobe = end_lobe.translate((-0.068, 0.0, -HANDLE_DROP / 2.0))
    right_lobe = end_lobe.translate((0.068, 0.0, -HANDLE_DROP / 2.0))

    pivot = cq.Workplane("YZ").circle(HANDLE_PIVOT_R).extrude(HANDLE_PIVOT_LEN)
    left_pivot = pivot.translate((-HANDLE_PIVOT_SPAN / 2.0 - HANDLE_PIVOT_LEN / 2.0, 0.0, 0.0))
    right_pivot = pivot.translate((HANDLE_PIVOT_SPAN / 2.0 - HANDLE_PIVOT_LEN / 2.0, 0.0, 0.0))

    return grip.union(left_lobe).union(right_lobe).union(left_pivot).union(right_pivot).combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_purifier")

    shell_white = model.material("shell_white", rgba=(0.92, 0.93, 0.94, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.76, 0.79, 0.76, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_build_shell_shape(), "shell"),
        material=shell_white,
        name="housing",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_shape(), "door"),
        material=shell_white,
        name="panel",
    )

    filter_cartridge = model.part("filter_cartridge")
    filter_cartridge.visual(
        mesh_from_cadquery(_build_filter_shape(), "filter_cartridge"),
        material=filter_gray,
        name="cartridge",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "handle"),
        material=shell_dark,
        name="grip",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=door,
        origin=Origin(xyz=(-DOOR_W / 2.0, BODY_D / 2.0 + DOOR_T / 2.0, DOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=8.0, velocity=1.6),
    )

    model.articulation(
        "filter_slide",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=filter_cartridge,
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - WALL - 0.020, FILTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=FILTER_TRAVEL, effort=25.0, velocity=0.20),
    )

    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0 - WALL - 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.85, effort=6.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    door = object_model.get_part("door")
    filter_cartridge = object_model.get_part("filter_cartridge")
    handle = object_model.get_part("handle")
    door_hinge = object_model.get_articulation("door_hinge")
    filter_slide = object_model.get_articulation("filter_slide")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.allow_overlap(
        handle,
        shell,
        elem_a="grip",
        elem_b="housing",
        reason="The folding carry handle uses captured pivot stubs that are simplified as nesting slightly into the recessed top-shell pocket.",
    )
    ctx.allow_overlap(
        filter_cartridge,
        shell,
        elem_a="cartridge",
        elem_b="housing",
        reason="The lower filter cartridge is represented as sliding inside a simplified purifier shell proxy rather than a fully detailed internal guide sleeve.",
    )

    ctx.expect_gap(
        door,
        shell,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        name="door sits just proud of the shell",
    )
    ctx.expect_overlap(
        door,
        shell,
        axes="xz",
        min_overlap=0.18,
        name="door covers the front opening",
    )

    ctx.expect_within(
        filter_cartridge,
        shell,
        axes="xz",
        margin=0.012,
        name="filter cartridge stays guided inside the shell opening",
    )
    ctx.expect_overlap(
        filter_cartridge,
        shell,
        axes="y",
        min_overlap=0.06,
        name="filter cartridge remains inserted at rest",
    )

    filter_rest = ctx.part_world_position(filter_cartridge)
    with ctx.pose({filter_slide: FILTER_TRAVEL}):
        ctx.expect_within(
            filter_cartridge,
            shell,
            axes="xz",
            margin=0.012,
            name="extended filter cartridge stays laterally guided",
        )
        ctx.expect_overlap(
            filter_cartridge,
            shell,
            axes="y",
            min_overlap=0.028,
            name="extended filter cartridge retains insertion",
        )
        filter_extended = ctx.part_world_position(filter_cartridge)

    ctx.check(
        "filter cartridge slides out the front",
        filter_rest is not None
        and filter_extended is not None
        and filter_extended[1] > filter_rest[1] + 0.05,
        details=f"rest={filter_rest}, extended={filter_extended}",
    )

    door_rest = ctx.part_element_world_aabb(door, elem="panel")
    with ctx.pose({door_hinge: 1.35}):
        door_open = ctx.part_element_world_aabb(door, elem="panel")
    ctx.check(
        "front door swings outward",
        door_rest is not None
        and door_open is not None
        and door_open[1][1] > door_rest[1][1] + 0.10,
        details=f"rest={door_rest}, open={door_open}",
    )

    shell_aabb = ctx.part_world_aabb(shell)
    handle_rest = ctx.part_element_world_aabb(handle, elem="grip")
    with ctx.pose({handle_pivot: 2.70}):
        handle_raised = ctx.part_element_world_aabb(handle, elem="grip")

    ctx.check(
        "folding handle hides inside the top recess at rest",
        shell_aabb is not None
        and handle_rest is not None
        and handle_rest[1][2] < shell_aabb[1][2] + 0.001,
        details=f"shell={shell_aabb}, handle_rest={handle_rest}",
    )
    ctx.check(
        "folding handle lifts above the shell when deployed",
        shell_aabb is not None
        and handle_raised is not None
        and handle_raised[1][2] > shell_aabb[1][2] + 0.015,
        details=f"shell={shell_aabb}, handle_raised={handle_raised}",
    )

    return ctx.report()


object_model = build_object_model()
