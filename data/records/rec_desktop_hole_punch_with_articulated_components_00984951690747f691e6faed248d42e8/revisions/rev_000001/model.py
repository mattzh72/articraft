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

HOLE_XS = (-0.108, 0.0, 0.108)
PLATE_THICKNESS = 0.004


def _base_plate_shape():
    plate = (
        cq.Workplane("XY")
        .box(0.34, 0.22, PLATE_THICKNESS)
        .translate((0.0, 0.0, PLATE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.018)
    )
    for x_pos in HOLE_XS:
        cutter = cq.Workplane("XY").circle(0.0066).extrude(0.012).translate((x_pos, 0.005, 0.0))
        plate = plate.cut(cutter)
    return plate


def _die_block_shape():
    die_block = (
        cq.Workplane("XY")
        .box(0.27, 0.082, 0.022)
        .translate((0.0, 0.032, 0.015))
        .edges("|Z")
        .fillet(0.007)
    )
    for x_pos in HOLE_XS:
        cutter = cq.Workplane("XY").circle(0.0066).extrude(0.030).translate((x_pos, 0.005, 0.0))
        die_block = die_block.cut(cutter)
    return die_block


def _lever_shape():
    lever_body = (
        cq.Workplane("XY")
        .box(0.25, 0.165, 0.012)
        .translate((0.0, -0.0825, 0.0))
        .edges("|Z")
        .fillet(0.012)
    )
    grip_pad = (
        cq.Workplane("XY")
        .box(0.20, 0.060, 0.018)
        .translate((0.0, -0.150, 0.003))
        .edges("|Z")
        .fillet(0.010)
    )
    hinge_barrel = cq.Workplane("YZ").circle(0.007).extrude(0.27).translate((-0.135, 0.0, 0.0))
    strut_0 = cq.Workplane("XY").box(0.030, 0.050, 0.014).translate((-0.090, -0.028, -0.002))
    strut_1 = cq.Workplane("XY").box(0.030, 0.050, 0.014).translate((0.090, -0.028, -0.002))
    return lever_body.union(grip_pad).union(hinge_barrel).union(strut_0).union(strut_1)


def _fence_shape():
    fence = (
        cq.Workplane("XY")
        .box(0.21, 0.016, 0.018)
        .edges("|Z")
        .fillet(0.003)
    )
    for x_pos in (-0.065, 0.065):
        slot = cq.Workplane("XY").box(0.012, 0.009, 0.022).translate((x_pos, 0.0, 0.0))
        fence = fence.cut(slot)
    return fence


def _add_button(model: ArticulatedObject, fence_part, name: str, x_offset: float):
    button = model.part(name)
    button.visual(
        Box((0.009, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material="button_dark",
        name="button_stem",
    )
    button.visual(
        Box((0.018, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material="button_red",
        name="button_cap",
    )

    joint = model.articulation(
        f"{name}_press",
        ArticulationType.PRISMATIC,
        parent=fence_part,
        child=button,
        origin=Origin(xyz=(x_offset, 0.0, 0.009)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.001),
    )
    return button, joint


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_hole_punch")

    model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("steel_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("fence_black", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("button_dark", rgba=(0.22, 0.22, 0.23, 1.0))
    model.material("button_red", rgba=(0.70, 0.18, 0.16, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_plate_shape(), "base_plate"), material="steel", name="base_plate")
    base.visual(mesh_from_cadquery(_die_block_shape(), "die_block"), material="steel", name="die_block")
    base.visual(
        Box((0.30, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.090, 0.011)),
        material="steel_dark",
        name="front_track",
    )
    for idx, x_pos in enumerate((-0.145, 0.145)):
        base.visual(
            Box((0.020, 0.018, 0.040)),
            origin=Origin(xyz=(x_pos, 0.096, 0.024)),
            material="steel",
            name=f"hinge_post_{idx}",
        )

    lever = model.part("lever")
    lever.visual(mesh_from_cadquery(_lever_shape(), "lever_shell"), material="steel", name="lever_shell")
    for idx, x_pos in enumerate(HOLE_XS):
        lever.visual(
            Cylinder(radius=0.0055, length=0.008),
            origin=Origin(xyz=(x_pos, -0.095, -0.010)),
            material="steel_dark",
            name=f"punch_{idx}",
        )

    fence = model.part("fence")
    fence.visual(mesh_from_cadquery(_fence_shape(), "fence_body"), material="fence_black", name="fence_body")

    model.articulation(
        "lever_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(0.0, 0.100, 0.042)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(67.0),
        ),
    )
    model.articulation(
        "fence_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fence,
        origin=Origin(xyz=(0.0, -0.103, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=-0.055, upper=0.055),
    )

    _add_button(model, fence, "button_0", -0.065)
    _add_button(model, fence, "button_1", 0.065)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lever = object_model.get_part("lever")
    fence = object_model.get_part("fence")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    lever_hinge = object_model.get_articulation("lever_hinge")
    fence_slide = object_model.get_articulation("fence_slide")
    button_0_press = object_model.get_articulation("button_0_press")
    button_1_press = object_model.get_articulation("button_1_press")

    ctx.expect_gap(
        lever,
        base,
        axis="z",
        positive_elem="punch_1",
        negative_elem="die_block",
        min_gap=0.001,
        max_gap=0.004,
        name="center punch hovers above the die block at rest",
    )
    ctx.expect_gap(
        base,
        fence,
        axis="y",
        positive_elem="front_track",
        negative_elem="fence_body",
        min_gap=0.0,
        max_gap=0.001,
        name="fence rides directly on the front track",
    )
    ctx.expect_overlap(
        base,
        fence,
        axes="xz",
        elem_a="front_track",
        elem_b="fence_body",
        min_overlap=0.010,
        name="fence stays engaged with the front track at rest",
    )
    ctx.expect_gap(
        button_0,
        fence,
        axis="z",
        positive_elem="button_cap",
        negative_elem="fence_body",
        min_gap=0.0,
        max_gap=0.0005,
        name="button 0 cap seats on the fence top at rest",
    )
    ctx.expect_gap(
        button_1,
        fence,
        axis="z",
        positive_elem="button_cap",
        negative_elem="fence_body",
        min_gap=0.0,
        max_gap=0.0005,
        name="button 1 cap seats on the fence top at rest",
    )

    closed_aabb = ctx.part_world_aabb(lever)
    lever_open = lever_hinge.motion_limits.upper if lever_hinge.motion_limits is not None else 0.0
    with ctx.pose({lever_hinge: lever_open}):
        open_aabb = ctx.part_world_aabb(lever)
        ctx.expect_gap(
            lever,
            base,
            axis="z",
            positive_elem="punch_1",
            negative_elem="die_block",
            min_gap=0.060,
            name="center punch lifts well clear when the lever opens",
        )
    ctx.check(
        "lever front lifts when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.055,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    slide_open = fence_slide.motion_limits.upper if fence_slide.motion_limits is not None else 0.0
    fence_rest_pos = ctx.part_world_position(fence)
    with ctx.pose({fence_slide: slide_open}):
        fence_open_pos = ctx.part_world_position(fence)
        ctx.expect_overlap(
            base,
            fence,
            axes="x",
            elem_a="front_track",
            elem_b="fence_body",
            min_overlap=0.090,
            name="fence keeps retained overlap with the front track at max travel",
        )
    ctx.check(
        "fence slides toward +x",
        fence_rest_pos is not None
        and fence_open_pos is not None
        and fence_open_pos[0] > fence_rest_pos[0] + 0.04,
        details=f"rest={fence_rest_pos}, open={fence_open_pos}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    button_travel = button_0_press.motion_limits.upper if button_0_press.motion_limits is not None else 0.0
    with ctx.pose({button_0_press: button_travel}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_held = ctx.part_world_position(button_1)
    ctx.check(
        "button 0 depresses independently",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_held is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.0008
        and abs(button_1_held[2] - button_1_rest[2]) < 1e-4,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_held={button_1_held}"
        ),
    )

    with ctx.pose({button_1_press: button_travel}):
        button_0_held = ctx.part_world_position(button_0)
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "button 1 depresses independently",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_held is not None
        and button_1_pressed is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.0008
        and abs(button_0_held[2] - button_0_rest[2]) < 1e-4,
        details=(
            f"button_0_rest={button_0_rest}, button_0_held={button_0_held}, "
            f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
