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


LOWER_W = 0.58
LOWER_D = 0.36
LOWER_H = 0.28
LOWER_BOTTOM_Z = 0.085
LOWER_CENTER_Z = LOWER_BOTTOM_Z + LOWER_H * 0.5

LID_W = 0.60
LID_D = 0.38
LID_H = 0.08
LID_BOTTOM_Z = 0.365

HANDLE_X = 0.185
HANDLE_Y = 0.206
HANDLE_COLLAPSED_Z = 0.458
HANDLE_TRAVEL = 0.22

POCKET_X = 0.165
POCKET_Y = 0.096
POCKET_HINGE_Y = 0.192
POCKET_HINGE_Z = 0.080

WHEEL_RADIUS = 0.095
WHEEL_WIDTH = 0.050
WHEEL_X = 0.343
WHEEL_Y = 0.145
WHEEL_Z = WHEEL_RADIUS


def _lower_body_shape():
    return (
        cq.Workplane("XY")
        .box(LOWER_W, LOWER_D, LOWER_H)
        .edges("|Z")
        .fillet(0.028)
        .edges(">Z")
        .fillet(0.010)
        .faces(">Z")
        .shell(-0.006)
    )


def _lid_shape():
    lid = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H)
        .edges("|Z")
        .fillet(0.022)
        .edges(">Z")
        .fillet(0.010)
    )

    underside_cavity = cq.Workplane("XY").box(0.53, 0.31, 0.044).translate((0.0, -0.005, -0.018))
    left_pocket = cq.Workplane("XY").box(0.222, 0.192, 0.024).translate((-POCKET_X, POCKET_Y, 0.028))
    right_pocket = cq.Workplane("XY").box(0.222, 0.192, 0.024).translate((POCKET_X, POCKET_Y, 0.028))
    handle_recess = cq.Workplane("XY").box(0.126, 0.060, 0.018).translate((0.0, 0.136, 0.031))

    return lid.cut(underside_cavity).cut(left_pocket).cut(right_pocket).cut(handle_recess)


def _add_cover(part, clear_material) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.220),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_material,
        name="barrel",
    )
    part.visual(
        Box((0.220, 0.182, 0.006)),
        origin=Origin(xyz=(0.0, -0.091, -0.003)),
        material=clear_material,
        name="panel",
    )
    part.visual(
        Box((0.060, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.176, -0.001)),
        material=clear_material,
        name="tab",
    )


def _add_wheel(part, rubber, steel, dark_steel) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.066, length=0.038),
        origin=spin_origin,
        material=dark_steel,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.035, length=0.054),
        origin=spin_origin,
        material=steel,
        name="hub",
    )
    part.visual(
        Box((0.010, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=steel,
        name="lug",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    black_plastic = model.material("black_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    axle_black = model.material("axle_black", rgba=(0.14, 0.14, 0.15, 1.0))
    latch_yellow = model.material("latch_yellow", rgba=(0.91, 0.72, 0.12, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.72, 0.82, 0.88, 0.34))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_cadquery(_lower_body_shape(), "toolbox_lower_body"),
        origin=Origin(xyz=(0.0, 0.0, LOWER_CENTER_Z)),
        material=black_plastic,
        name="shell",
    )
    lower_body.visual(
        Box((0.112, 0.050, LOWER_BOTTOM_Z)),
        origin=Origin(xyz=(-0.155, -0.145, LOWER_BOTTOM_Z * 0.5)),
        material=charcoal,
        name="foot_0",
    )
    lower_body.visual(
        Box((0.112, 0.050, LOWER_BOTTOM_Z)),
        origin=Origin(xyz=(0.155, -0.145, LOWER_BOTTOM_Z * 0.5)),
        material=charcoal,
        name="foot_1",
    )
    for i, x in enumerate((-HANDLE_X, HANDLE_X)):
        lower_body.visual(
            Box((0.036, 0.006, 0.270)),
            origin=Origin(xyz=(x, 0.214, 0.215)),
            material=charcoal,
            name=f"channel_{i}_back",
        )
        lower_body.visual(
            Box((0.006, 0.044, 0.270)),
            origin=Origin(xyz=(x - 0.014, 0.192, 0.215)),
            material=charcoal,
            name=f"channel_{i}_outer",
        )
        lower_body.visual(
            Box((0.006, 0.044, 0.270)),
            origin=Origin(xyz=(x + 0.014, 0.192, 0.215)),
            material=charcoal,
            name=f"channel_{i}_inner",
        )
    for i, x in enumerate((-WHEEL_X + 0.045, WHEEL_X - 0.045)):
        side = -1.0 if i == 0 else 1.0
        lower_body.visual(
            Box((0.026, 0.084, 0.110)),
            origin=Origin(xyz=(x, WHEEL_Y, 0.110)),
            material=axle_black,
            name=f"axle_mount_{i}",
        )
        lower_body.visual(
            Cylinder(radius=0.012, length=0.024),
            origin=Origin(xyz=(x + side * 0.006, WHEEL_Y, WHEEL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"axle_stub_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "toolbox_lid"),
        origin=Origin(xyz=(0.0, 0.0, LID_H * 0.5)),
        material=black_plastic,
        name="shell",
    )
    lid.visual(
        Box((0.208, 0.006, 0.016)),
        origin=Origin(xyz=(-POCKET_X, POCKET_Y, 0.024)),
        material=charcoal,
        name="left_cross_divider",
    )
    lid.visual(
        Box((0.006, 0.132, 0.016)),
        origin=Origin(xyz=(-POCKET_X, POCKET_Y, 0.024)),
        material=charcoal,
        name="left_center_divider",
    )
    lid.visual(
        Box((0.208, 0.006, 0.016)),
        origin=Origin(xyz=(POCKET_X, POCKET_Y, 0.024)),
        material=charcoal,
        name="right_cross_divider",
    )
    lid.visual(
        Box((0.006, 0.132, 0.016)),
        origin=Origin(xyz=(POCKET_X, POCKET_Y, 0.024)),
        material=charcoal,
        name="right_center_divider",
    )
    lid.visual(
        Box((0.120, 0.040, 0.008)),
        origin=Origin(xyz=(0.0, 0.135, 0.022)),
        material=charcoal,
        name="handle_pad",
    )
    lid.visual(
        Box((0.056, 0.016, 0.030)),
        origin=Origin(xyz=(-0.120, -0.188, 0.018)),
        material=latch_yellow,
        name="latch_0",
    )
    lid.visual(
        Box((0.056, 0.016, 0.030)),
        origin=Origin(xyz=(0.120, -0.188, 0.018)),
        material=latch_yellow,
        name="latch_1",
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.018, 0.010, 0.410)),
        origin=Origin(xyz=(-HANDLE_X, 0.0, -0.205)),
        material=aluminum,
        name="rail_0",
    )
    handle.visual(
        Box((0.018, 0.010, 0.410)),
        origin=Origin(xyz=(HANDLE_X, 0.0, -0.205)),
        material=aluminum,
        name="rail_1",
    )
    handle.visual(
        Box((0.420, 0.018, 0.016)),
        origin=Origin(),
        material=aluminum,
        name="bar",
    )
    handle.visual(
        Box((0.338, 0.028, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=charcoal,
        name="grip",
    )
    handle.visual(
        Box((0.020, 0.014, 0.022)),
        origin=Origin(xyz=(-HANDLE_X, 0.0, -0.328)),
        material=charcoal,
        name="shoe_0",
    )
    handle.visual(
        Box((0.020, 0.014, 0.022)),
        origin=Origin(xyz=(HANDLE_X, 0.0, -0.328)),
        material=charcoal,
        name="shoe_1",
    )

    cover_0 = model.part("cover_0")
    _add_cover(cover_0, clear_smoke)

    cover_1 = model.part("cover_1")
    _add_cover(cover_1, clear_smoke)

    wheel_0 = model.part("wheel_0")
    _add_wheel(wheel_0, rubber, steel, axle_black)

    wheel_1 = model.part("wheel_1")
    _add_wheel(wheel_1, rubber, steel, axle_black)

    model.articulation(
        "lid_mount",
        ArticulationType.FIXED,
        parent=lower_body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_BOTTOM_Z)),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_Y, HANDLE_COLLAPSED_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.45, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "cover_0_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cover_0,
        origin=Origin(xyz=(-POCKET_X, POCKET_HINGE_Y, POCKET_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cover_1_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cover_1,
        origin=Origin(xyz=(POCKET_X, POCKET_HINGE_Y, POCKET_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_body,
        child=wheel_0,
        origin=Origin(xyz=(-WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_body,
        child=wheel_1,
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_body = object_model.get_part("lower_body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    cover_0 = object_model.get_part("cover_0")
    cover_1 = object_model.get_part("cover_1")
    wheel_0 = object_model.get_part("wheel_0")

    handle_slide = object_model.get_articulation("handle_slide")
    cover_0_hinge = object_model.get_articulation("cover_0_hinge")
    cover_1_hinge = object_model.get_articulation("cover_1_hinge")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")

    ctx.expect_gap(
        lid,
        lower_body,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0005,
        name="lid closes tightly against the lower body seam",
    )
    ctx.expect_within(
        cover_0,
        lid,
        axes="xy",
        inner_elem="panel",
        margin=0.010,
        name="left organizer cover stays inside lid footprint",
    )
    ctx.expect_within(
        cover_1,
        lid,
        axes="xy",
        inner_elem="panel",
        margin=0.010,
        name="right organizer cover stays inside lid footprint",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        ctx.expect_overlap(
            handle,
            lower_body,
            axes="z",
            elem_a="rail_0",
            min_overlap=0.09,
            name="left handle rail remains inserted at full extension",
        )
        ctx.expect_overlap(
            handle,
            lower_body,
            axes="z",
            elem_a="rail_1",
            min_overlap=0.09,
            name="right handle rail remains inserted at full extension",
        )
        extended_handle_pos = ctx.part_world_position(handle)

    ctx.check(
        "handle extends upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.18,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    left_closed = ctx.part_element_world_aabb(cover_0, elem="panel")
    with ctx.pose({cover_0_hinge: 1.10}):
        left_open = ctx.part_element_world_aabb(cover_0, elem="panel")
    ctx.check(
        "left organizer cover opens upward",
        left_closed is not None
        and left_open is not None
        and left_open[1][2] > left_closed[1][2] + 0.10,
        details=f"closed={left_closed}, open={left_open}",
    )

    right_closed = ctx.part_element_world_aabb(cover_1, elem="panel")
    with ctx.pose({cover_1_hinge: 1.10}):
        right_open = ctx.part_element_world_aabb(cover_1, elem="panel")
    ctx.check(
        "right organizer cover opens upward",
        right_closed is not None
        and right_open is not None
        and right_open[1][2] > right_closed[1][2] + 0.10,
        details=f"closed={right_closed}, open={right_open}",
    )

    lug_start = ctx.part_element_world_aabb(wheel_0, elem="lug")
    with ctx.pose({wheel_0_spin: math.pi / 2.0}):
        lug_quarter_turn = ctx.part_element_world_aabb(wheel_0, elem="lug")
    ctx.check(
        "wheel rotates about its axle",
        lug_start is not None
        and lug_quarter_turn is not None
        and abs(lug_quarter_turn[1][2] - lug_start[1][2]) > 0.04,
        details=f"start={lug_start}, quarter_turn={lug_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
