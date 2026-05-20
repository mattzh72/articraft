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


BODY_BASE_Z = 0.085
BODY_WIDTH = 0.56
BODY_DEPTH = 0.35
LOWER_HEIGHT = 0.38
UPPER_HEIGHT = 0.13
TOP_Z = BODY_BASE_Z + LOWER_HEIGHT + UPPER_HEIGHT
ORGANIZER_REAR_Y = -0.006
SLEEVE_X = 0.172
SLEEVE_Y = -0.1905
SLEEVE_BOTTOM_Z = 0.325
SLEEVE_LENGTH = 0.22
SLEEVE_TOP_Z = SLEEVE_BOTTOM_Z + SLEEVE_LENGTH
HANDLE_TRAVEL = 0.28
RAIL_RADIUS = 0.010
RAIL_LENGTH = 0.52
RAIL_CENTER_Z = -0.12
WHEEL_RADIUS = 0.085
WHEEL_WIDTH = 0.048
WHEEL_CENTER_Y = -0.118
WHEEL_CENTER_Z = WHEEL_RADIUS
WHEEL_CENTER_X = 0.314


def _tube_shape(outer_radius: float, inner_radius: float, length: float):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _make_body_shell():
    upper_depth = 0.31
    organizer_width = 0.454
    organizer_depth = 0.156
    organizer_depth_cut = 0.043
    organizer_center_y = 0.072

    shell = cq.Workplane("XY").box(
        BODY_WIDTH,
        BODY_DEPTH,
        LOWER_HEIGHT,
        centered=(True, True, False),
    )
    upper = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 0.02, upper_depth, UPPER_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.01, LOWER_HEIGHT))
    )
    shell = shell.union(upper)
    shell = shell.edges("|Z").fillet(0.018)

    shell = (
        shell.faces(">Z")
        .workplane()
        .center(0.0, organizer_center_y)
        .rect(organizer_width, organizer_depth)
        .cutBlind(-organizer_depth_cut)
    )

    return shell


def _make_lid():
    lid = cq.Workplane("XY").box(0.466, 0.162, 0.016, centered=(True, False, False))
    lid = lid.edges("|Z").fillet(0.010)

    front_grip = (
        cq.Workplane("XY")
        .box(0.110, 0.020, 0.010, centered=(True, False, False))
        .translate((0.0, 0.142, 0.010))
    )
    lid = lid.union(front_grip)

    center_pad = (
        cq.Workplane("XY")
        .box(0.360, 0.115, 0.005, centered=(True, False, False))
        .translate((0.0, 0.020, 0.011))
    )
    lid = lid.union(center_pad)
    return lid


def _add_wheel_visuals(part, *, tire_material, rim_material, hub_material) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.034),
        origin=spin_origin,
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.040, length=0.056),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.062),
        origin=spin_origin,
        material=hub_material,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_tool_chest")

    shell_red = model.material("shell_red", rgba=(0.74, 0.14, 0.09, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.11, 0.12, 0.13, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.52, 0.62, 0.70, 0.34))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "tool_chest_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z)),
        material=shell_red,
        name="body_shell",
    )
    body.visual(
        Box((0.10, 0.090, 0.088)),
        origin=Origin(xyz=(0.17, 0.125, 0.044)),
        material=black_plastic,
        name="front_foot_0",
    )
    body.visual(
        Box((0.10, 0.090, 0.088)),
        origin=Origin(xyz=(-0.17, 0.125, 0.044)),
        material=black_plastic,
        name="front_foot_1",
    )
    body.visual(
        Box((0.010, 0.144, 0.028)),
        origin=Origin(xyz=(-0.108, 0.072, BODY_BASE_Z + 0.479)),
        material=black_plastic,
        name="tray_divider_0",
    )
    body.visual(
        Box((0.010, 0.144, 0.028)),
        origin=Origin(xyz=(0.108, 0.072, BODY_BASE_Z + 0.479)),
        material=black_plastic,
        name="tray_divider_1",
    )
    body.visual(
        Box((0.430, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.072, BODY_BASE_Z + 0.479)),
        material=black_plastic,
        name="tray_divider_2",
    )
    body.visual(
        Box((0.090, 0.045, 0.060)),
        origin=Origin(xyz=(0.238, -0.114, 0.126)),
        material=black_plastic,
        name="wheel_guard_0",
    )
    body.visual(
        Box((0.090, 0.045, 0.060)),
        origin=Origin(xyz=(-0.238, -0.114, 0.126)),
        material=black_plastic,
        name="wheel_guard_1",
    )
    for idx, (z, width) in enumerate(
        [
            (0.060, 0.44),
            (0.125, 0.46),
            (0.195, 0.48),
            (0.275, 0.50),
            (0.345, 0.44),
            (0.430, 0.38),
        ]
    ):
        body.visual(
            Box((width, 0.022, 0.016)),
            origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 - 0.010, BODY_BASE_Z + z)),
            material=shell_red,
            name=f"front_rib_{idx}",
        )
    for idx, side_x in enumerate((-BODY_WIDTH * 0.5 + 0.003, BODY_WIDTH * 0.5 - 0.003)):
        for rib_idx, z in enumerate((0.092, 0.167, 0.247, 0.327)):
            body.visual(
                Box((0.010, 0.18, 0.016)),
                origin=Origin(xyz=(side_x, 0.0, BODY_BASE_Z + z)),
                material=shell_red,
                name=f"side_rib_{idx}_{rib_idx}",
            )
    for idx, side in enumerate((-1.0, 1.0)):
        body.visual(
            mesh_from_cadquery(
                _tube_shape(0.016, 0.0125, SLEEVE_LENGTH),
                f"tool_chest_handle_sleeve_{idx}",
            ),
            origin=Origin(xyz=(side * SLEEVE_X, SLEEVE_Y, SLEEVE_BOTTOM_Z)),
            material=dark_steel,
            name=f"sleeve_{idx}",
        )

    organizer_lid = model.part("organizer_lid")
    organizer_lid.visual(
        mesh_from_cadquery(_make_lid(), "tool_chest_organizer_lid"),
        material=smoked_clear,
        name="lid_frame",
    )
    organizer_lid.visual(
        Box((0.090, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.153, 0.010)),
        material=black_plastic,
        name="lid_tab",
    )

    trolley_handle = model.part("trolley_handle")
    for idx, side in enumerate((-1.0, 1.0)):
        trolley_handle.visual(
            Cylinder(radius=RAIL_RADIUS, length=RAIL_LENGTH),
            origin=Origin(xyz=(side * SLEEVE_X, 0.0, RAIL_CENTER_Z)),
            material=aluminum,
            name=f"rail_{idx}",
        )
        trolley_handle.visual(
            Cylinder(radius=0.018, length=0.016),
            origin=Origin(xyz=(side * SLEEVE_X, 0.0, 0.008)),
            material=black_plastic,
            name=f"stop_collar_{idx}",
        )
    trolley_handle.visual(
        Box((0.398, 0.034, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.152)),
        material=black_plastic,
        name="grip",
    )
    trolley_handle.visual(
        Box((0.356, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=dark_steel,
        name="brace",
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(
        left_wheel,
        tire_material=rubber,
        rim_material=dark_steel,
        hub_material=aluminum,
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(
        right_wheel,
        tire_material=rubber,
        rim_material=dark_steel,
        hub_material=aluminum,
    )

    model.articulation(
        "organizer_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=organizer_lid,
        origin=Origin(xyz=(0.0, ORGANIZER_REAR_Y, TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=0.0, upper=1.24),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trolley_handle,
        origin=Origin(xyz=(0.0, SLEEVE_Y, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.45,
            lower=0.0,
            upper=HANDLE_TRAVEL,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=24.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=24.0),
    )

    return model


def _same_position(a, b, tol: float = 1e-6) -> bool:
    if a is None or b is None:
        return False
    return all(abs(av - bv) <= tol for av, bv in zip(a, b))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("organizer_lid")
    handle = object_model.get_part("trolley_handle")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("organizer_hinge")
    handle_slide = object_model.get_articulation("handle_slide")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_frame",
        elem_b="body_shell",
        min_overlap=0.15,
        name="organizer lid covers the tray opening",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="body_shell",
        max_gap=0.004,
        max_penetration=0.0,
        name="closed organizer lid sits nearly flush on the tray rim",
    )

    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="rail_0",
        outer_elem="sleeve_0",
        margin=0.004,
        name="left inner rail stays centered in the left sleeve",
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="rail_1",
        outer_elem="sleeve_1",
        margin=0.004,
        name="right inner rail stays centered in the right sleeve",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="rail_0",
        elem_b="sleeve_0",
        min_overlap=0.20,
        name="left rail remains deeply inserted when collapsed",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="rail_1",
        elem_b="sleeve_1",
        min_overlap=0.20,
        name="right rail remains deeply inserted when collapsed",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    rest_handle_pos = ctx.part_world_position(handle)
    left_wheel_pos = ctx.part_world_position(left_wheel)
    right_wheel_pos = ctx.part_world_position(right_wheel)

    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "organizer lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        extended_handle_pos = ctx.part_world_position(handle)
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="rail_0",
            outer_elem="sleeve_0",
            margin=0.004,
            name="left rail stays centered when extended",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="rail_1",
            outer_elem="sleeve_1",
            margin=0.004,
            name="right rail stays centered when extended",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="rail_0",
            elem_b="sleeve_0",
            min_overlap=0.09,
            name="left rail keeps retained insertion at full extension",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="rail_1",
            elem_b="sleeve_1",
            min_overlap=0.09,
            name="right rail keeps retained insertion at full extension",
        )

    ctx.check(
        "trolley handle extends upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.25,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    with ctx.pose({left_spin: 1.35, right_spin: -0.90}):
        spun_left_pos = ctx.part_world_position(left_wheel)
        spun_right_pos = ctx.part_world_position(right_wheel)

    ctx.check(
        "wheel spin keeps the left wheel on its axle",
        _same_position(left_wheel_pos, spun_left_pos),
        details=f"rest={left_wheel_pos}, spun={spun_left_pos}",
    )
    ctx.check(
        "wheel spin keeps the right wheel on its axle",
        _same_position(right_wheel_pos, spun_right_pos),
        details=f"rest={right_wheel_pos}, spun={spun_right_pos}",
    )

    return ctx.report()


object_model = build_object_model()
