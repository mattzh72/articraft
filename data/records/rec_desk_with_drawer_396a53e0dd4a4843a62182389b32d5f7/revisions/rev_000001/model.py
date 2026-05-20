from __future__ import annotations

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


TOP_WIDTH = 1.40
TOP_DEPTH = 0.70
TOP_THICKNESS = 0.028
TOP_UNDERSIDE_Z = 0.712

CABLE_OPENING_WIDTH = 0.52
CABLE_OPENING_DEPTH = 0.11
CABLE_DOOR_WIDTH = CABLE_OPENING_WIDTH
CABLE_DOOR_DEPTH = 0.106
CABLE_DOOR_THICKNESS = 0.012

DRAWER_WIDTH = 0.42
DRAWER_DEPTH = 0.34
DRAWER_HEIGHT = 0.08
DRAWER_WALL = 0.010
DRAWER_BOTTOM = 0.006
DRAWER_FRONT = 0.014
DRAWER_BACK = 0.010
DRAWER_CLEARANCE = 0.006
DRAWER_TRAVEL = 0.24

FRAME_TOP_Z = 0.690
LEG_THICKNESS = 0.050
LEG_DEPTH = 0.600
LEG_HEIGHT = 0.690
LEG_X = 0.575


def _make_drawer_shape():
    outer = (
        cq.Workplane("XY")
        .box(DRAWER_WIDTH, DRAWER_DEPTH, DRAWER_HEIGHT)
        .translate((0.0, -DRAWER_DEPTH / 2.0, -DRAWER_HEIGHT / 2.0))
    )
    inner_width = DRAWER_WIDTH - 2.0 * DRAWER_WALL
    inner_depth = DRAWER_DEPTH - DRAWER_FRONT - DRAWER_BACK
    inner = (
        cq.Workplane("XY")
        .box(
            inner_width,
            inner_depth,
            DRAWER_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                0.0,
                -(DRAWER_DEPTH - DRAWER_FRONT + DRAWER_BACK) / 2.0,
                -DRAWER_HEIGHT + DRAWER_BOTTOM,
            )
        )
    )
    finger_recess = (
        cq.Workplane("XY")
        .box(
            DRAWER_WIDTH * 0.40,
            0.020,
            0.010,
        )
        .translate((0.0, -DRAWER_DEPTH + 0.012, -0.010))
    )
    return outer.cut(inner).cut(finger_recess)


def _make_cable_door_shape():
    panel = (
        cq.Workplane("XY")
        .box(CABLE_DOOR_WIDTH, CABLE_DOOR_DEPTH, CABLE_DOOR_THICKNESS)
        .translate((0.0, -CABLE_DOOR_DEPTH / 2.0, -CABLE_DOOR_THICKNESS / 2.0))
    )
    stiffener = (
        cq.Workplane("XY")
        .box(CABLE_DOOR_WIDTH * 0.55, 0.016, 0.006)
        .translate(
            (
                0.0,
                -CABLE_DOOR_DEPTH + 0.014,
                -CABLE_DOOR_THICKNESS - 0.003,
            )
        )
    )
    return panel.union(stiffener)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_desk")

    oak = model.material("oak", rgba=(0.79, 0.70, 0.56, 1.0))
    graphite = model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.13, 0.14, 1.0))

    top = model.part("top")
    top.visual(
        Box((TOP_WIDTH, TOP_DEPTH - CABLE_OPENING_DEPTH + 0.002, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.054, TOP_THICKNESS / 2.0)),
        material=oak,
        name="top_shell",
    )
    for index, x_pos in enumerate((-0.48, 0.48)):
        top.visual(
            Box(
                (
                    (TOP_WIDTH - CABLE_OPENING_WIDTH) / 2.0,
                    CABLE_OPENING_DEPTH + 0.002,
                    TOP_THICKNESS,
                )
            ),
            origin=Origin(
                xyz=(
                    x_pos,
                    TOP_DEPTH / 2.0 - CABLE_OPENING_DEPTH / 2.0 - 0.001,
                    TOP_THICKNESS / 2.0,
                )
            ),
            material=oak,
            name=f"top_wing_{index}",
        )
    for index, x_pos in enumerate((-0.214, 0.214)):
        top.visual(
            Box((0.012, 0.420, 0.022)),
            origin=Origin(xyz=(x_pos, -0.110, -0.011)),
            material=graphite,
            name=f"runner_{index}",
        )
    top.visual(
        Box((CABLE_OPENING_WIDTH + 0.020, 0.012, 0.050)),
        origin=Origin(
            xyz=(0.0, TOP_DEPTH / 2.0 - CABLE_OPENING_DEPTH + 0.006, -0.025)
        ),
        material=graphite,
        name="cable_well_front",
    )
    for index, x_pos in enumerate(
        (
            -(CABLE_OPENING_WIDTH / 2.0 + 0.006),
            CABLE_OPENING_WIDTH / 2.0 + 0.006,
        )
    ):
        top.visual(
            Box((0.012, CABLE_OPENING_DEPTH, 0.050)),
            origin=Origin(
                xyz=(x_pos, TOP_DEPTH / 2.0 - CABLE_OPENING_DEPTH / 2.0, -0.025)
            ),
            material=graphite,
            name=f"cable_well_side_{index}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_make_drawer_shape(), "desk_drawer"),
        material=oak,
        name="drawer_shell",
    )
    for index, x_pos in enumerate((-0.202, 0.202)):
        drawer.visual(
            Box((0.016, 0.350, 0.016)),
            origin=Origin(xyz=(x_pos, -0.155, -0.015)),
            material=graphite,
            name=f"drawer_rail_{index}",
        )

    cable_door = model.part("cable_door")
    cable_door.visual(
        mesh_from_cadquery(_make_cable_door_shape(), "cable_door"),
        material=oak,
        name="door_panel",
    )
    cable_door.visual(
        Cylinder(radius=0.004, length=CABLE_DOOR_WIDTH - 0.040),
        origin=Origin(xyz=(0.0, -0.004, -0.004), rpy=(0.0, 1.57079632679, 0.0)),
        material=graphite,
        name="door_hinge_leaf",
    )

    frame = model.part("frame")
    frame.visual(
        Box((LEG_THICKNESS, LEG_DEPTH, LEG_HEIGHT)),
        origin=Origin(xyz=(-LEG_X, 0.0, LEG_HEIGHT / 2.0)),
        material=charcoal,
        name="leg_0",
    )
    frame.visual(
        Box((LEG_THICKNESS, LEG_DEPTH, LEG_HEIGHT)),
        origin=Origin(xyz=(LEG_X, 0.0, LEG_HEIGHT / 2.0)),
        material=charcoal,
        name="leg_1",
    )
    frame.visual(
        Box((1.100, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.170, 0.675)),
        material=charcoal,
        name="upper_stretcher",
    )
    frame.visual(
        Box((1.100, 0.035, 0.060)),
        origin=Origin(xyz=(0.0, 0.2825, 0.330)),
        material=charcoal,
        name="lower_stretcher",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-LEG_X, -0.180),
            (-LEG_X, 0.180),
            (LEG_X, -0.180),
            (LEG_X, 0.180),
        )
    ):
        frame.visual(
            Box((0.050, 0.110, TOP_UNDERSIDE_Z - FRAME_TOP_Z)),
            origin=Origin(
                xyz=(x_pos, y_pos, FRAME_TOP_Z + (TOP_UNDERSIDE_Z - FRAME_TOP_Z) / 2.0)
            ),
            material=charcoal,
            name=f"standoff_{index}",
        )

    model.articulation(
        "top_to_drawer",
        ArticulationType.PRISMATIC,
        parent=top,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.020, -DRAWER_CLEARANCE)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "top_to_cable_door",
        ArticulationType.REVOLUTE,
        parent=top,
        child=cable_door,
        origin=Origin(xyz=(0.0, TOP_DEPTH / 2.0, TOP_THICKNESS)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "top_to_frame",
        ArticulationType.FIXED,
        parent=top,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, -TOP_UNDERSIDE_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top = object_model.get_part("top")
    drawer = object_model.get_part("drawer")
    cable_door = object_model.get_part("cable_door")
    drawer_slide = object_model.get_articulation("top_to_drawer")
    door_hinge = object_model.get_articulation("top_to_cable_door")

    ctx.expect_gap(
        top,
        drawer,
        axis="z",
        positive_elem="top_shell",
        negative_elem="drawer_shell",
        min_gap=0.003,
        max_gap=0.012,
        name="drawer hangs just beneath the desktop",
    )
    ctx.expect_overlap(
        drawer,
        top,
        axes="y",
        elem_a="drawer_rail_0",
        elem_b="runner_0",
        min_overlap=0.18,
        name="closed drawer remains deeply engaged on the runners",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_gap(
            top,
            drawer,
            axis="z",
            positive_elem="top_shell",
            negative_elem="drawer_shell",
            min_gap=0.003,
            max_gap=0.012,
            name="extended drawer stays aligned under the desktop",
        )
        ctx.expect_overlap(
            drawer,
            top,
            axes="y",
            elem_a="drawer_rail_0",
            elem_b="runner_0",
            min_overlap=0.08,
            name="extended drawer retains runner engagement",
        )
        open_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends toward the seated user",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[1] < closed_drawer_pos[1] - 0.20,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    with ctx.pose({door_hinge: 1.05}):
        door_aabb = ctx.part_element_world_aabb(cable_door, elem="door_panel")
        top_aabb = ctx.part_element_world_aabb(top, elem="top_shell")
    ctx.check(
        "cable door lifts above the desktop when opened",
        door_aabb is not None
        and top_aabb is not None
        and door_aabb[1][2] > top_aabb[1][2] + 0.07,
        details=f"door={door_aabb}, top={top_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
