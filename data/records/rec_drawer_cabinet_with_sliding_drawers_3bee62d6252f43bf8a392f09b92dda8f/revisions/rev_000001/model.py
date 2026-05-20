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


BODY_DEPTH = 0.34
BODY_WIDTH = 0.42
BODY_HEIGHT = 0.20
WALL = 0.018
DRAWER_TRAVEL = 0.22


def _socket_collar_shape() -> cq.Workplane:
    """Annular underside socket that receives the next unit's stacking peg."""
    return cq.Workplane("XY").circle(0.026).circle(0.0185).extrude(-0.016)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modular_stacking_single_drawer_unit")

    model.material("warm_white_plastic", rgba=(0.86, 0.85, 0.80, 1.0))
    model.material("drawer_front_blue", rgba=(0.23, 0.34, 0.48, 1.0))
    model.material("drawer_interior", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("dark_handle", rgba=(0.06, 0.065, 0.07, 1.0))
    model.material("galvanized_rail", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("socket_shadow", rgba=(0.035, 0.035, 0.035, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material="warm_white_plastic",
        name="bottom_panel",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - WALL / 2.0)),
        material="warm_white_plastic",
        name="top_panel",
    )
    for index, y in enumerate((-(BODY_WIDTH / 2.0 - WALL / 2.0), BODY_WIDTH / 2.0 - WALL / 2.0)):
        body.visual(
            Box((BODY_DEPTH, WALL, BODY_HEIGHT)),
            origin=Origin(xyz=(0.0, y, BODY_HEIGHT / 2.0)),
            material="warm_white_plastic",
            name=f"side_panel_{index}",
        )
    body.visual(
        Box((WALL, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_DEPTH / 2.0 - WALL / 2.0), 0.0, BODY_HEIGHT / 2.0)),
        material="warm_white_plastic",
        name="rear_panel",
    )

    # A slightly proud face frame makes the open rectangular housing read as a molded module.
    body.visual(
        Box((0.014, BODY_WIDTH, 0.020)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 - 0.007, 0.0, BODY_HEIGHT - 0.010)),
        material="warm_white_plastic",
        name="front_top_lip",
    )
    body.visual(
        Box((0.014, BODY_WIDTH, 0.020)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 - 0.007, 0.0, 0.010)),
        material="warm_white_plastic",
        name="front_bottom_lip",
    )
    for index, y in enumerate((-(BODY_WIDTH / 2.0 - 0.010), BODY_WIDTH / 2.0 - 0.010)):
        body.visual(
            Box((0.014, 0.020, BODY_HEIGHT)),
            origin=Origin(xyz=(BODY_DEPTH / 2.0 - 0.007, y, BODY_HEIGHT / 2.0)),
            material="warm_white_plastic",
            name=f"front_side_lip_{index}",
        )

    # Fixed cabinet-side guide rails; the drawer carries the mating runners.
    for index, y in enumerate((-0.187, 0.187)):
        body.visual(
            Box((0.285, 0.010, 0.012)),
            origin=Origin(xyz=(0.015, y, 0.064)),
            material="galvanized_rail",
            name=f"guide_{index}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.026, 0.405, 0.165)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 + 0.013, 0.0, 0.100)),
        material="drawer_front_blue",
        name="front_panel",
    )
    drawer.visual(
        Box((0.304, 0.354, 0.012)),
        origin=Origin(xyz=(0.019, 0.0, 0.039)),
        material="drawer_interior",
        name="tray_bottom",
    )
    for index, y in enumerate((-0.174, 0.174)):
        drawer.visual(
            Box((0.304, 0.012, 0.108)),
            origin=Origin(xyz=(0.019, y, 0.088)),
            material="drawer_interior",
            name=f"tray_side_{index}",
        )
    drawer.visual(
        Box((0.012, 0.354, 0.108)),
        origin=Origin(xyz=(-0.127, 0.0, 0.088)),
        material="drawer_interior",
        name="tray_back",
    )
    for index, y in enumerate((-0.181, 0.181)):
        drawer.visual(
            Box((0.268, 0.008, 0.010)),
            origin=Origin(xyz=(0.012, y, 0.073)),
            material="galvanized_rail",
            name=f"runner_{index}",
        )

    drawer.visual(
        Cylinder(radius=0.009, length=0.220),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 + 0.044, 0.0, 0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_handle",
        name="handle_bar",
    )
    for index, y in enumerate((-0.080, 0.080)):
        drawer.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(BODY_DEPTH / 2.0 + 0.028, y, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_handle",
            name=f"handle_post_{index}",
        )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=DRAWER_TRAVEL),
    )

    peg_positions = (
        (-0.105, -0.145),
        (-0.105, 0.145),
        (0.105, -0.145),
        (0.105, 0.145),
    )
    for index, (x, y) in enumerate(peg_positions):
        peg = model.part(f"peg_{index}")
        peg.visual(
            Cylinder(radius=0.016, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
            material="warm_white_plastic",
            name="peg_post",
        )
        peg.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material="warm_white_plastic",
            name="peg_flange",
        )
        model.articulation(
            f"body_to_peg_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=peg,
            origin=Origin(xyz=(x, y, BODY_HEIGHT)),
        )

        socket = model.part(f"socket_{index}")
        socket.visual(
            mesh_from_cadquery(_socket_collar_shape(), f"socket_collar_{index}", tolerance=0.0006),
            origin=Origin(),
            material="socket_shadow",
            name="socket_collar",
        )
        model.articulation(
            f"body_to_socket_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=socket,
            origin=Origin(xyz=(x, y, 0.0)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("body_to_drawer")

    ctx.check(
        "drawer has bounded prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == DRAWER_TRAVEL,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )

    ctx.expect_gap(
        drawer,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="front_panel",
        name="drawer front seats flush on body",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="y",
        inner_elem="tray_bottom",
        outer_elem="bottom_panel",
        margin=0.006,
        name="drawer tray fits within body width",
    )
    for index in (0, 1):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a=f"runner_{index}",
            elem_b=f"guide_{index}",
            min_overlap=0.24,
            name=f"runner {index} overlaps guide rail length",
        )

    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray_bottom",
            elem_b="bottom_panel",
            min_overlap=0.070,
            name="extended drawer retains insertion",
        )
        extended_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends outward along front axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    for index in range(4):
        peg = object_model.get_part(f"peg_{index}")
        socket = object_model.get_part(f"socket_{index}")
        ctx.expect_contact(
            peg,
            body,
            elem_a="peg_flange",
            elem_b="top_panel",
            contact_tol=0.001,
            name=f"peg {index} fixed to top deck",
        )
        ctx.expect_contact(
            socket,
            body,
            elem_a="socket_collar",
            elem_b="bottom_panel",
            contact_tol=0.001,
            name=f"socket {index} fixed to underside",
        )
        ctx.expect_origin_distance(
            peg,
            socket,
            axes="xy",
            max_dist=0.001,
            name=f"peg {index} aligns with matching socket",
        )

    return ctx.report()


object_model = build_object_model()
