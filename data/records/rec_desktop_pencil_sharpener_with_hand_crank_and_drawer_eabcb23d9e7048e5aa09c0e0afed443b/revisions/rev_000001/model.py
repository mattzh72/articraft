from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_HEIGHT = 0.010
BODY_WIDTH = 0.074
BODY_FRONT_X = 0.041
DRAWER_TRAVEL = 0.038
SELECTOR_TRAVEL = 0.008
AXLE_X = -0.008
AXLE_Z = 0.086
AXLE_Y = 0.043
PORT_Z = 0.091
PORT_Y = 0.000
SELECTOR_Y = 0.018


def _build_housing_shape() -> cq.Workplane:
    side_profile = [
        (0.041, BASE_HEIGHT),
        (0.041, 0.112),
        (0.036, 0.128),
        (0.023, 0.140),
        (0.004, 0.146),
        (-0.015, 0.144),
        (-0.029, 0.137),
        (-0.038, 0.122),
        (-0.041, 0.094),
        (-0.041, BASE_HEIGHT),
    ]

    base = (
        cq.Workplane("XY")
        .box(0.102, 0.084, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )

    housing = cq.Workplane("XZ").polyline(side_profile).close().extrude(BODY_WIDTH / 2.0, both=True)

    side_boss = (
        cq.Workplane("XZ")
        .center(AXLE_X, AXLE_Z)
        .circle(0.011)
        .extrude(0.010)
        .translate((0.0, BODY_WIDTH / 2.0, 0.0))
    )

    port_bezel = (
        cq.Workplane("YZ")
        .center(PORT_Y, PORT_Z)
        .circle(0.010)
        .extrude(0.008)
        .translate((BODY_FRONT_X, 0.0, 0.0))
    )

    shell = base.union(housing).union(side_boss).union(port_bezel)

    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.068, 0.060, 0.038, centered=(True, True, False))
        .translate((0.008, 0.0, 0.014))
    )
    mechanism_cavity = (
        cq.Workplane("XY")
        .box(0.036, 0.030, 0.025, centered=(True, True, False))
        .translate((0.001, 0.0, 0.078))
    )
    pencil_bore = (
        cq.Workplane("YZ")
        .center(PORT_Y, PORT_Z)
        .circle(0.0048)
        .extrude(0.050)
        .translate((-0.005, 0.0, 0.0))
    )
    selector_slot = (
        cq.Workplane("YZ")
        .center(SELECTOR_Y, PORT_Z)
        .rect(0.005, 0.020)
        .extrude(0.012)
        .translate((0.030, 0.0, 0.0))
    )
    selector_channel = (
        cq.Workplane("XY")
        .box(0.020, 0.012, 0.026, centered=(True, True, True))
        .translate((0.026, SELECTOR_Y, PORT_Z))
    )
    axle_hole = (
        cq.Workplane("XZ")
        .center(AXLE_X, AXLE_Z)
        .circle(0.0041)
        .extrude(0.022)
        .translate((0.0, (BODY_WIDTH / 2.0) - 0.006, 0.0))
    )

    return shell.cut(drawer_cavity).cut(mechanism_cavity).cut(pencil_bore).cut(selector_slot).cut(selector_channel).cut(axle_hole)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mn + mx) * 0.5 for mn, mx in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_pencil_sharpener")

    enamel = model.material("enamel_green", rgba=(0.18, 0.34, 0.27, 1.0))
    drawer_metal = model.material("drawer_metal", rgba=(0.56, 0.62, 0.60, 1.0))
    crank_steel = model.material("crank_steel", rgba=(0.79, 0.81, 0.84, 1.0))
    bakelite = model.material("bakelite_black", rgba=(0.09, 0.08, 0.07, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_housing_shape(), "pencil_sharpener_body"),
        material=enamel,
        name="housing",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.102, 0.084, 0.146)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.006, 0.058, 0.038)),
        material=enamel,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.008, 0.020, 0.007)),
        origin=Origin(xyz=(0.005, 0.0, -0.010)),
        material=bakelite,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.060, 0.050, 0.0035)),
        origin=Origin(xyz=(-0.031, 0.0, -0.017)),
        material=drawer_metal,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.056, 0.003, 0.028)),
        origin=Origin(xyz=(-0.031, 0.0235, -0.005)),
        material=drawer_metal,
        name="tray_wall_0",
    )
    drawer.visual(
        Box((0.056, 0.003, 0.028)),
        origin=Origin(xyz=(-0.031, -0.0235, -0.005)),
        material=drawer_metal,
        name="tray_wall_1",
    )
    drawer.visual(
        Box((0.004, 0.050, 0.028)),
        origin=Origin(xyz=(-0.060, 0.0, -0.005)),
        material=drawer_metal,
        name="tray_back",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.068, 0.060, 0.040)),
        mass=0.16,
        origin=Origin(xyz=(-0.028, 0.0, -0.003)),
    )

    selector = model.part("selector")
    selector.visual(
        Box((0.004, 0.008, 0.009)),
        material=bakelite,
        name="selector_thumb",
    )
    selector.visual(
        Box((0.008, 0.003, 0.005)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=bakelite,
        name="selector_stem",
    )
    selector.visual(
        Box((0.010, 0.009, 0.008)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        material=bakelite,
        name="selector_shoe",
    )
    selector.inertial = Inertial.from_geometry(
        Box((0.016, 0.009, 0.010)),
        mass=0.01,
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=crank_steel,
        name="mount_washer",
    )
    crank.visual(
        Cylinder(radius=0.0032, length=0.020),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=crank_steel,
        name="axle_shaft",
    )
    crank.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=crank_steel,
        name="hub_cap",
    )
    crank.visual(
        Cylinder(radius=0.0030, length=0.010),
        origin=Origin(xyz=(0.0, 0.012, 0.005), rpy=(0.0, 0.0, 0.0)),
        material=crank_steel,
        name="arm_stand",
    )
    crank.visual(
        Cylinder(radius=0.0028, length=0.032),
        origin=Origin(xyz=(-0.016, 0.014, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=crank_steel,
        name="arm_bar",
    )
    crank.visual(
        Cylinder(radius=0.0028, length=0.026),
        origin=Origin(xyz=(-0.032, 0.014, 0.023), rpy=(0.0, 0.0, 0.0)),
        material=crank_steel,
        name="arm_riser",
    )
    crank.visual(
        Cylinder(radius=0.0023, length=0.026),
        origin=Origin(xyz=(-0.032, 0.022, 0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=crank_steel,
        name="handle_spindle",
    )
    crank.visual(
        Cylinder(radius=0.0050, length=0.018),
        origin=Origin(xyz=(-0.032, 0.029, 0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bakelite,
        name="handle_knob",
    )
    crank.inertial = Inertial.from_geometry(
        Box((0.070, 0.040, 0.045)),
        mass=0.12,
        origin=Origin(xyz=(-0.020, 0.020, 0.020)),
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.045, 0.0, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.12, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.PRISMATIC,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.040, SELECTOR_Y, 0.088)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=SELECTOR_TRAVEL),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(AXLE_X, AXLE_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    selector = object_model.get_part("selector")
    crank = object_model.get_part("crank")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    selector_joint = object_model.get_articulation("body_to_selector")
    crank_joint = object_model.get_articulation("body_to_crank")

    drawer_closed = ctx.part_world_position(drawer)
    ctx.expect_overlap(
        drawer,
        body,
        axes="yz",
        elem_a="drawer_front",
        elem_b="housing",
        min_overlap=0.035,
        name="drawer front aligns with the housing opening",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="tray_bottom",
        elem_b="housing",
        min_overlap=0.020,
        name="drawer remains inserted when closed",
    )

    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        drawer_open = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="yz",
            elem_a="drawer_front",
            elem_b="housing",
            min_overlap=0.035,
            name="drawer stays guided when extended",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray_bottom",
            elem_b="housing",
            min_overlap=0.015,
            name="drawer retains insertion at full extension",
        )
    ctx.check(
        "drawer slides out forward",
        drawer_closed is not None and drawer_open is not None and drawer_open[0] > drawer_closed[0] + 0.030,
        details=f"closed={drawer_closed}, open={drawer_open}",
    )

    selector_low = ctx.part_world_position(selector)
    ctx.expect_overlap(
        selector,
        body,
        axes="xy",
        elem_a="selector_shoe",
        elem_b="housing",
        min_overlap=0.006,
        name="selector stays captured in the guide at low position",
    )
    with ctx.pose({selector_joint: SELECTOR_TRAVEL}):
        selector_high = ctx.part_world_position(selector)
        ctx.expect_overlap(
            selector,
            body,
            axes="xy",
            elem_a="selector_shoe",
            elem_b="housing",
            min_overlap=0.006,
            name="selector stays captured in the guide at high position",
        )
    ctx.check(
        "selector slides beside the pencil port",
        selector_low is not None and selector_high is not None and selector_high[2] > selector_low[2] + 0.006,
        details=f"low={selector_low}, high={selector_high}",
    )

    body_aabb = ctx.part_element_world_aabb(body, elem="housing")
    knob_rest_aabb = ctx.part_element_world_aabb(crank, elem="handle_knob")
    with ctx.pose({crank_joint: math.pi}):
        knob_turn_aabb = ctx.part_element_world_aabb(crank, elem="handle_knob")

    rest_center = _aabb_center(knob_rest_aabb)
    turn_center = _aabb_center(knob_turn_aabb)
    body_side_y = body_aabb[1][1] if body_aabb is not None else None
    rest_min_y = knob_rest_aabb[0][1] if knob_rest_aabb is not None else None
    turn_min_y = knob_turn_aabb[0][1] if knob_turn_aabb is not None else None

    ctx.check(
        "crank handle sweeps around the side axle",
        rest_center is not None
        and turn_center is not None
        and abs(turn_center[0] - rest_center[0]) > 0.040
        and abs(turn_center[2] - rest_center[2]) > 0.040,
        details=f"rest={rest_center}, turned={turn_center}",
    )
    ctx.check(
        "crank handle stays outboard of the housing",
        body_side_y is not None
        and rest_min_y is not None
        and turn_min_y is not None
        and rest_min_y > body_side_y + 0.010
        and turn_min_y > body_side_y + 0.010,
        details=f"body_side_y={body_side_y}, rest_min_y={rest_min_y}, turn_min_y={turn_min_y}",
    )

    return ctx.report()


object_model = build_object_model()
