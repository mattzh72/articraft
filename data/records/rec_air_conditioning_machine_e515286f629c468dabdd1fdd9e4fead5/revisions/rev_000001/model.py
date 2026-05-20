from __future__ import annotations

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


BODY_WIDTH = 0.44
BODY_DEPTH = 0.36
BODY_HEIGHT = 0.79
BODY_WALL = 0.004
TOP_WALL = 0.020
BOTTOM_EXTENSION = 0.015
CORNER_RADIUS = 0.050
TOP_EDGE_RADIUS = 0.016

TOP_VENT_WIDTH = 0.240
TOP_VENT_DEPTH = 0.095
TOP_VENT_CENTER_Y = 0.035
TOP_VENT_HINGE_Y = TOP_VENT_CENTER_Y + TOP_VENT_DEPTH / 2.0 + 0.006
VENT_DOOR_WIDTH = TOP_VENT_WIDTH + 0.024
VENT_DOOR_DEPTH = TOP_VENT_DEPTH + 0.030
VENT_DOOR_THICKNESS = 0.005
VENT_DOOR_STANDOFF = 0.0

SERVICE_PANEL_WIDTH = 0.145
SERVICE_PANEL_HEIGHT = 0.245
SERVICE_PANEL_THICKNESS = 0.005
SERVICE_PANEL_BOTTOM_Z = 0.105
SERVICE_PANEL_REAR_Y = BODY_DEPTH / 2.0 - 0.026
SERVICE_PANEL_CENTER_Y = SERVICE_PANEL_REAR_Y - SERVICE_PANEL_WIDTH / 2.0
SERVICE_PANEL_STANDOFF = 0.0


def _rounded_tower(width: float, depth: float, height: float, corner_radius: float, top_radius: float):
    shape = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    shape = shape.edges("|Z").fillet(corner_radius)
    shape = shape.edges(">Z").fillet(top_radius)
    return shape


def _box_at(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    center_x: float,
    center_y: float,
    bottom_z: float,
):
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, False))
        .translate((center_x, center_y, bottom_z))
    )


def _build_housing_shape():
    outer = _rounded_tower(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, CORNER_RADIUS, TOP_EDGE_RADIUS)
    inner = _rounded_tower(
        BODY_WIDTH - 2.0 * BODY_WALL,
        BODY_DEPTH - 2.0 * BODY_WALL,
        BODY_HEIGHT - TOP_WALL + BOTTOM_EXTENSION,
        max(CORNER_RADIUS - BODY_WALL, 0.006),
        max(TOP_EDGE_RADIUS - BODY_WALL, 0.004),
    ).translate((0.0, 0.0, -BOTTOM_EXTENSION))
    housing = outer.cut(inner)

    control_pocket = _box_at(
        0.155,
        0.058,
        0.0028,
        center_x=0.0,
        center_y=-0.088,
        bottom_z=BODY_HEIGHT - 0.0028,
    )
    housing = housing.cut(control_pocket)

    vent_recess = (
        _box_at(
            VENT_DOOR_WIDTH,
            TOP_VENT_DEPTH + 0.020,
            0.0035,
            center_x=0.0,
            center_y=TOP_VENT_CENTER_Y,
            bottom_z=BODY_HEIGHT - 0.0035,
        )
    )
    housing = housing.cut(vent_recess)
    for slot_offset in (-0.030, -0.010, 0.010, 0.030):
        vent_slot = _box_at(
            TOP_VENT_WIDTH - 0.028,
            0.010,
            TOP_WALL + 0.010,
            center_x=0.0,
            center_y=TOP_VENT_CENTER_Y + slot_offset,
            bottom_z=BODY_HEIGHT - TOP_WALL - 0.002,
        )
        housing = housing.cut(vent_slot)

    slot_width = BODY_WIDTH - 0.120
    slot_depth = BODY_WALL * 8.0
    slot_height = 0.010
    for index in range(11):
        slot_z = 0.170 + index * 0.024
        slot = (
            cq.Workplane("XY")
            .box(slot_width, slot_depth, slot_height, centered=(True, True, True))
            .translate((0.0, -BODY_DEPTH / 2.0 + slot_depth / 2.0, slot_z))
        )
        housing = housing.cut(slot)

    access_opening_width = SERVICE_PANEL_WIDTH - 0.022
    access_opening_height = SERVICE_PANEL_HEIGHT - 0.028
    access_opening_center_y = SERVICE_PANEL_REAR_Y - 0.010 - access_opening_width / 2.0
    access_opening = (
        cq.Workplane("XY")
        .box(0.020, access_opening_width, access_opening_height, centered=(False, True, False))
        .translate((BODY_WIDTH / 2.0 - 0.012, access_opening_center_y, SERVICE_PANEL_BOTTOM_Z + 0.012))
    )
    housing = housing.cut(access_opening)

    exhaust_hole = (
        cq.Workplane("XY")
        .cylinder(0.045, 0.072, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, BODY_DEPTH / 2.0 - 0.010, 0.515))
    )
    housing = housing.cut(exhaust_hole)

    return housing


def _build_vent_door_shape():
    panel = (
        cq.Workplane("XY")
        .box(VENT_DOOR_WIDTH, VENT_DOOR_DEPTH, VENT_DOOR_THICKNESS, centered=(True, False, False))
        .translate((0.0, -VENT_DOOR_DEPTH, 0.0))
        .edges("|Z")
        .fillet(0.0018)
        .edges("<Y")
        .fillet(0.0012)
    )
    front_lip = (
        cq.Workplane("XY")
        .box(VENT_DOOR_WIDTH - 0.050, 0.010, 0.008, centered=(True, False, False))
        .translate((0.0, -VENT_DOOR_DEPTH - 0.010, 0.0))
        .edges("|Z")
        .fillet(0.0015)
    )
    return panel.union(front_lip)


def _build_service_panel_shape():
    panel = (
        cq.Workplane("XY")
        .box(
            SERVICE_PANEL_THICKNESS,
            SERVICE_PANEL_WIDTH,
            SERVICE_PANEL_HEIGHT,
            centered=(False, False, False),
        )
        .translate((0.0, -SERVICE_PANEL_WIDTH, 0.0))
        .edges("|Z")
        .fillet(0.0018)
    )
    finger_pull = (
        cq.Workplane("XY")
        .cylinder(0.020, 0.012, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((SERVICE_PANEL_THICKNESS - 0.002, -SERVICE_PANEL_WIDTH + 0.016, SERVICE_PANEL_HEIGHT * 0.55))
    )
    return panel.cut(finger_pull)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_air_conditioner")

    housing_white = model.material("housing_white", rgba=(0.91, 0.92, 0.90, 1.0))
    panel_white = model.material("panel_white", rgba=(0.95, 0.95, 0.94, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "portable_ac_housing"),
        material=housing_white,
        name="housing_shell",
    )

    vent_door = model.part("vent_door")
    vent_door.visual(
        mesh_from_cadquery(_build_vent_door_shape(), "portable_ac_vent_door"),
        material=panel_white,
        name="door_leaf",
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        mesh_from_cadquery(_build_service_panel_shape(), "portable_ac_service_panel"),
        material=panel_white,
        name="panel_cover",
    )

    model.articulation(
        "top_vent_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=vent_door,
        origin=Origin(xyz=(0.0, TOP_VENT_HINGE_Y, BODY_HEIGHT + VENT_DOOR_STANDOFF)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=12.0, velocity=1.4),
    )
    model.articulation(
        "service_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=service_panel,
        origin=Origin(
            xyz=(
                BODY_WIDTH / 2.0 + SERVICE_PANEL_STANDOFF,
                SERVICE_PANEL_REAR_Y,
                SERVICE_PANEL_BOTTOM_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=10.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    vent_door = object_model.get_part("vent_door")
    service_panel = object_model.get_part("service_panel")
    top_vent_hinge = object_model.get_articulation("top_vent_hinge")
    service_panel_hinge = object_model.get_articulation("service_panel_hinge")

    ctx.expect_gap(
        vent_door,
        housing,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        name="top vent door rests just above the top deck",
    )
    ctx.expect_overlap(
        vent_door,
        housing,
        axes="xy",
        min_overlap=0.11,
        name="top vent door covers the discharge area",
    )
    ctx.expect_gap(
        service_panel,
        housing,
        axis="x",
        min_gap=0.0,
        max_gap=0.004,
        name="service panel sits against the side housing",
    )
    ctx.expect_overlap(
        service_panel,
        housing,
        axes="yz",
        min_overlap=0.12,
        name="service panel covers the lower rear access bay",
    )

    closed_vent_aabb = ctx.part_world_aabb(vent_door)
    closed_service_aabb = ctx.part_world_aabb(service_panel)
    vent_upper = top_vent_hinge.motion_limits.upper if top_vent_hinge.motion_limits is not None else None
    service_upper = service_panel_hinge.motion_limits.upper if service_panel_hinge.motion_limits is not None else None

    opened_vent_aabb = None
    if vent_upper is not None:
        with ctx.pose({top_vent_hinge: vent_upper}):
            opened_vent_aabb = ctx.part_world_aabb(vent_door)

    opened_service_aabb = None
    if service_upper is not None:
        with ctx.pose({service_panel_hinge: service_upper}):
            opened_service_aabb = ctx.part_world_aabb(service_panel)

    vent_opens_upward = (
        closed_vent_aabb is not None
        and opened_vent_aabb is not None
        and opened_vent_aabb[1][2] > closed_vent_aabb[1][2] + 0.070
    )
    ctx.check(
        "top vent door opens upward",
        vent_opens_upward,
        details=f"closed={closed_vent_aabb}, opened={opened_vent_aabb}",
    )

    service_swings_outward = (
        closed_service_aabb is not None
        and opened_service_aabb is not None
        and opened_service_aabb[1][0] > closed_service_aabb[1][0] + 0.070
    )
    ctx.check(
        "service panel swings out from the side",
        service_swings_outward,
        details=f"closed={closed_service_aabb}, opened={opened_service_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
