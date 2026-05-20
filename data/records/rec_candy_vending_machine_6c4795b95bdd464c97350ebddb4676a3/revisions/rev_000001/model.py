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


BASE_WIDTH = 0.228
BASE_DEPTH = 0.190
BASE_BODY_HEIGHT = 0.118
SHOULDER_RING_HEIGHT = 0.016

CANISTER_OUTER_RADIUS = 0.106
CANISTER_INNER_RADIUS = 0.101
CANISTER_HEIGHT = 0.250
CANISTER_APOTHEM = CANISTER_OUTER_RADIUS * math.cos(math.pi / 8.0)

CAP_RADIUS = 0.110
CAP_THICKNESS = 0.012
CAP_CLEARANCE = 0.0

DOOR_WIDTH = 0.092
DOOR_HEIGHT = 0.074
DOOR_THICKNESS = 0.008


def _regular_polygon_points(radius: float, *, sides: int = 8, rotation: float = math.pi / 8.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(rotation + (2.0 * math.pi * index) / sides),
            radius * math.sin(rotation + (2.0 * math.pi * index) / sides),
        )
        for index in range(sides)
    ]


def _profile(points: list[tuple[float, float]]) -> cq.Workplane:
    wire = cq.Workplane("XY").moveTo(*points[0])
    for x_coord, y_coord in points[1:]:
        wire = wire.lineTo(x_coord, y_coord)
    return wire.close()


def _octagon_prism(radius: float, height: float) -> cq.Workplane:
    return _profile(_regular_polygon_points(radius)).extrude(height)


def _base_body_shape() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, 0.082, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )
    upper = (
        cq.Workplane("XY")
        .box(0.186, 0.150, 0.036, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, 0.082))
    )
    return lower.union(upper)


def _shoulder_ring_shape() -> cq.Workplane:
    outer = _octagon_prism(0.112, SHOULDER_RING_HEIGHT)
    inner = _octagon_prism(0.1015, SHOULDER_RING_HEIGHT)
    return outer.cut(inner)


def _canister_shape() -> cq.Workplane:
    outer = _octagon_prism(CANISTER_OUTER_RADIUS, CANISTER_HEIGHT)
    inner = _octagon_prism(CANISTER_INNER_RADIUS, CANISTER_HEIGHT)
    shell = outer.cut(inner)

    lower_band = _octagon_prism(CANISTER_OUTER_RADIUS, 0.020).cut(_octagon_prism(0.096, 0.020))
    top_band = (
        _octagon_prism(CANISTER_OUTER_RADIUS, 0.016)
        .cut(_octagon_prism(0.0975, 0.016))
        .translate((0.0, 0.0, CANISTER_HEIGHT - 0.016))
    )
    hinge_bridge = (
        cq.Workplane("XY")
        .box(0.074, 0.012, 0.010, centered=(True, True, False))
        .translate((0.0, -CANISTER_APOTHEM, CANISTER_HEIGHT - 0.010))
    )
    return shell.union(lower_band).union(top_band).union(hinge_bridge)


def _cap_shape() -> cq.Workplane:
    points = _regular_polygon_points(CAP_RADIUS)
    min_y = min(y_coord for _, y_coord in points)
    max_y = max(y_coord for _, y_coord in points)
    shifted = [(x_coord, y_coord - min_y) for x_coord, y_coord in points]

    panel = _profile(shifted).extrude(CAP_THICKNESS)
    hinge_leaf = (
        cq.Workplane("XY")
        .box(0.074, 0.010, 0.008, centered=(True, True, False))
        .translate((0.0, -0.005, 0.0))
    )
    finger_lip = (
        cq.Workplane("XY")
        .box(0.060, 0.009, 0.008, centered=(True, True, False))
        .translate((0.0, max_y - min_y + 0.0045, CAP_THICKNESS - 0.008))
    )
    return panel.union(hinge_leaf).union(finger_lip)


def _chute_tray_shape() -> cq.Workplane:
    tray = (
        cq.Workplane("XZ")
        .moveTo(-0.032, 0.0)
        .lineTo(0.032, 0.0)
        .lineTo(0.029, 0.005)
        .lineTo(0.023, 0.014)
        .lineTo(-0.023, 0.014)
        .lineTo(-0.029, 0.005)
        .close()
        .extrude(0.034, both=True)
        .translate((0.0, BASE_DEPTH / 2.0 + 0.017, 0.014))
    )
    lip = (
        cq.Workplane("XY")
        .box(0.060, 0.006, 0.005, centered=(True, True, False))
        .translate((0.0, BASE_DEPTH / 2.0 + 0.031, 0.024))
    )
    return tray.union(lip)


def _chute_hood_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(-0.038, 0.0)
        .lineTo(0.038, 0.0)
        .lineTo(0.030, 0.018)
        .lineTo(-0.030, 0.018)
        .close()
        .extrude(0.026, both=True)
        .translate((0.0, BASE_DEPTH / 2.0 + 0.011, 0.026))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="octagonal_jellybean_dispenser")

    cast_metal = model.material("cast_metal", rgba=(0.56, 0.53, 0.49, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.24, 0.25, 1.0))
    coin_metal = model.material("coin_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    chute_dark = model.material("chute_dark", rgba=(0.22, 0.18, 0.16, 1.0))
    clear_tint = model.material("clear_tint", rgba=(0.76, 0.92, 0.97, 0.28))
    cap_red = model.material("cap_red", rgba=(0.70, 0.12, 0.10, 1.0))
    knob_red = model.material("knob_red", rgba=(0.78, 0.10, 0.10, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_body_shape(), "base_body"), material=cast_metal, name="body")
    base.visual(
        mesh_from_cadquery(_shoulder_ring_shape(), "shoulder_ring"),
        origin=Origin(xyz=(0.0, 0.0, BASE_BODY_HEIGHT - SHOULDER_RING_HEIGHT)),
        material=dark_metal,
        name="shoulder_ring",
    )
    base.visual(
        Box((0.072, 0.016, 0.060)),
        origin=Origin(xyz=(0.0, BASE_DEPTH / 2.0 - 0.008, 0.084)),
        material=coin_metal,
        name="coin_plate",
    )
    base.visual(
        Box((0.026, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, BASE_DEPTH / 2.0 + 0.0005, 0.100)),
        material=dark_metal,
        name="coin_slot",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, BASE_DEPTH / 2.0 + 0.004, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_metal,
        name="hub",
    )
    base.visual(mesh_from_cadquery(_chute_hood_shape(), "chute_hood"), material=cast_metal, name="chute_hood")
    base.visual(mesh_from_cadquery(_chute_tray_shape(), "chute_tray"), material=chute_dark, name="chute_tray")

    canister = model.part("canister")
    canister.visual(mesh_from_cadquery(_canister_shape(), "canister_shell"), material=clear_tint, name="canister_shell")

    cap = model.part("cap")
    cap.visual(mesh_from_cadquery(_cap_shape(), "refill_cap"), material=cap_red, name="cap_panel")

    knob = model.part("dispense_knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_red,
        name="knob_body",
    )
    knob.visual(
        Box((0.010, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.015, 0.010)),
        material=coin_metal,
        name="pointer_rib",
    )

    door = model.part("service_door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0)),
        material=cast_metal,
        name="door_panel",
    )
    door.visual(
        Box((0.012, 0.005, 0.028)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.014, DOOR_THICKNESS + 0.0025, DOOR_HEIGHT / 2.0)),
        material=dark_metal,
        name="door_pull",
    )

    model.articulation(
        "base_to_canister",
        ArticulationType.FIXED,
        parent=base,
        child=canister,
        origin=Origin(xyz=(0.0, 0.0, BASE_BODY_HEIGHT)),
    )
    model.articulation(
        "canister_to_cap",
        ArticulationType.REVOLUTE,
        parent=canister,
        child=cap,
        origin=Origin(xyz=(0.0, -CANISTER_APOTHEM, CANISTER_HEIGHT + CAP_CLEARANCE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.4, lower=0.0, upper=1.30),
    )
    model.articulation(
        "base_to_dispense_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.0, BASE_DEPTH / 2.0 + 0.008, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )
    model.articulation(
        "base_to_service_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=door,
        origin=Origin(
            xyz=(
                -DOOR_WIDTH / 2.0,
                -(BASE_DEPTH / 2.0 + DOOR_THICKNESS),
                0.026,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    canister = object_model.get_part("canister")
    cap = object_model.get_part("cap")
    knob = object_model.get_part("dispense_knob")
    door = object_model.get_part("service_door")

    cap_hinge = object_model.get_articulation("canister_to_cap")
    door_hinge = object_model.get_articulation("base_to_service_door")
    knob_joint = object_model.get_articulation("base_to_dispense_knob")

    ctx.expect_gap(
        canister,
        base,
        axis="z",
        positive_elem="canister_shell",
        negative_elem="shoulder_ring",
        max_gap=0.0025,
        max_penetration=0.0,
        name="canister sits on shoulder ring",
    )
    ctx.expect_gap(
        cap,
        canister,
        axis="z",
        positive_elem="cap_panel",
        negative_elem="canister_shell",
        max_gap=0.004,
        max_penetration=0.0,
        name="closed cap clears canister rim",
    )
    ctx.expect_gap(
        knob,
        base,
        axis="y",
        positive_elem="shaft",
        negative_elem="hub",
        max_gap=0.001,
        max_penetration=0.0,
        name="dispense shaft sits on the front hub",
    )
    ctx.expect_overlap(
        door,
        base,
        axes="xz",
        elem_a="door_panel",
        elem_b="body",
        min_overlap=0.060,
        name="service door covers the back access area",
    )
    ctx.check(
        "dispense knob is continuous",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"limits={knob_joint.motion_limits}",
    )

    closed_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_panel")
    with ctx.pose({cap_hinge: 1.20}):
        open_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_panel")
    ctx.check(
        "refill cap lifts upward",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.080,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "service door swings out from the back",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.035,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
