from __future__ import annotations

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.46
BASE_DEPTH = 0.42
BASE_HEIGHT = 0.48
DOOR_WIDTH = 0.23
DOOR_HEIGHT = 0.20
DOOR_BOTTOM_Z = 0.10
COLUMN_WIDTH = 0.16
COLUMN_DEPTH = 0.12
COLUMN_HEIGHT = 1.10
COLUMN_CENTER_Y = 0.09
SPINE_WIDTH = 0.08
SPINE_DEPTH = 0.035
SPINE_CENTER_Y = -0.19
TOP_BRIDGE_DEPTH = 0.24
TOTAL_HEIGHT = BASE_HEIGHT + COLUMN_HEIGHT
BIN_WIDTH = 0.34
BIN_DEPTH = 0.28
BIN_HEIGHT = 0.24
BIN_WALL = 0.006
BIN_NOTCH_WIDTH = 0.18
BIN_NOTCH_DEPTH = 0.18
BIN_BOTTOMS = (0.58, 0.87, 1.16)
BIN_CENTER_Y = 0.025
KNOB_DIAMETER = 0.075
KNOB_DEPTH = 0.048
DOOR_THICKNESS = 0.018


def _zbox(
    sx: float,
    sy: float,
    sz: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z0: float = 0.0,
):
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz, centered=(True, True, False))
        .translate((x, y, z0))
    )


def _build_body_shape():
    body = _zbox(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)

    pickup_cut = _zbox(
        DOOR_WIDTH + 0.018,
        0.18,
        DOOR_HEIGHT + 0.024,
        y=BASE_DEPTH * 0.5 - 0.09 + 0.002,
        z0=DOOR_BOTTOM_Z - 0.012,
    )
    body = body.cut(pickup_cut)

    body = body.union(
        _zbox(
            COLUMN_WIDTH,
            COLUMN_DEPTH,
            COLUMN_HEIGHT,
            y=COLUMN_CENTER_Y,
            z0=BASE_HEIGHT,
        )
    )
    body = body.union(
        _zbox(
            SPINE_WIDTH,
            SPINE_DEPTH,
            COLUMN_HEIGHT + 0.04,
            y=SPINE_CENTER_Y,
            z0=BASE_HEIGHT,
        )
    )
    body = body.union(
        _zbox(
            0.22,
            TOP_BRIDGE_DEPTH,
            0.05,
            y=-0.03,
            z0=TOTAL_HEIGHT - 0.05,
        )
    )

    for bin_bottom in BIN_BOTTOMS:
        body = body.union(_zbox(0.18, 0.03, 0.018, y=-0.133, z0=bin_bottom + 0.01))
        body = body.union(_zbox(0.10, 0.07, 0.02, y=-0.1525, z0=bin_bottom + 0.015))
        body = body.union(_zbox(0.10, 0.04, 0.11, y=-0.1725, z0=bin_bottom + 0.005))

    return body


def _build_bin_shell_shape():
    outer = (
        cq.Workplane("XY")
        .box(BIN_WIDTH, BIN_DEPTH, BIN_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )
    notch = _zbox(
        BIN_NOTCH_WIDTH,
        BIN_NOTCH_DEPTH,
        BIN_HEIGHT + 0.02,
        y=BIN_DEPTH * 0.5 - BIN_NOTCH_DEPTH * 0.5 + 0.002,
        z0=-0.01,
    )
    outer = outer.cut(notch)

    inner = (
        cq.Workplane("XY")
        .box(
            BIN_WIDTH - 2.0 * BIN_WALL,
            BIN_DEPTH - 2.0 * BIN_WALL,
            BIN_HEIGHT - 2.0 * BIN_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BIN_WALL))
        .edges("|Z")
        .fillet(0.016)
    )
    inner = inner.cut(
        _zbox(
            BIN_NOTCH_WIDTH - 2.0 * BIN_WALL,
            BIN_NOTCH_DEPTH - BIN_WALL,
            BIN_HEIGHT,
            y=BIN_DEPTH * 0.5 - (BIN_NOTCH_DEPTH - BIN_WALL) * 0.5 + 0.001,
            z0=BIN_WALL,
        )
    )

    shell = outer.cut(inner)

    lip = _zbox(
        BIN_WIDTH + 0.01,
        BIN_DEPTH + 0.01,
        0.01,
        z0=BIN_HEIGHT - 0.01,
    ).cut(
        _zbox(
            BIN_WIDTH - 0.018,
            BIN_DEPTH - 0.018,
            0.012,
            z0=BIN_HEIGHT - 0.011,
        )
    )
    lip = lip.cut(
        _zbox(
            BIN_NOTCH_WIDTH + 0.01,
            BIN_NOTCH_DEPTH + 0.01,
            0.02,
            y=BIN_DEPTH * 0.5 - (BIN_NOTCH_DEPTH + 0.01) * 0.5 + 0.002,
            z0=BIN_HEIGHT - 0.015,
        )
    )
    return shell.union(lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_canister_candy_vending_machine")

    frame_finish = model.material("frame_finish", rgba=(0.66, 0.12, 0.10, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.93, 0.93, 0.95, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.85, 0.92, 0.97, 0.35))
    knob_finish = model.material("knob_finish", rgba=(0.96, 0.96, 0.94, 1.0))
    door_finish = model.material("door_finish", rgba=(0.87, 0.87, 0.89, 1.0))
    door_window = model.material("door_window", rgba=(0.70, 0.84, 0.94, 0.35))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "vending_body"),
        material=frame_finish,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(
            _zbox(DOOR_WIDTH + 0.05, 0.025, DOOR_HEIGHT + 0.08, y=0.198, z0=0.08),
            "pickup_frame",
        ),
        material=trim_finish,
        name="pickup_frame",
    )
    body.visual(
        mesh_from_cadquery(
            _zbox(0.12, 0.02, TOTAL_HEIGHT - BASE_HEIGHT - 0.08, y=0.152, z0=BASE_HEIGHT + 0.04),
            "column_face",
        ),
        material=trim_finish,
        name="column_face",
    )

    bin_mesh = mesh_from_cadquery(_build_bin_shell_shape(), "candy_bin")
    for index, bin_bottom in enumerate(BIN_BOTTOMS):
        bin_part = model.part(f"bin_{index}")
        bin_part.visual(
            bin_mesh,
            origin=Origin(xyz=(0.0, BIN_CENTER_Y, 0.0)),
            material=clear_bin,
            name="bin_shell",
        )
        model.articulation(
            f"body_to_bin_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=bin_part,
            origin=Origin(xyz=(0.0, 0.0, bin_bottom)),
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            KNOB_DIAMETER,
            KNOB_DEPTH,
            body_style="skirted",
            top_diameter=0.06,
            skirt=KnobSkirt(0.086, 0.008, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0024),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.001),
            center=False,
        ),
        "selector_knob",
    )
    for index, bin_bottom in enumerate(BIN_BOTTOMS):
        knob_part = model.part(f"selector_knob_{index}")
        knob_part.visual(
            knob_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=knob_finish,
            name="knob",
        )
        knob_part.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=trim_finish,
            name="hub",
        )
        model.articulation(
            f"body_to_selector_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob_part,
            origin=Origin(xyz=(0.0, 0.162, bin_bottom + 0.095)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=6.0),
        )

    door = model.part("pickup_door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0)),
        material=door_finish,
        name="door_panel",
    )
    door.visual(
        Box((DOOR_WIDTH * 0.78, 0.004, DOOR_HEIGHT * 0.52)),
        origin=Origin(xyz=(0.0, DOOR_THICKNESS + 0.002, DOOR_HEIGHT * 0.58)),
        material=door_window,
        name="door_window",
    )
    door.visual(
        Box((0.09, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, DOOR_THICKNESS + 0.006, DOOR_HEIGHT - 0.03)),
        material=trim_finish,
        name="door_pull",
    )
    model.articulation(
        "body_to_pickup_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, 0.2105, DOOR_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("pickup_door")
    door_joint = object_model.get_articulation("body_to_pickup_door")

    for index in range(3):
        ctx.allow_isolated_part(
            f"bin_{index}",
            reason="Each clear canister is intentionally modeled as a removable hopper held slightly off the simplified rear carriage so it stays visually separate from the mechanism column.",
        )

    ctx.expect_origin_gap("bin_1", "bin_0", axis="z", min_gap=0.28, max_gap=0.30, name="lower canisters remain stacked with a real vertical pitch")
    ctx.expect_origin_gap("bin_2", "bin_1", axis="z", min_gap=0.28, max_gap=0.30, name="upper canisters remain stacked with a real vertical pitch")

    for index in range(3):
        knob = object_model.get_part(f"selector_knob_{index}")
        ctx.expect_contact(
            knob,
            body,
            elem_a="hub",
            elem_b="column_face",
            contact_tol=0.001,
            name=f"selector knob {index} seats against the column face",
        )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_panel",
            negative_elem="pickup_frame",
            max_gap=0.001,
            max_penetration=0.0,
            name="pickup door closes flush to the front frame",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.0}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "pickup door swings outward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][1] > closed_aabb[1][1] + 0.08,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
