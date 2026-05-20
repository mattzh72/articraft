from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_RADIUS = 0.18
BASE_LOWER_H = 0.018
BASE_UPPER_H = 0.028
BASE_HEIGHT = BASE_LOWER_H + BASE_UPPER_H

COLUMN_RADIUS = 0.060
COLUMN_HEIGHT = 0.57

BODY_DEPTH = 0.20
BODY_WIDTH = 0.26
BODY_HEIGHT = 0.245
BODY_BASE_Z = 0.60
BODY_TOP_Z = BODY_BASE_Z + BODY_HEIGHT
BODY_FRONT_X = BODY_DEPTH / 2.0

NECK_RADIUS = 0.058
NECK_HEIGHT = 0.055

GLOBE_RADIUS = 0.195
GLOBE_THICKNESS = 0.007
GLOBE_CENTER_Z = 1.065
GLOBE_TOP_OPEN_RADIUS = 0.070
GLOBE_BOTTOM_OPEN_RADIUS = 0.056

TOP_COLLAR_OUTER_RADIUS = 0.085
TOP_COLLAR_INNER_RADIUS = 0.066
TOP_COLLAR_HEIGHT = 0.014
TOP_COLLAR_Z0 = 1.237
TOP_COLLAR_TOP_Z = TOP_COLLAR_Z0 + TOP_COLLAR_HEIGHT

LID_RADIUS = 0.078
LID_CENTER_X = 0.077
LID_HINGE_X = -0.074
LID_HINGE_Z = TOP_COLLAR_TOP_Z

COIN_PLATE_THICKNESS = 0.016
COIN_PLATE_WIDTH = 0.165
COIN_PLATE_HEIGHT = 0.205
KNOB_AXIS_X = 0.085
KNOB_AXIS_Z = BODY_BASE_Z + 0.185

CHUTE_OUTER_DEPTH = 0.072
CHUTE_OUTER_WIDTH = 0.124
CHUTE_OUTER_HEIGHT = 0.094
CHUTE_BASE_Z = BODY_BASE_Z + 0.028
CHUTE_FRONT_X = BODY_FRONT_X + CHUTE_OUTER_DEPTH - 0.006
FLAP_HINGE_X = CHUTE_FRONT_X + 0.016
FLAP_HINGE_Z = CHUTE_BASE_Z + 0.010


def _box(size_x: float, size_y: float, size_z: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def _rounded_box(
    size_x: float,
    size_y: float,
    size_z: float,
    radius: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(size_x, size_y, size_z)
    if radius > 0.0 and min(size_x, size_y) > (radius * 2.5):
        solid = solid.edges("|Z").fillet(radius)
    return solid.translate(center)


def _cyl_z(
    radius: float,
    height: float,
    z0: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z0))


def _cyl_x(
    radius: float,
    length: float,
    x0: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, y, z))


def _cyl_y(
    radius: float,
    length: float,
    y0: float,
    *,
    x: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y0, z))


def _build_body_shape() -> cq.Workplane:
    base = _cyl_z(BASE_RADIUS, BASE_LOWER_H, 0.0)
    base = base.union(_cyl_z(0.132, BASE_UPPER_H, BASE_LOWER_H))

    column = _cyl_z(COLUMN_RADIUS, COLUMN_HEIGHT, BASE_HEIGHT)
    shoulder = _cyl_z(0.086, 0.036, BODY_BASE_Z - 0.018)

    housing = _rounded_box(
        BODY_DEPTH,
        BODY_WIDTH,
        BODY_HEIGHT,
        0.024,
        (0.0, 0.0, BODY_BASE_Z + (BODY_HEIGHT / 2.0)),
    )
    neck = _cyl_z(0.070, 0.016, BODY_TOP_Z - 0.010)
    neck = neck.union(_cyl_z(NECK_RADIUS, NECK_HEIGHT, BODY_TOP_Z - 0.004))

    chute = _rounded_box(
        CHUTE_OUTER_DEPTH,
        CHUTE_OUTER_WIDTH,
        CHUTE_OUTER_HEIGHT,
        0.010,
        (
            BODY_FRONT_X + (CHUTE_OUTER_DEPTH / 2.0) - 0.006,
            0.0,
            CHUTE_BASE_Z + (CHUTE_OUTER_HEIGHT / 2.0),
        ),
    )
    chute_void = _box(
        0.090,
        0.088,
        0.062,
        (
            BODY_FRONT_X + 0.048,
            0.0,
            CHUTE_BASE_Z + 0.046,
        ),
    )

    knob_tunnel = _cyl_x(0.0135, 0.150, -0.030, z=KNOB_AXIS_Z)

    flap_knuckles = _cyl_y(0.005, 0.018, -0.036, x=FLAP_HINGE_X, z=FLAP_HINGE_Z)
    flap_knuckles = flap_knuckles.union(_cyl_y(0.005, 0.018, 0.018, x=FLAP_HINGE_X, z=FLAP_HINGE_Z))
    flap_ears = _box(0.024, 0.028, 0.020, (FLAP_HINGE_X - 0.008, -0.031, FLAP_HINGE_Z))
    flap_ears = flap_ears.union(_box(0.024, 0.028, 0.020, (FLAP_HINGE_X - 0.008, 0.031, FLAP_HINGE_Z)))

    body = base.union(column).union(shoulder).union(housing).union(neck).union(chute).union(flap_ears).union(flap_knuckles)
    body = body.cut(chute_void)
    body = body.cut(knob_tunnel)
    return body


def _build_coin_plate_shape() -> cq.Workplane:
    plate = _rounded_box(
        COIN_PLATE_THICKNESS,
        COIN_PLATE_WIDTH,
        COIN_PLATE_HEIGHT,
        0.016,
        (
            BODY_FRONT_X + (COIN_PLATE_THICKNESS / 2.0) + 0.001,
            0.0,
            BODY_BASE_Z + 0.142,
        ),
    )

    boss = _cyl_x(0.039, 0.018, BODY_FRONT_X + 0.004, z=KNOB_AXIS_Z)
    boss = boss.union(_cyl_x(0.028, 0.020, BODY_FRONT_X + 0.017, z=KNOB_AXIS_Z))

    coin_slot_hood = _box(
        0.018,
        0.066,
        0.020,
        (
            BODY_FRONT_X + 0.011,
            0.0,
            KNOB_AXIS_Z + 0.068,
        ),
    )

    slot_cut = _box(
        0.030,
        0.044,
        0.006,
        (
            BODY_FRONT_X + 0.010,
            0.0,
            KNOB_AXIS_Z + 0.066,
        ),
    )
    knob_hole = _cyl_x(0.0145, 0.060, BODY_FRONT_X - 0.010, z=KNOB_AXIS_Z)

    return plate.union(boss).union(coin_slot_hood).cut(slot_cut).cut(knob_hole)


def _build_globe_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").sphere(GLOBE_RADIUS).translate((0.0, 0.0, GLOBE_CENTER_Z))
    inner = cq.Workplane("XY").sphere(GLOBE_RADIUS - GLOBE_THICKNESS).translate((0.0, 0.0, GLOBE_CENTER_Z))
    globe = outer.cut(inner)
    globe = globe.cut(_cyl_z(GLOBE_TOP_OPEN_RADIUS, 0.260, GLOBE_CENTER_Z + 0.020))
    globe = globe.cut(_cyl_z(GLOBE_BOTTOM_OPEN_RADIUS, 0.260, GLOBE_CENTER_Z - GLOBE_RADIUS - 0.025))
    return globe


def _build_top_collar_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(TOP_COLLAR_OUTER_RADIUS)
        .circle(TOP_COLLAR_INNER_RADIUS)
        .extrude(TOP_COLLAR_HEIGHT)
        .translate((0.0, 0.0, TOP_COLLAR_Z0))
    )
    ring = ring.cut(_box(0.038, 0.034, 0.020, (LID_HINGE_X + 0.008, 0.0, TOP_COLLAR_Z0 + 0.010)))

    hinge_bridge = _box(0.020, 0.016, 0.010, (LID_HINGE_X + 0.006, -0.026, TOP_COLLAR_Z0 + 0.007))
    hinge_bridge = hinge_bridge.union(
        _box(0.020, 0.016, 0.010, (LID_HINGE_X + 0.006, 0.026, TOP_COLLAR_Z0 + 0.007))
    )
    hinge_knuckles = _cyl_y(0.006, 0.020, -0.035, x=LID_HINGE_X, z=LID_HINGE_Z)
    hinge_knuckles = hinge_knuckles.union(_cyl_y(0.006, 0.020, 0.015, x=LID_HINGE_X, z=LID_HINGE_Z))
    return ring.union(hinge_bridge).union(hinge_knuckles)


def _build_lid_cap_shape() -> cq.Workplane:
    cap = _cyl_z(LID_RADIUS, 0.008, 0.006, x=LID_CENTER_X)
    cap = cap.union(_cyl_z(0.066, 0.006, 0.014, x=LID_CENTER_X))
    finger_lip = _box(0.012, 0.050, 0.010, (LID_CENTER_X + 0.064, 0.0, 0.010))
    return cap.union(finger_lip)


def _build_lid_hinge_shape() -> cq.Workplane:
    rear_leaf = _box(0.038, 0.032, 0.004, (0.026, 0.0, 0.013))
    hinge_arm = _box(0.022, 0.014, 0.010, (0.010, -0.006, 0.006))
    knuckle = _cyl_y(0.006, 0.026, -0.013, x=0.0, z=0.0)
    return rear_leaf.union(hinge_arm).union(knuckle)


def _build_knob_cap_shape() -> cq.Workplane:
    hub = _cyl_x(0.024, 0.016, 0.054)
    cap = _cyl_x(0.045, 0.030, 0.068)
    face = _cyl_x(0.041, 0.010, 0.098)
    ridge = _cyl_x(0.048, 0.008, 0.066)
    slot = _box(0.014, 0.048, 0.006, (0.104, 0.0, 0.0))
    return hub.union(cap).union(face).union(ridge).cut(slot)


def _build_flap_panel_shape() -> cq.Workplane:
    panel = _box(0.005, 0.092, 0.058, (0.014, 0.0, 0.029))
    pull_lip = _box(0.012, 0.092, 0.010, (0.019, 0.0, 0.051))
    return panel.union(pull_lip)


def _build_flap_leaf_shape() -> cq.Workplane:
    return _box(0.028, 0.092, 0.008, (0.0185, 0.0, 0.004))


def _build_flap_hinge_shape() -> cq.Workplane:
    return _cyl_y(0.0045, 0.030, -0.015, x=0.0, z=0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_gumball_machine")

    model.material("body_red", rgba=(0.78, 0.07, 0.10, 1.0))
    model.material("cast_metal", rgba=(0.76, 0.76, 0.79, 1.0))
    model.material("globe_clear", rgba=(0.84, 0.93, 1.00, 0.28))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_pedestal"),
        material="body_red",
        name="pedestal",
    )
    body.visual(
        mesh_from_cadquery(_build_coin_plate_shape(), "coin_plate"),
        material="cast_metal",
        name="coin_plate",
    )
    body.visual(
        mesh_from_cadquery(_build_globe_shape(), "globe"),
        material="globe_clear",
        name="globe",
    )
    body.visual(
        mesh_from_cadquery(_build_top_collar_shape(), "top_collar"),
        material="cast_metal",
        name="top_collar",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_cap_shape(), "refill_lid_cap"),
        material="cast_metal",
        name="lid",
    )
    lid.visual(
        mesh_from_cadquery(_build_lid_hinge_shape(), "refill_lid_hinge"),
        material="cast_metal",
        name="hinge",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.104),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cast_metal",
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cast_metal",
        name="collar",
    )
    knob.visual(
        mesh_from_cadquery(_build_knob_cap_shape(), "dispense_knob"),
        material="cast_metal",
        name="cap",
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_build_flap_panel_shape(), "chute_flap_panel"),
        material="cast_metal",
        name="flap",
    )
    flap.visual(
        mesh_from_cadquery(_build_flap_leaf_shape(), "chute_flap_leaf"),
        material="cast_metal",
        name="leaf",
    )
    flap.visual(
        mesh_from_cadquery(_build_flap_hinge_shape(), "chute_flap_hinge"),
        material="cast_metal",
        name="hinge",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=6.0, velocity=1.8),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(KNOB_AXIS_X, 0.0, KNOB_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=3.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("knob")
    flap = object_model.get_part("flap")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="top_collar",
        elem_b="hinge",
        reason="The refill lid uses a simplified interleaved rear hinge barrel at the globe crown.",
    )
    ctx.allow_overlap(
        body,
        flap,
        elem_a="pedestal",
        elem_b="hinge",
        reason="The retrieval flap hinge barrels are simplified as an interleaved lower front hinge.",
    )

    lid_hinge = object_model.get_articulation("lid_hinge")
    flap_hinge = object_model.get_articulation("flap_hinge")

    with ctx.pose({lid_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid",
            negative_elem="top_collar",
            max_gap=0.010,
            max_penetration=0.002,
            name="refill lid seats on the globe collar",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid",
            elem_b="top_collar",
            min_overlap=0.090,
            name="refill lid covers the globe opening",
        )
        ctx.expect_gap(
            flap,
            body,
            axis="x",
            positive_elem="flap",
            negative_elem="pedestal",
            max_gap=0.010,
            max_penetration=0.002,
            name="retrieval flap closes against the chute housing",
        )
        ctx.expect_gap(
            knob,
            body,
            axis="x",
            positive_elem="cap",
            negative_elem="coin_plate",
            min_gap=0.000,
            max_gap=0.020,
            name="dispense knob sits just proud of the coin mechanism plate",
        )
        ctx.expect_within(
            knob,
            body,
            axes="yz",
            inner_elem="shaft",
            outer_elem="coin_plate",
            margin=0.012,
            name="dispense shaft stays centered within the coin mechanism footprint",
        )

    lid_limits = lid_hinge.motion_limits
    flap_limits = flap_hinge.motion_limits

    closed_lid_aabb = None
    open_lid_aabb = None
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0}):
            closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid")
        ctx.check(
            "refill lid opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.070,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    closed_flap_aabb = None
    open_flap_aabb = None
    if flap_limits is not None and flap_limits.upper is not None:
        with ctx.pose({flap_hinge: 0.0}):
            closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap")
        with ctx.pose({flap_hinge: flap_limits.upper}):
            open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap")
        ctx.check(
            "retrieval flap swings forward",
            closed_flap_aabb is not None
            and open_flap_aabb is not None
            and open_flap_aabb[1][0] > closed_flap_aabb[1][0] + 0.025,
            details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
