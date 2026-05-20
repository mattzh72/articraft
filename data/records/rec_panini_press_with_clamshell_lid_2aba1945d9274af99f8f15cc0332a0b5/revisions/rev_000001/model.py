from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_DEPTH = 0.37
BASE_WIDTH = 0.42
BASE_HEIGHT = 0.118
BASE_CORNER_RADIUS = 0.018

GRILL_X_OFFSET = 0.010
GRILL_DEPTH = 0.305
GRILL_WIDTH = 0.336
GRILL_POCKET_DEPTH = 0.012
GRILL_PLATE_THICKNESS = 0.010
GRILL_RIDGE_HEIGHT = 0.004
GRILL_RIDGE_COUNT = 8

FRONT_RECESS_DEPTH = 0.050
FRONT_RECESS_WIDTH = 0.100
FRONT_RECESS_HEIGHT = 0.058
FRONT_RECESS_Z = 0.056

HINGE_X = -0.155
HINGE_Z = 0.170
HINGE_BARREL_RADIUS = 0.011
BASE_BARREL_LENGTH = 0.048
UPPER_BARREL_LENGTH = 0.200

UPPER_DEPTH = 0.330
UPPER_WIDTH = 0.350
UPPER_HEIGHT = 0.084
UPPER_WALL = 0.008
UPPER_START_X = 0.008
UPPER_CENTER_X = UPPER_START_X + UPPER_DEPTH / 2.0
UPPER_CENTER_Z = -0.004
UPPER_GRILL_DEPTH = UPPER_DEPTH - 2.0 * UPPER_WALL
UPPER_GRILL_WIDTH = UPPER_WIDTH - 2.0 * UPPER_WALL
UPPER_GRILL_Z = -0.038

HANDLE_FRONT_X = 0.060
HANDLE_SPAN = 0.220
HANDLE_TUBE_RADIUS = 0.008
HANDLE_FOOT_DEPTH = 0.016
HANDLE_Z = -0.010

KNOB_SHAFT_LENGTH = 0.012
KNOB_SHAFT_RADIUS = 0.006
KNOB_ORIGIN_X = BASE_DEPTH / 2.0 - 0.020


def _cylinder_x(length: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )


def _cylinder_y(length: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )


def _grill_plate(depth: float, width: float, ridge_count: int) -> cq.Workplane:
    plate = cq.Workplane("XY").box(depth, width, GRILL_PLATE_THICKNESS)
    usable_depth = depth - 0.055
    ridge_pitch = usable_depth / (ridge_count - 1)
    ridge_start = -usable_depth / 2.0
    ridge_width = 0.010
    ridge_span = width * 0.90

    for ridge_index in range(ridge_count):
        ridge_x = ridge_start + ridge_index * ridge_pitch
        ridge = (
            cq.Workplane("XY")
            .box(ridge_width, ridge_span, GRILL_RIDGE_HEIGHT)
            .translate((ridge_x, 0.0, GRILL_PLATE_THICKNESS / 2.0 + GRILL_RIDGE_HEIGHT / 2.0))
        )
        plate = plate.union(ridge)

    return plate


def _base_housing_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT)
        .edges("|Z")
        .fillet(BASE_CORNER_RADIUS)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
    )

    grill_pocket = (
        cq.Workplane("XY")
        .box(GRILL_DEPTH + 0.004, GRILL_WIDTH + 0.004, GRILL_POCKET_DEPTH + 0.004)
        .translate(
            (
                GRILL_X_OFFSET,
                0.0,
                BASE_HEIGHT - GRILL_POCKET_DEPTH / 2.0 + 0.002,
            )
        )
    )
    front_recess = (
        cq.Workplane("XY")
        .box(FRONT_RECESS_DEPTH + 0.008, FRONT_RECESS_WIDTH, FRONT_RECESS_HEIGHT)
        .translate(
            (
                BASE_DEPTH / 2.0 - FRONT_RECESS_DEPTH / 2.0 + 0.006,
                0.0,
                FRONT_RECESS_Z,
            )
        )
    )

    return body.cut(grill_pocket).cut(front_recess)


def _hinge_bracket_shape() -> cq.Workplane:
    bracket = (
        cq.Workplane("XY")
        .box(0.022, 0.406, 0.020)
        .translate((-0.179, 0.0, BASE_HEIGHT + 0.010))
    )

    for y_sign in (-1.0, 1.0):
        arm = (
            cq.Workplane("XY")
            .box(0.030, 0.016, 0.072)
            .translate((-0.170, y_sign * 0.198, BASE_HEIGHT + 0.036))
        )
        barrel_y = -0.208 if y_sign < 0.0 else 0.190
        barrel = _cylinder_y(0.018, HINGE_BARREL_RADIUS).translate(
            (HINGE_X, barrel_y, HINGE_Z)
        )
        bracket = bracket.union(arm).union(barrel)

    for spacer_start in (-0.103, 0.100):
        spacer = _cylinder_y(0.003, HINGE_BARREL_RADIUS * 0.96).translate(
            (HINGE_X, spacer_start, HINGE_Z)
        )
        bracket = bracket.union(spacer)

    for connector_y in (-0.104, 0.104):
        rear_tie = (
            cq.Workplane("XY")
            .box(0.02444, 0.004, 0.006)
            .translate((-0.17778, connector_y, HINGE_Z))
        )
        drop_post = (
            cq.Workplane("XY")
            .box(0.010, 0.004, 0.032)
            .translate((-0.179, connector_y, 0.154))
        )
        bracket = bracket.union(rear_tie).union(drop_post)

    return bracket


def _upper_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(UPPER_DEPTH, UPPER_WIDTH, UPPER_HEIGHT)
        .edges("|Z")
        .fillet(0.012)
        .translate((UPPER_CENTER_X, 0.0, UPPER_CENTER_Z))
    )
    inner = (
        cq.Workplane("XY")
        .box(UPPER_DEPTH - 2.0 * UPPER_WALL, UPPER_WIDTH - 2.0 * UPPER_WALL, UPPER_HEIGHT - UPPER_WALL)
        .translate((UPPER_CENTER_X, 0.0, UPPER_CENTER_Z - UPPER_WALL / 2.0))
    )
    shell = outer.cut(inner)

    rear_bridge = (
        cq.Workplane("XY")
        .box(0.032, 0.250, 0.022)
        .translate((0.018, 0.0, 0.001))
    )
    center_barrel = _cylinder_y(UPPER_BARREL_LENGTH, HINGE_BARREL_RADIUS * 0.96).translate(
        (0.0, -UPPER_BARREL_LENGTH / 2.0, 0.0)
    )

    return shell.union(rear_bridge).union(center_barrel)


def _front_handle_shape() -> cq.Workplane:
    front_bar = _cylinder_y(HANDLE_SPAN, HANDLE_TUBE_RADIUS).translate(
        (HANDLE_FRONT_X, -HANDLE_SPAN / 2.0, HANDLE_Z)
    )
    handle = front_bar

    for y_sign in (-1.0, 1.0):
        y_pos = y_sign * HANDLE_SPAN / 2.0
        leg = _cylinder_x(HANDLE_FRONT_X - HANDLE_FOOT_DEPTH, HANDLE_TUBE_RADIUS).translate(
            (HANDLE_FOOT_DEPTH, y_pos, HANDLE_Z)
        )
        foot = (
            cq.Workplane("XY")
            .box(HANDLE_FOOT_DEPTH, 0.028, 0.016)
            .translate((HANDLE_FOOT_DEPTH / 2.0, y_pos, HANDLE_Z))
        )
        elbow = cq.Workplane("XY").sphere(HANDLE_TUBE_RADIUS).translate(
            (HANDLE_FRONT_X, y_pos, HANDLE_Z)
        )
        handle = handle.union(leg).union(foot).union(elbow)

    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_panini_press")

    steel = model.material("steel", rgba=(0.70, 0.71, 0.70, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.18, 0.18, 0.19, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_housing_shape(), "base_housing"),
        material=steel,
        name="base_housing",
    )
    base.visual(
        mesh_from_cadquery(_grill_plate(GRILL_DEPTH, GRILL_WIDTH, GRILL_RIDGE_COUNT), "lower_grill"),
        origin=Origin(
            xyz=(
                GRILL_X_OFFSET,
                0.0,
                BASE_HEIGHT - GRILL_POCKET_DEPTH + GRILL_PLATE_THICKNESS / 2.0,
            )
        ),
        material=dark_cast,
        name="lower_grill",
    )
    base.visual(
        Box((0.022, 0.406, 0.020)),
        origin=Origin(xyz=(-0.179, 0.0, BASE_HEIGHT + 0.010)),
        material=steel,
        name="hinge_bridge",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        base.visual(
            Box((0.030, 0.016, 0.072)),
            origin=Origin(xyz=(-0.170, y_sign * 0.198, BASE_HEIGHT + 0.036)),
            material=steel,
            name=f"hinge_arm_{index}",
        )
        base.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=0.018),
            origin=Origin(
                xyz=(HINGE_X, y_sign * 0.199, HINGE_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"hinge_barrel_{index}",
        )
    for index, center_y in enumerate((-0.1015, 0.1015)):
        tie_y = -0.104 if index == 0 else 0.104
        base.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS * 0.96, length=0.003),
            origin=Origin(
                xyz=(HINGE_X, center_y, HINGE_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"hinge_spacer_{index}",
        )
        base.visual(
            Box((0.02444, 0.004, 0.006)),
            origin=Origin(xyz=(-0.17778, tie_y, HINGE_Z)),
            material=steel,
            name=f"hinge_tie_{index}",
        )
        base.visual(
            Box((0.010, 0.004, 0.032)),
            origin=Origin(xyz=(-0.179, tie_y, 0.154)),
            material=steel,
            name=f"hinge_post_{index}",
        )
    base.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(
            xyz=(KNOB_ORIGIN_X - 0.028, 0.0, FRONT_RECESS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_cast,
        name="control_mount",
    )

    upper = model.part("upper_platen")
    upper.visual(
        mesh_from_cadquery(_upper_shell_shape(), "upper_shell"),
        material=steel,
        name="upper_shell",
    )
    upper.visual(
        mesh_from_cadquery(_grill_plate(UPPER_GRILL_DEPTH, UPPER_GRILL_WIDTH, GRILL_RIDGE_COUNT), "upper_grill"),
        origin=Origin(xyz=(UPPER_CENTER_X, 0.0, UPPER_GRILL_Z)),
        material=dark_cast,
        name="upper_grill",
    )

    handle = model.part("front_handle")
    handle.visual(
        mesh_from_cadquery(_front_handle_shape(), "front_handle"),
        material=handle_finish,
        name="front_handle",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.022,
                body_style="tapered",
                top_diameter=0.031,
                edge_radius=0.002,
                side_draft_deg=5.0,
                center=False,
            ),
            "timer_knob_cap",
        ),
        origin=Origin(xyz=(KNOB_SHAFT_LENGTH, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_finish,
        name="knob_cap",
    )
    timer_knob.visual(
        Cylinder(radius=KNOB_SHAFT_RADIUS, length=KNOB_SHAFT_LENGTH),
        origin=Origin(xyz=(KNOB_SHAFT_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_finish,
        name="knob_shaft",
    )
    timer_knob.visual(
        Cylinder(radius=0.0045, length=0.028),
        origin=Origin(xyz=(-0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_finish,
        name="knob_bushing",
    )

    model.articulation(
        "base_to_upper_platen",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "upper_platen_to_front_handle",
        ArticulationType.FIXED,
        parent=upper,
        child=handle,
        origin=Origin(xyz=(UPPER_START_X + UPPER_DEPTH, 0.0, -0.010)),
    )
    model.articulation(
        "base_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=timer_knob,
        origin=Origin(xyz=(KNOB_ORIGIN_X, 0.0, FRONT_RECESS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_platen")
    handle = object_model.get_part("front_handle")
    timer_knob = object_model.get_part("timer_knob")
    lid_hinge = object_model.get_articulation("base_to_upper_platen")

    ctx.expect_gap(
        upper,
        base,
        axis="z",
        positive_elem="upper_grill",
        negative_elem="lower_grill",
        min_gap=0.002,
        max_gap=0.010,
        name="closed grill faces keep a small cooking gap",
    )
    ctx.expect_overlap(
        upper,
        base,
        axes="xy",
        elem_a="upper_grill",
        elem_b="lower_grill",
        min_overlap=0.260,
        name="upper and lower grill plates share a broad cooking footprint",
    )
    ctx.expect_contact(
        handle,
        upper,
        name="front handle mounts to the upper platen shell",
    )
    knob_pos = ctx.part_world_position(timer_knob)
    ctx.check(
        "timer knob sits on the front control face",
        knob_pos is not None
        and knob_pos[0] > 0.15
        and abs(knob_pos[1]) < 0.01
        and 0.03 < knob_pos[2] < 0.08,
        details=f"knob_pos={knob_pos}",
    )

    closed_handle_pos = ctx.part_world_position(handle)
    open_handle_pos = None
    upper_limit = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if upper_limit is not None:
        with ctx.pose({lid_hinge: upper_limit}):
            open_handle_pos = ctx.part_world_position(handle)

    ctx.check(
        "upper platen opens upward from the rear hinge bracket",
        closed_handle_pos is not None
        and open_handle_pos is not None
        and open_handle_pos[2] > closed_handle_pos[2] + 0.16
        and open_handle_pos[0] < closed_handle_pos[0] - 0.03,
        details=f"closed_handle_pos={closed_handle_pos}, open_handle_pos={open_handle_pos}",
    )

    return ctx.report()


object_model = build_object_model()
