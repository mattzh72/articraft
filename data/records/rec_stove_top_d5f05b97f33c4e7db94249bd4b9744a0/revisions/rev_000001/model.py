from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


BODY_WIDTH = 0.53
BODY_DEPTH = 0.34
BODY_HEIGHT = 0.075
BODY_TOP_FILLET = 0.006
BODY_EDGE_FILLET = 0.004

BURNER_OFFSET_X = 0.115
BURNER_CENTER_Y = 0.034
BURNER_WELL_RADIUS = 0.055
BURNER_WELL_DEPTH = 0.004
BURNER_CAP_RADIUS = 0.028
BURNER_CAP_HEIGHT = 0.008
GRATE_BAR_HEIGHT = 0.005
GRATE_BAR_WIDTH = 0.008
GRATE_BAR_LENGTH_X = 0.132
GRATE_BAR_LENGTH_Y = 0.116
GRATE_LEG_RADIUS = 0.0045
GRATE_STAND_HEIGHT = 0.009

KNOB_OFFSET_X = 0.115
KNOB_CENTER_Z = 0.027
KNOB_HOLE_RADIUS = 0.007
KNOB_SHAFT_RADIUS = 0.0042
KNOB_SHAFT_LENGTH = 0.012
KNOB_STANDOFF = 0.002
KNOB_DIAMETER = 0.040
KNOB_HEIGHT = 0.022

HINGE_OUTER_RADIUS = 0.0044
HINGE_PIN_RADIUS = 0.0016
HINGE_CLEAR_RADIUS = 0.0022
HINGE_AXIS_Y = BODY_DEPTH / 2.0 - 0.008
HINGE_AXIS_Z = BODY_HEIGHT + 0.010
HINGE_GAP = 0.006
HINGE_SEGMENT_LENGTHS = (
    ("cover", 0.110),
    ("body", 0.065),
    ("cover", 0.110),
    ("body", 0.065),
    ("cover", 0.110),
)

COVER_WIDTH = BODY_WIDTH + 0.030
COVER_DEPTH = BODY_DEPTH + 0.006
COVER_SHEET_THICKNESS = 0.0018
COVER_TOP_Z = HINGE_OUTER_RADIUS + 0.0030 + COVER_SHEET_THICKNESS / 2.0
COVER_REAR_SETBACK = 0.014
COVER_SIDE_DROP = 0.024
COVER_SIDE_FLANGE_THICKNESS = 0.010
COVER_FRONT_DROP = 0.018
COVER_FRONT_FLANGE_THICKNESS = 0.010
COVER_KNUCKLE_TAB_DEPTH = 0.012
COVER_KNUCKLE_TAB_HEIGHT = 0.006


def _x_cylinder(length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length)


def _x_tube(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    inner = cq.Workplane("YZ").circle(inner_radius).extrude(length + 0.002).translate(
        (-0.001, 0.0, 0.0)
    )
    return outer.cut(inner)


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length)


def _hinge_segments() -> list[tuple[str, float, float]]:
    total_length = sum(length for _, length in HINGE_SEGMENT_LENGTHS)
    total_gap = HINGE_GAP * (len(HINGE_SEGMENT_LENGTHS) - 1)
    cursor = -(total_length + total_gap) / 2.0
    segments: list[tuple[str, float, float]] = []
    for owner, length in HINGE_SEGMENT_LENGTHS:
        center_x = cursor + length / 2.0
        segments.append((owner, center_x, length))
        cursor += length + HINGE_GAP
    return segments


def _burner_with_grate(center_x: float, center_y: float) -> cq.Workplane:
    burner_mount_z = BODY_HEIGHT - BURNER_WELL_DEPTH
    cap = (
        cq.Workplane("XY")
        .circle(BURNER_CAP_RADIUS)
        .extrude(BURNER_CAP_HEIGHT)
        .translate((center_x, center_y, burner_mount_z))
    )

    cross_bar_x = (
        cq.Workplane("XY")
        .box(GRATE_BAR_LENGTH_X, GRATE_BAR_WIDTH, GRATE_BAR_HEIGHT)
        .translate(
            (
                center_x,
                center_y,
                burner_mount_z + GRATE_STAND_HEIGHT + GRATE_BAR_HEIGHT / 2.0,
            )
        )
    )
    cross_bar_y = (
        cq.Workplane("XY")
        .box(GRATE_BAR_WIDTH, GRATE_BAR_LENGTH_Y, GRATE_BAR_HEIGHT)
        .translate(
            (
                center_x,
                center_y,
                burner_mount_z + GRATE_STAND_HEIGHT + GRATE_BAR_HEIGHT / 2.0,
            )
        )
    )

    grate = cap.union(cross_bar_x).union(cross_bar_y)
    leg_offsets = (
        (GRATE_BAR_LENGTH_X * 0.34, 0.0),
        (-GRATE_BAR_LENGTH_X * 0.34, 0.0),
        (0.0, GRATE_BAR_LENGTH_Y * 0.34),
        (0.0, -GRATE_BAR_LENGTH_Y * 0.34),
    )
    for dx, dy in leg_offsets:
        leg = (
            cq.Workplane("XY")
            .circle(GRATE_LEG_RADIUS)
            .extrude(GRATE_STAND_HEIGHT)
            .translate((center_x + dx, center_y + dy, burner_mount_z))
        )
        grate = grate.union(leg)
    return grate


def _build_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(BODY_TOP_FILLET)
        .faces(">Z")
        .edges()
        .fillet(BODY_EDGE_FILLET)
    )

    burner_centers = (
        (-BURNER_OFFSET_X, BURNER_CENTER_Y),
        (BURNER_OFFSET_X, BURNER_CENTER_Y),
    )
    body = (
        body.faces(">Z")
        .workplane()
        .pushPoints(list(burner_centers))
        .circle(BURNER_WELL_RADIUS)
        .cutBlind(-BURNER_WELL_DEPTH)
    )

    for x in (-KNOB_OFFSET_X, KNOB_OFFSET_X):
        knob_hole = (
            _y_cylinder(KNOB_HOLE_RADIUS, 0.055).translate(
                (x, -BODY_DEPTH / 2.0 - 0.018, KNOB_CENTER_Z)
            )
        )
        body = body.cut(knob_hole)

    for center_x, center_y in burner_centers:
        body = body.union(_burner_with_grate(center_x, center_y))

    pin_length = BODY_WIDTH - 0.034
    hinge_pin = _x_cylinder(pin_length, HINGE_PIN_RADIUS).translate(
        (-pin_length / 2.0, HINGE_AXIS_Y, HINGE_AXIS_Z)
    )
    body = body.union(hinge_pin)

    for owner, center_x, length in _hinge_segments():
        if owner != "body":
            continue
        knuckle = _x_cylinder(length, HINGE_OUTER_RADIUS).translate(
            (center_x - length / 2.0, HINGE_AXIS_Y, HINGE_AXIS_Z)
        )
        tab = (
            cq.Workplane("XY")
            .box(length, 0.010, 0.018)
            .translate((center_x, HINGE_AXIS_Y - 0.005, BODY_HEIGHT + 0.008))
        )
        body = body.union(knuckle).union(tab)

    return body


def _build_cover_shape() -> cq.Workplane:
    panel_depth = COVER_DEPTH - COVER_REAR_SETBACK
    cover = (
        cq.Workplane("XY")
        .box(COVER_WIDTH, panel_depth, COVER_SHEET_THICKNESS)
        .translate((0.0, -(COVER_DEPTH + COVER_REAR_SETBACK) / 2.0, COVER_TOP_Z))
    )

    side_y = -(COVER_DEPTH + COVER_REAR_SETBACK) / 2.0 + 0.004
    side_z = COVER_TOP_Z - COVER_SIDE_DROP / 2.0
    side_length = panel_depth - 0.008
    left_side = (
        cq.Workplane("XY")
        .box(COVER_SIDE_FLANGE_THICKNESS, side_length, COVER_SIDE_DROP)
        .translate(
            (
                -COVER_WIDTH / 2.0 + COVER_SIDE_FLANGE_THICKNESS / 2.0,
                side_y,
                side_z,
            )
        )
    )
    right_side = left_side.translate((COVER_WIDTH - COVER_SIDE_FLANGE_THICKNESS, 0.0, 0.0))
    front_lip = (
        cq.Workplane("XY")
        .box(
            COVER_WIDTH - 2.0 * COVER_SIDE_FLANGE_THICKNESS,
            COVER_FRONT_FLANGE_THICKNESS,
            COVER_FRONT_DROP,
        )
        .translate(
            (
                0.0,
                -COVER_DEPTH + COVER_FRONT_FLANGE_THICKNESS / 2.0,
                COVER_TOP_Z - COVER_FRONT_DROP / 2.0,
            )
        )
    )
    return cover.union(left_side).union(right_side).union(front_lip)


def _build_cover_hinge_shape() -> cq.Workplane:
    hinge_length = COVER_WIDTH - 0.036
    hinge_leaf = (
        cq.Workplane("XY")
        .box(COVER_WIDTH - 0.024, 0.014, 0.006)
        .translate((0.0, -0.018, 0.006))
    )
    hinge_bridge = (
        cq.Workplane("XY")
        .box(COVER_WIDTH - 0.024, 0.012, 0.007)
        .translate((0.0, -0.006, 0.0035))
    )
    barrel = _x_tube(hinge_length, HINGE_OUTER_RADIUS, HINGE_CLEAR_RADIUS).translate(
        (-hinge_length / 2.0, 0.0, 0.0)
    )
    return hinge_leaf.union(hinge_bridge).union(barrel)


def _add_control_knob(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    x_pos: float,
    knob_material,
    shaft_material,
):
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=KNOB_SHAFT_RADIUS, length=KNOB_SHAFT_LENGTH),
        origin=Origin(
            xyz=(0.0, KNOB_SHAFT_LENGTH / 2.0 - KNOB_STANDOFF, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=shaft_material,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_DIAMETER,
                KNOB_HEIGHT,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.048, 0.005, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            f"{name}_shell",
        ),
        origin=Origin(
            xyz=(0.0, -KNOB_STANDOFF, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=knob_material,
        name="knob_shell",
    )
    model.articulation(
        f"{name}_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(x_pos, -BODY_DEPTH / 2.0, KNOB_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camper_van_stove_top")

    steel = model.material("steel", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.28, 0.30, 1.0))
    control_black = model.material("control_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "camper_stove_body"),
        material=steel,
        name="body_shell",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_build_cover_shape(), "camper_stove_cover"),
        material=dark_steel,
        name="cover_shell",
    )
    cover_hinge = model.part("cover_hinge")
    cover_hinge.visual(
        mesh_from_cadquery(_build_cover_hinge_shape(), "camper_stove_cover_hinge"),
        material=dark_steel,
        name="cover_hinge_shell",
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.72,
        ),
    )
    model.articulation(
        "cover_to_cover_hinge",
        ArticulationType.FIXED,
        parent=cover,
        child=cover_hinge,
        origin=Origin(),
    )

    _add_control_knob(
        model,
        body,
        name="knob_0",
        x_pos=-KNOB_OFFSET_X,
        knob_material=control_black,
        shaft_material=steel,
    )
    _add_control_knob(
        model,
        body,
        name="knob_1",
        x_pos=KNOB_OFFSET_X,
        knob_material=control_black,
        shaft_material=steel,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    cover_hinge_part = object_model.get_part("cover_hinge")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    cover_hinge = object_model.get_articulation("cover_hinge")
    knob_0_spin = object_model.get_articulation("knob_0_spin")
    knob_1_spin = object_model.get_articulation("knob_1_spin")

    ctx.allow_overlap(
        body,
        cover_hinge_part,
        reason="The rear hinge leaf intentionally nests around the body-side hinge pin and alternating hinge barrels.",
    )
    ctx.allow_overlap(
        cover,
        cover_hinge_part,
        reason="The hinge leaf is intentionally tucked into the rear hem region of the cover shell.",
    )
    ctx.allow_overlap(
        body,
        knob_0,
        elem_a="body_shell",
        elem_b="shaft",
        reason="The short control shaft intentionally inserts through the fascia opening into the concealed valve body.",
    )
    ctx.allow_overlap(
        body,
        knob_1,
        elem_a="body_shell",
        elem_b="shaft",
        reason="The short control shaft intentionally inserts through the fascia opening into the concealed valve body.",
    )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            min_overlap=0.24,
            name="closed cover spans the cooktop footprint",
        )
        closed_cover_aabb = ctx.part_world_aabb(cover)

    ctx.expect_origin_gap(
        body,
        knob_0,
        axis="y",
        min_gap=0.16,
        max_gap=0.18,
        name="knob_0 sits on the front fascia",
    )
    ctx.expect_origin_gap(
        body,
        knob_1,
        axis="y",
        min_gap=0.16,
        max_gap=0.18,
        name="knob_1 sits on the front fascia",
    )
    ctx.expect_origin_gap(
        knob_0,
        body,
        axis="z",
        min_gap=0.02,
        max_gap=0.035,
        name="knob_0 sits below the burners",
    )
    ctx.expect_origin_gap(
        knob_1,
        body,
        axis="z",
        min_gap=0.02,
        max_gap=0.035,
        name="knob_1 sits below the burners",
    )

    with ctx.pose({cover_hinge: cover_hinge.motion_limits.upper}):
        opened_cover_aabb = ctx.part_world_aabb(cover)

    with ctx.pose({knob_0_spin: 1.2, knob_1_spin: -1.0}):
        posed_knob_0 = ctx.part_world_position(knob_0)
        posed_knob_1 = ctx.part_world_position(knob_1)

    ctx.check(
        "control knobs flank the burner line",
        posed_knob_0 is not None
        and posed_knob_1 is not None
        and posed_knob_0[0] < -0.08
        and posed_knob_1[0] > 0.08
        and abs(posed_knob_0[2] - KNOB_CENTER_Z) < 1e-6
        and abs(posed_knob_1[2] - KNOB_CENTER_Z) < 1e-6,
        details=f"posed_knob_0={posed_knob_0}, posed_knob_1={posed_knob_1}",
    )

    closed_max_z = None if closed_cover_aabb is None else float(closed_cover_aabb[1][2])
    opened_max_z = None if opened_cover_aabb is None else float(opened_cover_aabb[1][2])
    ctx.check(
        "opened cover rises above the cooktop",
        closed_max_z is not None
        and opened_max_z is not None
        and opened_max_z > closed_max_z + 0.20,
        details=f"closed_max_z={closed_max_z}, opened_max_z={opened_max_z}",
    )

    return ctx.report()


object_model = build_object_model()
