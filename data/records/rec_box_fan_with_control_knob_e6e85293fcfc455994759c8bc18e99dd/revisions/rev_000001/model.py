from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
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


HOUSING_DEPTH = 0.125
HOUSING_WIDTH = 0.335
HOUSING_HEIGHT = 0.335
HOUSING_WALL = 0.020
OPENING_WIDTH = HOUSING_WIDTH - 2.0 * HOUSING_WALL
OPENING_HEIGHT = HOUSING_HEIGHT - 2.0 * HOUSING_WALL
COVER_WIDTH = HOUSING_WIDTH - 0.020
COVER_HEIGHT = HOUSING_HEIGHT - 0.020

BASE_DEPTH = 0.205
BASE_WIDTH = 0.148
BASE_HEIGHT = 0.022
PAN_AXIS_Z = 0.225

FRONT_GRILLE_THICKNESS = 0.006
FRONT_GRILLE_BAR = 0.010

REAR_PANEL_THICKNESS = 0.006
REAR_SLOT_HEIGHT = 0.010

BLADE_AXIS_X = 0.072
BLADE_RADIUS = 0.104

KNOB_X = 0.052
KNOB_Y = HOUSING_WIDTH * 0.5
KNOB_Z = -0.104

DOOR_WIDTH = 0.084
DOOR_HEIGHT = 0.060
DOOR_THICKNESS = 0.005
DOOR_GAP = 0.002
DOOR_CENTER_Y = 0.089
DOOR_CENTER_Z = -0.082
HINGE_AXIS_X = -0.004
HINGE_RADIUS = 0.0028
HINGE_SEGMENT = 0.012
HINGE_CLEAR_GAP = 0.002
HINGE_OFFSET_Y = HINGE_RADIUS + 0.004


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _build_stand_shape() -> cq.Workplane:
    base = _box(
        (BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT),
        (0.006, 0.0, BASE_HEIGHT * 0.5),
    ).edges("|Z").fillet(0.012)

    column_height = PAN_AXIS_Z - BASE_HEIGHT
    column = _box(
        (0.020, 0.076, column_height),
        (-0.054, 0.0, BASE_HEIGHT + column_height * 0.5),
    )

    brace_height = PAN_AXIS_Z - 0.082
    brace = _box(
        (0.034, 0.024, brace_height),
        (-0.037, 0.0, 0.082 + brace_height * 0.5),
    )
    neck = _box(
        (0.032, 0.030, 0.050),
        (-0.0330, 0.0, PAN_AXIS_Z - 0.025),
    )

    top_pad = (
        cq.Workplane("YZ")
        .circle(0.032)
        .extrude(0.011)
        .translate((-0.017, 0.0, PAN_AXIS_Z))
    )

    return base.union(column).union(brace).union(neck).union(top_pad)


def _build_housing_shell() -> cq.Workplane:
    outer = _box(
        (HOUSING_DEPTH, HOUSING_WIDTH, HOUSING_HEIGHT),
        (HOUSING_DEPTH * 0.5, 0.0, 0.0),
    )
    outer = outer.edges("|Z").fillet(0.028)
    outer = outer.edges("|Y").fillet(0.012)

    inner = _box(
        (HOUSING_DEPTH + 0.004, OPENING_WIDTH, OPENING_HEIGHT),
        (HOUSING_DEPTH * 0.5, 0.0, 0.0),
    )
    shell = outer.cut(inner)

    motor = cq.Workplane("YZ").circle(0.034).extrude(0.042).translate((0.010, 0.0, 0.0))

    side_span = OPENING_WIDTH * 0.5 - 0.026
    top_span = OPENING_HEIGHT * 0.5 - 0.026

    side_support_right = _box((0.012, side_span + 0.010, 0.018), (0.028, 0.031 + side_span * 0.5, 0.0))
    side_support_left = _box((0.012, side_span + 0.010, 0.018), (0.028, -0.031 - side_span * 0.5, 0.0))
    upper_support = _box((0.012, 0.018, top_span + 0.010), (0.028, 0.0, 0.031 + top_span * 0.5))
    lower_support = _box((0.012, 0.018, top_span + 0.010), (0.028, 0.0, -0.031 - top_span * 0.5))

    return (
        shell.union(motor)
        .union(side_support_right)
        .union(side_support_left)
        .union(upper_support)
        .union(lower_support)
    )


def _build_front_grille() -> cq.Workplane:
    thickness_center = FRONT_GRILLE_THICKNESS * 0.5

    left_frame = _box(
        (FRONT_GRILLE_THICKNESS, FRONT_GRILLE_BAR, COVER_HEIGHT),
        (thickness_center, -COVER_WIDTH * 0.5 + FRONT_GRILLE_BAR * 0.5, 0.0),
    )
    right_frame = _box(
        (FRONT_GRILLE_THICKNESS, FRONT_GRILLE_BAR, COVER_HEIGHT),
        (thickness_center, COVER_WIDTH * 0.5 - FRONT_GRILLE_BAR * 0.5, 0.0),
    )
    top_frame = _box(
        (FRONT_GRILLE_THICKNESS, COVER_WIDTH, FRONT_GRILLE_BAR),
        (thickness_center, 0.0, COVER_HEIGHT * 0.5 - FRONT_GRILLE_BAR * 0.5),
    )
    bottom_frame = _box(
        (FRONT_GRILLE_THICKNESS, COVER_WIDTH, FRONT_GRILLE_BAR),
        (thickness_center, 0.0, -COVER_HEIGHT * 0.5 + FRONT_GRILLE_BAR * 0.5),
    )

    grille = left_frame.union(right_frame).union(top_frame).union(bottom_frame)

    vertical_height = COVER_HEIGHT - 2.0 * FRONT_GRILLE_BAR
    horizontal_width = COVER_WIDTH - 2.0 * FRONT_GRILLE_BAR
    for y in (-0.072, 0.0, 0.072):
        grille = grille.union(_box((FRONT_GRILLE_THICKNESS, 0.008, vertical_height), (thickness_center, y, 0.0)))
    for z in (-0.072, 0.0, 0.072):
        grille = grille.union(_box((FRONT_GRILLE_THICKNESS, horizontal_width, 0.008), (thickness_center, 0.0, z)))

    return grille


def _build_rear_panel() -> cq.Workplane:
    panel = _box(
        (REAR_PANEL_THICKNESS, COVER_WIDTH, COVER_HEIGHT),
        (-REAR_PANEL_THICKNESS * 0.5, 0.0, 0.0),
    )

    def cut_slot(center_y: float, center_z: float, width_y: float) -> cq.Workplane:
        return _box(
            (REAR_PANEL_THICKNESS + 0.004, width_y, REAR_SLOT_HEIGHT),
            (-REAR_PANEL_THICKNESS * 0.5, center_y, center_z),
        )

    slots = [
        cut_slot(-0.064, -0.105, 0.118),
        cut_slot(-0.064, -0.064, 0.118),
        cut_slot(-0.064, -0.023, 0.118),
        cut_slot(-0.064, 0.018, 0.118),
        cut_slot(-0.064, 0.059, 0.118),
        cut_slot(-0.064, 0.100, 0.118),
        cut_slot(0.050, 0.000, 0.090),
        cut_slot(0.050, 0.040, 0.090),
        cut_slot(0.050, 0.080, 0.090),
    ]
    for slot in slots:
        panel = panel.cut(slot)

    opening_width = DOOR_WIDTH + DOOR_GAP
    opening_height = DOOR_HEIGHT + DOOR_GAP
    door_opening = _box(
        (REAR_PANEL_THICKNESS + 0.004, opening_width, opening_height),
        (-REAR_PANEL_THICKNESS * 0.5, DOOR_CENTER_Y, DOOR_CENTER_Z),
    )
    panel = panel.cut(door_opening)

    hinge_edge_y = DOOR_CENTER_Y - opening_width * 0.5
    hinge_axis_y = hinge_edge_y + HINGE_OFFSET_Y
    upper_hinge_leaf = _box(
        (0.004, 0.005, HINGE_SEGMENT),
        (-0.002, hinge_edge_y + 0.0025, DOOR_CENTER_Z + 0.020),
    )
    lower_hinge_leaf = _box(
        (0.004, 0.005, HINGE_SEGMENT),
        (-0.002, hinge_edge_y + 0.0025, DOOR_CENTER_Z - 0.020),
    )
    upper_hinge_barrel = (
        cq.Workplane("XY")
        .circle(HINGE_RADIUS)
        .extrude(HINGE_SEGMENT)
        .translate((HINGE_AXIS_X, hinge_axis_y, DOOR_CENTER_Z + 0.020 - HINGE_SEGMENT * 0.5))
    )
    lower_hinge_barrel = (
        cq.Workplane("XY")
        .circle(HINGE_RADIUS)
        .extrude(HINGE_SEGMENT)
        .translate((HINGE_AXIS_X, hinge_axis_y, DOOR_CENTER_Z - 0.020 - HINGE_SEGMENT * 0.5))
    )

    return (
        panel.union(upper_hinge_leaf)
        .union(lower_hinge_leaf)
        .union(upper_hinge_barrel)
        .union(lower_hinge_barrel)
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_oscillating_fan")

    stand_finish = model.material("stand_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    housing_finish = model.material("housing_charcoal", rgba=(0.24, 0.25, 0.27, 1.0))
    grille_finish = model.material("grille_black", rgba=(0.11, 0.12, 0.13, 1.0))
    blade_finish = model.material("blade_black", rgba=(0.07, 0.07, 0.08, 1.0))
    trim_finish = model.material("trim_dark", rgba=(0.16, 0.16, 0.17, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_build_stand_shape(), "fan_stand"),
        material=stand_finish,
        name="stand_body",
    )
    stand.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, PAN_AXIS_Z + 0.02)),
        mass=2.4,
        origin=Origin(xyz=(0.01, 0.0, (PAN_AXIS_Z + 0.02) * 0.5)),
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "fan_housing_shell"),
        material=housing_finish,
        name="housing_shell",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_DEPTH + 0.016, HOUSING_WIDTH, HOUSING_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=((HOUSING_DEPTH - 0.008) * 0.5, 0.0, 0.0)),
    )

    front_grille = model.part("front_grille")
    front_grille.visual(
        mesh_from_cadquery(_build_front_grille(), "fan_front_grille"),
        material=grille_finish,
        name="grille_frame",
    )
    front_grille.inertial = Inertial.from_geometry(
        Box((FRONT_GRILLE_THICKNESS, COVER_WIDTH, COVER_HEIGHT)),
        mass=0.25,
        origin=Origin(xyz=(FRONT_GRILLE_THICKNESS * 0.5, 0.0, 0.0)),
    )

    rear_panel = model.part("rear_panel")
    rear_panel.visual(
        mesh_from_cadquery(_build_rear_panel(), "fan_rear_panel"),
        material=grille_finish,
        name="rear_plate",
    )
    rear_panel.inertial = Inertial.from_geometry(
        Box((0.012, COVER_WIDTH, COVER_HEIGHT)),
        mass=0.34,
        origin=Origin(xyz=(-0.001, 0.0, 0.0)),
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                BLADE_RADIUS,
                0.026,
                5,
                thickness=0.012,
                blade_pitch_deg=29.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="broad", camber=0.12),
                hub=FanRotorHub(
                    style="capped",
                    rear_collar_height=0.008,
                    rear_collar_radius=0.022,
                    bore_diameter=0.008,
                ),
            ),
            "fan_blade",
        ),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=blade_finish,
        name="rotor",
    )
    blade.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim_finish,
        name="axle_stub",
    )
    blade.inertial = Inertial.from_geometry(Box((0.026, 0.22, 0.22)), mass=0.20)

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=trim_finish,
        name="knob_collar",
    )
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.020,
                body_style="skirted",
                top_diameter=0.028,
                skirt=KnobSkirt(0.042, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
                center=False,
            ),
            "fan_selector_knob",
        ),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=trim_finish,
        name="knob_shell",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.024),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
    )

    service_door = model.part("service_door")
    hinge_y = DOOR_CENTER_Y - (DOOR_WIDTH + DOOR_GAP) * 0.5 + HINGE_OFFSET_Y
    service_door.visual(
        Box((DOOR_THICKNESS, DOOR_WIDTH - DOOR_GAP, DOOR_HEIGHT - DOOR_GAP)),
        origin=Origin(
            xyz=(
                abs(HINGE_AXIS_X) - DOOR_THICKNESS * 0.5,
                DOOR_WIDTH * 0.5,
                0.0,
            )
        ),
        material=trim_finish,
        name="door_panel",
    )
    service_door.visual(
        Box((0.004, 0.005, 0.028)),
        origin=Origin(xyz=(0.002, 0.0025, 0.0)),
        material=trim_finish,
        name="door_leaf",
    )
    service_door.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_finish,
        name="door_barrel",
    )
    service_door.visual(
        Box((0.0025, 0.012, 0.018)),
        origin=Origin(
            xyz=(
                -0.00125,
                DOOR_WIDTH - 0.010,
                0.0,
            )
        ),
        material=trim_finish,
        name="door_pull",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.010, DOOR_WIDTH, DOOR_HEIGHT)),
        mass=0.05,
        origin=Origin(xyz=(0.004, DOOR_WIDTH * 0.5, 0.0)),
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PAN_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.72,
            upper=0.72,
        ),
    )
    model.articulation(
        "housing_to_front_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=front_grille,
        origin=Origin(xyz=(HOUSING_DEPTH, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_rear_panel",
        ArticulationType.FIXED,
        parent=housing,
        child=rear_panel,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(BLADE_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.9, velocity=40.0),
    )
    model.articulation(
        "housing_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=selector_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=10.0),
    )
    model.articulation(
        "rear_panel_to_service_door",
        ArticulationType.REVOLUTE,
        parent=rear_panel,
        child=service_door,
        origin=Origin(xyz=(HINGE_AXIS_X, hinge_y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    rear_panel = object_model.get_part("rear_panel")
    blade = object_model.get_part("blade")
    selector_knob = object_model.get_part("selector_knob")
    service_door = object_model.get_part("service_door")
    stand = object_model.get_part("stand")

    pan_joint = object_model.get_articulation("stand_to_housing")
    door_joint = object_model.get_articulation("rear_panel_to_service_door")

    ctx.allow_overlap(
        blade,
        housing,
        elem_a="axle_stub",
        elem_b="housing_shell",
        reason="The rotor shaft is intentionally represented as entering the simplified central motor can.",
    )
    ctx.allow_overlap(
        rear_panel,
        service_door,
        elem_a="rear_plate",
        elem_b="door_barrel",
        reason="The small service door uses an interleaved simplified hinge barrel at the rear cover edge.",
    )
    ctx.allow_overlap(
        rear_panel,
        stand,
        elem_a="rear_plate",
        elem_b="stand_body",
        reason="The rear oscillation pad is simplified as a snug pan interface nested against the rear cover.",
    )

    ctx.expect_contact(front_grille, housing, name="front grille is mounted into the housing")
    ctx.expect_contact(rear_panel, housing, name="rear panel is seated in the housing")
    ctx.expect_within(
        blade,
        front_grille,
        axes="yz",
        inner_elem="rotor",
        outer_elem="grille_frame",
        margin=0.016,
        name="blade stays inside the square front opening",
    )
    ctx.expect_gap(
        front_grille,
        blade,
        axis="x",
        positive_elem="grille_frame",
        negative_elem="rotor",
        min_gap=0.022,
        name="front grille clears the rotor sweep",
    )
    ctx.expect_gap(
        blade,
        rear_panel,
        axis="x",
        positive_elem="rotor",
        negative_elem="rear_plate",
        min_gap=0.018,
        name="rotor clears the rear panel",
    )
    ctx.expect_origin_gap(
        selector_knob,
        housing,
        axis="y",
        min_gap=0.165,
        name="selector knob sits on the right side of the housing",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    if housing_aabb is not None:
        mins, maxs = housing_aabb
        width = float(maxs[1]) - float(mins[1])
        height = float(maxs[2]) - float(mins[2])
        ctx.check(
            "appliance cooling scale",
            0.31 <= width <= 0.37 and 0.31 <= height <= 0.37,
            details=f"housing_width={width:.3f}, housing_height={height:.3f}",
        )

    grille_closed = ctx.part_element_world_aabb(front_grille, elem="grille_frame")
    pan_upper = pan_joint.motion_limits.upper if pan_joint.motion_limits is not None else None
    if grille_closed is not None and pan_upper is not None:
        closed_center = _aabb_center(grille_closed)
        with ctx.pose({pan_joint: pan_upper}):
            grille_open = ctx.part_element_world_aabb(front_grille, elem="grille_frame")
        ctx.check(
            "housing pans toward positive y",
            grille_open is not None and _aabb_center(grille_open)[1] > closed_center[1] + 0.055,
            details=f"closed={closed_center}, opened={None if grille_open is None else _aabb_center(grille_open)}",
        )

    closed_door_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
    if closed_door_aabb is not None:
        ctx.check(
            "service door sits flush in the rear panel",
            abs(float(closed_door_aabb[1][0])) <= 0.0015,
            details=f"door_max_x={closed_door_aabb[1][0]:.4f}",
        )

    door_upper = door_joint.motion_limits.upper if door_joint.motion_limits is not None else None
    if closed_door_aabb is not None and door_upper is not None:
        with ctx.pose({door_joint: door_upper}):
            open_door_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
        ctx.check(
            "service door swings outward",
            open_door_aabb is not None and float(open_door_aabb[0][0]) < float(closed_door_aabb[0][0]) - 0.030,
            details=(
                f"closed_min_x={closed_door_aabb[0][0]:.4f}, "
                f"open_min_x={None if open_door_aabb is None else open_door_aabb[0][0]:.4f}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
