from __future__ import annotations

import math

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


BODY_RADIUS = 0.145
BASE_RADIUS = 0.150
BODY_HEIGHT = 0.520
BASE_HEIGHT = 0.018
SHELL_THICKNESS = 0.0035
REAR_FLAT = 0.095
REAR_PLANE_Y = -REAR_FLAT

TOP_SHOULDER_HEIGHT = 0.008
TOP_SHOULDER_OUTER_RADIUS = 0.128
TOP_SHOULDER_INNER_RADIUS = 0.099
VENT_RADIUS = 0.100

DOOR_OPENING_WIDTH = 0.194
DOOR_OPENING_HEIGHT = 0.370
DOOR_WIDTH = 0.208
DOOR_HEIGHT = 0.388
DOOR_THICKNESS = 0.005
DOOR_BOTTOM_Z = 0.074
HINGE_RADIUS = 0.006
HINGE_X = DOOR_WIDTH * 0.5 + 0.004
HINGE_Y = REAR_PLANE_Y - 0.004
HINGE_Z = DOOR_BOTTOM_Z + DOOR_HEIGHT * 0.5
BODY_BARREL_HEIGHT = 0.094
HINGE_GAP = 0.006
DOOR_BARREL_Z0 = -DOOR_HEIGHT * 0.5 + BODY_BARREL_HEIGHT + HINGE_GAP
DOOR_BARREL_HEIGHT = DOOR_HEIGHT - 2.0 * BODY_BARREL_HEIGHT - 2.0 * HINGE_GAP

FILTER_RADIUS = 0.093
FILTER_INNER_RADIUS = 0.062
FILTER_HEIGHT = 0.340
FILTER_CENTER_Y = 0.020
FILTER_JOINT_Y = REAR_PLANE_Y + SHELL_THICKNESS
FILTER_LOCAL_CENTER_Y = FILTER_CENTER_Y - FILTER_JOINT_Y
FILTER_TRAVEL = 0.110

RING_OUTER_RADIUS = 0.122
RING_INNER_RADIUS = 0.104
RING_HEIGHT = 0.016
RING_CENTER_Z = BODY_HEIGHT + RING_HEIGHT * 0.5


def _flat_keep_box(radius: float, height: float, rear_flat: float) -> cq.Workplane:
    front_extent = radius + 0.030
    y_size = front_extent + rear_flat
    y_center = (front_extent - rear_flat) * 0.5
    return (
        cq.Workplane("XY")
        .box(radius * 2.6, y_size, height, centered=(True, True, True))
        .translate((0.0, y_center, height * 0.5))
    )


def _vertical_cylinder(radius: float, height: float, *, x: float, y: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z0))


def _build_body_shape() -> cq.Workplane:
    main_shell = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_HEIGHT - TOP_SHOULDER_HEIGHT)
    main_shell = main_shell.intersect(_flat_keep_box(BODY_RADIUS, BODY_HEIGHT - TOP_SHOULDER_HEIGHT, REAR_FLAT))

    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT)
    base = base.intersect(_flat_keep_box(BASE_RADIUS, BASE_HEIGHT, REAR_FLAT))

    cavity = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS - SHELL_THICKNESS)
        .extrude(BODY_HEIGHT - TOP_SHOULDER_HEIGHT - BASE_HEIGHT - 0.006)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    cavity = cavity.intersect(
        _flat_keep_box(BODY_RADIUS - SHELL_THICKNESS, BODY_HEIGHT, REAR_FLAT - SHELL_THICKNESS)
    )

    housing = main_shell.union(base).cut(cavity)

    top_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_HEIGHT - TOP_SHOULDER_HEIGHT)
        .circle(TOP_SHOULDER_OUTER_RADIUS)
        .circle(TOP_SHOULDER_INNER_RADIUS)
        .extrude(TOP_SHOULDER_HEIGHT)
    )

    vent = cq.Workplane("XY").workplane(offset=BODY_HEIGHT - 0.006).circle(VENT_RADIUS).extrude(0.006)
    vent = vent.cut(cq.Workplane("XY").workplane(offset=BODY_HEIGHT - 0.007).circle(0.021).extrude(0.008))
    for angle_deg in range(0, 360, 45):
        slot = (
            cq.Workplane("XY")
            .box(0.012, 0.046, 0.010, centered=(True, True, True))
            .translate((0.043, 0.0, BODY_HEIGHT - 0.002))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        vent = vent.cut(slot)
    for angle_deg in (22.5, 67.5, 112.5, 157.5):
        spoke = (
            cq.Workplane("XY")
            .box(0.006, 0.130, 0.006, centered=(True, True, True))
            .translate((0.0, 0.0, BODY_HEIGHT - 0.003))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        vent = vent.union(spoke)

    housing = housing.union(top_shoulder).union(vent)

    rear_opening = (
        cq.Workplane("XY")
        .box(DOOR_OPENING_WIDTH, 0.050, DOOR_OPENING_HEIGHT, centered=(True, True, True))
        .translate((0.0, REAR_PLANE_Y + 0.025, HINGE_Z))
    )
    housing = housing.cut(rear_opening)

    top_stop = (
        cq.Workplane("XY")
        .box(0.086, 0.005, 0.012, centered=(True, True, True))
        .translate((0.0, REAR_PLANE_Y, HINGE_Z + DOOR_OPENING_HEIGHT * 0.5 + 0.011))
    )
    bottom_stop = (
        cq.Workplane("XY")
        .box(0.086, 0.005, 0.012, centered=(True, True, True))
        .translate((0.0, REAR_PLANE_Y, HINGE_Z - DOOR_OPENING_HEIGHT * 0.5 - 0.011))
    )
    rail_z_center = (0.106 + BASE_HEIGHT) * 0.5
    rail_height = 0.106 - BASE_HEIGHT
    left_rail = (
        cq.Workplane("XY")
        .box(0.018, 0.230, rail_height, centered=(True, True, True))
        .translate((-0.073, 0.015, rail_z_center))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(0.018, 0.230, rail_height, centered=(True, True, True))
        .translate((0.073, 0.015, rail_z_center))
    )
    housing = (
        housing.union(top_stop)
        .union(bottom_stop)
        .union(left_rail)
        .union(right_rail)
    )

    return housing


def _build_door_shape() -> cq.Workplane:
    panel_center_y = -0.001
    panel = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT, centered=(True, True, True))
        .translate((-DOOR_WIDTH * 0.5 - 0.006, panel_center_y, 0.0))
    )

    stile = (
        cq.Workplane("XY")
        .box(0.008, 0.008, DOOR_HEIGHT, centered=(True, True, True))
        .translate((-0.004, -0.001, 0.0))
    )

    pull = (
        cq.Workplane("XY")
        .box(0.022, 0.008, 0.120, centered=(True, True, True))
        .translate((-DOOR_WIDTH + 0.028, -0.005, 0.0))
    )
    return panel.union(stile).union(pull)


def _build_filter_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(FILTER_RADIUS)
        .extrude(FILTER_HEIGHT)
        .translate((0.0, FILTER_LOCAL_CENTER_Y, -FILTER_HEIGHT * 0.5))
    )
    outer = outer.cut(
        cq.Workplane("XY")
        .circle(FILTER_INNER_RADIUS)
        .extrude(FILTER_HEIGHT + 0.004)
        .translate((0.0, FILTER_LOCAL_CENTER_Y, -FILTER_HEIGHT * 0.5 - 0.002))
    )

    groove_cutter = (
        cq.Workplane("XY")
        .box(0.008, 0.028, FILTER_HEIGHT + 0.008, centered=(True, True, True))
        .translate((FILTER_RADIUS - 0.004, FILTER_LOCAL_CENTER_Y, 0.0))
    )
    for angle_deg in range(0, 360, 15):
        outer = outer.cut(
            groove_cutter.rotate(
                (0.0, FILTER_LOCAL_CENTER_Y, 0.0),
                (0.0, FILTER_LOCAL_CENTER_Y, 1.0),
                angle_deg,
            )
        )

    top_cap = _vertical_cylinder(
        FILTER_RADIUS,
        0.010,
        x=0.0,
        y=FILTER_LOCAL_CENTER_Y,
        z0=FILTER_HEIGHT * 0.5 - 0.010,
    )
    top_cap = top_cap.cut(
        _vertical_cylinder(
            FILTER_INNER_RADIUS,
            0.012,
            x=0.0,
            y=FILTER_LOCAL_CENTER_Y,
            z0=FILTER_HEIGHT * 0.5 - 0.011,
        )
    )

    bottom_cap = _vertical_cylinder(
        FILTER_RADIUS,
        0.010,
        x=0.0,
        y=FILTER_LOCAL_CENTER_Y,
        z0=-FILTER_HEIGHT * 0.5,
    )
    bottom_cap = bottom_cap.cut(
        _vertical_cylinder(
            FILTER_INNER_RADIUS,
            0.012,
            x=0.0,
            y=FILTER_LOCAL_CENTER_Y,
            z0=-FILTER_HEIGHT * 0.5 - 0.001,
        )
    )

    handle = (
        cq.Workplane("XY")
        .box(0.060, 0.014, 0.080, centered=(True, True, True))
        .translate((0.0, FILTER_LOCAL_CENTER_Y - FILTER_RADIUS + 0.010, 0.0))
    )

    return outer.union(top_cap).union(bottom_cap).union(handle)


def _build_control_ring_shape() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(RING_OUTER_RADIUS).circle(RING_INNER_RADIUS).extrude(RING_HEIGHT)
    ring = ring.translate((0.0, 0.0, -RING_HEIGHT * 0.5))
    indicator = (
        cq.Workplane("XY")
        .box(0.012, 0.004, 0.003, centered=(True, True, True))
        .translate((RING_OUTER_RADIUS - 0.010, 0.0, RING_HEIGHT * 0.5 + 0.0015))
    )
    return ring.union(indicator)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_air_purifier")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.94, 1.0))
    door_white = model.material("door_white", rgba=(0.90, 0.91, 0.90, 1.0))
    filter_offwhite = model.material("filter_offwhite", rgba=(0.93, 0.92, 0.86, 1.0))
    ring_graphite = model.material("ring_graphite", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "purifier_body"),
        material=body_white,
        name="housing",
    )

    rear_door = model.part("rear_door")
    rear_door.visual(
        mesh_from_cadquery(_build_door_shape(), "purifier_rear_door"),
        material=door_white,
        name="door_panel",
    )

    filter_part = model.part("filter")
    filter_part.visual(
        mesh_from_cadquery(_build_filter_shape(), "purifier_filter"),
        material=filter_offwhite,
        name="filter_cartridge",
    )

    control_ring = model.part("control_ring")
    control_ring.visual(
        mesh_from_cadquery(_build_control_ring_shape(), "purifier_control_ring"),
        material=ring_graphite,
        name="ring_shell",
    )

    model.articulation(
        "body_to_rear_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    model.articulation(
        "body_to_filter",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_part,
        origin=Origin(xyz=(0.0, FILTER_JOINT_Y, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_control_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=control_ring,
        origin=Origin(xyz=(0.0, 0.0, RING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    rear_door = object_model.get_part("rear_door")
    filter_part = object_model.get_part("filter")
    control_ring = object_model.get_part("control_ring")

    door_joint = object_model.get_articulation("body_to_rear_door")
    filter_joint = object_model.get_articulation("body_to_filter")
    ring_joint = object_model.get_articulation("body_to_control_ring")

    ctx.allow_overlap(
        body,
        filter_part,
        elem_a="housing",
        elem_b="filter_cartridge",
        reason="The service filter is intentionally represented as nested inside the purifier shell proxy.",
    )
    ctx.allow_overlap(
        body,
        rear_door,
        elem_a="housing",
        elem_b="door_panel",
        reason="The closed rear door is represented as a flush skin against simplified rear stop geometry.",
    )

    ctx.expect_overlap(
        rear_door,
        body,
        axes="xz",
        elem_a="door_panel",
        elem_b="housing",
        min_overlap=0.14,
        name="rear door covers the service opening",
    )
    ctx.expect_within(
        filter_part,
        body,
        axes="xz",
        margin=0.016,
        name="filter stays centered inside the cylindrical housing",
    )
    ctx.expect_gap(
        control_ring,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="control ring sits on the top shoulder without floating above it",
    )

    closed_door_aabb = ctx.part_element_world_aabb(rear_door, elem="door_panel")
    with ctx.pose({door_joint: math.radians(95.0)}):
        opened_door_aabb = ctx.part_element_world_aabb(rear_door, elem="door_panel")
    ctx.check(
        "rear door swings outward from the back",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < closed_door_aabb[0][1] - 0.06,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    rest_filter_pos = ctx.part_world_position(filter_part)
    with ctx.pose({filter_joint: FILTER_TRAVEL}):
        ctx.expect_within(
            filter_part,
            body,
            axes="xz",
            margin=0.016,
            name="extended filter remains guided by the housing",
        )
        ctx.expect_overlap(
            filter_part,
            body,
            axes="y",
            min_overlap=0.020,
            name="extended filter retains insertion in the purifier body",
        )
        extended_filter_pos = ctx.part_world_position(filter_part)
    ctx.check(
        "filter slides rearward through the access opening",
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[1] < rest_filter_pos[1] - 0.08,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    with ctx.pose({door_joint: math.radians(95.0), filter_joint: FILTER_TRAVEL}):
        extended_filter_aabb = ctx.part_world_aabb(filter_part)
    ctx.check(
        "extended filter protrudes behind the rear shell",
        extended_filter_aabb is not None and extended_filter_aabb[0][1] < REAR_PLANE_Y - 0.040,
        details=f"aabb={extended_filter_aabb}",
    )

    ring_rest_pos = ctx.part_world_position(control_ring)
    with ctx.pose({ring_joint: math.pi / 2.0}):
        ring_rotated_pos = ctx.part_world_position(control_ring)
    limits = ring_joint.motion_limits
    ctx.check(
        "top control ring is a continuous vertical-axis rotary control",
        ring_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(ring_joint.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None
        and ring_rest_pos is not None
        and ring_rotated_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(ring_rest_pos, ring_rotated_pos)),
        details=(
            f"type={ring_joint.articulation_type}, axis={ring_joint.axis}, "
            f"limits={limits}, rest={ring_rest_pos}, rotated={ring_rotated_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
