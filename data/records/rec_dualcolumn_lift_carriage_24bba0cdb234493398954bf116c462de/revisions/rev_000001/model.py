from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


POST_SPACING = 0.088
POST_X = POST_SPACING / 2.0
POST_RADIUS = 0.009
POST_SEAT_RADIUS = POST_RADIUS - 0.0002
POST_START_Z = 0.042
POST_TOP_Z = 0.314

BASE_CAN_RADIUS = 0.033
BASE_CAN_HEIGHT = 0.048
BASE_PLATE_WIDTH = 0.132
BASE_PLATE_DEPTH = 0.050
BASE_PLATE_THICKNESS = 0.012
SOCKET_OUTER_RADIUS = 0.015
SOCKET_HEIGHT = 0.014

LOWER_STOP_OUTER_RADIUS = 0.0145
LOWER_STOP_HEIGHT = 0.008
LOWER_STOP_BOTTOM_Z = BASE_CAN_HEIGHT + BASE_PLATE_THICKNESS + SOCKET_HEIGHT
LOWER_STOP_TOP_Z = LOWER_STOP_BOTTOM_Z + LOWER_STOP_HEIGHT

TOP_BRIDGE_WIDTH = 0.116
TOP_BRIDGE_DEPTH = 0.026
TOP_BRIDGE_THICKNESS = 0.012
TOP_BRIDGE_CENTER_Z = 0.308
TOP_BRIDGE_BOTTOM_Z = TOP_BRIDGE_CENTER_Z - TOP_BRIDGE_THICKNESS / 2.0

UPPER_STOP_OUTER_RADIUS = 0.014
UPPER_STOP_BOTTOM_Z = 0.268
UPPER_STOP_HEIGHT = TOP_BRIDGE_BOTTOM_Z - UPPER_STOP_BOTTOM_Z

GUIDE_OUTER_RADIUS = 0.016
GUIDE_BORE_RADIUS = 0.0104
GUIDE_HEIGHT = 0.062
LOWER_CARRIAGE_CENTER_Z = LOWER_STOP_TOP_Z + GUIDE_HEIGHT / 2.0
LIFT_TRAVEL = UPPER_STOP_BOTTOM_Z - LOWER_STOP_TOP_Z - GUIDE_HEIGHT

TABLE_WIDTH = 0.082
TABLE_DEPTH = 0.052
TABLE_THICKNESS = 0.008


def ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def make_base_can() -> cq.Workplane:
    plate_top_z = BASE_CAN_HEIGHT + BASE_PLATE_THICKNESS

    can = cq.Workplane("XY").circle(BASE_CAN_RADIUS).extrude(BASE_CAN_HEIGHT)
    plate = cq.Workplane("XY").box(
        BASE_PLATE_WIDTH,
        BASE_PLATE_DEPTH,
        BASE_PLATE_THICKNESS,
    ).translate((0.0, 0.0, BASE_CAN_HEIGHT + BASE_PLATE_THICKNESS / 2.0))

    front_web = cq.Workplane("XY").box(0.084, 0.008, 0.020).translate((0.0, 0.016, 0.038))
    rear_web = cq.Workplane("XY").box(0.084, 0.008, 0.020).translate((0.0, -0.016, 0.038))

    socket_left = (
        cq.Workplane("XY")
        .circle(SOCKET_OUTER_RADIUS)
        .extrude(SOCKET_HEIGHT)
        .translate((-POST_X, 0.0, plate_top_z))
    )
    socket_right = (
        cq.Workplane("XY")
        .circle(SOCKET_OUTER_RADIUS)
        .extrude(SOCKET_HEIGHT)
        .translate((POST_X, 0.0, plate_top_z))
    )

    base = can.union(plate).union(front_web).union(rear_web).union(socket_left).union(socket_right)

    for post_x in (-POST_X, POST_X):
        seat = (
            cq.Workplane("XY")
            .circle(POST_SEAT_RADIUS)
            .extrude(SOCKET_HEIGHT + BASE_PLATE_THICKNESS + 0.002)
            .translate((post_x, 0.0, BASE_CAN_HEIGHT - 0.001))
        )
        base = base.cut(seat)

    return base.edges("|Z").fillet(0.002)


def make_lower_stops() -> cq.Workplane:
    left = ring(LOWER_STOP_OUTER_RADIUS, POST_SEAT_RADIUS, LOWER_STOP_HEIGHT).translate(
        (-POST_X, 0.0, LOWER_STOP_BOTTOM_Z)
    )
    right = ring(LOWER_STOP_OUTER_RADIUS, POST_SEAT_RADIUS, LOWER_STOP_HEIGHT).translate(
        (POST_X, 0.0, LOWER_STOP_BOTTOM_Z)
    )
    return left.union(right)


def make_top_bridge() -> cq.Workplane:
    bridge = cq.Workplane("XY").box(
        TOP_BRIDGE_WIDTH,
        TOP_BRIDGE_DEPTH,
        TOP_BRIDGE_THICKNESS,
    ).translate((0.0, 0.0, TOP_BRIDGE_CENTER_Z))

    center_pad = cq.Workplane("XY").box(0.036, 0.020, 0.006).translate((0.0, 0.0, 0.317))
    bridge = bridge.union(center_pad).edges("|Z").fillet(0.002)

    for post_x in (-POST_X, POST_X):
        seat = (
            cq.Workplane("XY")
            .circle(POST_SEAT_RADIUS)
            .extrude(TOP_BRIDGE_THICKNESS + 0.002)
            .translate((post_x, 0.0, TOP_BRIDGE_BOTTOM_Z - 0.001))
        )
        bridge = bridge.cut(seat)

    return bridge


def make_upper_stops() -> cq.Workplane:
    left = ring(UPPER_STOP_OUTER_RADIUS, POST_SEAT_RADIUS, UPPER_STOP_HEIGHT).translate(
        (-POST_X, 0.0, UPPER_STOP_BOTTOM_Z)
    )
    right = ring(UPPER_STOP_OUTER_RADIUS, POST_SEAT_RADIUS, UPPER_STOP_HEIGHT).translate(
        (POST_X, 0.0, UPPER_STOP_BOTTOM_Z)
    )
    return left.union(right)


def make_post(post_x: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(POST_RADIUS)
        .extrude(POST_TOP_Z - POST_START_Z)
        .translate((post_x, 0.0, POST_START_Z))
    )


def make_guide(post_x: float) -> cq.Workplane:
    sleeve = (
        cq.Workplane("XY")
        .circle(GUIDE_OUTER_RADIUS)
        .extrude(GUIDE_HEIGHT)
        .translate((post_x, 0.0, -GUIDE_HEIGHT / 2.0))
    )
    block = cq.Workplane("XY").box(0.030, 0.024, GUIDE_HEIGHT).translate((post_x, 0.0, 0.0))
    guide = sleeve.union(block)

    bore = (
        cq.Workplane("XY")
        .circle(GUIDE_BORE_RADIUS)
        .extrude(GUIDE_HEIGHT + 0.004)
        .translate((post_x, 0.0, -GUIDE_HEIGHT / 2.0 - 0.002))
    )
    guide = guide.cut(bore)
    return guide.edges("|Z").fillet(0.0015)


def make_carriage_core() -> cq.Workplane:
    cross_block = cq.Workplane("XY").box(0.059, 0.018, 0.044).translate((0.0, 0.0, 0.0))
    nose_block = cq.Workplane("XY").box(0.036, 0.020, 0.044).translate((0.0, 0.014, 0.0))
    column = cq.Workplane("XY").box(0.020, 0.016, 0.031).translate((0.0, 0.034, 0.0455))
    left_web = cq.Workplane("XY").box(0.006, 0.018, 0.032).translate((-0.011, 0.029, 0.046))
    right_web = cq.Workplane("XY").box(0.006, 0.018, 0.032).translate((0.011, 0.029, 0.046))
    pedestal = cq.Workplane("XY").box(0.040, 0.022, 0.010).translate((0.0, 0.041, 0.064))

    core = cross_block.union(nose_block).union(column).union(left_web).union(right_web).union(pedestal)
    return core.edges("|Z").fillet(0.0015)


def make_table() -> cq.Workplane:
    table = cq.Workplane("XY").box(TABLE_WIDTH, TABLE_DEPTH, TABLE_THICKNESS).translate(
        (0.0, 0.045, 0.073)
    )
    return table.edges("|Z").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_lift_axis")

    model.material("frame_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("machined_plate", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("polished_post", rgba=(0.86, 0.88, 0.90, 1.0))
    model.material("carriage_body", rgba=(0.57, 0.60, 0.64, 1.0))
    model.material("table_plate", rgba=(0.80, 0.82, 0.84, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_base_can(), "base_can"),
        origin=Origin(),
        material="frame_dark",
        name="base_can",
    )
    frame.visual(
        mesh_from_cadquery(make_lower_stops(), "lower_stops"),
        origin=Origin(),
        material="machined_plate",
        name="lower_stops",
    )
    frame.visual(
        mesh_from_cadquery(make_top_bridge(), "top_bridge"),
        origin=Origin(),
        material="machined_plate",
        name="top_bridge",
    )
    frame.visual(
        mesh_from_cadquery(make_upper_stops(), "upper_stops"),
        origin=Origin(),
        material="machined_plate",
        name="upper_stops",
    )
    frame.visual(
        mesh_from_cadquery(make_post(-POST_X), "left_post"),
        origin=Origin(),
        material="polished_post",
        name="left_post",
    )
    frame.visual(
        mesh_from_cadquery(make_post(POST_X), "right_post"),
        origin=Origin(),
        material="polished_post",
        name="right_post",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_guide(-POST_X), "left_guide"),
        origin=Origin(),
        material="carriage_body",
        name="left_guide",
    )
    carriage.visual(
        mesh_from_cadquery(make_guide(POST_X), "right_guide"),
        origin=Origin(),
        material="carriage_body",
        name="right_guide",
    )
    carriage.visual(
        mesh_from_cadquery(make_carriage_core(), "carriage_core"),
        origin=Origin(),
        material="carriage_body",
        name="carriage_core",
    )
    carriage.visual(
        mesh_from_cadquery(make_table(), "table"),
        origin=Origin(),
        material="table_plate",
        name="table",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_CARRIAGE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")

    left_post = frame.get_visual("left_post")
    right_post = frame.get_visual("right_post")
    lower_stops = frame.get_visual("lower_stops")
    upper_stops = frame.get_visual("upper_stops")
    left_guide = carriage.get_visual("left_guide")
    right_guide = carriage.get_visual("right_guide")
    table = carriage.get_visual("table")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "prismatic_joint_present",
        lift.joint_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC joint, got {lift.joint_type}",
    )
    ctx.check(
        "lift_axis_vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0),
        f"expected axis (0, 0, 1), got {lift.axis}",
    )
    ctx.check(
        "lift_limits_match_stop_stack",
        lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and abs((lift.motion_limits.upper or 0.0) - LIFT_TRAVEL) < 1e-9,
        "prismatic travel does not match the modeled stop geometry",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_overlap(
            carriage,
            frame,
            elem_a=left_guide,
            elem_b=left_post,
            axes="xy",
            min_overlap=0.016,
            name="left_guide_wraps_left_post",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            elem_a=right_guide,
            elem_b=right_post,
            axes="xy",
            min_overlap=0.016,
            name="right_guide_wraps_right_post",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=left_guide,
            elem_b=lower_stops,
            name="left_guide_reaches_lower_stop",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=right_guide,
            elem_b=lower_stops,
            name="right_guide_reaches_lower_stop",
        )
        ctx.expect_gap(
            carriage,
            frame,
            positive_elem=table,
            negative_elem=frame.get_visual("base_can"),
            axis="z",
            min_gap=0.09,
            name="table_stays_well_above_base_can",
        )

    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_upper_limit")
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=left_guide,
            elem_b=upper_stops,
            name="left_guide_reaches_upper_stop",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=right_guide,
            elem_b=upper_stops,
            name="right_guide_reaches_upper_stop",
        )
        ctx.expect_gap(
            carriage,
            frame,
            positive_elem=table,
            negative_elem=frame.get_visual("top_bridge"),
            axis="y",
            min_gap=0.004,
            name="table_stays_forward_of_top_bridge",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
