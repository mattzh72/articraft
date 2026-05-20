from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


FRAME_WIDTH = 0.22
FRAME_HEIGHT = 0.68
FRAME_PLATE_THICKNESS = 0.018
RAIL_RADIUS = 0.012
RAIL_SPACING = 0.118
RAIL_X = RAIL_SPACING / 2.0
RAIL_Y = 0.032
RAIL_LENGTH = 0.54

TOP_STOP_Z = 0.286
TOP_STOP_SIZE = (0.168, 0.050, 0.058)
BOTTOM_STOP_Z = -0.300
BOTTOM_STOP_SIZE = (0.150, 0.045, 0.052)

SLIDE_LOWER = -0.10
SLIDE_UPPER = 0.12

HINGE_Y = 0.120
HINGE_Z = -0.122
HINGE_RADIUS = 0.012
CHEEK_INNER_X = 0.010
HUB_HALF_LENGTH = CHEEK_INNER_X
PITCH_LOWER = 0.0
PITCH_UPPER = 0.35

MESH_TOL = 0.0002
MESH_ANGULAR_TOL = 0.03


def cq_mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=MESH_TOL,
        angular_tolerance=MESH_ANGULAR_TOL,
    )


def make_frame_body() -> cq.Workplane:
    body = cq.Workplane("XY").box(FRAME_WIDTH, FRAME_PLATE_THICKNESS, FRAME_HEIGHT)

    head = cq.Workplane("XY").box(0.182, 0.072, 0.118).translate((0.0, 0.012, 0.252))
    lower_housing = cq.Workplane("XY").box(0.154, 0.040, 0.060).translate((0.0, 0.010, -0.286))
    center_rib = cq.Workplane("XY").box(0.092, 0.030, 0.230).translate((0.0, 0.010, 0.008))

    window = cq.Workplane("XY").box(0.080, 0.036, 0.248).translate((0.0, 0.0, 0.000))
    head_slot = cq.Workplane("XY").box(0.070, 0.042, 0.058).translate((0.0, 0.025, 0.250))

    return body.union(head).union(lower_housing).union(center_rib).cut(window).cut(head_slot)


def make_carriage_body() -> cq.Workplane:
    truck_left = cq.Workplane("XY").box(0.042, 0.028, 0.138).translate((-RAIL_X, 0.070, 0.030))
    truck_right = cq.Workplane("XY").box(0.042, 0.028, 0.138).translate((RAIL_X, 0.070, 0.030))
    upper_bridge = cq.Workplane("XY").box(0.074, 0.018, 0.100).translate((0.0, 0.082, 0.034))
    center_spine = cq.Workplane("XY").box(0.020, 0.014, 0.050).translate((0.0, 0.094, -0.040))
    left_cheek = cq.Workplane("XY").box(0.008, 0.014, 0.072).translate((-0.022, HINGE_Y, HINGE_Z))
    right_cheek = cq.Workplane("XY").box(0.008, 0.014, 0.072).translate((0.022, HINGE_Y, HINGE_Z))
    left_brace = cq.Workplane("XY").box(0.012, 0.012, 0.034).translate((-0.016, 0.106, -0.094))
    right_brace = cq.Workplane("XY").box(0.012, 0.012, 0.034).translate((0.016, 0.106, -0.094))
    cheek_header = cq.Workplane("XY").box(0.056, 0.010, 0.016).translate((0.0, 0.106, -0.078))

    body = (
        truck_left.union(truck_right)
        .union(upper_bridge)
        .union(center_spine)
        .union(cheek_header)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_brace)
        .union(right_brace)
    )

    for rail_x in (-RAIL_X, RAIL_X):
        truck_window = cq.Workplane("XY").box(0.022, 0.010, 0.096).translate((rail_x, 0.078, 0.028))
        body = body.cut(truck_window)

    hinge_bore = (
        cq.Workplane("YZ")
        .circle(HINGE_RADIUS)
        .extrude(0.060, both=True)
        .translate((0.0, HINGE_Y, HINGE_Z))
    )
    clevis_slot = cq.Workplane("XY").box(0.036, 0.026, 0.120).translate((0.0, 0.116, -0.118))
    lower_sweep_slot = cq.Workplane("XY").box(0.026, 0.020, 0.060).translate((0.0, 0.106, -0.144))

    return body.cut(hinge_bore).cut(clevis_slot).cut(lower_sweep_slot)


def make_wrist_body() -> cq.Workplane:
    left_stub = (
        cq.Workplane("YZ")
        .circle(HINGE_RADIUS)
        .extrude(0.012, both=True)
        .translate((-0.035, 0.0, 0.0))
    )
    right_stub = (
        cq.Workplane("YZ")
        .circle(HINGE_RADIUS)
        .extrude(0.012, both=True)
        .translate((0.035, 0.0, 0.0))
    )
    left_arm = cq.Workplane("XY").box(0.012, 0.010, 0.010).translate((-0.024, 0.024, -0.002))
    right_arm = cq.Workplane("XY").box(0.012, 0.010, 0.010).translate((0.024, 0.024, -0.002))
    front_bridge = cq.Workplane("XY").box(0.044, 0.010, 0.010).translate((0.0, 0.028, -0.002))
    spine = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.022, 0.006),
                (0.034, 0.006),
                (0.038, -0.032),
                (0.038, -0.108),
                (0.030, -0.156),
                (0.016, -0.156),
                (0.014, -0.098),
                (0.014, -0.006),
            ]
        )
        .close()
        .extrude(0.008, both=True)
    )
    front_rib = cq.Workplane("XY").box(0.010, 0.008, 0.044).translate((0.0, 0.034, -0.092))
    mount_block = cq.Workplane("XY").box(0.026, 0.016, 0.014).translate((0.0, 0.034, -0.152))
    return (
        left_stub.union(right_stub)
        .union(left_arm)
        .union(right_arm)
        .union(front_bridge)
        .union(spine)
        .union(front_rib)
        .union(mount_block)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gantry_z_wrist")

    frame_gray = model.material("frame_gray", rgba=(0.67, 0.70, 0.74, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.81, 1.0))
    carriage_black = model.material("carriage_black", rgba=(0.17, 0.19, 0.22, 1.0))
    pad_dark = model.material("pad_dark", rgba=(0.10, 0.11, 0.13, 1.0))
    wrist_dark = model.material("wrist_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    tool_silver = model.material("tool_silver", rgba=(0.74, 0.76, 0.79, 1.0))

    frame = model.part("frame")
    frame.visual(cq_mesh(make_frame_body(), "frame_body"), material=frame_gray, name="frame_body")
    frame.visual(
        Box(TOP_STOP_SIZE),
        origin=Origin(xyz=(0.0, 0.020, TOP_STOP_Z)),
        material=frame_gray,
        name="top_stop",
    )
    frame.visual(
        Box(BOTTOM_STOP_SIZE),
        origin=Origin(xyz=(0.0, 0.014, BOTTOM_STOP_Z)),
        material=frame_gray,
        name="bottom_stop",
    )
    frame.visual(
        Cylinder(radius=RAIL_RADIUS, length=RAIL_LENGTH),
        origin=Origin(xyz=(-RAIL_X, RAIL_Y, 0.0)),
        material=steel,
        name="left_rail",
    )
    frame.visual(
        Cylinder(radius=RAIL_RADIUS, length=RAIL_LENGTH),
        origin=Origin(xyz=(RAIL_X, RAIL_Y, 0.0)),
        material=steel,
        name="right_rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.078, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, 0.082, 0.034)),
        material=carriage_black,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.042, 0.028, 0.138)),
        origin=Origin(xyz=(-RAIL_X, 0.070, 0.030)),
        material=carriage_black,
        name="left_truck",
    )
    carriage.visual(
        Box((0.042, 0.028, 0.138)),
        origin=Origin(xyz=(RAIL_X, 0.070, 0.030)),
        material=carriage_black,
        name="right_truck",
    )
    carriage.visual(
        Box((0.020, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, 0.098, -0.054)),
        material=carriage_black,
        name="center_spine",
    )
    carriage.visual(
        Box((0.056, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.106, -0.078)),
        material=carriage_black,
        name="cheek_header",
    )
    carriage.visual(
        Box((0.008, 0.014, 0.072)),
        origin=Origin(xyz=(-0.022, 0.112, HINGE_Z)),
        material=carriage_black,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.008, 0.014, 0.072)),
        origin=Origin(xyz=(0.022, 0.112, HINGE_Z)),
        material=carriage_black,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.012, 0.016, 0.034)),
        origin=Origin(xyz=(-0.016, 0.106, -0.094)),
        material=carriage_black,
        name="left_brace",
    )
    carriage.visual(
        Box((0.012, 0.016, 0.034)),
        origin=Origin(xyz=(0.016, 0.106, -0.094)),
        material=carriage_black,
        name="right_brace",
    )
    carriage.visual(
        Box((0.026, 0.014, 0.112)),
        origin=Origin(xyz=(-RAIL_X, 0.051, 0.028)),
        material=pad_dark,
        name="left_rail_pad",
    )
    carriage.visual(
        Box((0.026, 0.014, 0.112)),
        origin=Origin(xyz=(RAIL_X, 0.051, 0.028)),
        material=pad_dark,
        name="right_rail_pad",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.006),
        origin=Origin(xyz=(-0.029, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    wrist.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.006),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    wrist.visual(
        Box((0.022, 0.008, 0.010)),
        origin=Origin(xyz=(-0.019, 0.008, -0.002)),
        material=wrist_dark,
        name="left_hub_arm",
    )
    wrist.visual(
        Box((0.022, 0.008, 0.010)),
        origin=Origin(xyz=(0.019, 0.008, -0.002)),
        material=wrist_dark,
        name="right_hub_arm",
    )
    wrist.visual(
        Box((0.030, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, -0.002)),
        material=wrist_dark,
        name="hub_bridge",
    )
    wrist.visual(
        Box((0.014, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.018, -0.014)),
        material=wrist_dark,
        name="wrist_neck",
    )
    wrist.visual(
        Box((0.016, 0.014, 0.080)),
        origin=Origin(xyz=(0.0, 0.026, -0.056)),
        material=wrist_dark,
        name="wrist_body",
    )
    wrist.visual(
        Box((0.026, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.030, -0.114)),
        material=wrist_dark,
        name="tool_mount",
    )
    wrist.visual(
        Box((0.068, 0.046, 0.008)),
        origin=Origin(xyz=(0.0, 0.034, -0.138)),
        material=tool_silver,
        name="tool_plate",
    )

    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.30,
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.0,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist")
    z_slide = object_model.get_articulation("z_slide")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    left_rail = frame.get_visual("left_rail")
    right_rail = frame.get_visual("right_rail")
    top_stop = frame.get_visual("top_stop")
    bottom_stop = frame.get_visual("bottom_stop")
    left_rail_pad = carriage.get_visual("left_rail_pad")
    right_rail_pad = carriage.get_visual("right_rail_pad")
    left_cheek = carriage.get_visual("left_cheek")
    right_cheek = carriage.get_visual("right_cheek")
    left_trunnion = wrist.get_visual("left_trunnion")
    right_trunnion = wrist.get_visual("right_trunnion")
    tool_plate = wrist.get_visual("tool_plate")

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
        "z_slide_axis_is_vertical",
        tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical slide axis, got {z_slide.axis}",
    )
    ctx.check(
        "wrist_axis_is_horizontal",
        tuple(wrist_pitch.axis) == (1.0, 0.0, 0.0),
        details=f"Expected horizontal wrist axis, got {wrist_pitch.axis}",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a=left_rail,
        elem_b=left_rail_pad,
        contact_tol=0.0005,
        name="left_rail_supports_carriage",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a=right_rail,
        elem_b=right_rail_pad,
        contact_tol=0.0005,
        name="right_rail_supports_carriage",
    )
    ctx.expect_contact(
        wrist,
        carriage,
        elem_a=left_trunnion,
        elem_b=left_cheek,
        contact_tol=0.0005,
        name="left_trunnion_supported_by_left_cheek",
    )
    ctx.expect_contact(
        wrist,
        carriage,
        elem_a=right_trunnion,
        elem_b=right_cheek,
        contact_tol=0.0005,
        name="right_trunnion_supported_by_right_cheek",
    )

    with ctx.pose({z_slide: SLIDE_UPPER}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem=top_stop,
            min_gap=0.015,
            name="carriage_clears_top_stop_at_upper_travel",
        )

    with ctx.pose({z_slide: SLIDE_LOWER}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            negative_elem=bottom_stop,
            min_gap=0.006,
            name="carriage_clears_bottom_stop_at_lower_travel",
        )

    with ctx.pose({z_slide: SLIDE_UPPER, wrist_pitch: PITCH_LOWER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_upper_back_pitch_pose")

    with ctx.pose({z_slide: SLIDE_LOWER, wrist_pitch: PITCH_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_lower_forward_pitch_pose")
        ctx.expect_gap(
            wrist,
            frame,
            axis="y",
            positive_elem=tool_plate,
            min_gap=0.080,
            name="tool_plate_stays_forward_of_frame_when_pitched_out",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
