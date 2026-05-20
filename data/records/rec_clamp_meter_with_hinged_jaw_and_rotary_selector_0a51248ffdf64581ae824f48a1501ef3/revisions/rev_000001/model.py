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

BODY_WIDTH = 0.110
BODY_DEPTH = 0.050
BODY_HEIGHT = 0.168
WAIST_WIDTH = 0.096
WAIST_DEPTH = 0.054
WAIST_HEIGHT = 0.028
WAIST_Z = 0.128
HEAD_WIDTH = 0.124
HEAD_DEPTH = 0.060
HEAD_HEIGHT = 0.054
HEAD_Z = 0.148

JAW_CENTER_Z = 0.222
JAW_OUTER_R = 0.043
JAW_INNER_R = 0.028
JAW_DEPTH = 0.060
JAW_HINGE_ANGLE_DEG = 140.0

KNOB_CENTER_Z = 0.086
KNOB_PLATE_RADIUS = 0.038
KNOB_FRONT_Y = BODY_DEPTH * 0.5 + 0.004

STAND_HINGE_Z = 0.030
STAND_HINGE_Y = -(BODY_DEPTH * 0.5 + 0.0007)

NCV_STROKE = 0.0012
RANGE_STROKE = 0.0012


def _arc_points(
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    center_x: float = 0.0,
    center_z: float = 0.0,
    segments: int = 28,
) -> list[tuple[float, float]]:
    if end_deg < start_deg:
        end_deg += 360.0
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = math.radians(start_deg + (end_deg - start_deg) * t)
        points.append(
            (
                center_x + radius * math.cos(angle),
                center_z + radius * math.sin(angle),
            )
        )
    return points


def _ring_segment_shape(
    *,
    center_z: float,
    inner_r: float,
    outer_r: float,
    start_deg: float,
    end_deg: float,
    depth: float,
    segments: int = 28,
):
    outer = _arc_points(
        outer_r,
        start_deg,
        end_deg,
        center_z=center_z,
        segments=segments,
    )
    inner = _arc_points(
        inner_r,
        start_deg,
        end_deg,
        center_z=center_z,
        segments=segments,
    )
    profile = outer + inner[::-1]
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(depth)
        .translate((0.0, depth * 0.5, 0.0))
    )


def _body_shell_shape():
    lower = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )
    waist = cq.Workplane("XY").box(
        WAIST_WIDTH,
        WAIST_DEPTH,
        WAIST_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, WAIST_Z))
    head = (
        cq.Workplane("XY")
        .box(HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, HEAD_Z))
        .edges("|Z")
        .fillet(0.008)
    )
    return lower.union(waist).union(head)


def _fixed_jaw_shape():
    return _ring_segment_shape(
        center_z=JAW_CENTER_Z,
        inner_r=JAW_INNER_R,
        outer_r=JAW_OUTER_R,
        start_deg=146.0,
        end_deg=384.0,
        depth=JAW_DEPTH,
        segments=36,
    )


def _jaw_shape():
    hinge_angle = math.radians(JAW_HINGE_ANGLE_DEG)
    hinge_x = JAW_OUTER_R * math.cos(hinge_angle)
    hinge_z = JAW_CENTER_Z + JAW_OUTER_R * math.sin(hinge_angle)
    moving_center_z = JAW_CENTER_Z + 0.006

    arc = _ring_segment_shape(
        center_z=moving_center_z,
        inner_r=JAW_INNER_R,
        outer_r=JAW_OUTER_R,
        start_deg=36.0,
        end_deg=132.0,
        depth=JAW_DEPTH,
        segments=32,
    )
    lug = (
        cq.Workplane("XY")
        .box(0.032, 0.044, 0.012, centered=(True, True, True))
        .translate((-0.038, 0.0, 0.25364))
    )
    return arc.union(lug).translate((-hinge_x, 0.0, -hinge_z))


def _stand_shape():
    outline = [
        (-0.036, 0.000),
        (-0.036, 0.078),
        (-0.024, 0.098),
        (0.024, 0.098),
        (0.036, 0.078),
        (0.036, 0.000),
    ]
    plate = (
        cq.Workplane("XZ")
        .polyline(outline)
        .close()
        .extrude(0.004)
    )
    slot = (
        cq.Workplane("XY")
        .box(0.018, 0.008, 0.028, centered=(True, True, False))
        .translate((0.0, -0.002, 0.050))
    )
    return plate.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_clamp_meter")

    shell_mat = model.material("shell_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    grip_mat = model.material("rubber_black", rgba=(0.10, 0.11, 0.12, 1.0))
    dial_mat = model.material("dial_amber", rgba=(0.90, 0.63, 0.10, 1.0))
    glass_mat = model.material("display_glass", rgba=(0.20, 0.30, 0.27, 0.45))
    bezel_mat = model.material("bezel_black", rgba=(0.12, 0.13, 0.14, 1.0))
    ncv_mat = model.material("ncv_red", rgba=(0.82, 0.16, 0.14, 1.0))
    button_mat = model.material("button_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    stand_mat = model.material("stand_graphite", rgba=(0.15, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "clamp_meter_body_shell"),
        material=shell_mat,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_fixed_jaw_shape(), "clamp_meter_fixed_jaw"),
        material=shell_mat,
        name="fixed_jaw",
    )
    hinge_angle = math.radians(JAW_HINGE_ANGLE_DEG)
    jaw_hinge_x = JAW_OUTER_R * math.cos(hinge_angle)
    jaw_hinge_z = JAW_CENTER_Z + JAW_OUTER_R * math.sin(hinge_angle)
    body.visual(
        Box((0.012, HEAD_DEPTH, 0.016)),
        origin=Origin(xyz=(jaw_hinge_x - 0.014, 0.0, jaw_hinge_z - 0.010)),
        material=shell_mat,
        name="jaw_hinge_support",
    )
    body.visual(
        Box((0.010, BODY_DEPTH + 0.002, 0.118)),
        origin=Origin(xyz=(-0.055, 0.0, 0.072)),
        material=grip_mat,
        name="grip_left",
    )
    body.visual(
        Box((0.010, BODY_DEPTH + 0.002, 0.118)),
        origin=Origin(xyz=(0.055, 0.0, 0.072)),
        material=grip_mat,
        name="grip_right",
    )
    body.visual(
        Box((0.080, 0.004, 0.052)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 + 0.0019, 0.128)),
        material=bezel_mat,
        name="display_bezel",
    )
    body.visual(
        Box((0.068, 0.003, 0.040)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 + 0.0026, 0.128)),
        material=glass_mat,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=KNOB_PLATE_RADIUS, length=0.004),
        origin=Origin(
            xyz=(0.0, BODY_DEPTH * 0.5 + 0.0020, KNOB_CENTER_Z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=bezel_mat,
        name="dial_plate",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.003),
        origin=Origin(
            xyz=(-0.028, HEAD_DEPTH * 0.5 + 0.0014, 0.155),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=bezel_mat,
        name="ncv_seat",
    )
    body.visual(
        Box((0.020, 0.003, 0.014)),
        origin=Origin(xyz=(0.039, BODY_DEPTH * 0.5 + 0.0014, 0.098)),
        material=bezel_mat,
        name="range_seat",
    )
    body.visual(
        Box((0.072, 0.005, 0.018)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 + 0.0023, 0.022)),
        material=grip_mat,
        name="lower_bumper",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_jaw_shape(), "clamp_meter_jaw"),
        material=shell_mat,
        name="jaw_arc",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dial_mat,
        name="knob_skirt",
    )
    selector_knob.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dial_mat,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.006, 0.0025, 0.018)),
        origin=Origin(xyz=(0.0, 0.019, 0.011)),
        material=bezel_mat,
        name="knob_pointer",
    )

    ncv_button = model.part("ncv_button")
    ncv_button.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=ncv_mat,
        name="ncv_cap",
    )

    range_button = model.part("range_button")
    range_button.visual(
        Box((0.016, 0.004, 0.011)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=button_mat,
        name="range_cap",
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shape(), "clamp_meter_stand"),
        material=stand_mat,
        name="stand_panel",
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(jaw_hinge_x, 0.0, jaw_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.0, KNOB_FRONT_Y, KNOB_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )
    model.articulation(
        "body_to_ncv_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=ncv_button,
        origin=Origin(xyz=(-0.028, HEAD_DEPTH * 0.5 + 0.0029, 0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=NCV_STROKE,
        ),
    )
    model.articulation(
        "body_to_range_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=range_button,
        origin=Origin(xyz=(0.039, BODY_DEPTH * 0.5 + 0.0029, 0.098)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=RANGE_STROKE,
        ),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, STAND_HINGE_Y, STAND_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    selector_knob = object_model.get_part("selector_knob")
    ncv_button = object_model.get_part("ncv_button")
    range_button = object_model.get_part("range_button")
    stand = object_model.get_part("stand")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    ncv_joint = object_model.get_articulation("body_to_ncv_button")
    range_joint = object_model.get_articulation("body_to_range_button")
    stand_joint = object_model.get_articulation("body_to_stand")

    ctx.expect_overlap(
        selector_knob,
        body,
        axes="xz",
        elem_a="knob_skirt",
        elem_b="dial_plate",
        min_overlap=0.055,
        name="selector knob stays centered on the dial plate",
    )
    ctx.expect_gap(
        selector_knob,
        body,
        axis="y",
        positive_elem="knob_skirt",
        negative_elem="dial_plate",
        max_gap=0.0006,
        max_penetration=1e-6,
        name="selector knob seats against the dial plate",
    )
    ctx.expect_overlap(
        ncv_button,
        body,
        axes="xz",
        elem_a="ncv_cap",
        elem_b="ncv_seat",
        min_overlap=0.014,
        name="ncv button stays registered on its shoulder seat",
    )
    ctx.expect_gap(
        ncv_button,
        body,
        axis="y",
        positive_elem="ncv_cap",
        negative_elem="ncv_seat",
        max_gap=0.0002,
        max_penetration=1e-6,
        name="ncv button sits on the shoulder seat",
    )
    ctx.expect_overlap(
        range_button,
        body,
        axes="xz",
        elem_a="range_cap",
        elem_b="range_seat",
        min_overlap=0.010,
        name="range button stays registered beside the dial",
    )
    ctx.expect_gap(
        range_button,
        body,
        axis="y",
        positive_elem="range_cap",
        negative_elem="range_seat",
        max_gap=0.0002,
        max_penetration=1e-6,
        name="range button sits on its seat",
    )

    jaw_limits = jaw_joint.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None:
        closed_jaw_aabb = ctx.part_world_aabb(jaw)
        with ctx.pose({jaw_joint: jaw_limits.upper}):
            open_jaw_aabb = ctx.part_world_aabb(jaw)
        ctx.check(
            "jaw opens upward away from the body",
            closed_jaw_aabb is not None
            and open_jaw_aabb is not None
            and open_jaw_aabb[1][2] > closed_jaw_aabb[1][2] + 0.010,
            details=f"closed={closed_jaw_aabb}, open={open_jaw_aabb}",
        )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        closed_stand_aabb = ctx.part_world_aabb(stand)
        with ctx.pose({stand_joint: stand_limits.upper}):
            open_stand_aabb = ctx.part_world_aabb(stand)
        ctx.check(
            "rear stand swings backward from the housing",
            closed_stand_aabb is not None
            and open_stand_aabb is not None
            and open_stand_aabb[0][1] < closed_stand_aabb[0][1] - 0.030,
            details=f"closed={closed_stand_aabb}, open={open_stand_aabb}",
        )

    ncv_rest = ctx.part_world_position(ncv_button)
    with ctx.pose({ncv_joint: NCV_STROKE}):
        ncv_pressed = ctx.part_world_position(ncv_button)
    ctx.check(
        "ncv button presses inward",
        ncv_rest is not None
        and ncv_pressed is not None
        and ncv_pressed[1] < ncv_rest[1] - 0.0010,
        details=f"rest={ncv_rest}, pressed={ncv_pressed}",
    )

    range_rest = ctx.part_world_position(range_button)
    with ctx.pose({range_joint: RANGE_STROKE}):
        range_pressed = ctx.part_world_position(range_button)
    ctx.check(
        "range button presses inward",
        range_rest is not None
        and range_pressed is not None
        and range_pressed[1] < range_rest[1] - 0.0010,
        details=f"rest={range_rest}, pressed={range_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
