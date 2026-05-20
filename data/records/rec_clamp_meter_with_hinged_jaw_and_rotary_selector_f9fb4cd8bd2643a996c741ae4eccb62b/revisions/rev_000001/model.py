from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
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

BODY_THICKNESS = 0.044
BODY_FRONT_Y = BODY_THICKNESS * 0.5
BODY_BACK_Y = -BODY_FRONT_Y

JAW_OPENING_CENTER_X = 0.013
JAW_OPENING_CENTER_Z = 0.219
JAW_INNER_RADIUS = 0.023
JAW_OUTER_RADIUS = 0.038

JAW_HINGE_X = -0.017
JAW_HINGE_Z = 0.244

DIAL_CENTER_Z = 0.070
DISPLAY_CENTER_Z = 0.137
BUTTON_CENTER_Z = 0.098
BUTTON_SPACING_X = 0.032

STAND_HINGE_Z = 0.030


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_along_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
):
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length)
        .translate((0.0, center[1] - length * 0.5, 0.0))
    )


def _cylinder_along_x(
    radius: float,
    length: float,
    center: tuple[float, float, float],
):
    return (
        cq.Workplane("YZ")
        .center(center[1], center[2])
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length * 0.5, 0.0, 0.0))
    )


def _arc_points(
    center_x: float,
    center_z: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start + (end - start) * t
        points.append(
            (
                center_x + radius * math.cos(angle),
                center_z + radius * math.sin(angle),
            )
        )
    return points


def _body_mesh():
    handle = _box((0.080, BODY_THICKNESS, 0.156), (0.0, 0.0, 0.078))
    shoulder = _box((0.090, BODY_THICKNESS, 0.050), (0.0, 0.0, 0.145))
    head = _box((0.104, BODY_THICKNESS, 0.102), (0.0, 0.0, 0.205))

    body = handle.union(shoulder).union(head)

    jaw_opening = _cylinder_along_y(
        JAW_INNER_RADIUS,
        BODY_THICKNESS + 0.010,
        (JAW_OPENING_CENTER_X, 0.0, JAW_OPENING_CENTER_Z),
    )
    top_gap = _box((0.084, BODY_THICKNESS + 0.016, 0.052), (0.020, 0.0, 0.248))
    throat_relief = _box((0.040, BODY_THICKNESS + 0.016, 0.030), (0.036, 0.0, 0.216))
    left_hinge_clearance = _box((0.036, BODY_THICKNESS + 0.016, 0.032), (-0.010, 0.0, 0.245))

    display_pocket = _box((0.062, 0.0045, 0.040), (0.0, BODY_FRONT_Y - 0.00225, DISPLAY_CENTER_Z))
    dial_pocket = _cylinder_along_y(0.028, 0.0045, (0.0, BODY_FRONT_Y - 0.00225, DIAL_CENTER_Z))
    left_button_pocket = _box(
        (0.018, 0.0065, 0.012),
        (-BUTTON_SPACING_X * 0.5, BODY_FRONT_Y - 0.00325, BUTTON_CENTER_Z),
    )
    right_button_pocket = _box(
        (0.018, 0.0065, 0.012),
        (BUTTON_SPACING_X * 0.5, BODY_FRONT_Y - 0.00325, BUTTON_CENTER_Z),
    )

    rear_stand_relief = _box((0.050, 0.0045, 0.080), (0.0, BODY_BACK_Y + 0.00225, 0.074))

    body = (
        body.cut(jaw_opening)
        .cut(top_gap)
        .cut(throat_relief)
        .cut(left_hinge_clearance)
        .cut(display_pocket)
        .cut(dial_pocket)
        .cut(left_button_pocket)
        .cut(right_button_pocket)
        .cut(rear_stand_relief)
    )
    return mesh_from_cadquery(body, "clamp_meter_body")


def _jaw_mesh():
    outer = _arc_points(0.029, -0.025, 0.037, 150.0, 18.0, segments=18)
    inner = _arc_points(0.029, -0.025, 0.023, 34.0, 132.0, segments=14)
    profile = (
        outer
        + [(0.060, -0.016), (0.054, -0.006)]
        + inner
        + [
            (0.012, -0.004),
            (0.004, 0.006),
            (-0.006, 0.006),
            (-0.006, -0.004),
        ]
    )
    return mesh_from_geometry(
        ExtrudeGeometry.centered(profile, 0.038),
        "clamp_meter_jaw",
    )


def _stand_mesh():
    plate = _box((0.050, 0.005, 0.082), (0.0, -0.0025, 0.041))
    foot = _box((0.054, 0.008, 0.010), (0.0, -0.004, 0.006))
    hinge_barrel = _cylinder_along_x(0.0045, 0.034, (0.0, -0.0025, 0.0))
    center_rib = _box((0.018, 0.005, 0.032), (0.0, -0.0025, 0.018))
    stand = plate.union(foot).union(hinge_barrel).union(center_rib)
    return mesh_from_cadquery(stand, "clamp_meter_stand")


def _grip_mesh():
    grip = cq.Workplane("XY").box(0.066, 0.009, 0.080)
    grip = grip.edges("|Y").fillet(0.010)
    grip = grip.edges("|Z").fillet(0.0035)
    heel = cq.Workplane("XY").box(0.052, 0.010, 0.028).translate((0.0, 0.0, -0.034))
    heel = heel.edges("|Y").fillet(0.008)
    return mesh_from_cadquery(grip.union(heel), "clamp_meter_grip")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_clamp_meter")

    body_orange = model.material("body_orange", rgba=(0.91, 0.47, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    dial_grey = model.material("dial_grey", rgba=(0.24, 0.25, 0.27, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.30, 0.54, 0.58, 0.60))
    button_grey = model.material("button_grey", rgba=(0.28, 0.29, 0.31, 1.0))

    body = model.part("body")
    body.visual(_body_mesh(), material=body_orange, name="shell")
    body.visual(
        _grip_mesh(),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0045, 0.055)),
        material=charcoal,
        name="front_grip",
    )
    body.visual(
        Box((0.0122, 0.040, 0.014)),
        origin=Origin(xyz=(JAW_HINGE_X - 0.0116, 0.0, JAW_HINGE_Z - 0.001)),
        material=body_orange,
        name="jaw_hinge_block",
    )

    jaw = model.part("jaw")
    jaw.visual(
        _jaw_mesh(),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="jaw_shell",
    )
    jaw.visual(
        Cylinder(radius=0.0055, length=0.040),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="jaw_barrel",
    )

    display = model.part("display")
    display.visual(
        Box((0.058, 0.0036, 0.036)),
        origin=Origin(xyz=(0.0, 0.0018, 0.0)),
        material=charcoal,
        name="display_bezel",
    )
    display.visual(
        Box((0.050, 0.0020, 0.028)),
        origin=Origin(xyz=(0.0, 0.0028, 0.0)),
        material=screen_glass,
        name="display_glass",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.012,
                body_style="skirted",
                top_diameter=0.035,
                skirt=KnobSkirt(0.054, 0.0045, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.0007,
                    angle_deg=0.0,
                ),
                center=False,
            ),
            "clamp_meter_dial",
        ),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dial_grey,
        name="dial_shell",
    )
    dial.visual(
        Box((0.003, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.011, 0.005)),
        material=charcoal,
        name="dial_pointer",
    )

    for index, x_pos in enumerate((-BUTTON_SPACING_X * 0.5, BUTTON_SPACING_X * 0.5)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.014, 0.0036, 0.008)),
            origin=Origin(xyz=(0.0, 0.0018, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.0052, 0.0055)),
            origin=Origin(xyz=(0.0, -0.0026, 0.0)),
            material=button_grey,
            name="button_stem",
        )

        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BODY_FRONT_Y, BUTTON_CENTER_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.050,
                lower=0.0,
                upper=0.0024,
            ),
        )

    stand = model.part("stand")
    stand.visual(_stand_mesh(), material=charcoal, name="stand_shell")

    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.00455, DISPLAY_CENTER_Z)),
    )
    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(JAW_HINGE_X, 0.0, JAW_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0040, DIAL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
        ),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, BODY_BACK_Y, STAND_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    display = object_model.get_part("display")
    dial = object_model.get_part("dial")
    jaw = object_model.get_part("jaw")
    stand = object_model.get_part("stand")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    stand_joint = object_model.get_articulation("body_to_stand")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")

    ctx.expect_origin_gap(
        display,
        button_0,
        axis="z",
        min_gap=0.030,
        name="left button sits below the display",
    )
    ctx.expect_origin_gap(
        display,
        button_1,
        axis="z",
        min_gap=0.030,
        name="right button sits below the display",
    )
    ctx.expect_origin_gap(
        button_0,
        dial,
        axis="z",
        min_gap=0.020,
        name="buttons sit above the function dial",
    )
    ctx.expect_origin_distance(
        button_0,
        button_1,
        axes="x",
        min_dist=0.028,
        max_dist=0.040,
        name="mode buttons remain separate controls",
    )
    ctx.expect_origin_distance(
        display,
        dial,
        axes="x",
        max_dist=0.004,
        name="display stays centered over the dial",
    )

    jaw_limits = jaw_joint.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None:
        rest_jaw_aabb = ctx.part_world_aabb(jaw)
        with ctx.pose({jaw_joint: jaw_limits.upper}):
            open_jaw_aabb = ctx.part_world_aabb(jaw)
        ctx.check(
            "jaw opens upward",
            rest_jaw_aabb is not None
            and open_jaw_aabb is not None
            and open_jaw_aabb[1][2] > rest_jaw_aabb[1][2] + 0.035,
            details=f"rest={rest_jaw_aabb}, open={open_jaw_aabb}",
        )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        rest_stand_aabb = ctx.part_world_aabb(stand)
        with ctx.pose({stand_joint: stand_limits.upper}):
            open_stand_aabb = ctx.part_world_aabb(stand)
        ctx.check(
            "rear stand folds backward",
            rest_stand_aabb is not None
            and open_stand_aabb is not None
            and open_stand_aabb[0][1] < rest_stand_aabb[0][1] - 0.040,
            details=f"rest={rest_stand_aabb}, open={open_stand_aabb}",
        )

    rest_button_0 = ctx.part_world_position(button_0)
    with ctx.pose({button_0_joint: 0.0024}):
        pressed_button_0 = ctx.part_world_position(button_0)
    ctx.check(
        "left button presses inward",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and pressed_button_0[1] < rest_button_0[1] - 0.002,
        details=f"rest={rest_button_0}, pressed={pressed_button_0}",
    )

    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: 0.0024}):
        pressed_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "right button presses inward",
        rest_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_1[1] < rest_button_1[1] - 0.002,
        details=f"rest={rest_button_1}, pressed={pressed_button_1}",
    )

    rest_pointer_aabb = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.pi * 0.5}):
        turned_pointer_aabb = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    if rest_pointer_aabb is not None and turned_pointer_aabb is not None:
        rest_pointer_center = tuple(
            (float(rest_pointer_aabb[0][axis]) + float(rest_pointer_aabb[1][axis])) * 0.5
            for axis in range(3)
        )
        turned_pointer_center = tuple(
            (float(turned_pointer_aabb[0][axis]) + float(turned_pointer_aabb[1][axis])) * 0.5
            for axis in range(3)
        )
    else:
        rest_pointer_center = None
        turned_pointer_center = None
    ctx.check(
        "function dial rotates its pointer",
        rest_pointer_center is not None
        and turned_pointer_center is not None
        and abs(turned_pointer_center[0] - rest_pointer_center[0]) > 0.004,
        details=f"rest={rest_pointer_center}, turned={turned_pointer_center}",
    )

    return ctx.report()


object_model = build_object_model()
