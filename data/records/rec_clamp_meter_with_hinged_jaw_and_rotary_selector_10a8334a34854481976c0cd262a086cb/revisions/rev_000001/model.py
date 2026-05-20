from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_DEPTH = 0.046
FRONT_Y = BODY_DEPTH * 0.5
BACK_Y = -BODY_DEPTH * 0.5
JAW_CENTER_Z = 0.191
JAW_OUTER_RADIUS = 0.037
JAW_INNER_RADIUS = 0.025
JAW_HINGE_Z = JAW_CENTER_Z + JAW_OUTER_RADIUS


def _arc_points(
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int = 24,
    center_x: float = 0.0,
    center_z: float = 0.0,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = math.radians(start_deg + (end_deg - start_deg) * t)
        points.append((center_x + radius * math.cos(angle), center_z + radius * math.sin(angle)))
    return points


def _annulus_segment(
    *,
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    thickness: float,
    center_x: float = 0.0,
    center_z: float = 0.0,
    segments: int = 28,
):
    outer_arc = _arc_points(
        outer_radius,
        start_deg,
        end_deg,
        segments=segments,
        center_x=center_x,
        center_z=center_z,
    )
    inner_arc = list(
        reversed(
            _arc_points(
                inner_radius,
                start_deg,
                end_deg,
                segments=segments,
                center_x=center_x,
                center_z=center_z,
            )
        )
    )
    profile = outer_arc + inner_arc
    return cq.Workplane("XZ").polyline(profile).close().extrude(thickness).translate((0.0, thickness * 0.5, 0.0))


def _cylinder_y(radius: float, length: float, *, center_x: float, center_y: float, center_z: float):
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x, center_y + length * 0.5, center_z))
    )


def _cylinder_x(radius: float, length: float, *, center_x: float, center_y: float, center_z: float):
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x - length * 0.5, center_y, center_z))
    )


def _front_rect_cut(width: float, height: float, depth: float, *, center_x: float, center_z: float):
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .rect(width, height)
        .extrude(depth)
        .translate((0.0, FRONT_Y, 0.0))
    )


def _front_circle_cut(radius: float, depth: float, *, center_x: float, center_z: float):
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(radius)
        .extrude(depth)
        .translate((0.0, FRONT_Y, 0.0))
    )


def _housing_profile() -> list[tuple[float, float]]:
    right_side = [
        (0.022, 0.000),
        (0.029, 0.040),
        (0.038, 0.086),
        (0.046, 0.130),
        (0.049, 0.154),
        (0.042, 0.160),
        (0.032, 0.164),
        (0.020, 0.166),
    ]
    return right_side + [(-x, z) for x, z in reversed(right_side)]


def _make_body_shape():
    housing = (
        cq.Workplane("XZ")
        .polyline(_housing_profile())
        .close()
        .extrude(BODY_DEPTH)
        .translate((0.0, BODY_DEPTH * 0.5, 0.0))
    )

    fixed_jaw = _annulus_segment(
        outer_radius=JAW_OUTER_RADIUS,
        inner_radius=JAW_INNER_RADIUS,
        start_deg=230.0,
        end_deg=440.0,
        thickness=0.020,
        center_z=JAW_CENTER_Z,
    )

    hinge_front_cheek = cq.Workplane("XY").box(0.014, 0.008, 0.050).translate((0.0, 0.014, 0.201))
    hinge_back_cheek = cq.Workplane("XY").box(0.014, 0.008, 0.050).translate((0.0, -0.014, 0.201))
    hinge_front_post = cq.Workplane("XY").box(0.010, 0.008, 0.044).translate((0.0, 0.014, 0.154))
    hinge_back_post = cq.Workplane("XY").box(0.010, 0.008, 0.044).translate((0.0, -0.014, 0.154))
    hinge_front_lug = _cylinder_y(0.0045, 0.008, center_x=0.0, center_y=0.014, center_z=JAW_HINGE_Z)
    hinge_back_lug = _cylinder_y(0.0045, 0.008, center_x=0.0, center_y=-0.014, center_z=JAW_HINGE_Z)

    body = housing.union(fixed_jaw)
    body = body.union(hinge_front_cheek).union(hinge_back_cheek)
    body = body.union(hinge_front_post).union(hinge_back_post)
    body = body.union(hinge_front_lug).union(hinge_back_lug)
    body = body.cut(_front_circle_cut(0.028, 0.005, center_x=0.0, center_z=0.096))
    for button_x in (-0.018, 0.0, 0.018):
        body = body.cut(_front_rect_cut(0.014, 0.012, 0.007, center_x=button_x, center_z=0.060))
    body = body.cut(cq.Workplane("XY").box(0.050, 0.022, 0.070).translate((-0.014, 0.0, 0.197)))

    return body


def _make_upper_jaw_shape():
    jaw = _annulus_segment(
        outer_radius=JAW_OUTER_RADIUS,
        inner_radius=JAW_INNER_RADIUS,
        start_deg=92.0,
        end_deg=225.0,
        thickness=0.016,
        center_z=JAW_CENTER_Z - JAW_HINGE_Z,
    )
    barrel = _cylinder_y(0.0045, 0.016, center_x=0.0, center_y=0.0, center_z=0.0)
    front_collar = _cylinder_y(0.0045, 0.002, center_x=0.0, center_y=0.009, center_z=0.0)
    back_collar = _cylinder_y(0.0045, 0.002, center_x=0.0, center_y=-0.009, center_z=0.0)
    return jaw.union(barrel).union(front_collar).union(back_collar)


def _make_stand_shape():
    tab_left = cq.Workplane("XY").box(0.012, 0.004, 0.010).translate((-0.020, -0.002, 0.005))
    tab_right = cq.Workplane("XY").box(0.012, 0.004, 0.010).translate((0.020, -0.002, 0.005))
    rail_left = cq.Workplane("XY").box(0.012, 0.004, 0.090).translate((-0.020, -0.002, 0.050))
    rail_right = cq.Workplane("XY").box(0.012, 0.004, 0.090).translate((0.020, -0.002, 0.050))
    foot = cq.Workplane("XY").box(0.056, 0.006, 0.010).translate((0.0, -0.003, 0.095))
    return tab_left.union(tab_right).union(rail_left).union(rail_right).union(foot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_meter")

    housing_dark = model.material("housing_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    jaw_dark = model.material("jaw_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    dial_black = model.material("dial_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_grey = model.material("button_grey", rgba=(0.63, 0.66, 0.70, 1.0))
    stand_dark = model.material("stand_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    display_glass = model.material("display_glass", rgba=(0.38, 0.58, 0.62, 0.50))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "clamp_meter_body"),
        material=housing_dark,
        name="body_shell",
    )

    display_window = model.part("display_window")
    display_window.visual(
        Box((0.050, 0.002, 0.028)),
        material=display_glass,
        name="display_window",
    )
    model.articulation(
        "body_to_display_window",
        ArticulationType.FIXED,
        parent=body,
        child=display_window,
        origin=Origin(xyz=(0.0, FRONT_Y + 0.001, 0.147)),
    )

    upper_jaw = model.part("upper_jaw")
    upper_jaw.visual(
        mesh_from_cadquery(_make_upper_jaw_shape(), "clamp_meter_upper_jaw"),
        material=jaw_dark,
        name="jaw_shell",
    )
    model.articulation(
        "body_to_upper_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_jaw,
        origin=Origin(xyz=(0.0, 0.0, JAW_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.0),
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.049,
                0.014,
                body_style="skirted",
                top_diameter=0.040,
                edge_radius=0.0012,
                grip=KnobGrip(style="fluted", count=16, depth=0.0010),
                center=False,
            ),
            "clamp_meter_selector",
        ),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dial_black,
        name="dial",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.003, 0.096)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=6.0),
    )

    for index, button_x in enumerate((-0.018, 0.0, 0.018)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.012, 0.0065, 0.010)),
            origin=Origin(xyz=(0.0, 0.00325, 0.005)),
            material=button_grey,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, FRONT_Y - 0.0045, 0.060)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.0015),
        )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_stand_shape(), "clamp_meter_stand"),
        material=stand_dark,
        name="stand_frame",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, BACK_Y, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    display_window = object_model.get_part("display_window")
    upper_jaw = object_model.get_part("upper_jaw")
    stand = object_model.get_part("stand")
    jaw_hinge = object_model.get_articulation("body_to_upper_jaw")
    stand_hinge = object_model.get_articulation("body_to_stand")

    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]

    ctx.allow_overlap(
        body,
        upper_jaw,
        reason="The clamp jaw uses a simplified interleaved hinge at the head, so the closed-pose pivot collars intentionally nest into the body hinge ears.",
    )

    ctx.expect_origin_distance(
        selector,
        body,
        axes="x",
        max_dist=0.002,
        name="selector stays centered on the body",
    )
    ctx.expect_origin_gap(
        display_window,
        buttons[1],
        axis="z",
        min_gap=0.070,
        name="display sits above the front buttons",
    )
    ctx.expect_origin_gap(
        selector,
        buttons[1],
        axis="z",
        min_gap=0.025,
        max_gap=0.050,
        name="buttons sit below the selector dial",
    )

    jaw_limits = jaw_hinge.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None:
        closed_jaw = ctx.part_world_aabb(upper_jaw)
        with ctx.pose({jaw_hinge: jaw_limits.upper}):
            opened_jaw = ctx.part_world_aabb(upper_jaw)
        ctx.check(
            "upper jaw opens upward and outward",
            closed_jaw is not None
            and opened_jaw is not None
            and opened_jaw[0][2] > closed_jaw[0][2] + 0.015
            and opened_jaw[0][0] < closed_jaw[0][0] - 0.006,
            details=f"closed={closed_jaw}, opened={opened_jaw}",
        )

    stand_limits = stand_hinge.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        closed_stand = ctx.part_world_aabb(stand)
        with ctx.pose({stand_hinge: stand_limits.upper}):
            opened_stand = ctx.part_world_aabb(stand)
        ctx.check(
            "rear stand folds out behind the meter",
            closed_stand is not None
            and opened_stand is not None
            and opened_stand[0][1] < closed_stand[0][1] - 0.030,
            details=f"closed={closed_stand}, opened={opened_stand}",
        )

    for index, (button, button_joint) in enumerate(zip(buttons, button_joints)):
        limits = button_joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.0010,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
