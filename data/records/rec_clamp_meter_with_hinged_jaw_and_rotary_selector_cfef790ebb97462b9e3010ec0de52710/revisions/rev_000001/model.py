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


BODY_DEPTH = 0.058
BODY_WIDTH = 0.106
BODY_FRONT_X = BODY_DEPTH * 0.5
JAW_CENTER_Z = 0.235
JAW_OUTER_RADIUS = 0.060
JAW_INNER_RADIUS = 0.037
JAW_DEPTH = 0.034
JAW_HINGE_ANGLE_DEG = 25.0


def _annular_sector_profile(
    *,
    center_y: float,
    center_z: float,
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    segments: int = 48,
) -> list[tuple[float, float]]:
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    if end <= start:
        end += math.tau

    outer_points: list[tuple[float, float]] = []
    inner_points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start + (end - start) * t
        outer_points.append(
            (
                center_y + outer_radius * math.cos(angle),
                center_z + outer_radius * math.sin(angle),
            )
        )
    for index in range(segments, -1, -1):
        t = index / segments
        angle = start + (end - start) * t
        inner_points.append(
            (
                center_y + inner_radius * math.cos(angle),
                center_z + inner_radius * math.sin(angle),
            )
        )
    return outer_points + inner_points


def _extrude_profile_on_yz(profile: list[tuple[float, float]], depth: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(depth)
        .translate((-depth * 0.5, 0.0, 0.0))
    )


def _x_cylinder(*, radius: float, length: float, x_center: float, y: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(length)
        .translate((x_center - length * 0.5, 0.0, 0.0))
    )


def _make_body_solid() -> cq.Workplane:
    lower = cq.Workplane("XY").box(BODY_DEPTH, BODY_WIDTH, 0.118, centered=(True, True, False))
    mid = (
        cq.Workplane("XY")
        .box(BODY_DEPTH * 0.98, 0.096, 0.046, centered=(True, True, False))
        .translate((0.0, 0.0, 0.114))
    )
    neck = (
        cq.Workplane("XY")
        .box(BODY_DEPTH * 0.96, 0.082, 0.040, centered=(True, True, False))
        .translate((0.0, 0.0, 0.156))
    )
    head = (
        cq.Workplane("XY")
        .box(BODY_DEPTH * 0.95, 0.070, 0.028, centered=(True, True, False))
        .translate((0.0, 0.0, 0.192))
    )
    jaw_bridge = (
        cq.Workplane("XY")
        .box(BODY_DEPTH * 0.90, 0.030, 0.052, centered=(True, True, False))
        .translate((0.0, 0.0, 0.168))
    )
    rear_mount = (
        cq.Workplane("XY")
        .box(0.012, 0.074, 0.014, centered=(True, True, False))
        .translate((-0.023, 0.0, 0.016))
    )

    fixed_jaw = _extrude_profile_on_yz(
        _annular_sector_profile(
            center_y=0.0,
            center_z=JAW_CENTER_Z,
            outer_radius=JAW_OUTER_RADIUS,
            inner_radius=JAW_INNER_RADIUS,
            start_deg=146.0,
            end_deg=384.0,
        ),
        JAW_DEPTH,
    )
    display_recess = (
        cq.Workplane("YZ")
        .center(0.0, 0.152)
        .rect(0.078, 0.054)
        .extrude(0.004)
        .translate((BODY_FRONT_X - 0.004, 0.0, 0.0))
    )

    return (
        lower.union(mid)
        .union(neck)
        .union(head)
        .union(jaw_bridge)
        .union(rear_mount)
        .union(fixed_jaw)
        .cut(display_recess)
    )


def _make_jaw_gate_solid() -> cq.Workplane:
    hinge_y = JAW_OUTER_RADIUS * math.cos(math.radians(JAW_HINGE_ANGLE_DEG))
    hinge_z = JAW_CENTER_Z + JAW_OUTER_RADIUS * math.sin(math.radians(JAW_HINGE_ANGLE_DEG))

    jaw_gate = _extrude_profile_on_yz(
        _annular_sector_profile(
            center_y=0.0,
            center_z=JAW_CENTER_Z,
            outer_radius=JAW_OUTER_RADIUS - 0.0015,
            inner_radius=JAW_INNER_RADIUS + 0.0015,
            start_deg=26.0,
            end_deg=144.0,
        ),
        JAW_DEPTH,
    )
    return jaw_gate.translate((0.0, -hinge_y, -hinge_z))


def _make_stand_solid() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(0.004, 0.074, 0.112, centered=(True, True, False))
        .translate((-0.003, 0.0, 0.0))
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(0.006, 0.046, 0.078, centered=(True, True, False))
        .translate((-0.003, 0.0, 0.018))
    )
    foot = (
        cq.Workplane("XY")
        .box(0.010, 0.074, 0.010, centered=(True, True, False))
        .translate((-0.006, 0.0, 0.102))
    )
    pivot_tab = (
        cq.Workplane("XY")
        .box(0.008, 0.074, 0.010, centered=(True, True, False))
        .translate((-0.004, 0.0, 0.000))
    )
    return outer.cut(inner_cut).union(foot).union(pivot_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_clamp_meter")

    body_yellow = model.material("body_yellow", rgba=(0.86, 0.71, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.11, 0.12, 0.13, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.19, 0.20, 0.22, 1.0))
    mid_grey = model.material("mid_grey", rgba=(0.36, 0.38, 0.40, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.23, 0.43, 0.50, 0.55))
    screen_bezel = model.material("screen_bezel", rgba=(0.08, 0.09, 0.10, 1.0))
    accent = model.material("accent", rgba=(0.82, 0.84, 0.85, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_solid(), "clamp_meter_body"),
        material=body_yellow,
        name="body_shell",
    )
    hinge_y = JAW_OUTER_RADIUS * math.cos(math.radians(JAW_HINGE_ANGLE_DEG))
    hinge_z = JAW_CENTER_Z + JAW_OUTER_RADIUS * math.sin(math.radians(JAW_HINGE_ANGLE_DEG))
    body.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(-0.012, hinge_y, hinge_z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_grey,
        name="rear_lug",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.012, hinge_y, hinge_z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_grey,
        name="front_lug",
    )

    display = model.part("display")
    display.visual(
        Box((0.006, 0.072, 0.048)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=screen_bezel,
        name="display_bezel",
    )
    display.visual(
        Box((0.002, 0.058, 0.032)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=screen_glass,
        name="display_glass",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(BODY_FRONT_X - 0.004, 0.0, 0.152)),
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_make_jaw_gate_solid(), "clamp_meter_jaw_gate"),
        material=rubber_black,
        name="jaw_gate",
    )
    jaw.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_grey,
        name="hinge_barrel",
    )
    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(48.0),
        ),
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_grey,
        name="knob_skirt",
    )
    selector.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=rubber_black,
        name="knob_grip",
    )
    selector.visual(
        Box((0.004, 0.004, 0.014)),
        origin=Origin(xyz=(0.015, 0.0, 0.010)),
        material=accent,
        name="pointer",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, 0.088)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    button_specs = (
        ("button_top", (0.008, 0.024, 0.010), (BODY_FRONT_X, 0.0, 0.118)),
        ("button_bottom", (0.008, 0.024, 0.010), (BODY_FRONT_X, 0.0, 0.054)),
        ("button_left", (0.008, 0.010, 0.024), (BODY_FRONT_X, -0.031, 0.088)),
        ("button_right", (0.008, 0.010, 0.024), (BODY_FRONT_X, 0.031, 0.088)),
    )
    for button_name, size, origin_xyz in button_specs:
        button = model.part(button_name)
        button.visual(
            Box(size),
            origin=Origin(xyz=(size[0] * 0.5, 0.0, 0.0)),
            material=mid_grey,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=origin_xyz),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_stand_solid(), "clamp_meter_stand"),
        material=dark_grey,
        name="stand_frame",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(-BODY_FRONT_X, 0.0, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    stand_joint = object_model.get_articulation("body_to_stand")

    ctx.expect_gap(
        selector,
        body,
        axis="x",
        positive_elem="knob_skirt",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="selector seats on the front face",
    )
    ctx.expect_contact(
        "display",
        body,
        elem_a="display_bezel",
        elem_b="body_shell",
        name="display module contacts its recess",
    )

    ctx.allow_overlap(
        body,
        jaw,
        elem_a="body_shell",
        elem_b="hinge_barrel",
        reason="The jaw hinge barrel rotates inside the partially enclosed head hinge pocket.",
    )

    for button_name in ("button_top", "button_bottom", "button_left", "button_right"):
        ctx.expect_gap(
            button_name,
            body,
            axis="x",
            positive_elem="button_cap",
            negative_elem="body_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{button_name} rests on the front face",
        )

    selector_pos = ctx.part_world_position(selector)
    top_pos = ctx.part_world_position("button_top")
    bottom_pos = ctx.part_world_position("button_bottom")
    left_pos = ctx.part_world_position("button_left")
    right_pos = ctx.part_world_position("button_right")
    ctx.check(
        "buttons surround the selector",
        (
            selector_pos is not None
            and top_pos is not None
            and bottom_pos is not None
            and left_pos is not None
            and right_pos is not None
            and top_pos[2] > selector_pos[2] + 0.02
            and bottom_pos[2] < selector_pos[2] - 0.02
            and left_pos[1] < selector_pos[1] - 0.02
            and right_pos[1] > selector_pos[1] + 0.02
        ),
        details=(
            f"selector={selector_pos}, top={top_pos}, bottom={bottom_pos}, "
            f"left={left_pos}, right={right_pos}"
        ),
    )

    for button_name in ("button_top", "button_bottom", "button_left", "button_right"):
        articulation = object_model.get_articulation(f"body_to_{button_name}")
        limits = articulation.motion_limits
        rest = ctx.part_world_position(button_name)
        pressed = None
        if limits is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.upper}):
                pressed = ctx.part_world_position(button_name)
        ctx.check(
            f"{button_name} presses inward",
            rest is not None and pressed is not None and pressed[0] < rest[0] - 0.001,
            details=f"rest={rest}, pressed={pressed}",
        )

    jaw_closed = ctx.part_element_world_aabb(jaw, elem="jaw_gate")
    jaw_open = None
    if jaw_joint.motion_limits is not None and jaw_joint.motion_limits.upper is not None:
        with ctx.pose({jaw_joint: jaw_joint.motion_limits.upper}):
            jaw_open = ctx.part_element_world_aabb(jaw, elem="jaw_gate")
    ctx.check(
        "jaw swings open above the head",
        jaw_closed is not None
        and jaw_open is not None
        and jaw_open[1][2] > jaw_closed[1][2] + 0.05,
        details=f"closed={jaw_closed}, open={jaw_open}",
    )

    stand_closed = ctx.part_world_aabb(stand)
    stand_open = None
    if stand_joint.motion_limits is not None and stand_joint.motion_limits.upper is not None:
        with ctx.pose({stand_joint: stand_joint.motion_limits.upper}):
            stand_open = ctx.part_world_aabb(stand)
    ctx.check(
        "kickstand swings rearward",
        stand_closed is not None
        and stand_open is not None
        and stand_open[0][0] < stand_closed[0][0] - 0.035,
        details=f"closed={stand_closed}, open={stand_open}",
    )

    return ctx.report()


object_model = build_object_model()
