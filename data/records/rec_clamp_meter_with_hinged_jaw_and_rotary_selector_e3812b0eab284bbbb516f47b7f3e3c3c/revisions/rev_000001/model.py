from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _arc_points(
    center_x: float,
    center_z: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int = 32,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = math.radians(start_deg + (end_deg - start_deg) * t)
        points.append((center_x + radius * math.cos(angle), center_z + radius * math.sin(angle)))
    return points


def _mirror_profile(right_side: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return right_side + [(-x, z) for x, z in reversed(right_side)]


def _body_shell_shape() -> cq.Workplane:
    profile = _mirror_profile(
        [
            (0.036, 0.000),
            (0.039, 0.012),
            (0.041, 0.055),
            (0.041, 0.098),
            (0.039, 0.128),
            (0.033, 0.152),
            (0.024, 0.167),
            (0.018, 0.174),
        ]
    )
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.024, both=True)


def _annular_sector(
    *,
    center_z: float,
    inner_radius: float,
    outer_radius: float,
    start_deg: float,
    end_deg: float,
    thickness: float,
) -> cq.Workplane:
    outer = _arc_points(0.0, center_z, outer_radius, start_deg, end_deg)
    inner = _arc_points(0.0, center_z, inner_radius, end_deg, start_deg)
    return cq.Workplane("XZ").polyline(outer + inner).close().extrude(thickness * 0.5, both=True)


def _hinge_neck_shape() -> cq.Workplane:
    shoulder = cq.Workplane("XY").box(0.050, 0.024, 0.018).translate((0.0, 0.0, 0.174))
    neck = cq.Workplane("XY").box(0.020, 0.018, 0.016).translate((0.0, 0.0, 0.190))
    return shoulder.union(neck)


def _jaw_ear_shape(y_offset: float) -> cq.Workplane:
    ear = cq.Workplane("XY").box(0.024, 0.008, 0.020).translate((0.0, y_offset, 0.210))
    barrel_0 = (
        cq.Workplane("XZ")
        .center(0.0, 0.222)
        .circle(0.0048)
        .extrude(0.004, both=True)
        .translate((0.0, y_offset, 0.0))
    )
    return ear.union(barrel_0)


def _fixed_jaw_shape() -> cq.Workplane:
    return _annular_sector(
        center_z=0.186,
        inner_radius=0.024,
        outer_radius=0.036,
        start_deg=104.0,
        end_deg=358.0,
        thickness=0.020,
    )


def _jaw_shape() -> cq.Workplane:
    jaw_arc = _annular_sector(
        center_z=-0.036,
        inner_radius=0.024,
        outer_radius=0.036,
        start_deg=2.0,
        end_deg=102.0,
        thickness=0.018,
    )
    hinge_barrel = cq.Workplane("XZ").center(0.0, 0.0).circle(0.0041).extrude(0.0095, both=True)
    return jaw_arc.union(hinge_barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_meter")

    body_orange = model.material("body_orange", rgba=(0.93, 0.63, 0.12, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_black = model.material("panel_black", rgba=(0.09, 0.10, 0.11, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.22, 0.34, 0.40, 0.60))
    control_grey = model.material("control_grey", rgba=(0.71, 0.73, 0.75, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "clamp_meter_body_shell"),
        material=body_orange,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_hinge_neck_shape(), "clamp_meter_hinge_neck"),
        material=plastic_black,
        name="hinge_neck",
    )
    body.visual(
        mesh_from_cadquery(_jaw_ear_shape(-0.0135), "clamp_meter_jaw_ear_0"),
        material=plastic_black,
        name="jaw_ear_0",
    )
    body.visual(
        mesh_from_cadquery(_jaw_ear_shape(0.0135), "clamp_meter_jaw_ear_1"),
        material=plastic_black,
        name="jaw_ear_1",
    )
    body.visual(
        mesh_from_cadquery(_fixed_jaw_shape(), "clamp_meter_fixed_jaw"),
        material=plastic_black,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.064, 0.002, 0.108)),
        origin=Origin(xyz=(0.0, 0.024, 0.094)),
        material=panel_black,
        name="front_panel",
    )
    body.visual(
        Box((0.050, 0.002, 0.030)),
        origin=Origin(xyz=(0.0, 0.025, 0.119)),
        material=screen_glass,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.004),
        origin=Origin(xyz=(0.0, 0.022, 0.072), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=panel_black,
        name="dial_bezel",
    )
    body.visual(
        Box((0.004, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, 0.025, 0.101)),
        material=control_grey,
        name="dial_index",
    )
    body.visual(
        Box((0.012, 0.052, 0.080)),
        origin=Origin(xyz=(-0.031, 0.0, 0.064)),
        material=plastic_black,
        name="side_grip_0",
    )
    body.visual(
        Box((0.012, 0.052, 0.080)),
        origin=Origin(xyz=(0.031, 0.0, 0.064)),
        material=plastic_black,
        name="side_grip_1",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.090, 0.056, 0.225)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_jaw_shape(), "clamp_meter_jaw"),
        material=plastic_black,
        name="jaw_shell",
    )
    jaw.inertial = Inertial.from_geometry(
        Box((0.074, 0.024, 0.052)),
        mass=0.10,
        origin=Origin(xyz=(0.018, 0.0, -0.026)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.013,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.052, 0.004, flare=0.04),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                center=False,
            ),
            "clamp_meter_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="dial_shell",
    )
    dial.visual(
        Box((0.004, 0.0025, 0.016)),
        origin=Origin(xyz=(0.0, 0.01425, 0.013)),
        material=control_grey,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.052, 0.016, 0.052)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
    )

    ncv_button = model.part("ncv_button")
    ncv_button.visual(
        Box((0.015, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=control_grey,
        name="button_cap",
    )
    ncv_button.visual(
        Box((0.011, 0.0015, 0.006)),
        origin=Origin(xyz=(0.0, 0.00375, 0.0)),
        material=panel_black,
        name="button_top",
    )
    ncv_button.inertial = Inertial.from_geometry(
        Box((0.015, 0.0045, 0.010)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.00225, 0.0)),
    )

    range_button = model.part("range_button")
    range_button.visual(
        Box((0.013, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=control_grey,
        name="button_cap",
    )
    range_button.visual(
        Box((0.009, 0.0015, 0.008)),
        origin=Origin(xyz=(0.0, 0.00375, 0.0)),
        material=panel_black,
        name="button_top",
    )
    range_button.inertial = Inertial.from_geometry(
        Box((0.013, 0.0045, 0.012)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.00225, 0.0)),
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.222)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=0.82,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.024, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=6.0,
        ),
    )
    model.articulation(
        "body_to_ncv_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=ncv_button,
        origin=Origin(xyz=(-0.020, 0.024, 0.133)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.002,
        ),
    )
    model.articulation(
        "body_to_range_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=range_button,
        origin=Origin(xyz=(0.031, 0.024, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.002,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    dial = object_model.get_part("dial")
    ncv_button = object_model.get_part("ncv_button")
    range_button = object_model.get_part("range_button")
    jaw_hinge = object_model.get_articulation("body_to_jaw")
    dial_joint = object_model.get_articulation("body_to_dial")
    ncv_joint = object_model.get_articulation("body_to_ncv_button")
    range_joint = object_model.get_articulation("body_to_range_button")
    limits = jaw_hinge.motion_limits

    ctx.expect_overlap(jaw, body, axes="y", min_overlap=0.015, name="jaw matches meter thickness")
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        elem_a="dial_shell",
        elem_b="dial_bezel",
        min_gap=0.0,
        max_gap=0.0015,
        name="dial sits on bezel without sinking into body",
    )

    closed_jaw_box = None
    open_jaw_box = None

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({jaw_hinge: limits.lower}):
            ctx.expect_contact(
                jaw,
                body,
                elem_a="jaw_shell",
                elem_b="fixed_jaw",
                contact_tol=0.001,
                name="closed jaw seats against fixed jaw",
            )
            closed_jaw_box = ctx.part_world_aabb(jaw)
        with ctx.pose({jaw_hinge: limits.upper}):
            open_jaw_box = ctx.part_world_aabb(jaw)

    jaw_opens_outward = (
        closed_jaw_box is not None
        and open_jaw_box is not None
        and float(open_jaw_box[1][0]) > float(closed_jaw_box[1][0]) + 0.012
        and float(open_jaw_box[1][2]) > float(closed_jaw_box[1][2]) + 0.008
    )
    ctx.check(
        "jaw opens upward and outward",
        jaw_opens_outward,
        details=f"closed_jaw_box={closed_jaw_box}, open_jaw_box={open_jaw_box}",
    )

    pointer_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        pointer_quarter_turn = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    dial_rotates = (
        pointer_rest is not None
        and pointer_quarter_turn is not None
        and abs(
            float(pointer_quarter_turn[1][0] + pointer_quarter_turn[0][0])
            - float(pointer_rest[1][0] + pointer_rest[0][0])
        )
        > 0.010
    )
    ctx.check(
        "dial pointer visibly rotates around the selector axis",
        dial_rotates,
        details=f"pointer_rest={pointer_rest}, pointer_quarter_turn={pointer_quarter_turn}",
    )

    ncv_rest = ctx.part_world_position(ncv_button)
    range_rest = ctx.part_world_position(range_button)
    ncv_pressed = None
    range_pressed = None
    if ncv_joint.motion_limits is not None and ncv_joint.motion_limits.upper is not None:
        with ctx.pose({ncv_joint: ncv_joint.motion_limits.upper}):
            ncv_pressed = ctx.part_world_position(ncv_button)
    if range_joint.motion_limits is not None and range_joint.motion_limits.upper is not None:
        with ctx.pose({range_joint: range_joint.motion_limits.upper}):
            range_pressed = ctx.part_world_position(range_button)

    ctx.check(
        "ncv button presses inward",
        ncv_rest is not None and ncv_pressed is not None and float(ncv_pressed[1]) < float(ncv_rest[1]) - 0.0015,
        details=f"ncv_rest={ncv_rest}, ncv_pressed={ncv_pressed}",
    )
    ctx.check(
        "range button presses inward",
        range_rest is not None
        and range_pressed is not None
        and float(range_pressed[1]) < float(range_rest[1]) - 0.0015,
        details=f"range_rest={range_rest}, range_pressed={range_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
