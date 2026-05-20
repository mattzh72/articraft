from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.200
BODY_D = 0.182
BODY_H = 0.340

SIDE_WALL = 0.005
TOP_WALL = 0.005
BOTTOM_WALL = 0.008
FRONT_WALL = 0.006

HINGE_AXIS_X = BODY_W * 0.5 - 0.001
HINGE_AXIS_Y = -BODY_D * 0.5 + 0.0045
HINGE_AXIS_Z = (BOTTOM_WALL + (BODY_H - TOP_WALL)) * 0.5

SERVICE_PANEL_W = 0.198
SERVICE_PANEL_H = 0.333
SERVICE_PANEL_T = 0.004

FILTER_TRAY_W = 0.176
FILTER_TRAY_H = 0.300
FILTER_TRAY_D = 0.108
FILTER_TRAY_TRAVEL = 0.062
FILTER_TRAY_JOINT_Y = -0.065

KNOB_AXIS_X = BODY_W * 0.5 + 0.002
KNOB_Y = 0.014
KNOB_Z = 0.214


def _body_shell_mesh():
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )

    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * SIDE_WALL,
            BODY_D - FRONT_WALL,
            BODY_H - BOTTOM_WALL - TOP_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, -FRONT_WALL * 0.5, BOTTOM_WALL))
    )
    shell = shell.cut(cavity)

    for x_pos in (-0.050, -0.025, 0.0, 0.025, 0.050):
        front_slot = (
            cq.Workplane("XY")
            .box(0.013, 0.016, 0.118, centered=(True, True, False))
            .translate((x_pos, BODY_D * 0.5 - FRONT_WALL * 0.5, 0.080))
        )
        shell = shell.cut(front_slot)

    for x_pos in (-0.054, -0.027, 0.0, 0.027, 0.054):
        top_slot = (
            cq.Workplane("XY")
            .box(0.015, 0.060, 0.014, centered=(True, True, False))
            .translate((x_pos, -0.002, BODY_H - 0.014))
        )
        shell = shell.cut(top_slot)

    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.92, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.78, 0.79, 0.77, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    filter_green = model.material("filter_green", rgba=(0.54, 0.68, 0.55, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_mesh(), "air_purifier_body_shell"),
        material=shell_white,
        name="shell",
    )
    body.visual(
        Box((0.164, 0.004, 0.122)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - FRONT_WALL - 0.002, 0.141)),
        material=charcoal,
        name="front_intake_backing",
    )
    body.visual(
        Box((0.142, 0.064, 0.004)),
        origin=Origin(xyz=(0.0, -0.002, BODY_H - TOP_WALL - 0.002)),
        material=charcoal,
        name="top_outlet_backing",
    )
    body.visual(
        Box((0.006, 0.060, 0.250)),
        origin=Origin(xyz=(0.092, -0.035, HINGE_AXIS_Z)),
        material=warm_grey,
        name="guide_0",
    )
    body.visual(
        Box((0.006, 0.060, 0.250)),
        origin=Origin(xyz=(-0.092, -0.035, HINGE_AXIS_Z)),
        material=warm_grey,
        name="guide_1",
    )
    for index, z_pos in enumerate((HINGE_AXIS_Z - 0.128, HINGE_AXIS_Z, HINGE_AXIS_Z + 0.128)):
        body.visual(
            Cylinder(radius=0.0035, length=0.068),
            origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, z_pos)),
            material=warm_grey,
            name=f"hinge_knuckle_{index}",
        )
        body.visual(
            Box((0.008, 0.004, 0.068)),
            origin=Origin(xyz=(BODY_W * 0.5 - 0.004, HINGE_AXIS_Y + 0.0055, z_pos)),
            material=warm_grey,
            name=f"hinge_rib_{index}",
        )
    body.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(
            xyz=(BODY_W * 0.5, KNOB_Y, KNOB_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_trim,
        name="knob_seat",
    )
    body.visual(
        Box((0.052, 0.018, 0.005)),
        origin=Origin(xyz=(-0.045, -0.040, 0.0025)),
        material=dark_trim,
        name="foot_0",
    )
    body.visual(
        Box((0.052, 0.018, 0.005)),
        origin=Origin(xyz=(0.045, -0.040, 0.0025)),
        material=dark_trim,
        name="foot_1",
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((SERVICE_PANEL_W, SERVICE_PANEL_T, SERVICE_PANEL_H)),
        origin=Origin(xyz=(-SERVICE_PANEL_W * 0.5, -0.0025, 0.0)),
        material=shell_white,
        name="panel_face",
    )
    service_panel.visual(
        Box((0.034, 0.008, 0.050)),
        origin=Origin(xyz=(-0.164, -0.007, -0.005)),
        material=warm_grey,
        name="panel_pull",
    )
    for index, z_pos in enumerate((-0.064, 0.064)):
        service_panel.visual(
            Cylinder(radius=0.0035, length=0.068),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=warm_grey,
            name=f"panel_knuckle_{index}",
        )

    filter_tray = model.part("filter_tray")
    filter_tray.visual(
        Box((0.010, FILTER_TRAY_D, FILTER_TRAY_H)),
        origin=Origin(xyz=(0.084, FILTER_TRAY_D * 0.5, 0.0)),
        material=charcoal,
        name="tray_side_0",
    )
    filter_tray.visual(
        Box((0.010, FILTER_TRAY_D, FILTER_TRAY_H)),
        origin=Origin(xyz=(-0.084, FILTER_TRAY_D * 0.5, 0.0)),
        material=charcoal,
        name="tray_side_1",
    )
    filter_tray.visual(
        Box((0.166, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, 0.145)),
        material=charcoal,
        name="tray_top",
    )
    filter_tray.visual(
        Box((0.166, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, -0.145)),
        material=charcoal,
        name="tray_bottom",
    )
    filter_tray.visual(
        Box((0.158, 0.072, 0.270)),
        origin=Origin(xyz=(0.0, 0.052, 0.0)),
        material=filter_green,
        name="filter_media",
    )
    filter_tray.visual(
        Box((0.020, 0.022, 0.060)),
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
        material=charcoal,
        name="tray_spine",
    )
    filter_tray.visual(
        Box((0.054, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=charcoal,
        name="tray_pull",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="skirted",
                top_diameter=0.028,
                skirt=KnobSkirt(0.039, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=22.0),
                center=False,
            ),
            "air_purifier_speed_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_trim,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(
            xyz=(0.003, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=charcoal,
        name="shaft",
    )

    model.articulation(
        "body_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_panel,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "body_to_filter_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_tray,
        origin=Origin(xyz=(0.0, FILTER_TRAY_JOINT_Y, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.10,
            lower=0.0,
            upper=FILTER_TRAY_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_knob,
        origin=Origin(xyz=(KNOB_AXIS_X, KNOB_Y, KNOB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    return model


def _aabb_size(aabb):
    mins, maxs = aabb
    return tuple(float(maxs[index] - mins[index]) for index in range(3))


def _aabb_center(aabb):
    mins, maxs = aabb
    return tuple(float((mins[index] + maxs[index]) * 0.5) for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    service_panel = object_model.get_part("service_panel")
    filter_tray = object_model.get_part("filter_tray")
    speed_knob = object_model.get_part("speed_knob")

    panel_hinge = object_model.get_articulation("body_to_service_panel")
    tray_slide = object_model.get_articulation("body_to_filter_tray")
    knob_joint = object_model.get_articulation("body_to_speed_knob")

    ctx.expect_overlap(
        service_panel,
        body,
        axes="xz",
        min_overlap=0.180,
        name="service panel covers the rear service opening",
    )
    ctx.expect_within(
        filter_tray,
        body,
        axes="xz",
        margin=0.012,
        name="filter tray stays centered within the purifier body",
    )
    ctx.expect_overlap(
        filter_tray,
        body,
        axes="y",
        min_overlap=0.075,
        name="filter tray remains inserted at rest",
    )

    body_aabb = ctx.part_world_aabb(body)
    panel_face_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_face")
    if body_aabb is not None and panel_face_aabb is not None:
        body_rear_y = float(body_aabb[0][1])
        panel_rear_y = float(panel_face_aabb[0][1])
        panel_front_y = float(panel_face_aabb[1][1])
        ctx.check(
            "service panel sits flush on the rear plane",
            abs(panel_rear_y - body_rear_y) <= 0.002 and panel_front_y > body_rear_y,
            details=(
                f"body_rear_y={body_rear_y:.4f}, "
                f"panel_rear_y={panel_rear_y:.4f}, panel_front_y={panel_front_y:.4f}"
            ),
        )

    tray_rest_pos = ctx.part_world_position(filter_tray)
    closed_panel_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_face")
    extended_tray_pos = None
    open_panel_aabb = None
    with ctx.pose({panel_hinge: math.radians(95.0), tray_slide: FILTER_TRAY_TRAVEL}):
        ctx.expect_overlap(
            filter_tray,
            body,
            axes="y",
            min_overlap=0.038,
            name="extended filter tray retains insertion in the short guides",
        )
        ctx.expect_within(
            filter_tray,
            body,
            axes="z",
            margin=0.010,
            name="extended filter tray stays vertically aligned",
        )
        extended_tray_pos = ctx.part_world_position(filter_tray)
        open_panel_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_face")

    ctx.check(
        "filter tray extends rearward",
        tray_rest_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] < tray_rest_pos[1] - 0.050,
        details=f"rest={tray_rest_pos}, extended={extended_tray_pos}",
    )

    if closed_panel_aabb is not None and open_panel_aabb is not None:
        closed_center = _aabb_center(closed_panel_aabb)
        open_center = _aabb_center(open_panel_aabb)
        ctx.check(
            "service panel swings outward from the rear edge",
            open_center[1] < closed_center[1] - 0.045 and open_center[0] > closed_center[0] + 0.080,
            details=f"closed_center={closed_center}, open_center={open_center}",
        )

    knob_aabb = ctx.part_world_aabb(speed_knob)
    body_center = ctx.part_world_position(body)
    knob_center = ctx.part_world_position(speed_knob)
    if knob_aabb is not None and knob_center is not None and body_center is not None:
        knob_size = _aabb_size(knob_aabb)
        ctx.check(
            "speed knob is mounted on the purifier side",
            knob_center[0] > body_center[0] + BODY_W * 0.45 and 0.020 <= knob_size[1] <= 0.050,
            details=f"knob_center={knob_center}, body_center={body_center}, knob_size={knob_size}",
        )

    limits = knob_joint.motion_limits
    ctx.check(
        "speed knob uses continuous rotation limits",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"limits={limits!r}",
    )

    return ctx.report()


object_model = build_object_model()
