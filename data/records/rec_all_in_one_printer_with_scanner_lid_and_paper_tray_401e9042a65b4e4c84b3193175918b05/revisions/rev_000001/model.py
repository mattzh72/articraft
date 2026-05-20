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


BODY_DEPTH = 0.49
BODY_WIDTH = 0.47
BODY_HEIGHT = 0.63

SCANNER_TOP_Z = 0.62
SCANNER_HINGE_X = -0.18
SCANNER_HINGE_Z = 0.620

CASSETTE_ENTRY_X = 0.219
PANEL_PIVOT_X = 0.284
PANEL_PIVOT_Y = 0.080
PANEL_PIVOT_Z = 0.384


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _body_shape() -> cq.Workplane:
    lower = _box((BODY_DEPTH, BODY_WIDTH, 0.41), (0.0, 0.0, 0.205))
    upper = _box((0.43, 0.45, 0.21), (-0.02, 0.0, 0.515))
    panel_pedestal = _box((0.085, 0.19, 0.075), (0.202, 0.0, 0.3825))
    panel_chin = _box((0.055, 0.20, 0.040), (0.220, 0.0, 0.315))
    panel_pivot_rail = _box((0.040, 0.176, 0.022), (0.252, 0.078, 0.396))
    body = lower.union(upper).union(panel_pedestal).union(panel_chin).union(panel_pivot_rail)

    cassette_cavity = _box((0.38, 0.392, 0.128), (0.055, 0.0, 0.089))
    output_cavity = _box((0.22, 0.320, 0.052), (0.135, 0.0, 0.262))
    scanner_well = _box((0.318, 0.316, 0.034), (-0.020, 0.0, 0.603))
    service_cavity = _box((0.212, 0.110, 0.192), (-0.005, 0.180, 0.292))
    internal_channel = _box((0.260, 0.180, 0.080), (0.055, 0.075, 0.252))
    body = body.cut(cassette_cavity)
    body = body.cut(output_cavity)
    body = body.cut(scanner_well)
    body = body.cut(service_cavity)
    body = body.cut(internal_channel)

    output_lip = _box((0.048, 0.320, 0.006), (0.226, 0.0, 0.232))
    return body.union(output_lip)


def _flatbed_lid_shape() -> cq.Workplane:
    lid_outer = _box((0.346, 0.366, 0.028), (0.173, 0.0, 0.014))
    lid_inner = _box((0.304, 0.324, 0.024), (0.173, 0.0, 0.012))
    lid_shell = lid_outer.cut(lid_inner)

    adf_base_outer = _box((0.182, 0.342, 0.060), (0.091, 0.0, 0.058))
    adf_tunnel = _box((0.142, 0.298, 0.036), (0.102, 0.0, 0.048))
    adf_front_slot = _box((0.060, 0.286, 0.020), (0.018, 0.0, 0.044))
    adf_base = adf_base_outer.cut(adf_tunnel).cut(adf_front_slot)

    front_grip = _box((0.015, 0.110, 0.010), (0.339, 0.0, 0.010))
    hinge_bar = (
        cq.Workplane("YZ")
        .center(0.0, 0.012)
        .circle(0.008)
        .extrude(0.330)
        .translate((0.004, -0.165, 0.0))
    )
    return lid_shell.union(adf_base).union(front_grip).union(hinge_bar)


def _adf_lid_shape() -> cq.Workplane:
    cover_outer = _box((0.172, 0.344, 0.021), (0.086, 0.0, 0.0105))
    cover_inner = _box((0.146, 0.312, 0.017), (0.090, 0.0, 0.0085))
    cover_shell = cover_outer.cut(cover_inner)
    front_lip = _box((0.018, 0.210, 0.010), (0.165, 0.0, 0.010))
    hinge_bar = (
        cq.Workplane("YZ")
        .center(0.0, 0.010)
        .circle(0.006)
        .extrude(0.330)
        .translate((0.003, -0.165, 0.0))
    )
    return cover_shell.union(front_lip).union(hinge_bar)


def _cassette_shape() -> cq.Workplane:
    tray_outer = _box((0.320, 0.374, 0.089), (-0.160, 0.0, 0.0445))
    tray_inner = _box((0.292, 0.330, 0.066), (-0.160, 0.0, 0.053))
    tray = tray_outer.cut(tray_inner)

    fascia = _box((0.034, 0.388, 0.108), (0.014, 0.0, 0.054))
    handle_recess = _box((0.020, 0.116, 0.038), (0.018, 0.0, 0.060))
    label_recess = _box((0.006, 0.088, 0.016), (0.029, 0.0, 0.090))
    fascia = fascia.cut(handle_recess).cut(label_recess)

    return tray.union(fascia)


def _service_door_shape() -> cq.Workplane:
    outer = _box((0.206, 0.010, 0.194), (0.103, 0.005, 0.097))
    inner = _box((0.170, 0.008, 0.152), (0.112, 0.004, 0.098))
    latch = _box((0.016, 0.006, 0.034), (0.182, 0.011, 0.100))
    return outer.cut(inner).union(latch)


def _control_panel_shape() -> cq.Workplane:
    housing = _box((0.032, 0.168, 0.088), (0.016, 0.0, 0.044))

    screen_recess = _box((0.008, 0.094, 0.050), (0.028, -0.018, 0.056))
    housing = housing.cut(screen_recess)

    for y in (-0.048, -0.022, 0.004, 0.030):
        pocket = _box((0.004, 0.019, 0.012), (0.030, y, 0.018))
        housing = housing.cut(pocket)

    dial_hole = (
        cq.Workplane("YZ")
        .center(0.058, 0.026)
        .circle(0.009)
        .extrude(0.012)
        .translate((0.020, 0.0, 0.0))
    )
    left_trunnion = (
        cq.Workplane("XZ")
        .center(0.000, 0.012)
        .circle(0.008)
        .extrude(0.010)
        .translate((0.0, -0.079, 0.0))
    )
    right_trunnion = (
        cq.Workplane("XZ")
        .center(0.000, 0.012)
        .circle(0.008)
        .extrude(0.010)
        .translate((0.0, 0.069, 0.0))
    )
    pivot_spine = _box((0.004, 0.150, 0.010), (-0.002, -0.005, 0.012))
    return housing.cut(dial_hole).union(left_trunnion).union(right_trunnion).union(pivot_spine)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_printer")

    body_color = model.material("body_color", rgba=(0.81, 0.83, 0.85, 1.0))
    trim_color = model.material("trim_color", rgba=(0.38, 0.40, 0.44, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    glass_color = model.material("glass_color", rgba=(0.10, 0.15, 0.18, 0.55))
    button_color = model.material("button_color", rgba=(0.30, 0.32, 0.35, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "printer_body"),
        material=body_color,
        name="body_shell",
    )

    flatbed_lid = model.part("flatbed_lid")
    flatbed_lid.visual(
        mesh_from_cadquery(_flatbed_lid_shape(), "flatbed_lid"),
        material=body_color,
        name="lid_shell",
    )

    adf_lid = model.part("adf_lid")
    adf_lid.visual(
        mesh_from_cadquery(_adf_lid_shape(), "adf_lid"),
        material=trim_color,
        name="adf_cover",
    )

    cassette = model.part("cassette")
    cassette.visual(
        Box((0.320, 0.374, 0.006)),
        origin=Origin(xyz=(-0.160, 0.0, 0.003)),
        material=body_color,
        name="cassette_floor",
    )
    cassette.visual(
        Box((0.320, 0.008, 0.072)),
        origin=Origin(xyz=(-0.160, -0.183, 0.039)),
        material=body_color,
        name="cassette_wall_0",
    )
    cassette.visual(
        Box((0.320, 0.008, 0.072)),
        origin=Origin(xyz=(-0.160, 0.183, 0.039)),
        material=body_color,
        name="cassette_wall_1",
    )
    cassette.visual(
        Box((0.008, 0.374, 0.072)),
        origin=Origin(xyz=(-0.316, 0.0, 0.039)),
        material=body_color,
        name="cassette_back",
    )
    cassette.visual(
        Box((0.240, 0.010, 0.016)),
        origin=Origin(xyz=(-0.160, -0.191, 0.024)),
        material=body_color,
        name="cassette_runner_0",
    )
    cassette.visual(
        Box((0.240, 0.010, 0.016)),
        origin=Origin(xyz=(-0.160, 0.191, 0.024)),
        material=body_color,
        name="cassette_runner_1",
    )
    cassette.visual(
        Box((0.050, 0.300, 0.028)),
        origin=Origin(xyz=(0.007, 0.0, 0.017)),
        material=body_color,
        name="cassette_front",
    )
    cassette.visual(
        Box((0.012, 0.388, 0.108)),
        origin=Origin(xyz=(0.038, 0.0, 0.054)),
        material=body_color,
        name="cassette_fascia",
    )

    service_door = model.part("service_door")
    service_door.visual(
        mesh_from_cadquery(_service_door_shape(), "service_door"),
        material=trim_color,
        name="door_shell",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(_control_panel_shape(), "control_panel"),
        material=trim_color,
        name="panel_shell",
    )
    control_panel.visual(
        Box((0.002, 0.090, 0.046)),
        origin=Origin(xyz=(0.025, -0.018, 0.056)),
        material=glass_color,
        name="touchscreen",
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="dial_knob",
    )
    selector_dial.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="dial_shaft",
    )

    key_positions = (-0.048, -0.022, 0.004, 0.030)
    for index, y in enumerate(key_positions):
        key = model.part(f"key_{index}")
        key.visual(
            Box((0.003, 0.017, 0.010)),
            origin=Origin(xyz=(0.0015, 0.0, 0.0)),
            material=button_color,
            name="key_cap",
        )
        key.visual(
            Box((0.004, 0.009, 0.006)),
            origin=Origin(xyz=(-0.001, 0.0, 0.0)),
            material=button_color,
            name="key_stem",
        )
        model.articulation(
            f"control_panel_to_key_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=key,
            origin=Origin(xyz=(0.030, y, 0.018)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0012,
            ),
        )

    model.articulation(
        "body_to_flatbed_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flatbed_lid,
        origin=Origin(xyz=(SCANNER_HINGE_X, 0.0, SCANNER_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.28,
        ),
    )
    model.articulation(
        "flatbed_lid_to_adf_lid",
        ArticulationType.REVOLUTE,
        parent=flatbed_lid,
        child=adf_lid,
        origin=Origin(xyz=(0.010, 0.0, 0.088)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "body_to_cassette",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cassette,
        origin=Origin(xyz=(CASSETTE_ENTRY_X, 0.0, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=0.19,
        ),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(-0.108, 0.235, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_panel,
        origin=Origin(
            xyz=(PANEL_PIVOT_X, PANEL_PIVOT_Y, PANEL_PIVOT_Z),
            rpy=(0.0, -0.35, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.6,
            lower=0.0,
            upper=0.45,
        ),
    )
    model.articulation(
        "control_panel_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=selector_dial,
        origin=Origin(xyz=(0.031, 0.058, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=7.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flatbed_lid = object_model.get_part("flatbed_lid")
    adf_lid = object_model.get_part("adf_lid")
    cassette = object_model.get_part("cassette")
    service_door = object_model.get_part("service_door")
    control_panel = object_model.get_part("control_panel")
    selector_dial = object_model.get_part("selector_dial")

    lid_hinge = object_model.get_articulation("body_to_flatbed_lid")
    adf_hinge = object_model.get_articulation("flatbed_lid_to_adf_lid")
    cassette_slide = object_model.get_articulation("body_to_cassette")
    service_hinge = object_model.get_articulation("body_to_service_door")
    panel_tilt = object_model.get_articulation("body_to_control_panel")
    dial_joint = object_model.get_articulation("control_panel_to_selector_dial")

    ctx.allow_overlap(
        body,
        control_panel,
        elem_a="body_shell",
        elem_b="panel_shell",
        reason="The tilting control panel uses a simplified hinge spine that intentionally nests into the body pivot bracket.",
    )
    ctx.allow_overlap(
        body,
        cassette,
        elem_a="body_shell",
        elem_b="cassette_runner_0",
        reason="The cassette runner is simplified as sliding within the body side guide.",
    )
    ctx.allow_overlap(
        body,
        cassette,
        elem_a="body_shell",
        elem_b="cassette_runner_1",
        reason="The cassette runner is simplified as sliding within the body side guide.",
    )
    ctx.allow_overlap(
        selector_dial,
        control_panel,
        elem_a="dial_shaft",
        elem_b="panel_shell",
        reason="The selector dial shaft intentionally passes into the control panel housing.",
    )
    ctx.allow_isolated_part(
        selector_dial,
        reason="The selector dial is modeled with a tiny bushing clearance to read as panel-mounted without forcing a mesh collision.",
    )
    for index in range(4):
        ctx.allow_overlap(
            f"key_{index}",
            control_panel,
            elem_a="key_stem",
            elem_b="panel_shell",
            reason="Each hard key uses a simplified stem that intentionally enters the control panel housing.",
        )

    ctx.expect_gap(
        flatbed_lid,
        body,
        axis="z",
        max_gap=0.010,
        max_penetration=0.0,
        name="flatbed lid sits on scanner deck",
    )
    ctx.expect_overlap(
        flatbed_lid,
        body,
        axes="xy",
        min_overlap=0.25,
        name="flatbed lid covers scanner opening footprint",
    )
    flatbed_closed_top = None
    flatbed_open_top = None
    adf_closed_top = None
    adf_open_top = None
    cassette_closed_pos = ctx.part_world_position(cassette)
    cassette_open_pos = None
    door_closed_pos = ctx.part_world_position(service_door)
    door_open_pos = None
    door_closed_aabb = ctx.part_element_world_aabb(service_door, elem="door_shell")
    door_open_aabb = None
    panel_closed_aabb = ctx.part_element_world_aabb(control_panel, elem="panel_shell")
    panel_open_aabb = None
    body_rest_aabb = ctx.part_world_aabb(body)
    cassette_rest_aabb = ctx.part_world_aabb(cassette)
    cassette_fascia_aabb = ctx.part_element_world_aabb(cassette, elem="cassette_fascia")

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_aabb = ctx.part_world_aabb(flatbed_lid)
        if lid_aabb is not None:
            flatbed_open_top = float(lid_aabb[1][2])

    lid_aabb_rest = ctx.part_world_aabb(flatbed_lid)
    if lid_aabb_rest is not None:
        flatbed_closed_top = float(lid_aabb_rest[1][2])

    with ctx.pose({adf_hinge: adf_hinge.motion_limits.upper}):
        adf_aabb = ctx.part_world_aabb(adf_lid)
        if adf_aabb is not None:
            adf_open_top = float(adf_aabb[1][2])

    adf_aabb_rest = ctx.part_world_aabb(adf_lid)
    if adf_aabb_rest is not None:
        adf_closed_top = float(adf_aabb_rest[1][2])

    with ctx.pose({cassette_slide: cassette_slide.motion_limits.upper}):
        cassette_open_pos = ctx.part_world_position(cassette)
        ctx.expect_overlap(
            cassette,
            body,
            axes="x",
            min_overlap=0.14,
            elem_a="cassette_floor",
            elem_b="body_shell",
            name="cassette stays retained at full extension",
        )

    with ctx.pose({service_hinge: service_hinge.motion_limits.upper}):
        door_open_pos = ctx.part_world_position(service_door)
        door_open_aabb = ctx.part_element_world_aabb(service_door, elem="door_shell")

    with ctx.pose({panel_tilt: panel_tilt.motion_limits.upper}):
        panel_open_aabb = ctx.part_element_world_aabb(control_panel, elem="panel_shell")

    ctx.check(
        "cassette front closes the body opening",
        cassette_fascia_aabb is not None
        and 0.252 <= float(cassette_fascia_aabb[1][0]) <= 0.266,
        details=f"cassette_fascia={cassette_fascia_aabb}, body={body_rest_aabb}, cassette={cassette_rest_aabb}",
    )
    ctx.check(
        "flatbed lid opens upward",
        flatbed_closed_top is not None and flatbed_open_top is not None and flatbed_open_top > flatbed_closed_top + 0.12,
        details=f"closed_top={flatbed_closed_top}, open_top={flatbed_open_top}",
    )
    ctx.check(
        "adf lid opens upward",
        adf_closed_top is not None and adf_open_top is not None and adf_open_top > adf_closed_top + 0.06,
        details=f"closed_top={adf_closed_top}, open_top={adf_open_top}",
    )
    ctx.check(
        "cassette slides forward",
        cassette_closed_pos is not None
        and cassette_open_pos is not None
        and cassette_open_pos[0] > cassette_closed_pos[0] + 0.16,
        details=f"closed={cassette_closed_pos}, open={cassette_open_pos}",
    )
    ctx.check(
        "service door swings outward",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and float(door_open_aabb[1][1]) > float(door_closed_aabb[1][1]) + 0.08,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}, origins=({door_closed_pos}, {door_open_pos})",
    )
    ctx.check(
        "control panel tilts upward",
        panel_closed_aabb is not None
        and panel_open_aabb is not None
        and float(panel_open_aabb[1][0]) > float(panel_closed_aabb[1][0]) + 0.008,
        details=f"closed={panel_closed_aabb}, open={panel_open_aabb}",
    )
    ctx.check(
        "selector dial is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"joint_type={dial_joint.articulation_type}, limits={dial_joint.motion_limits}",
    )
    ctx.check(
        "four independent hard keys are present",
        all(object_model.get_part(f"key_{index}") is not None for index in range(4)),
        details="Expected key_0 through key_3 parts.",
    )
    ctx.expect_contact(
        selector_dial,
        control_panel,
        contact_tol=0.0012,
        elem_a="dial_shaft",
        elem_b="panel_shell",
        name="selector dial mounts through the panel face",
    )

    for index in range(4):
        key = object_model.get_part(f"key_{index}")
        joint = object_model.get_articulation(f"control_panel_to_key_{index}")
        key_rest = ctx.part_world_position(key)
        key_pressed = None
        with ctx.pose({joint: joint.motion_limits.upper}):
            key_pressed = ctx.part_world_position(key)
        ctx.check(
            f"key_{index} presses inward",
            key_rest is not None and key_pressed is not None and key_pressed[0] < key_rest[0] - 0.001,
            details=f"rest={key_rest}, pressed={key_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
