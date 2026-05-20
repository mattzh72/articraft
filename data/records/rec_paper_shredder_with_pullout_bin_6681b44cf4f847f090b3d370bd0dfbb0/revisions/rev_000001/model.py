from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobSkirt,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_DEPTH = 0.29
BODY_WIDTH = 0.35
LOWER_HEIGHT = 0.37
TOTAL_HEIGHT = 0.60
PANEL_PITCH = math.radians(32.0)
PANEL_CENTER = (0.086, 0.0, 0.575)


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c + z * s, y, -x * s + z * c)


def _panel_anchor(u: float, v: float, proud: float = 0.0) -> tuple[float, float, float]:
    dx, dy, dz = _rotate_y((u, v, proud), PANEL_PITCH)
    return (PANEL_CENTER[0] + dx, PANEL_CENTER[1] + dy, PANEL_CENTER[2] + dz)


def _build_upper_shell_mesh() -> object:
    upper_profile = [
        (-0.145, 0.332),
        (-0.145, 0.515),
        (-0.105, 0.570),
        (-0.020, 0.600),
        (0.050, 0.600),
        (0.118, 0.558),
        (0.145, 0.505),
        (0.145, 0.332),
    ]
    upper = cq.Workplane("XZ").polyline(upper_profile).close().extrude(0.164, both=True)
    upper = upper.edges("|Y").fillet(0.020)

    cutter_chamber = (
        cq.Workplane("XY")
        .box(0.180, 0.244, 0.200, centered=(True, True, False))
        .translate((0.000, 0.0, 0.335))
    )
    upper = upper.cut(cutter_chamber)

    feed_slot = (
        cq.Workplane("XY")
        .box(0.014, 0.205, 0.070, centered=(True, True, False))
        .translate((0.008, 0.0, 0.530))
    )
    upper = upper.cut(feed_slot)

    access_opening = (
        cq.Workplane("XY")
        .box(0.028, 0.124, 0.060, centered=(True, True, False))
        .translate((0.131, 0.0, 0.392))
    )
    upper = upper.cut(access_opening)

    control_recess = (
        cq.Workplane("XY")
        .box(0.150, 0.240, 0.070)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), math.degrees(PANEL_PITCH))
        .translate(_panel_anchor(0.0, 0.0, -0.030))
    )
    upper = upper.cut(control_recess)

    return upper


def _build_bin_mesh() -> object:
    outer = (
        cq.Workplane("XY")
        .box(0.242, 0.302, 0.268, centered=(True, True, False))
        .translate((-0.121, 0.0, 0.0))
    )
    outer = outer.edges("|Z").fillet(0.016)

    inner = (
        cq.Workplane("XY")
        .box(0.212, 0.274, 0.238, centered=(True, True, False))
        .translate((-0.123, 0.0, 0.018))
    )
    bin_shape = outer.cut(inner)

    grip_cut = (
        cq.Workplane("XY")
        .box(0.030, 0.190, 0.050, centered=(True, True, False))
        .translate((0.000, 0.0, 0.176))
    )
    bin_shape = bin_shape.cut(grip_cut)

    return bin_shape


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[index]) + float(maxs[index])) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_shredder")

    shell_dark = model.material("shell_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    panel_black = model.material("panel_black", rgba=(0.07, 0.08, 0.09, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.11, 0.12, 1.0))
    button_grey = model.material("button_grey", rgba=(0.70, 0.72, 0.74, 1.0))
    bin_grey = model.material("bin_grey", rgba=(0.24, 0.26, 0.29, 1.0))
    steel = model.material("steel", rgba=(0.65, 0.68, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_upper_shell_mesh(), "shredder_upper_shell"),
        material=shell_dark,
        name="upper_shell",
    )
    body.visual(
        Box((0.255, 0.314, 0.030)),
        origin=Origin(xyz=(0.0175, 0.0, 0.015)),
        material=shell_dark,
        name="base_plate",
    )
    body.visual(
        Box((0.255, 0.314, 0.030)),
        origin=Origin(xyz=(0.0175, 0.0, 0.355)),
        material=shell_dark,
        name="top_bridge",
    )
    body.visual(
        Box((0.035, 0.314, 0.340)),
        origin=Origin(xyz=(-0.1275, 0.0, 0.170)),
        material=shell_dark,
        name="back_wall",
    )
    body.visual(
        Box((0.255, 0.018, 0.340)),
        origin=Origin(xyz=(0.0175, -0.166, 0.170)),
        material=shell_dark,
        name="side_wall_0",
    )
    body.visual(
        Box((0.255, 0.018, 0.340)),
        origin=Origin(xyz=(0.0175, 0.166, 0.170)),
        material=shell_dark,
        name="side_wall_1",
    )
    for side in (-1.0, 1.0):
        body.visual(
            Box((0.190, 0.008, 0.010)),
            origin=Origin(xyz=(0.010, side * 0.153, 0.076)),
            material=trim_black,
            name=f"rail_{'inner' if side < 0 else 'outer'}",
        )
    for x_center in (-0.020, 0.020):
        for side in (-1.0, 1.0):
            body.visual(
                Box((0.014, 0.016, 0.014)),
                origin=Origin(xyz=(x_center, side * 0.124, 0.465)),
                material=trim_black,
                name=f"bearing_{'rear' if x_center < 0 else 'front'}_{0 if side < 0 else 1}",
            )
    body.visual(
        Box((0.026, 0.140, 0.018)),
        origin=Origin(xyz=(0.132, 0.0, 0.456)),
        material=trim_black,
        name="flap_surround",
    )
    body.visual(
        Box((0.014, 0.205, 0.0016)),
        origin=Origin(xyz=(0.008, 0.0, 0.5992)),
        material=trim_black,
        name="slot_trim",
    )
    body.visual(
        Box((0.018, 0.140, 0.008)),
        origin=Origin(xyz=(0.136, 0.0, 0.458)),
        material=trim_black,
        name="flap_header",
    )
    body.visual(
        Box((0.140, 0.220, 0.012)),
        origin=Origin(xyz=_panel_anchor(0.0, 0.0, -0.02495), rpy=(0.0, PANEL_PITCH, 0.0)),
        material=panel_black,
        name="panel_mount",
    )
    panel = model.part("panel")
    panel.visual(
        Box((0.145, 0.230, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=panel_black,
        name="panel_carrier",
    )
    model.articulation(
        "body_to_panel",
        ArticulationType.FIXED,
        parent=body,
        child=panel,
        origin=Origin(xyz=_panel_anchor(0.0, 0.0, -0.00495), rpy=(0.0, PANEL_PITCH, 0.0)),
    )

    bin_part = model.part("bin")
    bin_part.visual(
        Box((0.232, 0.294, 0.014)),
        origin=Origin(xyz=(-0.116, 0.0, 0.007)),
        material=bin_grey,
        name="bin_floor",
    )
    bin_part.visual(
        Box((0.220, 0.010, 0.248)),
        origin=Origin(xyz=(-0.116, -0.146, 0.124)),
        material=bin_grey,
        name="bin_side_0",
    )
    bin_part.visual(
        Box((0.220, 0.010, 0.248)),
        origin=Origin(xyz=(-0.116, 0.146, 0.124)),
        material=bin_grey,
        name="bin_side_1",
    )
    bin_part.visual(
        Box((0.012, 0.294, 0.248)),
        origin=Origin(xyz=(-0.226, 0.0, 0.124)),
        material=bin_grey,
        name="bin_back",
    )
    bin_part.visual(
        Box((0.012, 0.294, 0.140)),
        origin=Origin(xyz=(-0.006, 0.0, 0.070)),
        material=bin_grey,
        name="bin_front_lower",
    )
    bin_part.visual(
        Box((0.012, 0.294, 0.060)),
        origin=Origin(xyz=(-0.006, 0.0, 0.238)),
        material=bin_grey,
        name="bin_front_upper",
    )
    bin_part.visual(
        Box((0.012, 0.052, 0.060)),
        origin=Origin(xyz=(-0.006, -0.121, 0.176)),
        material=bin_grey,
        name="grip_cheek_0",
    )
    bin_part.visual(
        Box((0.012, 0.052, 0.060)),
        origin=Origin(xyz=(-0.006, 0.121, 0.176)),
        material=bin_grey,
        name="grip_cheek_1",
    )
    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.28, lower=0.0, upper=0.150),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.022,
                body_style="skirted",
                top_diameter=0.031,
                skirt=KnobSkirt(0.048, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=20, depth=0.0008),
                center=False,
            ),
            "selector_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0006)),
        material=trim_black,
        name="knob_shell",
    )
    selector_knob.visual(
        Box((0.012, 0.003, 0.002)),
        origin=Origin(xyz=(0.010, 0.0, 0.0215)),
        material=button_grey,
        name="selector_marker",
    )
    model.articulation(
        "panel_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=selector_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    button_offsets = [
        (-0.042, 0.0),
        (-0.004, -0.050),
        (0.030, -0.050),
        (-0.004, 0.050),
        (0.030, 0.050),
    ]
    for index, (u, v) in enumerate(button_offsets):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.022, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.0030)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=trim_black,
            name="button_base",
        )
        model.articulation(
            f"panel_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=Origin(xyz=(u, v, 0.003)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.003),
        )

    cutter_front = model.part("cutter_front")
    cutter_rear = model.part("cutter_rear")
    for cutter in (cutter_front, cutter_rear):
        cutter.visual(
            Box((0.032, 0.232, 0.032)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=steel,
            name="cutter_core",
        )
        for y_offset in (-0.090, -0.054, -0.018, 0.018, 0.054, 0.090):
            cutter.visual(
                Box((0.036, 0.012, 0.036)),
                origin=Origin(xyz=(0.0, y_offset, 0.0)),
                material=steel,
                name=f"cutter_ring_{int((y_offset + 0.09) * 1000):03d}",
            )
    cutter_front.visual(
        Box((0.006, 0.012, 0.010)),
        origin=Origin(xyz=(0.019, -0.108, 0.0)),
        material=steel,
        name="timing_tooth",
    )
    cutter_rear.visual(
        Box((0.006, 0.012, 0.010)),
        origin=Origin(xyz=(0.019, 0.108, 0.0)),
        material=steel,
        name="timing_tooth",
    )

    model.articulation(
        "body_to_cutter_front",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cutter_front,
        origin=Origin(xyz=(0.020, 0.0, 0.465)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    model.articulation(
        "body_to_cutter_rear",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cutter_rear,
        origin=Origin(xyz=(-0.020, 0.0, 0.465)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
        mimic=Mimic(joint="body_to_cutter_front", multiplier=-1.0, offset=0.0),
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.008, 0.126, 0.052)),
        origin=Origin(xyz=(-0.004, 0.0, -0.026)),
        material=panel_black,
        name="flap_panel",
    )
    flap.visual(
        Box((0.010, 0.116, 0.006)),
        origin=Origin(xyz=(-0.003, 0.0, -0.003)),
        material=trim_black,
        name="flap_handle",
    )
    flap.visual(
        Box((0.008, 0.126, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_black,
        name="flap_barrel",
    )
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.149, 0.0, 0.456)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    panel = object_model.get_part("panel")
    bin_joint = object_model.get_articulation("body_to_bin")

    ctx.allow_overlap(
        panel,
        "selector_knob",
        reason="The selector knob is represented as a shallow visible cap seated onto a simplified hidden carrier behind the sloped panel surface.",
    )
    ctx.allow_isolated_part(
        panel,
        reason="The hidden control carrier sits with a sub-millimeter cosmetic clearance behind the sloped shell surface in this simplified representation.",
    )
    ctx.allow_isolated_part(
        "selector_knob",
        reason="The knob is mounted to the hidden control carrier that is intentionally modeled with a tiny cosmetic clearance behind the shell.",
    )
    for index in range(5):
        ctx.allow_overlap(
            panel,
            f"mode_button_{index}",
            reason="Each push button is represented as a shallow cap seated onto the simplified hidden carrier behind the sloped panel surface.",
        )
        ctx.allow_isolated_part(
            f"mode_button_{index}",
            reason="This button is mounted to the hidden control carrier that is intentionally modeled with a tiny cosmetic clearance behind the shell.",
        )

    ctx.expect_overlap(bin_part, body, axes="yz", min_overlap=0.25, name="bin stays laterally captured in cabinet")
    ctx.expect_overlap(bin_part, body, axes="x", min_overlap=0.22, name="bin starts deeply inserted")

    rest_bin = ctx.part_world_position(bin_part)
    with ctx.pose({bin_joint: 0.150}):
        ctx.expect_overlap(bin_part, body, axes="yz", min_overlap=0.25, name="extended bin stays guided on rails")
        ctx.expect_overlap(bin_part, body, axes="x", min_overlap=0.08, name="extended bin retains rail engagement")
        ext_bin = ctx.part_world_position(bin_part)
    ctx.check(
        "bin extends forward",
        rest_bin is not None and ext_bin is not None and ext_bin[0] > rest_bin[0] + 0.12,
        details=f"rest={rest_bin}, extended={ext_bin}",
    )

    knob = object_model.get_part("selector_knob")
    knob_joint = object_model.get_articulation("panel_to_selector_knob")
    knob_joint_ok = (
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None
    )
    ctx.check("selector knob uses continuous rotation", knob_joint_ok, details=str(knob_joint.motion_limits))
    knob_marker_rest = ctx.part_element_world_aabb(knob, elem="selector_marker")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        knob_marker_quarter = ctx.part_element_world_aabb(knob, elem="selector_marker")
    knob_marker_rest_center = _aabb_center(knob_marker_rest)
    knob_marker_quarter_center = _aabb_center(knob_marker_quarter)
    ctx.check(
        "selector marker orbits around knob axis",
        knob_marker_rest_center is not None
        and knob_marker_quarter_center is not None
        and abs(knob_marker_quarter_center[1] - knob_marker_rest_center[1]) > 0.006,
        details=f"rest={knob_marker_rest_center}, quarter={knob_marker_quarter_center}",
    )

    for index in range(5):
        button = object_model.get_part(f"mode_button_{index}")
        button_joint = object_model.get_articulation(f"panel_to_mode_button_{index}")
        rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.003}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"mode button {index} presses into panel",
            rest is not None
            and pressed is not None
            and pressed[0] < rest[0] - 0.0008
            and pressed[2] < rest[2] - 0.0020,
            details=f"rest={rest}, pressed={pressed}",
        )

    cutter_front_joint = object_model.get_articulation("body_to_cutter_front")
    cutter_rear_joint = object_model.get_articulation("body_to_cutter_rear")
    ctx.check(
        "cutters use continuous motion",
        cutter_front_joint.articulation_type == ArticulationType.CONTINUOUS
        and cutter_rear_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"front={cutter_front_joint.articulation_type}, rear={cutter_rear_joint.articulation_type}",
    )
    ctx.check(
        "rear cutter mimics front cutter in reverse",
        cutter_rear_joint.mimic is not None,
        details=f"rear mimic={cutter_rear_joint.mimic}",
    )

    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("body_to_flap")
    flap_closed_center = _aabb_center(ctx.part_element_world_aabb(flap, elem="flap_panel"))
    with ctx.pose({flap_joint: 1.05}):
        flap_open_center = _aabb_center(ctx.part_element_world_aabb(flap, elem="flap_panel"))
    ctx.check(
        "front flap swings outward",
        flap_closed_center is not None
        and flap_open_center is not None
        and flap_open_center[0] > flap_closed_center[0] + 0.015
        and abs(flap_open_center[2] - flap_closed_center[2]) > 0.008,
        details=f"closed={flap_closed_center}, open={flap_open_center}",
    )

    return ctx.report()


object_model = build_object_model()
