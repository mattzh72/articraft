from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHOULDER_Z = 0.1049
FIRST_LENGTH = 0.230
SECOND_LENGTH = 0.380
TERMINAL_LENGTH = 0.205

FIRST_LAYER_Z = 0.020
SECOND_LAYER_Z = 0.050
TERMINAL_LAYER_Z = 0.080

FIRST_THICKNESS = 0.014
SECOND_THICKNESS = 0.014
TERMINAL_THICKNESS = 0.012

SHAFT_RADIUS = 0.0065
BORE_RADIUS = 0.0130
SLEEVE_RADIUS = 0.0106
WASHER_OUTER_RADIUS = 0.021
WASHER_INNER_RADIUS = 0.0098
WASHER_THICKNESS = 0.004


def _cylinder_solid(radius: float, height: float, center_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, center_z - height / 2.0))
    )


def _annular_disk(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _capsule_plate(
    length: float,
    width: float,
    boss_radius: float,
    thickness: float,
    center_z: float,
    *,
    bore_radius: float = BORE_RADIUS,
) -> cq.Workplane:
    """Flat two-boss link plate with real through bores at each joint axis."""
    bar = cq.Workplane("XY").center(length / 2.0, 0.0).rect(length, width).extrude(thickness)
    proximal = cq.Workplane("XY").circle(boss_radius).extrude(thickness)
    distal = cq.Workplane("XY").center(length, 0.0).circle(boss_radius).extrude(thickness)
    body = bar.union(proximal).union(distal)

    for x in (0.0, length):
        cutter = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(bore_radius)
            .extrude(thickness * 5.0)
            .translate((0.0, 0.0, -2.0 * thickness))
        )
        body = body.cut(cutter)

    return body.translate((0.0, 0.0, center_z - thickness / 2.0))


def _pedestal_shell() -> cq.Workplane:
    """Low calibration pedestal with a real shoulder-bolt bore through the bearing stack."""
    base = _cylinder_solid(0.165, 0.026, 0.013)
    column = _cylinder_solid(0.070, 0.058, 0.055)
    top_bearing = _cylinder_solid(0.054, 0.032, 0.096)
    body = base.union(column).union(top_bearing)

    central_bore = (
        cq.Workplane("XY")
        .circle(BORE_RADIUS + 0.0025)
        .extrude(0.130)
        .translate((0.0, 0.0, 0.020))
    )
    return body.cut(central_bore)


def _tool_plate_shape() -> cq.Workplane:
    plate_length = 0.105
    plate_width = 0.082
    thickness = 0.010
    center_x = TERMINAL_LENGTH + 0.043
    center_z = TERMINAL_LAYER_Z

    plate = (
        cq.Workplane("XY")
        .center(center_x, 0.0)
        .rect(plate_length, plate_width)
        .extrude(thickness)
        .translate((0.0, 0.0, center_z - thickness / 2.0))
    )
    for dx in (-0.030, 0.030):
        for dy in (-0.024, 0.024):
            hole = (
                cq.Workplane("XY")
                .center(center_x + dx, dy)
                .circle(0.0048)
                .extrude(thickness * 5.0)
                .translate((0.0, 0.0, center_z - 2.5 * thickness))
            )
            plate = plate.cut(hole)
    return plate


def _add_joint_hardware(
    part,
    *,
    x: float,
    layer_z: float,
    plate_thickness: float,
    washer_mesh,
    material: str,
    prefix: str,
    top_cap: bool = False,
    bottom_nut_z: float | None = None,
) -> None:
    """Add ring washers and, when this side carries the shoulder bolt, a shaft and cap."""
    top = layer_z + plate_thickness / 2.0
    bottom = layer_z - plate_thickness / 2.0
    embed = 0.0006

    part.visual(
        washer_mesh,
        origin=Origin(xyz=(x, 0.0, top + WASHER_THICKNESS / 2.0 - embed)),
        material=material,
        name=f"{prefix}_top_washer",
    )
    part.visual(
        washer_mesh,
        origin=Origin(xyz=(x, 0.0, bottom - WASHER_THICKNESS / 2.0 + embed)),
        material=material,
        name=f"{prefix}_bottom_washer",
    )

    if top_cap:
        shaft_low = -0.010 if bottom_nut_z is None else bottom_nut_z + 0.002
        shaft_high = top + WASHER_THICKNESS + 0.012
        shaft_length = shaft_high - shaft_low
        part.visual(
            Cylinder(radius=SLEEVE_RADIUS, length=plate_thickness + 2.0 * WASHER_THICKNESS),
            origin=Origin(xyz=(x, 0.0, layer_z)),
            material="brushed_steel",
            name=f"{prefix}_shoulder_sleeve",
        )
        part.visual(
            Cylinder(radius=SHAFT_RADIUS, length=shaft_length),
            origin=Origin(xyz=(x, 0.0, shaft_low + shaft_length / 2.0)),
            material="brushed_steel",
            name=f"{prefix}_shoulder_bolt",
        )
        part.visual(
            Cylinder(radius=0.0145, length=0.006),
            origin=Origin(xyz=(x, 0.0, shaft_high + 0.003 - 0.001)),
            material="black_oxide",
            name=f"{prefix}_cap_screw",
        )
        if bottom_nut_z is not None:
            part.visual(
                Cylinder(radius=0.0125, length=0.0055),
                origin=Origin(xyz=(x, 0.0, bottom_nut_z)),
                material="black_oxide",
                name=f"{prefix}_locknut",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_serial_arm")

    model.material("base_graphite", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("hard_anodized", rgba=(0.55, 0.58, 0.58, 1.0))
    model.material("dark_anodized", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    model.material("black_oxide", rgba=(0.01, 0.012, 0.014, 1.0))
    model.material("tool_blue", rgba=(0.12, 0.22, 0.34, 1.0))

    washer_mesh = mesh_from_cadquery(
        _annular_disk(WASHER_OUTER_RADIUS, WASHER_INNER_RADIUS, WASHER_THICKNESS),
        "spacer_washer",
        tolerance=0.0006,
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shell(), "pedestal_shell", tolerance=0.0008),
        material="base_graphite",
        name="pedestal_shell",
    )
    pedestal.visual(
        washer_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material="brushed_steel",
        name="shoulder_spacer",
    )
    for i, (sx, sy) in enumerate(
        ((0.110, 0.110), (-0.110, 0.110), (-0.110, -0.110), (0.110, -0.110))
    ):
        pedestal.visual(
            Cylinder(radius=0.0095, length=0.006),
            origin=Origin(xyz=(sx, sy, 0.028)),
            material="black_oxide",
            name=f"base_screw_{i}",
        )
    # Low stop lugs sit below the first link sweep and read as adjustable travel stops.
    for i, angle in enumerate((-2.25, 2.25)):
        stop_x = 0.107 * math.cos(angle)
        stop_y = 0.107 * math.sin(angle)
        pedestal.visual(
            Cylinder(radius=0.006, length=0.062),
            origin=Origin(xyz=(stop_x, stop_y, 0.0565)),
            material="dark_anodized",
            name=f"shoulder_stop_post_{i}",
        )
        pedestal.visual(
            Box((0.025, 0.018, 0.020)),
            origin=Origin(
                xyz=(stop_x, stop_y, 0.095),
                rpy=(0.0, 0.0, angle),
            ),
            material="dark_anodized",
            name=f"shoulder_stop_{i}",
        )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(
            _capsule_plate(
                FIRST_LENGTH,
                width=0.055,
                boss_radius=0.042,
                thickness=FIRST_THICKNESS,
                center_z=FIRST_LAYER_Z,
            ),
            "first_link_plate",
            tolerance=0.0007,
        ),
        material="hard_anodized",
        name="first_plate",
    )
    _add_joint_hardware(
        first_link,
        x=0.0,
        layer_z=FIRST_LAYER_Z,
        plate_thickness=FIRST_THICKNESS,
        washer_mesh=washer_mesh,
        material="brushed_steel",
        prefix="shoulder",
        top_cap=True,
        bottom_nut_z=None,
    )
    _add_joint_hardware(
        first_link,
        x=FIRST_LENGTH,
        layer_z=FIRST_LAYER_Z,
        plate_thickness=FIRST_THICKNESS,
        washer_mesh=washer_mesh,
        material="brushed_steel",
        prefix="elbow",
    )
    # Small black side plugs mark the precision dowel locations without intruding into the pivot bores.
    for i, y in enumerate((-0.019, 0.019)):
        first_link.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(FIRST_LENGTH * 0.50, y, FIRST_LAYER_Z + FIRST_THICKNESS / 2.0 + 0.001)),
            material="black_oxide",
            name=f"first_dowel_{i}",
        )

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(
            _capsule_plate(
                SECOND_LENGTH,
                width=0.050,
                boss_radius=0.039,
                thickness=SECOND_THICKNESS,
                center_z=SECOND_LAYER_Z,
            ),
            "second_link_plate",
            tolerance=0.0007,
        ),
        material="hard_anodized",
        name="second_plate",
    )
    _add_joint_hardware(
        second_link,
        x=0.0,
        layer_z=SECOND_LAYER_Z,
        plate_thickness=SECOND_THICKNESS,
        washer_mesh=washer_mesh,
        material="brushed_steel",
        prefix="elbow",
        top_cap=True,
        bottom_nut_z=0.010,
    )
    _add_joint_hardware(
        second_link,
        x=SECOND_LENGTH,
        layer_z=SECOND_LAYER_Z,
        plate_thickness=SECOND_THICKNESS,
        washer_mesh=washer_mesh,
        material="brushed_steel",
        prefix="wrist",
    )
    for i, x in enumerate((0.145, 0.235)):
        second_link.visual(
            Box((0.018, 0.006, 0.003)),
            origin=Origin(xyz=(x, 0.0, SECOND_LAYER_Z + SECOND_THICKNESS / 2.0 + 0.0015)),
            material="dark_anodized",
            name=f"second_slot_cover_{i}",
        )

    terminal_link = model.part("terminal_link")
    terminal_link.visual(
        mesh_from_cadquery(
            _capsule_plate(
                TERMINAL_LENGTH,
                width=0.034,
                boss_radius=0.033,
                thickness=TERMINAL_THICKNESS,
                center_z=TERMINAL_LAYER_Z,
            ),
            "terminal_link_plate",
            tolerance=0.0007,
        ),
        material="hard_anodized",
        name="terminal_plate",
    )
    terminal_link.visual(
        mesh_from_cadquery(_tool_plate_shape(), "tool_plate", tolerance=0.0007),
        material="tool_blue",
        name="tool_plate",
    )
    _add_joint_hardware(
        terminal_link,
        x=0.0,
        layer_z=TERMINAL_LAYER_Z,
        plate_thickness=TERMINAL_THICKNESS,
        washer_mesh=washer_mesh,
        material="brushed_steel",
        prefix="wrist",
        top_cap=True,
        bottom_nut_z=0.038,
    )
    for i, (dx, dy) in enumerate(
        ((-0.030, -0.024), (-0.030, 0.024), (0.030, -0.024), (0.030, 0.024))
    ):
        terminal_link.visual(
            Cylinder(radius=0.0075, length=0.004),
            origin=Origin(
                xyz=(TERMINAL_LENGTH + 0.043 + dx, dy, TERMINAL_LAYER_Z + 0.007)
            ),
            material="black_oxide",
            name=f"tool_screw_{i}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.4, lower=-2.05, upper=2.05
        ),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0, velocity=1.6, lower=-2.40, upper=2.40
        ),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=terminal_link,
        origin=Origin(xyz=(SECOND_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-2.25, upper=2.25
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    first = object_model.get_part("first_link")
    second = object_model.get_part("second_link")
    terminal = object_model.get_part("terminal_link")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    # The arm is intentionally a stacked planar linkage: the links share vertical
    # joint axes but run in separated machined layers so they can fold without
    # scraping covers, washers, or neighboring link plates.
    ctx.expect_gap(
        first,
        pedestal,
        axis="z",
        min_gap=0.004,
        positive_elem="first_plate",
        negative_elem="pedestal_shell",
        name="shoulder bearing clears first link plate",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        min_gap=0.010,
        positive_elem="second_plate",
        negative_elem="first_plate",
        name="elbow stacked plates have folding clearance",
    )
    ctx.expect_gap(
        terminal,
        second,
        axis="z",
        min_gap=0.010,
        positive_elem="terminal_plate",
        negative_elem="second_plate",
        name="wrist stacked plates have folding clearance",
    )
    ctx.expect_overlap(
        first,
        pedestal,
        axes="xy",
        min_overlap=0.012,
        elem_a="shoulder_shoulder_bolt",
        elem_b="pedestal_shell",
        name="shoulder bolt is carried in pedestal bearing",
    )
    ctx.expect_overlap(
        second,
        first,
        axes="xy",
        min_overlap=0.012,
        elem_a="elbow_shoulder_bolt",
        elem_b="first_plate",
        name="elbow bolt is centered through first link bore",
    )
    ctx.expect_overlap(
        terminal,
        second,
        axes="xy",
        min_overlap=0.012,
        elem_a="wrist_shoulder_bolt",
        elem_b="second_plate",
        name="wrist bolt is centered through second link bore",
    )

    with ctx.pose({shoulder: 0.85, elbow: -1.85, wrist: 1.45}):
        ctx.expect_gap(
            second,
            first,
            axis="z",
            min_gap=0.010,
            positive_elem="second_plate",
            negative_elem="first_plate",
            name="folded elbow pose keeps link layers separated",
        )
        ctx.expect_gap(
            terminal,
            second,
            axis="z",
            min_gap=0.010,
            positive_elem="terminal_plate",
            negative_elem="second_plate",
            name="folded wrist pose keeps link layers separated",
        )

    rest_tip = ctx.part_world_position(terminal)
    with ctx.pose({shoulder: -0.75, elbow: 1.35, wrist: -1.10}):
        moved_tip = ctx.part_world_position(terminal)
    ctx.check(
        "serial revolute chain moves the terminal link",
        rest_tip is not None
        and moved_tip is not None
        and math.dist(rest_tip[:2], moved_tip[:2]) > 0.12,
        details=f"rest={rest_tip}, moved={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
