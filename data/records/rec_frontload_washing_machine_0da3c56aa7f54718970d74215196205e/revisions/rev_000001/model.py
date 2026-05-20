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


BODY_W = 1.30
BODY_D = 0.66
BODY_H = 0.86
FOOT_H = 0.04
BODY_TOP = FOOT_H + BODY_H
WALL = 0.035
FRONT_Y = -BODY_D / 2.0

DRUM_Z = FOOT_H + 0.45
LEFT_X = -0.32
RIGHT_X = 0.32
OPENING_R = 0.245
DOOR_OUTER_R = 0.275
DOOR_INNER_R = 0.170
DOOR_CENTER_X = DOOR_OUTER_R + 0.010
HINGE_Y = FRONT_Y - 0.032
DOOR_PLANE_Y = -0.025


def _ring_on_xz(center_x: float, center_z: float, outer_r: float, inner_r: float, depth: float, y_center: float):
    """Annular solid in the XZ plane, extruded along Y."""
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(outer_r)
        .circle(inner_r)
        .extrude(depth)
        .translate((0.0, y_center + depth / 2.0, 0.0))
    )


def _make_body_shell():
    panel_depth = WALL
    shell = (
        cq.Workplane("XZ")
        .center(0.0, FOOT_H + BODY_H / 2.0)
        .rect(BODY_W, BODY_H)
        .extrude(panel_depth)
        .translate((0.0, FRONT_Y + panel_depth, 0.0))
    )

    cut_depth = 0.14
    for cx in (LEFT_X, RIGHT_X):
        cutter = (
            cq.Workplane("XZ")
            .center(cx, DRUM_Z)
            .circle(OPENING_R)
            .extrude(cut_depth)
            .translate((0.0, FRONT_Y + cut_depth, 0.0))
        )
        shell = shell.cut(cutter)

    try:
        shell = shell.edges("|Z").fillet(0.018)
    except Exception:
        pass
    return shell


def _make_front_rim(cx: float):
    return _ring_on_xz(cx, DRUM_Z, OPENING_R + 0.030, OPENING_R + 0.004, 0.026, FRONT_Y - 0.013)


def _make_door_ring():
    return _ring_on_xz(DOOR_CENTER_X, 0.0, DOOR_OUTER_R, DOOR_INNER_R, 0.035, DOOR_PLANE_Y)


def _make_drum():
    drum_r = 0.205
    drum_l = 0.315
    wall = 0.012

    tube = (
        cq.Workplane("XZ")
        .circle(drum_r)
        .circle(drum_r - wall)
        .extrude(drum_l)
        .translate((0.0, drum_l / 2.0, 0.0))
    )
    front_rim = (
        cq.Workplane("XZ")
        .circle(drum_r + 0.014)
        .circle(drum_r - 0.034)
        .extrude(0.020)
        .translate((0.0, -drum_l / 2.0 + 0.010, 0.0))
    )
    back_disc = (
        cq.Workplane("XZ")
        .circle(drum_r - 0.006)
        .extrude(0.012)
        .translate((0.0, drum_l / 2.0, 0.0))
    )
    for ix in range(-4, 5):
        for iz in range(-4, 5):
            x = ix * 0.038
            z = iz * 0.038
            if x * x + z * z < (drum_r - 0.045) ** 2 and (ix + iz) % 2 == 0:
                hole = (
                    cq.Workplane("XZ")
                    .center(x, z)
                    .circle(0.010)
                    .extrude(0.030)
                    .translate((0.0, drum_l / 2.0 + 0.006, 0.0))
                )
                back_disc = back_disc.cut(hole)

    drum = tube.union(front_rim).union(back_disc)
    for angle in (0.0, 120.0, 240.0):
        fin = (
            cq.Workplane("XY")
            .box(0.035, drum_l * 0.72, 0.064)
            .translate((0.0, 0.0, drum_r - 0.028))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        )
        drum = drum.union(fin)
    return drum


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_tub_washing_machine")

    porcelain = model.material("warm_white_porcelain", rgba=(0.92, 0.93, 0.90, 1.0))
    dark = model.material("charcoal_rubber", rgba=(0.04, 0.045, 0.05, 1.0))
    seam = model.material("dark_recess_lines", rgba=(0.02, 0.025, 0.03, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.70, 0.73, 0.72, 1.0))
    glass = model.material("smoky_transparent_glass", rgba=(0.35, 0.55, 0.68, 0.42))
    panel = model.material("pale_control_panel", rgba=(0.78, 0.82, 0.84, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "cabinet_shell", tolerance=0.002, angular_tolerance=0.15),
        material=porcelain,
        name="cabinet_shell",
    )
    body.visual(
        Box((WALL, BODY_D + 0.006, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + WALL / 2.0, 0.0, FOOT_H + BODY_H / 2.0)),
        material=porcelain,
        name="side_panel_0",
    )
    body.visual(
        Box((WALL, BODY_D + 0.006, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - WALL / 2.0, 0.0, FOOT_H + BODY_H / 2.0)),
        material=porcelain,
        name="side_panel_1",
    )
    body.visual(
        Box((BODY_W, BODY_D + 0.006, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP - WALL / 2.0)),
        material=porcelain,
        name="top_panel",
    )
    body.visual(
        Box((BODY_W, BODY_D + 0.006, WALL)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + WALL / 2.0)),
        material=porcelain,
        name="bottom_panel",
    )
    body.visual(
        Box((BODY_W, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - WALL / 2.0, FOOT_H + BODY_H / 2.0)),
        material=porcelain,
        name="back_panel",
    )
    body.visual(
        Box((0.035, BODY_D - 2.0 * WALL, BODY_H - 2.0 * WALL)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + BODY_H / 2.0)),
        material=porcelain,
        name="center_bulkhead",
    )
    for name, cx in (("wash_rim", LEFT_X), ("spin_rim", RIGHT_X)):
        body.visual(
            mesh_from_cadquery(_make_front_rim(cx), name, tolerance=0.0015, angular_tolerance=0.12),
            material=porcelain,
            name=name,
        )

    # Raised rear control plinth and surface details that make the two tubs read as separate compartments.
    body.visual(
        Box((0.62, 0.120, 0.030)),
        origin=Origin(xyz=(0.0, 0.215, BODY_TOP + 0.015)),
        material=panel,
        name="control_plinth",
    )
    body.visual(
        Box((0.014, 0.500, 0.006)),
        origin=Origin(xyz=(0.0, -0.080, BODY_TOP + 0.002)),
        material=seam,
        name="top_divider_seam",
    )
    body.visual(
        Box((0.012, 0.010, 0.700)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, FOOT_H + 0.390)),
        material=seam,
        name="front_divider_seam",
    )
    for i, x0 in enumerate((LEFT_X, RIGHT_X)):
        body.visual(
            Box((0.440, 0.010, 0.006)),
            origin=Origin(xyz=(x0, -0.150, BODY_TOP + 0.002)),
            material=seam,
            name=f"lid_seam_{i}_front",
        )
        body.visual(
            Box((0.440, 0.010, 0.006)),
            origin=Origin(xyz=(x0, 0.090, BODY_TOP + 0.002)),
            material=seam,
            name=f"lid_seam_{i}_rear",
        )
        body.visual(
            Box((0.010, 0.240, 0.006)),
            origin=Origin(xyz=(x0 - 0.220, -0.030, BODY_TOP + 0.002)),
            material=seam,
            name=f"lid_seam_{i}_side_0",
        )
        body.visual(
            Box((0.010, 0.240, 0.006)),
            origin=Origin(xyz=(x0 + 0.220, -0.030, BODY_TOP + 0.002)),
            material=seam,
            name=f"lid_seam_{i}_side_1",
        )

    for x in (-0.54, 0.54):
        for y in (-0.24, 0.24):
            body.visual(
                Box((0.110, 0.080, FOOT_H)),
                origin=Origin(xyz=(x, y, FOOT_H / 2.0)),
                material=dark,
                name=f"foot_{len(body.visuals)}",
            )

    def add_body_hinge(prefix: str, hinge_x: float):
        for index, zoff in enumerate((-0.205, 0.205)):
            body.visual(
                Box((0.050, 0.026, 0.145)),
                origin=Origin(xyz=(hinge_x + 0.015, FRONT_Y - 0.012, DRUM_Z + zoff)),
                material=porcelain,
                name=f"{prefix}_hinge_plate_{index}",
            )
            body.visual(
                Cylinder(radius=0.017, length=0.115),
                origin=Origin(xyz=(hinge_x, HINGE_Y, DRUM_Z + zoff)),
                material=dark,
                name=f"{prefix}_hinge_barrel_{index}",
            )

    wash_hinge_x = LEFT_X - DOOR_OUTER_R - 0.025
    spin_hinge_x = RIGHT_X - DOOR_OUTER_R - 0.025
    add_body_hinge("wash", wash_hinge_x)
    add_body_hinge("spin", spin_hinge_x)
    body.visual(
        Cylinder(radius=0.006, length=0.570),
        origin=Origin(xyz=(wash_hinge_x, HINGE_Y, DRUM_Z)),
        material=steel,
        name="wash_hinge_pin",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.570),
        origin=Origin(xyz=(spin_hinge_x, HINGE_Y, DRUM_Z)),
        material=steel,
        name="spin_hinge_pin",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(LEFT_X, BODY_D / 2.0 - WALL - 0.010, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wash_bearing",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(RIGHT_X, BODY_D / 2.0 - WALL - 0.010, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="spin_bearing",
    )

    drum_mesh = _make_drum()
    wash_drum = model.part("wash_drum")
    wash_drum.visual(
        mesh_from_cadquery(drum_mesh, "wash_drum_cage", tolerance=0.0015, angular_tolerance=0.10),
        material=steel,
        name="perforated_cage",
    )
    wash_drum.visual(
        Cylinder(radius=0.026, length=0.600),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="axle",
    )

    spin_drum = model.part("spin_drum")
    spin_drum.visual(
        mesh_from_cadquery(drum_mesh, "spin_drum_cage", tolerance=0.0015, angular_tolerance=0.10),
        material=steel,
        name="perforated_cage",
    )
    spin_drum.visual(
        Cylinder(radius=0.022, length=0.600),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="axle",
    )

    def add_door(name: str):
        door = model.part(name)
        door.visual(
            mesh_from_cadquery(_make_door_ring(), f"{name}_ring", tolerance=0.0015, angular_tolerance=0.10),
            material=dark,
            name="outer_ring",
        )
        door.visual(
            Cylinder(radius=0.178, length=0.012),
            origin=Origin(xyz=(DOOR_CENTER_X, DOOR_PLANE_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=glass,
            name="glass_window",
        )
        door.visual(
            Cylinder(radius=0.018, length=0.145),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark,
            name="door_knuckle",
        )
        door.visual(
            Box((0.122, 0.022, 0.036)),
            origin=Origin(xyz=(0.073, -0.007, 0.0)),
            material=dark,
            name="hinge_strap",
        )
        door.visual(
            Box((0.026, 0.036, 0.125)),
            origin=Origin(xyz=(DOOR_CENTER_X + DOOR_OUTER_R + 0.010, DOOR_PLANE_Y - 0.010, 0.0)),
            material=dark,
            name="pull_handle",
        )
        return door

    wash_door = add_door("wash_door")
    spin_door = add_door("spin_door")

    def add_dial(name: str, x: float):
        dial = model.part(name)
        dial.visual(
            Cylinder(radius=0.045, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=dark,
            name="dial_cap",
        )
        dial.visual(
            Box((0.064, 0.010, 0.007)),
            origin=Origin(xyz=(0.010, 0.0, 0.031)),
            material=porcelain,
            name="pointer_mark",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=dial,
            origin=Origin(xyz=(x, 0.215, BODY_TOP + 0.030)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=0.0, upper=4.7),
        )

    model.articulation(
        "body_to_wash_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wash_drum,
        origin=Origin(xyz=(LEFT_X, 0.0, DRUM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "body_to_spin_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spin_drum,
        origin=Origin(xyz=(RIGHT_X, 0.0, DRUM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=35.0),
    )
    model.articulation(
        "body_to_wash_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wash_door,
        origin=Origin(xyz=(wash_hinge_x, HINGE_Y, DRUM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_spin_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spin_door,
        origin=Origin(xyz=(spin_hinge_x, HINGE_Y, DRUM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=0.0, upper=1.75),
    )

    add_dial("wash_dial", -0.205)
    add_dial("mode_dial", 0.0)
    add_dial("spin_dial", 0.205)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wash_drum = object_model.get_part("wash_drum")
    spin_drum = object_model.get_part("spin_drum")
    wash_door = object_model.get_part("wash_door")
    spin_door = object_model.get_part("spin_door")

    for joint_name in ("body_to_wash_drum", "body_to_spin_drum"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name} is continuous", joint.articulation_type == ArticulationType.CONTINUOUS)
        ctx.check(f"{joint_name} axle points front to rear", tuple(joint.axis) == (0.0, 1.0, 0.0))

    ctx.allow_overlap(
        body,
        wash_drum,
        elem_a="wash_bearing",
        elem_b="axle",
        reason="The rear bushing is a solid proxy for a captured revolute axle.",
    )
    ctx.allow_overlap(
        body,
        wash_drum,
        elem_a="back_panel",
        elem_b="axle",
        reason="The axle passes through an unmodeled small clearance hole in the rear cabinet panel.",
    )
    ctx.expect_within(
        wash_drum,
        body,
        axes="xz",
        inner_elem="axle",
        outer_elem="wash_bearing",
        margin=0.0,
        name="wash axle is centered in rear bushing",
    )
    ctx.expect_overlap(
        wash_drum,
        body,
        axes="y",
        elem_a="axle",
        elem_b="wash_bearing",
        min_overlap=0.040,
        name="wash axle remains captured in rear bushing",
    )

    ctx.allow_overlap(
        body,
        spin_drum,
        elem_a="spin_bearing",
        elem_b="axle",
        reason="The rear bushing is a solid proxy for a captured revolute axle.",
    )
    ctx.allow_overlap(
        body,
        spin_drum,
        elem_a="back_panel",
        elem_b="axle",
        reason="The axle passes through an unmodeled small clearance hole in the rear cabinet panel.",
    )
    ctx.expect_within(
        spin_drum,
        body,
        axes="xz",
        inner_elem="axle",
        outer_elem="spin_bearing",
        margin=0.0,
        name="spin axle is centered in rear bushing",
    )
    ctx.expect_overlap(
        spin_drum,
        body,
        axes="y",
        elem_a="axle",
        elem_b="spin_bearing",
        min_overlap=0.040,
        name="spin axle remains captured in rear bushing",
    )

    ctx.allow_overlap(
        body,
        wash_door,
        elem_a="wash_hinge_pin",
        elem_b="door_knuckle",
        reason="The porthole door knuckle rotates around a captured hinge pin.",
    )
    ctx.expect_within(
        body,
        wash_door,
        axes="xy",
        inner_elem="wash_hinge_pin",
        outer_elem="door_knuckle",
        margin=0.001,
        name="wash hinge pin is coaxial with door knuckle",
    )
    ctx.expect_overlap(
        body,
        wash_door,
        axes="z",
        elem_a="wash_hinge_pin",
        elem_b="door_knuckle",
        min_overlap=0.12,
        name="wash hinge pin passes through door knuckle",
    )

    ctx.allow_overlap(
        body,
        spin_door,
        elem_a="spin_hinge_pin",
        elem_b="door_knuckle",
        reason="The porthole door knuckle rotates around a captured hinge pin.",
    )
    ctx.expect_within(
        body,
        spin_door,
        axes="xy",
        inner_elem="spin_hinge_pin",
        outer_elem="door_knuckle",
        margin=0.001,
        name="spin hinge pin is coaxial with door knuckle",
    )
    ctx.expect_overlap(
        body,
        spin_door,
        axes="z",
        elem_a="spin_hinge_pin",
        elem_b="door_knuckle",
        min_overlap=0.12,
        name="spin hinge pin passes through door knuckle",
    )

    ctx.expect_overlap(
        wash_door,
        wash_drum,
        axes="xz",
        min_overlap=0.32,
        elem_a="glass_window",
        elem_b="perforated_cage",
        name="wash porthole covers wash drum",
    )
    ctx.expect_overlap(
        spin_door,
        spin_drum,
        axes="xz",
        min_overlap=0.32,
        elem_a="glass_window",
        elem_b="perforated_cage",
        name="spin porthole covers spin drum",
    )

    wash_hinge = object_model.get_articulation("body_to_wash_door")
    spin_hinge = object_model.get_articulation("body_to_spin_door")
    wash_closed = ctx.part_world_aabb(wash_door)
    spin_closed = ctx.part_world_aabb(spin_door)
    with ctx.pose({wash_hinge: 1.25, spin_hinge: 1.25}):
        wash_open = ctx.part_world_aabb(wash_door)
        spin_open = ctx.part_world_aabb(spin_door)
    ctx.check(
        "wash door opens outward from front",
        wash_closed is not None and wash_open is not None and wash_open[0][1] < wash_closed[0][1] - 0.08,
        details=f"closed={wash_closed}, open={wash_open}",
    )
    ctx.check(
        "spin door opens outward from front",
        spin_closed is not None and spin_open is not None and spin_open[0][1] < spin_closed[0][1] - 0.08,
        details=f"closed={spin_closed}, open={spin_open}",
    )

    return ctx.report()


object_model = build_object_model()
