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


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _safe_shell() -> cq.Workplane:
    """Broad flanged in-wall safe carcass with a real open storage cavity."""
    body_w, body_d, body_h = 0.46, 0.32, 0.33
    cavity_w, cavity_d, cavity_h = 0.42, 0.305, 0.275
    flange_w, flange_t, flange_h = 0.56, 0.047, 0.43
    opening_w, opening_h = 0.44, 0.295

    body = _box_at((body_w, body_d, body_h), (0.0, body_d / 2.0, 0.0))
    cavity_cut = _box_at(
        (cavity_w, cavity_d, cavity_h),
        (0.0, -0.020 + cavity_d / 2.0, 0.0),
    )
    shell = body.cut(cavity_cut)

    flange = _box_at((flange_w, flange_t, flange_h), (0.0, -0.035 + flange_t / 2.0, 0.0))
    flange_cut = _box_at((opening_w, flange_t + 0.020, opening_h), (0.0, -0.035 + flange_t / 2.0, 0.0))
    flange = flange.cut(flange_cut)

    return shell.union(flange)


def _door_panel() -> cq.Workplane:
    """Compact slab door in a hinge-line local frame; geometry extends to -X."""
    door_w, door_t, door_h = 0.39, 0.052, 0.25
    hinge_clearance = 0.018
    front_y = -door_t / 2.0
    main = _box_at((door_w, door_t, door_h), (-hinge_clearance - door_w / 2.0, 0.0, 0.0))

    # Low raised perimeter on the face gives the hotel-safe door its inset-panel read.
    rib_t, rib_y = 0.012, 0.006
    rib_y_center = front_y - rib_y / 2.0
    left = _box_at((rib_t, rib_y, 0.205), (-hinge_clearance - door_w + 0.035, rib_y_center, 0.0))
    right = _box_at((rib_t, rib_y, 0.205), (-hinge_clearance - 0.035, rib_y_center, 0.0))
    top = _box_at((0.305, rib_y, rib_t), (-hinge_clearance - door_w / 2.0, rib_y_center, 0.112))
    bottom = _box_at((0.305, rib_y, rib_t), (-hinge_clearance - door_w / 2.0, rib_y_center, -0.112))
    hinge_leaf = _box_at((0.015, 0.026, door_h), (-0.018, 0.0, 0.0))
    return main.union(left).union(right).union(top).union(bottom).union(hinge_leaf)


def _tray_pan() -> cq.Workplane:
    """Shallow document tray with lips, in a center local frame."""
    tray_w, tray_d = 0.34, 0.20
    base_t, lip_h, lip_t = 0.006, 0.025, 0.012
    base = _box_at((tray_w, tray_d, base_t), (0.0, 0.0, 0.0))
    side_z = (lip_h - base_t) / 2.0
    left = _box_at((lip_t, tray_d, lip_h), (-tray_w / 2.0 + lip_t / 2.0, 0.0, side_z))
    right = _box_at((lip_t, tray_d, lip_h), (tray_w / 2.0 - lip_t / 2.0, 0.0, side_z))
    rear = _box_at((tray_w, lip_t, lip_h), (0.0, tray_d / 2.0 - lip_t / 2.0, side_z))
    front = _box_at((tray_w, lip_t, lip_h), (0.0, -tray_d / 2.0 + lip_t / 2.0, side_z))
    return base.union(left).union(right).union(rear).union(front)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_wall_safe")

    steel = model.material("powder_coated_steel", rgba=(0.22, 0.23, 0.23, 1.0))
    dark = model.material("charcoal_door", rgba=(0.055, 0.060, 0.065, 1.0))
    inner = model.material("dark_interior", rgba=(0.10, 0.105, 0.11, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.70, 0.66, 1.0))
    black = model.material("black_knurled_plastic", rgba=(0.015, 0.015, 0.018, 1.0))
    white = model.material("engraved_white_marks", rgba=(0.90, 0.88, 0.80, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_safe_shell(), "safe_shell"), material=steel, name="safe_shell")
    body.visual(
        Box((0.425, 0.260, 0.012)),
        origin=Origin(xyz=(0.0, 0.145, -0.030)),
        material=inner,
        name="storage_shelf",
    )
    for idx, x in enumerate((-0.185, 0.185)):
        body.visual(
            Box((0.060, 0.240, 0.016)),
            origin=Origin(xyz=(x, 0.135, -0.096)),
            material=inner,
            name=f"runner_{idx}",
        )
    body.visual(
        Cylinder(radius=0.010, length=0.305),
        origin=Origin(xyz=(0.205, -0.050, 0.0)),
        material=chrome,
        name="hinge_pin",
    )
    for idx, z in enumerate((-0.055, 0.055)):
        body.visual(
            Box((0.040, 0.010, 0.024)),
            origin=Origin(xyz=(0.225, -0.040, z)),
            material=chrome,
            name=f"hinge_bracket_{idx}",
        )

    door = model.part("door")
    door.visual(mesh_from_cadquery(_door_panel(), "door_panel"), material=dark, name="door_panel")
    door.visual(
        Cylinder(radius=0.050, length=0.006),
        origin=Origin(xyz=(-0.213, -0.029, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dial_boss",
    )
    door.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(-0.213, -0.029, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_boss",
    )
    for name, z, length in (
        ("door_knuckle_bottom", -0.104, 0.052),
        ("door_knuckle_middle", 0.000, 0.060),
        ("door_knuckle_top", 0.104, 0.052),
    ):
        door.visual(
            Cylinder(radius=0.016, length=length),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=chrome,
            name=name,
        )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.205, -0.050, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dial_cap",
    )
    for i in range(12):
        angle = i * math.tau / 12.0
        r = 0.037
        tick_w = 0.004 if i % 3 else 0.006
        tick_h = 0.010 if i % 3 else 0.014
        dial.visual(
            Box((tick_w, 0.006, tick_h)),
            origin=Origin(xyz=(r * math.cos(angle), -0.0190, r * math.sin(angle))),
            material=white,
            name=f"dial_tick_{i}",
        )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.213, -0.032, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_hub",
    )
    handle.visual(
        Box((0.110, 0.022, 0.024)),
        origin=Origin(xyz=(0.052, -0.026, 0.0)),
        material=chrome,
        name="handle_spoke",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.110, -0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_tip",
    )
    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(-0.213, -0.032, -0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_tray_pan(), "document_tray"), material=steel, name="tray_pan")
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.135, -0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")
    door_hinge = object_model.get_articulation("door_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    handle_turn = object_model.get_articulation("handle_turn")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.check(
        "dial is continuous",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"dial joint type={dial_spin.articulation_type}",
    )
    ctx.check(
        "door uses vertical side hinge",
        door_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    for knuckle_name in ("door_knuckle_bottom", "door_knuckle_middle", "door_knuckle_top"):
        ctx.allow_overlap(
            body,
            door,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The fixed hinge pin is intentionally captured inside the door hinge knuckle proxy.",
        )
        ctx.expect_within(
            body,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.0005,
            name=f"hinge pin centered in {knuckle_name}",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.045,
            name=f"hinge pin spans {knuckle_name}",
        )

    ctx.expect_gap(
        door,
        dial,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem="dial_boss",
        negative_elem="dial_body",
        name="dial seats on door face",
    )
    ctx.expect_gap(
        door,
        handle,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem="handle_boss",
        negative_elem="handle_hub",
        name="handle hub seats on door face",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward from front flange",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.10,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    for runner_name in ("runner_0", "runner_1"):
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0001,
            positive_elem="tray_pan",
            negative_elem=runner_name,
            name=f"tray rests on {runner_name}",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.18,
            elem_a="tray_pan",
            elem_b=runner_name,
            name=f"tray retained on {runner_name}",
        )

    rest_tray = ctx.part_world_aabb(tray)
    with ctx.pose({tray_slide: 0.160}):
        extended_tray = ctx.part_world_aabb(tray)
        for runner_name in ("runner_0", "runner_1"):
            ctx.expect_overlap(
                tray,
                body,
                axes="y",
                min_overlap=0.055,
                elem_a="tray_pan",
                elem_b=runner_name,
                name=f"extended tray remains on {runner_name}",
            )
            ctx.expect_gap(
                tray,
                body,
                axis="z",
                max_gap=0.0015,
                max_penetration=0.0001,
                positive_elem="tray_pan",
                negative_elem=runner_name,
                name=f"extended tray stays on {runner_name}",
            )
    ctx.check(
        "tray slides out toward the user",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[0][1] < rest_tray[0][1] - 0.12,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    return ctx.report()


object_model = build_object_model()
