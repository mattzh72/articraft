from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_wall_safe")

    satin_steel = model.material("satin_steel", rgba=(0.55, 0.57, 0.57, 1.0))
    darker_steel = model.material("darker_steel", rgba=(0.34, 0.36, 0.37, 1.0))
    dark_cavity = model.material("shadowed_interior", rgba=(0.05, 0.055, 0.06, 1.0))
    black = model.material("black_dial", rgba=(0.02, 0.021, 0.023, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.78, 0.76, 0.70, 1.0))
    slot_dark = model.material("slot_black", rgba=(0.0, 0.0, 0.0, 1.0))

    body = model.part("body")
    flange_mesh = mesh_from_geometry(
        BezelGeometry(
            opening_size=(0.410, 0.285),
            outer_size=(0.560, 0.390),
            depth=0.028,
            opening_shape="rounded_rect",
            outer_shape="rounded_rect",
            opening_corner_radius=0.012,
            outer_corner_radius=0.024,
        ),
        "broad_front_flange",
    )
    body.visual(
        flange_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="front_flange",
    )
    # Four simple in-wall sleeve walls and a dark rear plate make the safe read
    # as a hollow wall-mounted box instead of a flat decorative frame.
    body.visual(
        Box((0.026, 0.286, 0.300)),
        origin=Origin(xyz=(-0.218, 0.157, 0.0)),
        material=darker_steel,
        name="left_sleeve",
    )
    body.visual(
        Box((0.026, 0.286, 0.300)),
        origin=Origin(xyz=(0.218, 0.157, 0.0)),
        material=darker_steel,
        name="right_sleeve",
    )
    body.visual(
        Box((0.410, 0.286, 0.024)),
        origin=Origin(xyz=(0.0, 0.157, 0.155)),
        material=darker_steel,
        name="top_sleeve",
    )
    body.visual(
        Box((0.410, 0.286, 0.024)),
        origin=Origin(xyz=(0.0, 0.157, -0.155)),
        material=darker_steel,
        name="bottom_sleeve",
    )
    body.visual(
        Box((0.438, 0.018, 0.324)),
        origin=Origin(xyz=(0.0, 0.309, 0.0)),
        material=dark_cavity,
        name="rear_wall",
    )
    body.visual(
        Box((0.052, 0.011, 0.270)),
        origin=Origin(xyz=(0.199, -0.0195, 0.0)),
        material=satin_steel,
        name="hinge_leaf",
    )

    door = model.part("door")
    door_mesh = mesh_from_cadquery(
        cq.Workplane("XY")
        .box(0.340, 0.026, 0.230)
        .edges("|Z")
        .fillet(0.006),
        "compact_door_slab",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    door.visual(
        door_mesh,
        origin=Origin(xyz=(-0.170, 0.0, 0.0)),
        material=darker_steel,
        name="door_slab",
    )
    # A pressed perimeter lip on the slab emphasizes that the door is a separate
    # plate sitting inside the broad flange.
    door.visual(
        Box((0.300, 0.003, 0.010)),
        origin=Origin(xyz=(-0.170, -0.0145, 0.098)),
        material=satin_steel,
        name="top_door_lip",
    )
    door.visual(
        Box((0.300, 0.003, 0.010)),
        origin=Origin(xyz=(-0.170, -0.0145, -0.098)),
        material=satin_steel,
        name="bottom_door_lip",
    )
    door.visual(
        Box((0.010, 0.003, 0.188)),
        origin=Origin(xyz=(-0.318, -0.0145, 0.0)),
        material=satin_steel,
        name="free_edge_lip",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.255),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.170, -0.033, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.074,
                0.018,
                body_style="cylindrical",
                edge_radius=0.0012,
                grip=KnobGrip(style="fluted", count=48, depth=0.0013),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "combination_dial",
        ),
        material=black,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.039, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00125)),
        material=chrome,
        name="dial_back_ring",
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.170, -0.013, 0.044), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=chrome,
        name="front_hub",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        handle.visual(
            Box((0.076, 0.014, 0.012)),
            origin=Origin(
                xyz=(0.030 * math.cos(angle), 0.030 * math.sin(angle), 0.011),
                rpy=(0.0, 0.0, angle),
            ),
            material=chrome,
            name=f"spoke_{i}",
        )
        handle.visual(
            Sphere(radius=0.010),
            origin=Origin(
                xyz=(0.061 * math.cos(angle), 0.061 * math.sin(angle), 0.011)
            ),
            material=chrome,
            name=f"spoke_tip_{i}",
        )
    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(-0.075, -0.013, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-1.2, upper=1.2),
    )

    key_cover = model.part("key_cover")
    key_cover.visual(
        Box((0.088, 0.006, 0.045)),
        origin=Origin(xyz=(0.0, -0.0075, -0.0225)),
        material=satin_steel,
        name="cover_plate",
    )
    key_cover.visual(
        Cylinder(radius=0.0045, length=0.098),
        origin=Origin(xyz=(0.0, -0.0045, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="cover_hinge_barrel",
    )
    key_cover.visual(
        Box((0.030, 0.0015, 0.004)),
        origin=Origin(xyz=(0.0, -0.0102, -0.023)),
        material=slot_dark,
        name="key_slot_mark",
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=key_cover,
        origin=Origin(xyz=(-0.170, -0.013, -0.034)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.5, lower=0.0, upper=1.15),
    )

    # Keep references alive for simple static linters and to make the intended
    # primary hinge explicit in the model metadata.
    model.meta["primary_door_hinge"] = door_hinge.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    key_cover = object_model.get_part("key_cover")
    door_hinge = object_model.get_articulation("door_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    handle_turn = object_model.get_articulation("handle_turn")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.check(
        "door has bounded vertical hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.0,
    )
    ctx.check(
        "dial is continuous",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.check(
        "handle and cover are hinged controls",
        handle_turn.articulation_type == ArticulationType.REVOLUTE
        and cover_hinge.articulation_type == ArticulationType.REVOLUTE,
    )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_flange",
        negative_elem="door_slab",
        min_gap=0.002,
        max_gap=0.010,
        name="closed door stands proud of front flange",
    )
    ctx.expect_gap(
        door,
        dial,
        axis="y",
        positive_elem="door_slab",
        negative_elem="dial_back_ring",
        min_gap=-0.0005,
        max_gap=0.002,
        name="dial is seated on door face",
    )
    ctx.expect_gap(
        door,
        key_cover,
        axis="y",
        positive_elem="door_slab",
        negative_elem="cover_hinge_barrel",
        min_gap=0.000,
        max_gap=0.002,
        name="key cover sits separately below dial",
    )

    rest_door = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({door_hinge: 1.0}):
        open_door = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "door swings outward from right hinge",
        rest_door is not None
        and open_door is not None
        and open_door[0][1] < rest_door[0][1] - 0.060,
        details=f"closed={rest_door}, open={open_door}",
    )

    rest_cover = ctx.part_element_world_aabb(key_cover, elem="cover_plate")
    with ctx.pose({cover_hinge: 0.9}):
        open_cover = ctx.part_element_world_aabb(key_cover, elem="cover_plate")
    ctx.check(
        "key cover flips outward on small hinge",
        rest_cover is not None
        and open_cover is not None
        and open_cover[0][1] < rest_cover[0][1] - 0.020,
        details=f"closed={rest_cover}, open={open_cover}",
    )

    return ctx.report()


object_model = build_object_model()
