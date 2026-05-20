from __future__ import annotations

from math import cos, pi, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    wall = model.material("painted_wall", color=(0.78, 0.75, 0.68, 1.0))
    frame_steel = model.material("dark_blued_steel", color=(0.08, 0.095, 0.105, 1.0))
    shadow = model.material("shadowed_interior", color=(0.015, 0.017, 0.020, 1.0))
    door_steel = model.material("thick_door_steel", color=(0.18, 0.205, 0.225, 1.0))
    face_steel = model.material("brushed_face_steel", color=(0.26, 0.29, 0.31, 1.0))
    black = model.material("matte_black_control", color=(0.005, 0.005, 0.006, 1.0))
    brass = model.material("aged_brass", color=(0.72, 0.55, 0.28, 1.0))

    body = model.part("safe_body")

    # Four wall pieces leave a visible square recess instead of a solid backplate
    # over the safe.  They overlap the steel frame slightly so the root reads as
    # one installed assembly rather than floating trim.
    body.visual(
        Box((0.020, 0.740, 0.110)),
        origin=Origin(xyz=(-0.016, 0.0, 0.297)),
        material=wall,
        name="wall_top",
    )
    body.visual(
        Box((0.020, 0.740, 0.110)),
        origin=Origin(xyz=(-0.016, 0.0, -0.297)),
        material=wall,
        name="wall_bottom",
    )
    body.visual(
        Box((0.020, 0.110, 0.740)),
        origin=Origin(xyz=(-0.016, 0.297, 0.0)),
        material=wall,
        name="wall_side_0",
    )
    body.visual(
        Box((0.020, 0.110, 0.740)),
        origin=Origin(xyz=(-0.016, -0.297, 0.0)),
        material=wall,
        name="wall_side_1",
    )

    # Recessed steel safe box: a dark rear plate and four deep side walls that
    # make the safe read as installed into the wall cavity.
    body.visual(
        Box((0.014, 0.390, 0.390)),
        origin=Origin(xyz=(-0.187, 0.0, 0.0)),
        material=shadow,
        name="back_plate",
    )
    body.visual(
        Box((0.185, 0.390, 0.018)),
        origin=Origin(xyz=(-0.0925, 0.0, 0.204)),
        material=frame_steel,
        name="recess_top",
    )
    body.visual(
        Box((0.185, 0.390, 0.018)),
        origin=Origin(xyz=(-0.0925, 0.0, -0.204)),
        material=frame_steel,
        name="recess_bottom",
    )
    body.visual(
        Box((0.185, 0.018, 0.390)),
        origin=Origin(xyz=(-0.0925, 0.204, 0.0)),
        material=frame_steel,
        name="recess_side_0",
    )
    body.visual(
        Box((0.185, 0.018, 0.390)),
        origin=Origin(xyz=(-0.0925, -0.204, 0.0)),
        material=frame_steel,
        name="recess_side_1",
    )

    # Heavy square face frame around the door opening.
    body.visual(
        Box((0.022, 0.490, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=frame_steel,
        name="front_ring_top",
    )
    body.visual(
        Box((0.022, 0.490, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=frame_steel,
        name="front_ring_bottom",
    )
    body.visual(
        Box((0.022, 0.050, 0.490)),
        origin=Origin(xyz=(0.0, 0.220, 0.0)),
        material=frame_steel,
        name="front_ring_side_0",
    )
    body.visual(
        Box((0.022, 0.050, 0.490)),
        origin=Origin(xyz=(0.0, -0.220, 0.0)),
        material=frame_steel,
        name="front_ring_side_1",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.380),
        origin=Origin(xyz=(0.026, 0.202, 0.0)),
        material=black,
        name="frame_hinge_pin",
    )
    body.visual(
        Box((0.035, 0.008, 0.370)),
        origin=Origin(xyz=(0.0205, 0.199, 0.0)),
        material=black,
        name="frame_hinge_leaf",
    )

    door = model.part("door")
    door.visual(
        Box((0.055, 0.360, 0.360)),
        origin=Origin(xyz=(0.0275, -0.180, 0.0)),
        material=door_steel,
        name="door_slab",
    )
    door.visual(
        Box((0.008, 0.300, 0.300)),
        origin=Origin(xyz=(0.059, -0.180, 0.0)),
        material=face_steel,
        name="face_panel",
    )
    # Narrow dark grooves on the raised face panel emphasize the deep, separate
    # door slab without needing a decorative texture.
    door.visual(
        Box((0.003, 0.302, 0.006)),
        origin=Origin(xyz=(0.064, -0.180, 0.153)),
        material=black,
        name="face_groove_top",
    )
    door.visual(
        Box((0.003, 0.302, 0.006)),
        origin=Origin(xyz=(0.064, -0.180, -0.153)),
        material=black,
        name="face_groove_bottom",
    )
    door.visual(
        Box((0.003, 0.006, 0.302)),
        origin=Origin(xyz=(0.064, -0.028, 0.0)),
        material=black,
        name="face_groove_hinge",
    )
    door.visual(
        Box((0.003, 0.006, 0.302)),
        origin=Origin(xyz=(0.064, -0.332, 0.0)),
        material=black,
        name="face_groove_free",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.340),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=black,
        name="door_hinge_barrel",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        # The door's local -Y direction spans across the opening from the
        # right-hand hinge line; +Z rotation swings that free edge outward.
        origin=Origin(xyz=(0.012, 0.185, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    dial = model.part("dial")
    dial_cap = mesh_from_geometry(
        KnobGeometry(
            0.086,
            0.024,
            body_style="faceted",
            base_diameter=0.090,
            top_diameter=0.076,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=32, depth=0.0010),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "combination_dial_cap",
    )
    dial.visual(
        dial_cap,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="dial_center",
    )

    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.063, -0.180, 0.068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.025, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="handle_hub",
    )
    spoke_length = 0.070
    for idx, angle_deg in enumerate((-90.0, 30.0, 150.0)):
        angle = radians(angle_deg)
        dy = cos(angle)
        dz = sin(angle)
        handle.visual(
            Box((0.012, spoke_length, 0.012)),
            origin=Origin(
                xyz=(0.020, 0.5 * spoke_length * dy, 0.5 * spoke_length * dz),
                rpy=(angle, 0.0, 0.0),
            ),
            material=black,
            name=f"spoke_{idx}",
        )
        handle.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.020, spoke_length * dy, spoke_length * dz)),
            material=black,
            name=f"spoke_tip_{idx}",
        )

    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.063, -0.180, -0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-1.57, upper=1.57),
    )

    key_cover = model.part("key_cover")
    key_cover.visual(
        Box((0.006, 0.045, 0.060)),
        origin=Origin(xyz=(0.003, -0.0225, 0.0)),
        material=face_steel,
        name="cover_flap",
    )
    key_cover.visual(
        Cylinder(radius=0.004, length=0.066),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=black,
        name="cover_hinge_barrel",
    )
    key_cover.visual(
        Box((0.003, 0.016, 0.007)),
        origin=Origin(xyz=(0.007, -0.0225, -0.021)),
        material=black,
        name="cover_lip",
    )

    model.articulation(
        "door_to_key_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=key_cover,
        # A small vertical hinge lets the flap swing away from the door face.
        origin=Origin(xyz=(0.063, -0.080, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    key_cover = object_model.get_part("key_cover")
    door_hinge = object_model.get_articulation("body_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_turn = object_model.get_articulation("door_to_handle")
    cover_hinge = object_model.get_articulation("door_to_key_cover")

    ctx.expect_gap(
        door,
        body,
        axis="x",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="door_slab",
        negative_elem="front_ring_top",
        name="door slab stands proud of frame ring",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="door_hinge_barrel",
        elem_b="frame_hinge_leaf",
        contact_tol=0.0005,
        name="door hinge barrel is carried by the frame hinge leaf",
    )
    ctx.expect_within(
        door,
        body,
        axes="yz",
        margin=0.002,
        inner_elem="door_slab",
        outer_elem="back_plate",
        name="door fits within square recessed safe body",
    )
    ctx.expect_gap(
        dial,
        door,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="dial_cap",
        negative_elem="face_panel",
        name="dial mounts on the door face",
    )
    ctx.expect_gap(
        handle,
        door,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="handle_hub",
        negative_elem="face_panel",
        name="handle hub mounts on the door face",
    )
    ctx.expect_gap(
        key_cover,
        door,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="cover_flap",
        negative_elem="face_panel",
        name="key cover flap sits on the door face",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.0, dial_spin: pi, handle_turn: 1.0, cover_hinge: 1.0}):
        opened_aabb = ctx.part_world_aabb(door)
        opened_cover = ctx.part_world_aabb(key_cover)

    ctx.check(
        "right-hinged door opens outward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][0] > closed_aabb[1][0] + 0.035,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )
    ctx.check(
        "key flap swings outward on its hinge",
        opened_cover is not None
        and closed_aabb is not None
        and opened_cover[1][0] > closed_aabb[1][0] + 0.015,
        details=f"door_closed={closed_aabb}, cover_opened={opened_cover}",
    )

    return ctx.report()


object_model = build_object_model()
