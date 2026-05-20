from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    gunmetal = model.material("gunmetal", rgba=(0.13, 0.15, 0.16, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.06, 0.07, 0.075, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black = model.material("black_dial", rgba=(0.015, 0.015, 0.018, 1.0))
    white = model.material("white_mark", rgba=(0.92, 0.90, 0.84, 1.0))
    felt = model.material("green_felt", rgba=(0.05, 0.28, 0.13, 1.0))
    tray_paint = model.material("tray_paint", rgba=(0.26, 0.25, 0.23, 1.0))
    wall_paint = model.material("painted_wall", rgba=(0.78, 0.74, 0.66, 1.0))

    body = model.part("body")
    # A shallow square steel box recessed into a surrounding wall patch.
    body.visual(
        Box((0.17, 0.024, 0.90)),
        origin=Origin(xyz=(-0.365, -0.150, 0.0)),
        material=wall_paint,
        name="wall_left",
    )
    body.visual(
        Box((0.17, 0.024, 0.90)),
        origin=Origin(xyz=(0.365, -0.150, 0.0)),
        material=wall_paint,
        name="wall_right",
    )
    body.visual(
        Box((0.56, 0.024, 0.17)),
        origin=Origin(xyz=(0.0, -0.150, 0.365)),
        material=wall_paint,
        name="wall_top",
    )
    body.visual(
        Box((0.56, 0.024, 0.17)),
        origin=Origin(xyz=(0.0, -0.150, -0.365)),
        material=wall_paint,
        name="wall_bottom",
    )

    body.visual(
        Box((0.040, 0.220, 0.440)),
        origin=Origin(xyz=(-0.220, 0.0, 0.0)),
        material=gunmetal,
        name="left_wall",
    )
    body.visual(
        Box((0.040, 0.220, 0.440)),
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        material=gunmetal,
        name="right_wall",
    )
    body.visual(
        Box((0.480, 0.220, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=gunmetal,
        name="top_wall",
    )
    body.visual(
        Box((0.480, 0.220, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=gunmetal,
        name="bottom_wall",
    )
    body.visual(
        Box((0.440, 0.018, 0.440)),
        origin=Origin(xyz=(0.0, 0.111, 0.0)),
        material=dark_steel,
        name="back_wall",
    )

    # Thick square front frame around the actual safe opening.
    body.visual(
        Box((0.080, 0.036, 0.520)),
        origin=Origin(xyz=(-0.240, -0.126, 0.0)),
        material=dark_steel,
        name="frame_left",
    )
    body.visual(
        Box((0.080, 0.036, 0.520)),
        origin=Origin(xyz=(0.240, -0.126, 0.0)),
        material=dark_steel,
        name="frame_right",
    )
    body.visual(
        Box((0.560, 0.036, 0.080)),
        origin=Origin(xyz=(0.0, -0.126, 0.240)),
        material=dark_steel,
        name="frame_top",
    )
    body.visual(
        Box((0.560, 0.036, 0.080)),
        origin=Origin(xyz=(0.0, -0.126, -0.240)),
        material=dark_steel,
        name="frame_bottom",
    )

    # Short tray runners fixed to the side walls; they do not reach the back.
    body.visual(
        Box((0.025, 0.120, 0.018)),
        origin=Origin(xyz=(-0.1885, -0.015, -0.115)),
        material=satin_steel,
        name="left_runner",
    )
    body.visual(
        Box((0.025, 0.120, 0.018)),
        origin=Origin(xyz=(0.1885, -0.015, -0.115)),
        material=satin_steel,
        name="right_runner",
    )

    # Exposed hinge knuckles fixed to the right frame.
    body.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.205, -0.165, -0.145)),
        material=satin_steel,
        name="body_hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.205, -0.165, 0.055)),
        material=satin_steel,
        name="body_hinge_barrel_1",
    )
    for zc, name in [(-0.145, "body_hinge_leaf_0"), (0.055, "body_hinge_leaf_1")]:
        body.visual(
            Box((0.050, 0.012, 0.055)),
            origin=Origin(xyz=(0.230, -0.147, zc)),
            material=satin_steel,
            name=name,
        )

    door = model.part("door")
    # The door frame is the vertical hinge line.  The slab extends leftward
    # into the square opening and is deliberately thick like a safe door.
    door.visual(
        Box((0.360, 0.050, 0.360)),
        origin=Origin(xyz=(-0.185, 0.035, 0.0)),
        material=dark_steel,
        name="door_slab",
    )
    door.visual(
        Box((0.285, 0.006, 0.285)),
        origin=Origin(xyz=(-0.185, 0.0075, 0.0)),
        material=gunmetal,
        name="recessed_face",
    )
    door.visual(
        Box((0.070, 0.004, 0.010)),
        origin=Origin(xyz=(-0.185, 0.003, 0.126)),
        material=white,
        name="dial_index",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=satin_steel,
        name="door_hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=satin_steel,
        name="door_hinge_barrel_1",
    )
    for zc, name in [(-0.055, "door_hinge_leaf_0"), (0.145, "door_hinge_leaf_1")]:
        door.visual(
            Box((0.046, 0.012, 0.055)),
            origin=Origin(xyz=(-0.018, 0.006, zc)),
            material=satin_steel,
            name=name,
        )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.090,
            0.020,
            body_style="cylindrical",
            edge_radius=0.0015,
            grip=KnobGrip(style="ribbed", count=48, depth=0.0010),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        ),
        "combination_dial",
    )
    dial.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="dial_shaft",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_cap",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="handle_hub",
    )
    for idx, angle in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        dx = math.cos(angle)
        dz = math.sin(angle)
        handle.visual(
            Box((0.075, 0.012, 0.012)),
            origin=Origin(
                xyz=(0.036 * dx, -0.029, 0.036 * dz),
                rpy=(0.0, -angle, 0.0),
            ),
            material=satin_steel,
            name=f"spoke_{idx}",
        )
        handle.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.073 * dx, -0.029, 0.073 * dz)),
            material=satin_steel,
            name=f"spoke_knob_{idx}",
        )

    tray = model.part("tray")
    tray.visual(
        Box((0.360, 0.130, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tray_paint,
        name="tray_floor",
    )
    tray.visual(
        Box((0.012, 0.130, 0.040)),
        origin=Origin(xyz=(-0.174, 0.0, 0.014)),
        material=tray_paint,
        name="left_lip",
    )
    tray.visual(
        Box((0.012, 0.130, 0.040)),
        origin=Origin(xyz=(0.174, 0.0, 0.014)),
        material=tray_paint,
        name="right_lip",
    )
    tray.visual(
        Box((0.360, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, -0.059, 0.014)),
        material=tray_paint,
        name="front_lip",
    )
    tray.visual(
        Box((0.360, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, 0.059, 0.014)),
        material=tray_paint,
        name="rear_lip",
    )
    tray.visual(
        Box((0.305, 0.080, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=felt,
        name="felt_pad",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.205, -0.165, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.185, 0.0045, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=6.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(-0.185, 0.0045, -0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.015, -0.100)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.25, lower=0.0, upper=0.100),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")
    door_hinge = object_model.get_articulation("body_to_door")
    tray_slide = object_model.get_articulation("body_to_tray")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_turn = object_model.get_articulation("door_to_handle")

    ctx.check(
        "dial is continuous",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_spin.articulation_type}",
    )
    ctx.check(
        "handle has limited rotation",
        handle_turn.motion_limits is not None
        and handle_turn.motion_limits.lower < 0.0
        and handle_turn.motion_limits.upper > 0.5,
        details=f"limits={handle_turn.motion_limits}",
    )

    ctx.expect_contact(
        tray,
        body,
        elem_a="tray_floor",
        elem_b="left_runner",
        name="tray rests on left runner",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="tray_floor",
        elem_b="right_runner",
        name="tray rests on right runner",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        elem_a="tray_floor",
        elem_b="left_runner",
        min_overlap=0.10,
        name="tray retained on runner at rest",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward from right hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.08,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.100}):
        extended_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            elem_a="tray_floor",
            elem_b="left_runner",
            min_overlap=0.020,
            name="extended tray remains on runner",
        )
    ctx.check(
        "tray slides outward",
        rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.08,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_shaft",
        elem_b="recessed_face",
        name="dial shaft seats on door face",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_hub",
        elem_b="recessed_face",
        name="handle hub seats on door face",
    )

    return ctx.report()


object_model = build_object_model()
