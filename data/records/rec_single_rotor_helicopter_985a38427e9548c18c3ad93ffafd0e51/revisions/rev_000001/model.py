from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_SECTIONS = [
    (-2.35, 0.12, 0.18, 0.06),
    (-1.85, 0.96, 1.04, 0.14),
    (-0.95, 1.72, 1.42, 0.08),
    (0.00, 1.84, 1.52, 0.03),
    (0.95, 1.72, 1.40, 0.00),
    (1.55, 1.28, 1.00, -0.04),
    (2.10, 0.72, 0.64, 0.04),
    (3.45, 0.40, 0.40, 0.13),
    (4.65, 0.28, 0.30, 0.20),
]
INTERIOR_SECTIONS = [
    (-1.55, 1.18, 0.96, -0.02),
    (-0.60, 1.36, 1.10, -0.03),
    (0.70, 1.32, 1.04, -0.06),
    (1.45, 0.86, 0.64, -0.10),
]

DOOR_WIDTH = 0.90
DOOR_HEIGHT = 1.10
DOOR_THICKNESS = 0.045
DOOR_CENTER_X = -0.24
DOOR_CENTER_Z = -0.04
DOOR_HINGE_Y = 0.9195

HATCH_LENGTH = 0.66
HATCH_WIDTH = 0.44
HATCH_THICKNESS = 0.028
HATCH_CENTER_X = 1.10
HATCH_HINGE_Z = 0.80

MAIN_ROTOR_MAST_Z = 1.10
TAIL_ROTOR_CENTER = (4.93, 0.47, 0.52)


def _loft_ellipse_sections(sections: list[tuple[float, float, float, float]]) -> cq.Workplane:
    x0, width0, height0, z0 = sections[0]
    loft = cq.Workplane("YZ").workplane(offset=x0).center(0.0, z0).ellipse(width0 * 0.5, height0 * 0.5)
    last_x = x0
    for x_pos, width, height, z_center in sections[1:]:
        loft = loft.workplane(offset=x_pos - last_x).center(0.0, z_center).ellipse(width * 0.5, height * 0.5)
        last_x = x_pos
    return loft.loft(combine=True)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_fuselage_shell() -> cq.Workplane:
    fuselage = _loft_ellipse_sections(BODY_SECTIONS)

    engine_deck = _box((1.05, 0.76, 0.30), (0.55, 0.00, 0.69)).edges("|Z").fillet(0.08)
    fin = (
        cq.Workplane("XZ")
        .polyline([(3.42, 0.18), (4.18, 0.95), (4.84, 0.76), (4.60, 0.34), (3.82, 0.22)])
        .close()
        .extrude(0.08)
        .translate((0.0, -0.04, 0.0))
    )
    rotor_mount = _box((0.34, 0.32, 0.18), (4.82, 0.16, 0.52)).edges("|Y").fillet(0.02)

    fuselage = fuselage.union(engine_deck).union(fin).union(rotor_mount)

    for side in (-1.0, 1.0):
        stabilizer = (
            _box((0.82, 0.60, 0.03), (3.60, side * 0.30, 0.23))
            .rotate((3.60, 0.0, 0.23), (3.60, 0.0, 1.23), -11.0 if side > 0.0 else 11.0)
        )
        fuselage = fuselage.union(stabilizer)

    cabin_void = _loft_ellipse_sections(INTERIOR_SECTIONS)
    fuselage = fuselage.cut(cabin_void)

    door_cut = (
        cq.Workplane("XZ")
        .workplane(offset=0.96)
        .center(DOOR_CENTER_X, DOOR_CENTER_Z)
        .rect(DOOR_WIDTH + 0.04, DOOR_HEIGHT + 0.04)
        .extrude(-0.72)
    )
    hatch_cut = (
        cq.Workplane("XY")
        .workplane(offset=1.00)
        .center(HATCH_CENTER_X, 0.0)
        .rect(HATCH_LENGTH + 0.04, HATCH_WIDTH + 0.04)
        .extrude(-0.42)
    )
    cockpit_window_cut = (
        cq.Workplane("XZ")
        .polyline([(-1.96, 0.10), (-1.62, 0.56), (-1.02, 0.78), (-0.66, 0.60), (-0.78, 0.18), (-1.44, 0.06)])
        .close()
        .extrude(0.92)
        .translate((0.0, -0.46, 0.0))
    )
    fuselage = fuselage.cut(door_cut).cut(hatch_cut).cut(cockpit_window_cut)

    gear_posts: cq.Workplane | None = None
    for side in (-1.0, 1.0):
        for x_pos, z_center, height in ((-0.68, -0.50, 0.56), (0.86, -0.54, 0.52)):
            post = _box((0.10, 0.10, height), (x_pos, side * 0.80, z_center)).edges("|Z").fillet(0.015)
            fairing = _box((0.34, 0.28, 0.20), (x_pos, side * 0.56, -0.26)).edges("|Z").fillet(0.03)
            brace = _box((0.14, 0.12, 0.24), (x_pos, side * 0.74, -0.48)).edges("|Z").fillet(0.02)
            if gear_posts is None:
                gear_posts = post.union(fairing).union(brace)
            else:
                gear_posts = gear_posts.union(post).union(fairing).union(brace)

    return fuselage.union(gear_posts)


def _build_landing_wheels() -> cq.Workplane:
    wheels: cq.Workplane | None = None
    for side in (-1.0, 1.0):
        for x_pos, z_pos in ((-0.68, -0.80), (0.86, -0.80)):
            wheel = (
                cq.Workplane("XY")
                .circle(0.18)
                .extrude(0.10)
                .translate((0.0, 0.0, -0.05))
                .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
                .translate((x_pos, side * 0.80, z_pos))
            )
            wheels = wheel if wheels is None else wheels.union(wheel)
    return wheels


def _build_main_rotor_hub() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.12).extrude(0.12).translate((0.0, 0.0, -0.04))
    mast = cq.Workplane("XY").circle(0.038).extrude(0.26).translate((0.0, 0.0, -0.26))
    cap = cq.Workplane("XY").circle(0.07).extrude(0.05).translate((0.0, 0.0, 0.08))
    return hub.union(mast).union(cap)


def _build_main_rotor_blades() -> cq.Workplane:
    blade_profile = [
        (0.18, -0.13),
        (0.70, -0.12),
        (2.30, -0.09),
        (3.45, -0.05),
        (3.72, 0.00),
        (3.45, 0.05),
        (2.30, 0.09),
        (0.70, 0.12),
        (0.18, 0.13),
    ]
    blade = cq.Workplane("XY").polyline(blade_profile).close().extrude(0.028).translate((0.0, 0.0, -0.014))
    cuff = _box((0.32, 0.22, 0.05), (0.16, 0.0, 0.0)).edges("|Z").fillet(0.03)
    blade = blade.union(cuff)

    blades: cq.Workplane | None = None
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        rotated = blade.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        blades = rotated if blades is None else blades.union(rotated)
    return blades


def _build_tail_rotor_hub() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.075).extrude(0.10).translate((0.0, 0.0, -0.05))
    shaft = cq.Workplane("XY").circle(0.026).extrude(0.10).translate((0.0, 0.0, -0.15))
    return hub.union(shaft).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)


def _build_tail_rotor_blades() -> cq.Workplane:
    blade_profile = [
        (0.02, -0.06),
        (0.20, -0.055),
        (0.46, -0.035),
        (0.62, 0.00),
        (0.46, 0.035),
        (0.20, 0.055),
        (0.02, 0.06),
    ]
    blade = cq.Workplane("XY").polyline(blade_profile).close().extrude(0.018).translate((0.0, 0.0, -0.009))
    cuff = _box((0.18, 0.038, 0.15), (0.09, 0.0, 0.0))
    blade = blade.union(cuff)

    blades: cq.Workplane | None = None
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        rotated = blade.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        blades = rotated if blades is None else blades.union(rotated)
    return blades.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)


def _build_passenger_door() -> cq.Workplane:
    panel = _box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT), (DOOR_WIDTH * 0.5, DOOR_THICKNESS * 0.5, 0.0))
    panel = panel.edges("|Z").fillet(0.022)
    window_cut = _box((0.38, DOOR_THICKNESS + 0.03, 0.34), (0.50, DOOR_THICKNESS * 0.5, 0.22))
    panel = panel.cut(window_cut)
    handle = _box((0.06, 0.016, 0.22), (0.68, DOOR_THICKNESS + 0.006, 0.0)).edges("|Z").fillet(0.007)

    for z_pos in (-0.34, 0.00, 0.34):
        barrel = cq.Workplane("XY").circle(0.015).extrude(0.16).translate((0.02, DOOR_THICKNESS * 0.5, z_pos - 0.08))
        panel = panel.union(barrel)

    return panel.union(handle)


def _build_baggage_hatch_panel() -> cq.Workplane:
    return _box((HATCH_LENGTH, HATCH_WIDTH, HATCH_THICKNESS), (HATCH_LENGTH * 0.5, 0.0, HATCH_THICKNESS * 0.5)).edges(
        "|Z"
    ).fillet(0.018)


def _build_baggage_hatch_hinges() -> cq.Workplane:
    hinges: cq.Workplane | None = None
    for y_pos in (-0.12, 0.12):
        hinge_block = _box((0.08, 0.07, 0.04), (-0.02, y_pos, 0.0)).edges("|Z").fillet(0.008)
        hinges = hinge_block if hinges is None else hinges.union(hinge_block)
    return hinges


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corporate_helicopter")

    fuselage_paint = model.material("fuselage_paint", rgba=(0.88, 0.90, 0.94, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.35, 0.38, 0.42, 1.0))
    rotor_dark = model.material("rotor_dark", rgba=(0.10, 0.11, 0.13, 1.0))
    tire_black = model.material("tire_black", rgba=(0.06, 0.06, 0.07, 1.0))

    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_cadquery(_build_fuselage_shell(), "fuselage_shell"),
        material=fuselage_paint,
        name="body_shell",
    )
    fuselage.visual(
        mesh_from_cadquery(_build_landing_wheels(), "landing_wheels"),
        material=tire_black,
        name="landing_wheels",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        mesh_from_cadquery(_build_main_rotor_hub(), "main_rotor_hub"),
        material=trim_gray,
        name="rotor_hub",
    )
    main_rotor.visual(
        mesh_from_cadquery(_build_main_rotor_blades(), "main_rotor_blades"),
        material=rotor_dark,
        name="rotor_blades",
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        mesh_from_cadquery(_build_tail_rotor_hub(), "tail_rotor_hub"),
        material=trim_gray,
        name="tail_rotor_hub",
    )
    tail_rotor.visual(
        mesh_from_cadquery(_build_tail_rotor_blades(), "tail_rotor_blades"),
        material=rotor_dark,
        name="tail_rotor_blades",
    )

    passenger_door = model.part("passenger_door")
    passenger_door.visual(
        mesh_from_cadquery(_build_passenger_door(), "passenger_door"),
        material=fuselage_paint,
        name="door_panel",
    )

    baggage_hatch = model.part("baggage_hatch")
    baggage_hatch.visual(
        mesh_from_cadquery(_build_baggage_hatch_panel(), "baggage_hatch_panel"),
        material=fuselage_paint,
        name="hatch_panel",
    )
    baggage_hatch.visual(
        mesh_from_cadquery(_build_baggage_hatch_hinges(), "baggage_hatch_hinges"),
        material=trim_gray,
        name="hatch_hinges",
    )

    model.articulation(
        "fuselage_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.0, MAIN_ROTOR_MAST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=40.0),
    )
    model.articulation(
        "fuselage_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=TAIL_ROTOR_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=55.0),
    )
    model.articulation(
        "fuselage_to_baggage_hatch",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=baggage_hatch,
        origin=Origin(xyz=(HATCH_CENTER_X - HATCH_LENGTH * 0.5, 0.0, HATCH_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=25.0, velocity=1.2),
    )
    model.articulation(
        "fuselage_to_passenger_door",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=passenger_door,
        origin=Origin(xyz=(DOOR_CENTER_X - DOOR_WIDTH * 0.5, DOOR_HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=20.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    passenger_door = object_model.get_part("passenger_door")
    baggage_hatch = object_model.get_part("baggage_hatch")

    main_joint = object_model.get_articulation("fuselage_to_main_rotor")
    tail_joint = object_model.get_articulation("fuselage_to_tail_rotor")
    door_joint = object_model.get_articulation("fuselage_to_passenger_door")
    hatch_joint = object_model.get_articulation("fuselage_to_baggage_hatch")

    ctx.allow_overlap(
        fuselage,
        main_rotor,
        elem_a="body_shell",
        elem_b="rotor_hub",
        reason="The mast is intentionally shown entering the roof gearbox fairing instead of modeling the internal mast socket as a separate cavity.",
    )
    ctx.allow_overlap(
        fuselage,
        tail_rotor,
        elem_a="body_shell",
        elem_b="tail_rotor_hub",
        reason="The tail rotor gearbox shaft is intentionally represented as seated inside the tail pylon housing rather than exposing a separate socket cavity.",
    )
    ctx.allow_overlap(
        fuselage,
        baggage_hatch,
        elem_a="body_shell",
        elem_b="hatch_hinges",
        reason="The hinge knuckles are intentionally represented as embedded into the roof hinge pocket instead of modeling the hidden socket geometry separately.",
    )

    ctx.check(
        "main rotor uses continuous mast joint",
        main_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(main_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_joint.articulation_type!r}, axis={main_joint.axis!r}",
    )
    ctx.check(
        "tail rotor uses continuous transverse joint",
        tail_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(tail_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={tail_joint.articulation_type!r}, axis={tail_joint.axis!r}",
    )
    ctx.check(
        "door has outward side hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_joint.axis) == (0.0, 0.0, 1.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper >= 1.3,
        details=f"type={door_joint.articulation_type!r}, axis={door_joint.axis!r}, limits={door_joint.motion_limits!r}",
    )
    ctx.check(
        "baggage hatch has top hinge",
        hatch_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(hatch_joint.axis) == (0.0, -1.0, 0.0)
        and hatch_joint.motion_limits is not None
        and hatch_joint.motion_limits.upper is not None
        and hatch_joint.motion_limits.upper >= 1.2,
        details=f"type={hatch_joint.articulation_type!r}, axis={hatch_joint.axis!r}, limits={hatch_joint.motion_limits!r}",
    )

    ctx.expect_overlap(
        passenger_door,
        fuselage,
        axes="xz",
        elem_a="door_panel",
        elem_b="body_shell",
        min_overlap=0.80,
        name="door covers the side cabin opening",
    )
    ctx.expect_overlap(
        baggage_hatch,
        fuselage,
        axes="xy",
        elem_a="hatch_panel",
        elem_b="body_shell",
        min_overlap=0.35,
        name="hatch covers the aft baggage opening",
    )
    ctx.expect_overlap(
        main_rotor,
        fuselage,
        axes="xy",
        elem_a="rotor_hub",
        elem_b="body_shell",
        min_overlap=0.10,
        name="main rotor hub is centered over the mast fairing",
    )
    ctx.expect_overlap(
        tail_rotor,
        fuselage,
        axes="xz",
        elem_a="tail_rotor_hub",
        elem_b="body_shell",
        min_overlap=0.08,
        name="tail rotor hub sits on the tail pylon",
    )

    main_pos = ctx.part_world_position(main_rotor)
    tail_pos = ctx.part_world_position(tail_rotor)
    ctx.check(
        "tail rotor is aft on the starboard fin",
        main_pos is not None
        and tail_pos is not None
        and tail_pos[0] > main_pos[0] + 4.0
        and tail_pos[1] > 0.10
        and tail_pos[2] > 0.30,
        details=f"main={main_pos!r}, tail={tail_pos!r}",
    )

    door_closed = ctx.part_element_world_aabb(passenger_door, elem="door_panel")
    with ctx.pose({door_joint: 1.20}):
        door_open = ctx.part_element_world_aabb(passenger_door, elem="door_panel")
    ctx.check(
        "door swings outward from the cabin side",
        door_closed is not None
        and door_open is not None
        and door_open[1][1] > door_closed[1][1] + 0.40,
        details=f"closed={door_closed!r}, open={door_open!r}",
    )

    hatch_closed = ctx.part_element_world_aabb(baggage_hatch, elem="hatch_panel")
    with ctx.pose({hatch_joint: 1.05}):
        hatch_open = ctx.part_element_world_aabb(baggage_hatch, elem="hatch_panel")
    ctx.check(
        "baggage hatch lifts upward",
        hatch_closed is not None
        and hatch_open is not None
        and hatch_open[1][2] > hatch_closed[1][2] + 0.24,
        details=f"closed={hatch_closed!r}, open={hatch_open!r}",
    )

    return ctx.report()


object_model = build_object_model()
