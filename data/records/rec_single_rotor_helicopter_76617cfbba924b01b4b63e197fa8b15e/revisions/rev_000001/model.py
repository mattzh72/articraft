from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_LENGTH = 5.35
BODY_WIDTH = 1.62
BODY_HEIGHT = 2.18

DOOR_THICKNESS = 0.03
DOOR_HINGE_X = 0.34
DOOR_HINGE_Y = 0.66
DOOR_HINGE_Z = 1.24

HATCH_THICKNESS = 0.028
HATCH_HINGE_X = -1.78
HATCH_HINGE_Y = 0.45
HATCH_HINGE_Z = 1.17

MAIN_MAST_Z = 2.14
TAIL_ROTOR_X = -5.24
TAIL_ROTOR_Z = 1.24


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name)


def _body_shell_shape() -> cq.Workplane:
    fuselage = (
        cq.Workplane("XZ")
        .polyline(
            [
                (1.42, 1.00),
                (1.16, 0.72),
                (0.62, 0.45),
                (-0.18, 0.40),
                (-0.96, 0.45),
                (-1.72, 0.82),
                (-1.78, 1.06),
                (-1.18, 1.13),
                (0.10, 1.15),
                (0.92, 1.10),
            ]
        )
        .close()
        .extrude(0.68, both=True)
    )
    fuselage = fuselage.cut(cq.Workplane("XY").box(3.10, 1.70, 0.96).translate((0.00, 0.00, 1.58)))
    fuselage = fuselage.cut(cq.Workplane("XY").box(1.14, 1.70, 0.90).translate((-0.10, 0.00, 1.22)))
    fuselage = fuselage.cut(cq.Workplane("XY").box(0.64, 0.24, 0.46).translate((-1.46, 0.46, 1.16)))

    aft_fairing = cq.Workplane("XY").box(0.62, 0.50, 0.42).translate((-1.48, 0.00, 0.98))
    roof_spine = cq.Workplane("XY").box(1.34, 0.34, 0.22).translate((-0.10, 0.00, 1.74))
    front_header = cq.Workplane("XY").box(0.22, 1.04, 0.10).translate((0.56, 0.00, 1.68))
    rear_header = cq.Workplane("XY").box(0.14, 1.02, 0.10).translate((-0.60, 0.00, 1.67))
    windshield_post = cq.Workplane("XY").box(0.12, 0.16, 0.68).translate((0.92, 0.00, 1.30))

    left_a_pillar = (
        cq.Workplane("XY")
        .box(0.10, 0.10, 0.78)
        .translate((0.42, 0.54, 1.27))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 16.0)
    )
    right_a_pillar = (
        cq.Workplane("XY")
        .box(0.10, 0.10, 0.78)
        .translate((0.42, -0.54, 1.27))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 16.0)
    )
    left_b_pillar = cq.Workplane("XY").box(0.10, 0.12, 0.74).translate((-0.62, 0.54, 1.24))
    right_b_pillar = cq.Workplane("XY").box(0.10, 0.12, 0.74).translate((-0.62, -0.54, 1.24))

    return (
        fuselage.union(aft_fairing)
        .union(roof_spine)
        .union(front_header)
        .union(rear_header)
        .union(windshield_post)
        .union(left_a_pillar)
        .union(right_a_pillar)
        .union(left_b_pillar)
        .union(right_b_pillar)
    )


def _windshield_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(0.48, 1.00)
        .threePointArc((1.02, 1.02), (1.24, 1.18))
        .lineTo(0.76, 1.67)
        .lineTo(0.54, 1.64)
        .close()
        .extrude(0.50, both=True)
    )


def _door_frame_shape() -> cq.Workplane:
    profile = [
        (0.00, -0.44),
        (-0.18, -0.45),
        (-0.72, -0.40),
        (-0.84, -0.05),
        (-0.82, 0.22),
        (-0.64, 0.38),
        (-0.14, 0.40),
        (0.00, 0.30),
    ]
    window = [
        (-0.10, -0.04),
        (-0.56, -0.02),
        (-0.68, 0.20),
        (-0.56, 0.31),
        (-0.18, 0.33),
        (-0.08, 0.22),
    ]
    shell = cq.Workplane("XZ").polyline(profile).close().extrude(DOOR_THICKNESS)
    shell = shell.cut(cq.Workplane("XZ").polyline(window).close().extrude(DOOR_THICKNESS + 0.002).translate((0.0, -0.001, 0.0)))

    for zc in (-0.28, 0.00, 0.27):
        shell = shell.union(cq.Workplane("XY").circle(0.020).extrude(0.08).translate((0.00, DOOR_THICKNESS * 0.5, zc)))
    return shell


def _door_window_shape() -> cq.Workplane:
    return cq.Workplane("XZ").polyline(
        [
            (-0.10, -0.04),
            (-0.56, -0.02),
            (-0.68, 0.20),
            (-0.56, 0.31),
            (-0.18, 0.33),
            (-0.08, 0.22),
        ]
    ).close().extrude(0.012)


def _hatch_panel_shape() -> cq.Workplane:
    profile = [
        (0.00, -0.22),
        (0.54, -0.20),
        (0.62, 0.05),
        (0.54, 0.21),
        (0.10, 0.24),
        (0.00, 0.17),
    ]
    shell = cq.Workplane("XZ").polyline(profile).close().extrude(HATCH_THICKNESS)
    shell = shell.union(cq.Workplane("XY").circle(0.016).extrude(0.06).translate((0.00, HATCH_THICKNESS * 0.5, 0.0)))
    shell = shell.union(cq.Workplane("XY").circle(0.016).extrude(0.06).translate((0.00, HATCH_THICKNESS * 0.5, 0.16)))
    shell = shell.union(cq.Workplane("XY").circle(0.016).extrude(0.06).translate((0.00, HATCH_THICKNESS * 0.5, -0.16)))
    return shell


def _main_rotor_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.11).extrude(0.10)
    blade = (
        cq.Workplane("XY")
        .polyline([(0.08, -0.10), (2.25, -0.055), (2.80, -0.018), (2.82, 0.0), (2.80, 0.018), (2.25, 0.055), (0.08, 0.10)])
        .close()
        .extrude(0.028)
        .translate((0.0, 0.0, 0.07))
    )
    rotor = hub.union(blade)
    rotor = rotor.union(blade.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 120.0))
    rotor = rotor.union(blade.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 240.0))
    return rotor


def _skid_mesh(name: str, y: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (1.05, y, 0.16),
                (0.95, y, 0.12),
                (0.20, y, 0.10),
                (-1.05, y, 0.10),
                (-1.72, y, 0.12),
                (-1.86, y, 0.18),
            ],
            radius=0.045,
            samples_per_segment=18,
            radial_segments=18,
        ),
        name,
    )


def _cross_tube_mesh(name: str, x: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (x, 0.82, 0.15),
                (x + 0.02, 0.48, 0.38),
                (x, 0.0, 0.58),
                (x - 0.02, -0.48, 0.38),
                (x, -0.82, 0.15),
            ],
            radius=0.032,
            samples_per_segment=16,
            radial_segments=16,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tour_helicopter")

    body_paint = model.material("body_paint", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.16, 0.18, 0.21, 1.0))
    skid_steel = model.material("skid_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    glass = model.material("glass", rgba=(0.56, 0.72, 0.82, 0.35))
    cabin_trim = model.material("cabin_trim", rgba=(0.24, 0.25, 0.27, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.13, 0.13, 0.14, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.22, 0.23, 0.25, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=1250.0,
        origin=Origin(xyz=(-1.20, 0.0, 1.00)),
    )
    body.visual(_mesh(_body_shell_shape(), "body_shell"), material=body_paint, name="body_shell")
    body.visual(_mesh(_windshield_shape(), "windshield"), material=glass, name="windshield")
    body.visual(
        Cylinder(radius=0.11, length=2.80),
        origin=Origin(xyz=(-3.12, 0.0, 1.18), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_paint,
        name="tail_boom",
    )
    body.visual(
        Cylinder(radius=0.07, length=1.20),
        origin=Origin(xyz=(-4.36, 0.0, 1.22), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_paint,
        name="tail_boom_tip",
    )
    body.visual(Box((0.34, 0.08, 0.74)), origin=Origin(xyz=(-4.56, 0.0, 1.55)), material=body_paint, name="vertical_fin")
    body.visual(Box((0.58, 0.18, 0.04)), origin=Origin(xyz=(-4.12, 0.0, 1.18)), material=body_paint, name="horizontal_stab")
    body.visual(
        Cylinder(radius=0.035, length=0.16),
        origin=Origin(xyz=(-5.035, 0.0, 1.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_paint,
        name="tail_shaft",
    )
    body.visual(Box((0.08, 0.14, 0.14)), origin=Origin(xyz=(-5.155, 0.0, 1.23)), material=body_paint, name="tail_gearbox")
    body.visual(Box((1.32, 1.02, 0.08)), origin=Origin(xyz=(-0.10, 0.0, 0.60)), material=cabin_trim, name="cabin_floor")
    body.visual(Box((0.04, 0.06, 0.86)), origin=Origin(xyz=(DOOR_HINGE_X, 0.570, DOOR_HINGE_Z)), material=trim_dark, name="left_hinge_post")
    body.visual(Box((0.04, 0.06, 0.86)), origin=Origin(xyz=(DOOR_HINGE_X, -0.5951, DOOR_HINGE_Z)), material=trim_dark, name="right_hinge_post")
    body.visual(Box((0.04, 0.04, 0.48)), origin=Origin(xyz=(HATCH_HINGE_X, 0.375, HATCH_HINGE_Z)), material=trim_dark, name="hatch_hinge_post")
    body.visual(Box((0.32, 0.40, 0.12)), origin=Origin(xyz=(0.12, 0.28, 0.68)), material=seat_vinyl, name="left_front_seat_base")
    body.visual(Box((0.32, 0.40, 0.12)), origin=Origin(xyz=(0.12, -0.28, 0.68)), material=seat_vinyl, name="right_front_seat_base")
    body.visual(Box((0.10, 0.38, 0.34)), origin=Origin(xyz=(-0.02, 0.28, 0.87)), material=seat_vinyl, name="left_front_seat_back")
    body.visual(Box((0.10, 0.38, 0.34)), origin=Origin(xyz=(-0.02, -0.28, 0.87)), material=seat_vinyl, name="right_front_seat_back")
    body.visual(Box((0.58, 0.48, 0.14)), origin=Origin(xyz=(-0.58, 0.0, 0.68)), material=seat_vinyl, name="rear_bench_base")
    body.visual(Box((0.12, 0.50, 0.32)), origin=Origin(xyz=(-0.80, 0.0, 0.85)), material=seat_vinyl, name="rear_bench_back")
    body.visual(Box((0.22, 0.18, 0.34)), origin=Origin(xyz=(0.56, 0.0, 0.78)), material=trim_dark, name="console")
    body.visual(Cylinder(radius=0.11, length=0.30), origin=Origin(xyz=(0.06, 0.0, 1.79)), material=trim_dark, name="mast_fairing")
    body.visual(Cylinder(radius=0.050, length=0.38), origin=Origin(xyz=(0.06, 0.0, 1.95)), material=trim_dark, name="mast")
    body.visual(_skid_mesh("left_skid", 0.82), material=skid_steel, name="left_skid")
    body.visual(_skid_mesh("right_skid", -0.82), material=skid_steel, name="right_skid")
    body.visual(_cross_tube_mesh("front_cross_tube", 0.44), material=skid_steel, name="front_cross_tube")
    body.visual(_cross_tube_mesh("rear_cross_tube", -0.58), material=skid_steel, name="rear_cross_tube")

    left_door = model.part("left_door")
    left_door.inertial = Inertial.from_geometry(Box((0.88, 0.04, 0.90)), mass=16.0, origin=Origin(xyz=(-0.42, -0.015, 0.0)))
    left_door.visual(
        _mesh(_door_frame_shape(), "left_door_frame"),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS, 0.0)),
        material=body_paint,
        name="door_frame",
    )
    left_door.visual(
        _mesh(_door_window_shape(), "left_door_window"),
        origin=Origin(xyz=(0.0, -0.021, 0.0)),
        material=glass,
        name="door_window",
    )
    left_door.visual(Box((0.06, 0.024, 0.10)), origin=Origin(xyz=(-0.46, -0.012, -0.06)), material=trim_dark, name="door_handle")

    right_door = model.part("right_door")
    right_door.inertial = Inertial.from_geometry(Box((0.88, 0.04, 0.90)), mass=16.0, origin=Origin(xyz=(-0.42, 0.015, 0.0)))
    right_door.visual(_mesh(_door_frame_shape(), "right_door_frame"), material=body_paint, name="door_frame")
    right_door.visual(_mesh(_door_window_shape(), "right_door_window"), origin=Origin(xyz=(0.0, 0.009, 0.0)), material=glass, name="door_window")
    right_door.visual(Box((0.06, 0.024, 0.10)), origin=Origin(xyz=(-0.46, 0.012, -0.06)), material=trim_dark, name="door_handle")

    luggage_hatch = model.part("luggage_hatch")
    luggage_hatch.inertial = Inertial.from_geometry(Box((0.64, 0.04, 0.50)), mass=9.0, origin=Origin(xyz=(0.30, -0.014, 0.0)))
    luggage_hatch.visual(
        _mesh(_hatch_panel_shape(), "luggage_hatch_panel"),
        origin=Origin(xyz=(0.0, -HATCH_THICKNESS, 0.0)),
        material=body_paint,
        name="hatch_panel",
    )
    luggage_hatch.visual(Box((0.07, 0.028, 0.08)), origin=Origin(xyz=(0.22, -0.014, -0.03)), material=trim_dark, name="hatch_handle")

    main_rotor = model.part("main_rotor")
    main_rotor.inertial = Inertial.from_geometry(Box((5.7, 5.7, 0.18)), mass=32.0, origin=Origin(xyz=(0.0, 0.0, 0.08)))
    main_rotor.visual(_mesh(_main_rotor_shape(), "main_rotor"), material=rotor_gray, name="main_rotor")

    tail_rotor = model.part("tail_rotor")
    tail_rotor.inertial = Inertial.from_geometry(Box((0.60, 0.12, 0.60)), mass=4.5)
    tail_rotor.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="tail_hub",
    )
    tail_rotor.visual(Box((0.05, 0.02, 0.255)), origin=Origin(xyz=(0.0, 0.0, 0.1725)), material=rotor_gray, name="tail_blade_up")
    tail_rotor.visual(Box((0.05, 0.02, 0.255)), origin=Origin(xyz=(0.0, 0.0, -0.1725)), material=rotor_gray, name="tail_blade_down")

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=40.0, velocity=1.2),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(DOOR_HINGE_X, -DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=40.0, velocity=1.2),
    )
    model.articulation(
        "luggage_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=luggage_hatch,
        origin=Origin(xyz=(HATCH_HINGE_X, HATCH_HINGE_Y, HATCH_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=25.0, velocity=1.0),
    )
    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=main_rotor,
        origin=Origin(xyz=(0.06, 0.0, MAIN_MAST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=35.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=tail_rotor,
        origin=Origin(xyz=(TAIL_ROTOR_X, 0.0, TAIL_ROTOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=55.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    luggage_hatch = object_model.get_part("luggage_hatch")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    hatch_hinge = object_model.get_articulation("luggage_hatch_hinge")
    main_spin = object_model.get_articulation("main_rotor_spin")
    tail_spin = object_model.get_articulation("tail_rotor_spin")

    ctx.expect_overlap(left_door, body, axes="xz", min_overlap=0.60, name="left door closes over the cabin opening")
    ctx.expect_overlap(right_door, body, axes="xz", min_overlap=0.60, name="right door closes over the cabin opening")
    ctx.expect_overlap(luggage_hatch, body, axes="xz", min_overlap=0.34, name="luggage hatch closes over the baggage bay")
    ctx.expect_gap(main_rotor, body, axis="z", min_gap=0.0, max_penetration=0.0, name="main rotor sits on the mast without penetrating the roof")

    def center_of_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    left_rest = center_of_aabb(ctx.part_element_world_aabb(left_door, elem="door_frame"))
    right_rest = center_of_aabb(ctx.part_element_world_aabb(right_door, elem="door_frame"))
    hatch_rest = center_of_aabb(ctx.part_element_world_aabb(luggage_hatch, elem="hatch_panel"))

    with ctx.pose({left_hinge: 1.10, right_hinge: 1.10, hatch_hinge: 0.95}):
        left_open = center_of_aabb(ctx.part_element_world_aabb(left_door, elem="door_frame"))
        right_open = center_of_aabb(ctx.part_element_world_aabb(right_door, elem="door_frame"))
        hatch_open = center_of_aabb(ctx.part_element_world_aabb(luggage_hatch, elem="hatch_panel"))

    ctx.check(
        "left door opens outward",
        left_rest is not None and left_open is not None and left_open[1] > left_rest[1] + 0.14,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right door opens outward",
        right_rest is not None and right_open is not None and right_open[1] < right_rest[1] - 0.14,
        details=f"rest={right_rest}, open={right_open}",
    )
    ctx.check(
        "luggage hatch opens on the left side",
        hatch_rest is not None and hatch_open is not None and hatch_open[1] > hatch_rest[1] + 0.10,
        details=f"rest={hatch_rest}, open={hatch_open}",
    )
    ctx.check(
        "main rotor joint is continuous",
        main_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={main_spin.articulation_type!r}",
    )
    ctx.check(
        "tail rotor joint is continuous",
        tail_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={tail_spin.articulation_type!r}",
    )

    tail_pos = ctx.part_world_position(tail_rotor)
    ctx.check(
        "tail rotor is mounted aft on the tail",
        tail_pos is not None and tail_pos[0] < -5.0 and tail_pos[2] > 1.0,
        details=f"tail_pos={tail_pos}",
    )

    return ctx.report()


object_model = build_object_model()
