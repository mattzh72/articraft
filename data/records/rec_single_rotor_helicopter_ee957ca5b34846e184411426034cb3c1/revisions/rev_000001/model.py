from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_main_rotor(part, hub_finish, blade_finish) -> None:
    part.visual(
        Cylinder(radius=0.15, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=hub_finish,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.045, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=hub_finish,
        name="mast_cuff",
    )
    part.visual(
        Box((0.32, 0.22, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=hub_finish,
        name="spider",
    )

    blade_length = 3.45
    blade_chord = 0.18
    root_offset = 0.22
    for index, yaw in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        part.visual(
            Box((0.34, 0.09, 0.06)),
            origin=Origin(xyz=(0.22, 0.0, 0.05), rpy=(0.0, 0.0, yaw)),
            material=hub_finish,
            name=f"grip_{index}",
        )
        part.visual(
            Box((blade_length, blade_chord, 0.04)),
            origin=Origin(xyz=(root_offset + blade_length / 2.0, 0.0, 0.02), rpy=(0.0, 0.03, yaw)),
            material=blade_finish,
            name=f"blade_{index}",
        )


def _add_tail_rotor(part, hub_finish, blade_finish) -> None:
    axial_offset = 0.08
    part.visual(
        Cylinder(radius=0.06, length=0.14),
        origin=Origin(xyz=(axial_offset, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_finish,
        name="hub",
    )
    part.visual(
        Box((0.10, 0.14, 0.14)),
        origin=Origin(xyz=(axial_offset, 0.0, 0.0)),
        material=hub_finish,
        name="center_block",
    )

    blade_length = 0.50
    blade_chord = 0.11
    for index, roll in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        part.visual(
            Box((0.08, 0.16, 0.05)),
            origin=Origin(xyz=(axial_offset, 0.10, 0.0), rpy=(roll, 0.0, 0.0)),
            material=hub_finish,
            name=f"grip_{index}",
        )
        part.visual(
            Box((0.04, blade_length, blade_chord)),
            origin=Origin(xyz=(axial_offset, 0.29, 0.0), rpy=(roll, 0.0, 0.0)),
            material=blade_finish,
            name=f"blade_{index}",
        )


def _add_cowling_door(part, panel_finish, hinge_finish, *, side: str) -> None:
    if side == "left":
        x_offset = -0.20
        pitch = -0.28
        hinge_x = -0.02
    else:
        x_offset = 0.20
        pitch = 0.28
        hinge_x = 0.02

    part.visual(
        Box((0.46, 0.74, 0.024)),
        origin=Origin(xyz=(x_offset, 0.0, 0.0), rpy=(0.0, pitch, 0.0)),
        material=panel_finish,
        name="panel",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.68),
        origin=Origin(xyz=(hinge_x, 0.0, 0.004), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_finish,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.24, 0.72, 0.04)),
        origin=Origin(xyz=(x_offset * 0.65, 0.0, -0.012), rpy=(0.0, pitch, 0.0)),
        material=hinge_finish,
        name="hinge_flange",
    )


def _add_baggage_door(part, panel_finish, hinge_finish) -> None:
    part.visual(
        Box((0.022, 0.50, 0.52)),
        origin=Origin(xyz=(0.011, 0.25, 0.0)),
        material=panel_finish,
        name="panel",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_finish,
        name="hinge_barrel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_helicopter")

    airframe = model.material("airframe", rgba=(0.85, 0.56, 0.20, 1.0))
    glazing = model.material("glazing", rgba=(0.10, 0.13, 0.16, 1.0))
    rotor_finish = model.material("rotor_finish", rgba=(0.26, 0.27, 0.29, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    skid_finish = model.material("skid_finish", rgba=(0.22, 0.23, 0.24, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    fuselage_sections = [
        (1.60, -1.02, -0.40, 0.30),
        (1.22, -1.14, -0.20, 0.94),
        (0.78, -1.28, -0.08, 1.32),
        (0.18, -1.34, 0.00, 1.48),
        (-0.28, -1.30, -0.04, 1.44),
        (-0.72, -1.12, -0.12, 1.08),
        (-1.08, -0.96, -0.18, 0.70),
        (-1.30, -0.82, -0.22, 0.42),
    ]
    body.visual(
        _save_mesh(
            "utility_helicopter_fuselage",
            superellipse_side_loft(fuselage_sections, exponents=2.3, segments=64),
        ),
        material=airframe,
        name="fuselage",
    )
    body.visual(
        Box((0.20, 0.88, 0.12)),
        origin=Origin(xyz=(0.0, -0.48, -0.02)),
        material=airframe,
        name="engine_spine",
    )
    body.visual(
        Box((0.48, 0.76, 0.14)),
        origin=Origin(xyz=(0.0, -0.56, -0.14)),
        material=airframe,
        name="engine_deck",
    )
    body.visual(
        Box((0.92, 0.04, 0.54)),
        origin=Origin(xyz=(0.0, 1.06, -0.46), rpy=(0.48, 0.0, 0.0)),
        material=glazing,
        name="windshield",
    )
    body.visual(
        Box((0.04, 0.56, 0.42)),
        origin=Origin(xyz=(0.63, 0.72, -0.50)),
        material=glazing,
        name="starboard_cabin_window",
    )
    body.visual(
        Box((0.04, 0.56, 0.42)),
        origin=Origin(xyz=(-0.63, 0.72, -0.50)),
        material=glazing,
        name="port_cabin_window",
    )
    body.visual(
        Box((0.34, 0.24, 0.16)),
        origin=Origin(xyz=(0.0, -0.06, -0.03)),
        material=rotor_finish,
        name="mast_base",
    )
    body.visual(
        Cylinder(radius=0.10, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=rotor_finish,
        name="mast",
    )
    body.visual(
        _save_mesh(
            "utility_helicopter_tail_boom",
            tube_from_spline_points(
                [
                    (0.0, -1.12, -0.33),
                    (0.0, -1.90, -0.16),
                    (0.0, -2.85, -0.01),
                    (0.0, -4.00, 0.11),
                ],
                radius=0.115,
                samples_per_segment=18,
                radial_segments=20,
            ),
        ),
        material=airframe,
        name="tail_boom",
    )
    body.visual(
        Box((0.12, 0.56, 0.78)),
        origin=Origin(xyz=(0.0, -3.92, 0.37)),
        material=airframe,
        name="tail_fin",
    )
    body.visual(
        Box((0.74, 0.18, 0.05)),
        origin=Origin(xyz=(-0.05, -3.47, 0.20)),
        material=airframe,
        name="tail_stabilizer",
    )
    body.visual(
        Cylinder(radius=0.05, length=0.36),
        origin=Origin(xyz=(0.17, -4.02, 0.12), rpy=(0.0, pi / 2.0, 0.0)),
        material=rotor_finish,
        name="tail_gearbox",
    )
    body.visual(
        Box((0.04, 0.74, 0.08)),
        origin=Origin(xyz=(0.156, -0.56, -0.04)),
        material=hinge_finish,
        name="starboard_cowling_rail",
    )
    body.visual(
        Box((0.04, 0.74, 0.08)),
        origin=Origin(xyz=(-0.156, -0.56, -0.04)),
        material=hinge_finish,
        name="port_cowling_rail",
    )

    for side_sign, side_prefix in ((1.0, "starboard"), (-1.0, "port")):
        skid_path = [
            (0.86 * side_sign, 0.92, -1.35),
            (0.89 * side_sign, 0.25, -1.39),
            (0.88 * side_sign, -0.48, -1.39),
            (0.82 * side_sign, -1.04, -1.34),
        ]
        body.visual(
            _save_mesh(
                f"utility_helicopter_{side_prefix}_skid",
                tube_from_spline_points(
                    skid_path,
                    radius=0.045,
                    samples_per_segment=12,
                    radial_segments=16,
                ),
            ),
            material=skid_finish,
            name=f"{side_prefix}_skid",
        )
        for strut_index, y_pos in enumerate((0.56, -0.42)):
            body.visual(
                _save_mesh(
                    f"utility_helicopter_{side_prefix}_strut_{strut_index}",
                    tube_from_spline_points(
                        [
                            (0.26 * side_sign, y_pos, -1.00),
                            (0.54 * side_sign, y_pos, -1.10),
                            (0.88 * side_sign, y_pos, -1.40),
                        ],
                        radius=0.030,
                        samples_per_segment=10,
                        radial_segments=14,
                    ),
                ),
                material=skid_finish,
                name=f"{side_prefix}_strut_{strut_index}",
            )

    main_rotor = model.part("main_rotor")
    _add_main_rotor(main_rotor, rotor_finish, blade_finish)

    tail_rotor = model.part("tail_rotor")
    _add_tail_rotor(tail_rotor, rotor_finish, blade_finish)

    left_cowling_door = model.part("left_cowling_door")
    _add_cowling_door(left_cowling_door, airframe, hinge_finish, side="left")

    right_cowling_door = model.part("right_cowling_door")
    _add_cowling_door(right_cowling_door, airframe, hinge_finish, side="right")

    baggage_door = model.part("baggage_door")
    _add_baggage_door(baggage_door, airframe, hinge_finish)

    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=40.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=tail_rotor,
        origin=Origin(xyz=(0.34, -4.02, 0.12)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=75.0),
    )
    model.articulation(
        "left_cowling_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_cowling_door,
        origin=Origin(xyz=(-0.174, -0.56, -0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=0.0, upper=1.18),
    )
    model.articulation(
        "right_cowling_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_cowling_door,
        origin=Origin(xyz=(0.174, -0.56, -0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=0.0, upper=1.18),
    )
    model.articulation(
        "baggage_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=baggage_door,
        origin=Origin(xyz=(0.478, -1.10, -0.66)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    left_cowling_door = object_model.get_part("left_cowling_door")
    right_cowling_door = object_model.get_part("right_cowling_door")
    baggage_door = object_model.get_part("baggage_door")

    main_rotor_spin = object_model.get_articulation("main_rotor_spin")
    tail_rotor_spin = object_model.get_articulation("tail_rotor_spin")
    left_cowling_hinge = object_model.get_articulation("left_cowling_hinge")
    right_cowling_hinge = object_model.get_articulation("right_cowling_hinge")
    baggage_door_hinge = object_model.get_articulation("baggage_door_hinge")

    ctx.allow_overlap(
        baggage_door,
        body,
        elem_a="panel",
        elem_b="fuselage",
        reason="The fuselage is authored as a closed shell proxy, so the separate baggage door skin sits flush on that sidewall.",
    )

    ctx.check(
        "main rotor spins about vertical mast axis",
        main_rotor_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(main_rotor_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_rotor_spin.articulation_type}, axis={main_rotor_spin.axis!r}",
    )
    ctx.check(
        "tail rotor spins about transverse tail axis",
        tail_rotor_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(tail_rotor_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={tail_rotor_spin.articulation_type}, axis={tail_rotor_spin.axis!r}",
    )

    left_limit = left_cowling_hinge.motion_limits.upper if left_cowling_hinge.motion_limits is not None else None
    right_limit = right_cowling_hinge.motion_limits.upper if right_cowling_hinge.motion_limits is not None else None
    baggage_limit = baggage_door_hinge.motion_limits.upper if baggage_door_hinge.motion_limits is not None else None

    with ctx.pose({left_cowling_hinge: 0.0, right_cowling_hinge: 0.0, baggage_door_hinge: 0.0}):
        left_closed = ctx.part_world_aabb(left_cowling_door)
        right_closed = ctx.part_world_aabb(right_cowling_door)
        baggage_closed = ctx.part_world_aabb(baggage_door)

    with ctx.pose({left_cowling_hinge: left_limit or 0.0}):
        left_open = ctx.part_world_aabb(left_cowling_door)
    with ctx.pose({right_cowling_hinge: right_limit or 0.0}):
        right_open = ctx.part_world_aabb(right_cowling_door)
    with ctx.pose({baggage_door_hinge: baggage_limit or 0.0}):
        baggage_open = ctx.part_world_aabb(baggage_door)

    ctx.check(
        "left cowling door opens upward",
        left_closed is not None and left_open is not None and left_open[1][2] > left_closed[1][2] + 0.18,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right cowling door opens upward",
        right_closed is not None and right_open is not None and right_open[1][2] > right_closed[1][2] + 0.18,
        details=f"closed={right_closed}, open={right_open}",
    )
    ctx.check(
        "baggage door swings outward from the aft side",
        baggage_closed is not None and baggage_open is not None and baggage_open[1][0] > baggage_closed[1][0] + 0.18,
        details=f"closed={baggage_closed}, open={baggage_open}",
    )

    ctx.check(
        "main rotor stays above the cabin roof",
        ctx.part_world_aabb(main_rotor) is not None and ctx.part_world_aabb(body) is not None and ctx.part_world_aabb(main_rotor)[0][2] > -0.02,
        details=f"main_rotor={ctx.part_world_aabb(main_rotor)}, body={ctx.part_world_aabb(body)}",
    )
    ctx.check(
        "tail rotor sits aft of the fin post",
        ctx.part_world_aabb(tail_rotor) is not None and ctx.part_world_aabb(tail_rotor)[1][1] < -3.40,
        details=f"tail_rotor={ctx.part_world_aabb(tail_rotor)}",
    )

    return ctx.report()


object_model = build_object_model()
