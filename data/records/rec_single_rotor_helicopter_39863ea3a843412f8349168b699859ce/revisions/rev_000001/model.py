from __future__ import annotations

import math

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
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xz_section(
    station_y: float,
    width: float,
    height: float,
    *,
    center_z: float,
    corner: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, station_y, z + center_z)
        for x, z in rounded_rect_profile(width, height, corner, corner_segments=8)
    ]


def _build_fuselage_shell():
    sections = [
        _xz_section(2.20, 0.18, 0.30, center_z=1.12, corner=0.05),
        _xz_section(1.72, 0.88, 1.12, center_z=1.20, corner=0.16),
        _xz_section(1.10, 1.72, 1.58, center_z=1.30, corner=0.22),
        _xz_section(0.30, 1.96, 1.82, center_z=1.38, corner=0.26),
        _xz_section(-0.55, 1.92, 1.72, center_z=1.43, corner=0.24),
        _xz_section(-1.38, 1.70, 1.46, center_z=1.40, corner=0.20),
        _xz_section(-2.25, 1.00, 0.86, center_z=1.33, corner=0.12),
        _xz_section(-3.55, 0.56, 0.48, center_z=1.34, corner=0.08),
        _xz_section(-5.15, 0.34, 0.30, center_z=1.46, corner=0.05),
        _xz_section(-6.60, 0.22, 0.20, center_z=1.56, corner=0.04),
    ]
    return repair_loft(section_loft(sections))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_utility_helicopter")

    body_red = model.material("body_red", rgba=(0.76, 0.10, 0.07, 1.0))
    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    metal = model.material("metal", rgba=(0.63, 0.66, 0.70, 1.0))
    window_tint = model.material("window_tint", rgba=(0.16, 0.22, 0.28, 0.55))

    fuselage = model.part("fuselage")
    fuselage.visual(
        _save_mesh("fuselage_shell", _build_fuselage_shell()),
        material=body_red,
        name="shell",
    )

    right_skid = tube_from_spline_points(
        [
            (1.12, 1.70, 0.10),
            (1.10, 0.78, 0.08),
            (1.08, -0.50, 0.08),
            (1.04, -1.90, 0.10),
        ],
        radius=0.055,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    left_skid = tube_from_spline_points(
        [
            (-1.12, 1.70, 0.10),
            (-1.10, 0.78, 0.08),
            (-1.08, -0.50, 0.08),
            (-1.04, -1.90, 0.10),
        ],
        radius=0.055,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    forward_cross_tube = tube_from_spline_points(
        [
            (-1.08, 0.78, 0.10),
            (-0.70, 0.79, 0.44),
            (0.00, 0.80, 0.62),
            (0.70, 0.79, 0.44),
            (1.08, 0.78, 0.10),
        ],
        radius=0.042,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    aft_cross_tube = tube_from_spline_points(
        [
            (-1.04, -0.82, 0.10),
            (-0.66, -0.82, 0.43),
            (0.00, -0.82, 0.58),
            (0.66, -0.82, 0.43),
            (1.04, -0.82, 0.10),
        ],
        radius=0.042,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    fuselage.visual(_save_mesh("left_skid", left_skid), material=dark_gray, name="left_skid")
    fuselage.visual(_save_mesh("right_skid", right_skid), material=dark_gray, name="right_skid")
    fuselage.visual(
        _save_mesh("forward_cross_tube", forward_cross_tube),
        material=metal,
        name="forward_cross_tube",
    )
    fuselage.visual(
        _save_mesh("aft_cross_tube", aft_cross_tube),
        material=metal,
        name="aft_cross_tube",
    )

    fuselage.visual(
        Cylinder(radius=0.24, length=0.46),
        origin=Origin(xyz=(0.0, -0.10, 2.41)),
        material=body_white,
        name="mast_fairing",
    )
    fuselage.visual(
        Box((0.18, 1.16, 1.62)),
        origin=Origin(xyz=(0.0, -6.06, 2.11)),
        material=body_white,
        name="tail_fin",
    )
    fuselage.visual(
        Box((0.52, 0.18, 0.40)),
        origin=Origin(xyz=(0.26, -6.36, 1.74)),
        material=body_white,
        name="tail_rotor_pylon",
    )
    fuselage.visual(
        Box((1.18, 0.30, 0.10)),
        origin=Origin(xyz=(0.0, -5.08, 1.34)),
        material=body_white,
        name="horizontal_stabilizer",
    )
    fuselage.visual(
        Cylinder(radius=0.025, length=0.66),
        origin=Origin(xyz=(0.95, -0.20, 1.39)),
        material=metal,
        name="right_service_hinge_mount",
    )
    fuselage.visual(
        Cylinder(radius=0.025, length=0.66),
        origin=Origin(xyz=(-0.95, -0.20, 1.39)),
        material=metal,
        name="left_service_hinge_mount",
    )
    fuselage.visual(
        Cylinder(radius=0.033, length=1.12),
        origin=Origin(xyz=(0.95, 0.72, 1.20)),
        material=metal,
        name="crew_hinge_mount",
    )

    fuselage.visual(
        Box((1.58, 0.05, 0.74)),
        origin=Origin(xyz=(0.0, 1.60, 1.60), rpy=(-0.56, 0.0, 0.0)),
        material=window_tint,
        name="windshield",
    )
    fuselage.visual(
        Box((0.05, 0.92, 0.58)),
        origin=Origin(xyz=(0.90, 1.02, 1.54), rpy=(0.0, -0.10, 0.0)),
        material=window_tint,
        name="right_cockpit_window",
    )
    fuselage.visual(
        Box((0.05, 0.92, 0.58)),
        origin=Origin(xyz=(-0.90, 1.02, 1.54), rpy=(0.0, 0.10, 0.0)),
        material=window_tint,
        name="left_cockpit_window",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.12, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=metal,
        name="mast",
    )
    main_rotor.visual(
        Cylinder(radius=0.26, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=rotor_gray,
        name="hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=rotor_gray,
        name="hub_cap",
    )
    main_blade_length = 4.90
    main_blade_chord = 0.28
    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        radial_offset = 0.42 + main_blade_length / 2.0
        cuff_offset = 0.22 + 0.68 / 2.0
        main_rotor.visual(
            Box((0.68, 0.34, 0.05)),
            origin=Origin(
                xyz=(
                    math.cos(angle) * cuff_offset,
                    math.sin(angle) * cuff_offset,
                    0.07,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=rotor_gray,
            name=f"cuff_{index}",
        )
        main_rotor.visual(
            Box((main_blade_length, main_blade_chord, 0.035)),
            origin=Origin(
                xyz=(
                    math.cos(angle) * radial_offset,
                    math.sin(angle) * radial_offset,
                    0.07,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_gray,
            name=f"blade_{index}",
        )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.08, length=0.18),
        origin=Origin(xyz=(0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub",
    )
    tail_rotor.visual(
        Cylinder(radius=0.12, length=0.06),
        origin=Origin(xyz=(0.03, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_gray,
        name="gearbox_plate",
    )
    tail_blade_length = 0.78
    for index in range(4):
        angle = index * (math.pi / 2.0)
        tail_rotor.visual(
            Box((0.020, tail_blade_length, 0.085)),
            origin=Origin(
                xyz=(
                    0.020,
                    math.cos(angle) * (0.12 + tail_blade_length / 2.0),
                    math.sin(angle) * (0.12 + tail_blade_length / 2.0),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_gray,
            name=f"blade_{index}",
        )

    right_service_door = model.part("right_service_door")
    right_service_door.visual(
        Box((0.028, 0.78, 0.62)),
        origin=Origin(xyz=(0.014, -0.39, 0.31)),
        material=body_white,
        name="panel",
    )
    right_service_door.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.030, -0.62, 0.29), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="handle",
    )

    left_service_door = model.part("left_service_door")
    left_service_door.visual(
        Box((0.028, 0.78, 0.62)),
        origin=Origin(xyz=(-0.014, -0.39, 0.31)),
        material=body_white,
        name="panel",
    )
    left_service_door.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(-0.030, -0.62, 0.29), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="handle",
    )

    crew_door = model.part("crew_door")
    crew_door.visual(
        Box((0.028, 0.92, 1.08)),
        origin=Origin(xyz=(0.014, -0.46, 0.54)),
        material=body_white,
        name="panel",
    )
    crew_door.visual(
        Box((0.016, 0.40, 0.38)),
        origin=Origin(xyz=(0.022, -0.22, 0.76)),
        material=window_tint,
        name="window",
    )
    crew_door.visual(
        Cylinder(radius=0.015, length=0.060),
        origin=Origin(xyz=(0.032, -0.70, 0.48), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="handle",
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.0, -0.05, 2.64)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=25.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(0.52, -6.36, 1.74)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=40.0),
    )
    model.articulation(
        "right_service_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=right_service_door,
        origin=Origin(xyz=(0.975, -0.20, 1.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=0.0,
            upper=2.15,
        ),
    )
    model.articulation(
        "left_service_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=left_service_door,
        origin=Origin(xyz=(-0.975, -0.20, 1.06)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=0.0,
            upper=2.15,
        ),
    )
    model.articulation(
        "crew_door_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=crew_door,
        origin=Origin(xyz=(0.983, 0.72, 0.64)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.3,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def elem_center(part_name: str, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    fuselage = object_model.get_part("fuselage")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    right_service_door = object_model.get_part("right_service_door")
    left_service_door = object_model.get_part("left_service_door")
    crew_door = object_model.get_part("crew_door")

    right_service_hinge = object_model.get_articulation("right_service_hinge")
    left_service_hinge = object_model.get_articulation("left_service_hinge")
    crew_door_hinge = object_model.get_articulation("crew_door_hinge")

    ctx.expect_contact(
        right_service_door,
        fuselage,
        elem_a="panel",
        elem_b="right_service_hinge_mount",
        name="right service door is hinge-supported",
    )
    ctx.expect_overlap(
        right_service_door,
        fuselage,
        axes="yz",
        min_overlap=0.40,
        elem_a="panel",
        elem_b="shell",
        name="right service door covers engine bay opening zone",
    )
    ctx.expect_contact(
        left_service_door,
        fuselage,
        elem_a="panel",
        elem_b="left_service_hinge_mount",
        name="left service door is hinge-supported",
    )
    ctx.expect_overlap(
        left_service_door,
        fuselage,
        axes="yz",
        min_overlap=0.40,
        elem_a="panel",
        elem_b="shell",
        name="left service door covers engine bay opening zone",
    )
    ctx.expect_contact(
        crew_door,
        fuselage,
        elem_a="panel",
        elem_b="crew_hinge_mount",
        name="crew door is hinge-supported",
    )
    ctx.expect_overlap(
        crew_door,
        fuselage,
        axes="yz",
        min_overlap=0.55,
        elem_a="panel",
        elem_b="shell",
        name="crew door covers cabin opening zone",
    )

    rest_right = elem_center("right_service_door", "panel")
    rest_left = elem_center("left_service_door", "panel")
    rest_crew = elem_center("crew_door", "panel")

    with ctx.pose({right_service_hinge: 1.20}):
        open_right = elem_center("right_service_door", "panel")
    with ctx.pose({left_service_hinge: 1.20}):
        open_left = elem_center("left_service_door", "panel")
    with ctx.pose({crew_door_hinge: 1.10}):
        open_crew = elem_center("crew_door", "panel")

    ctx.check(
        "right service door swings outward",
        rest_right is not None and open_right is not None and open_right[0] > rest_right[0] + 0.15,
        details=f"rest={rest_right}, open={open_right}",
    )
    ctx.check(
        "left service door swings outward",
        rest_left is not None and open_left is not None and open_left[0] < rest_left[0] - 0.15,
        details=f"rest={rest_left}, open={open_left}",
    )
    ctx.check(
        "crew door swings outward",
        rest_crew is not None and open_crew is not None and open_crew[0] > rest_crew[0] + 0.20,
        details=f"rest={rest_crew}, open={open_crew}",
    )

    main_rotor_pos = ctx.part_world_position(main_rotor)
    tail_rotor_pos = ctx.part_world_position(tail_rotor)
    fuselage_pos = ctx.part_world_position(fuselage)
    ctx.check(
        "main rotor sits above fuselage",
        main_rotor_pos is not None and fuselage_pos is not None and main_rotor_pos[2] > fuselage_pos[2] + 2.4,
        details=f"fuselage={fuselage_pos}, rotor={main_rotor_pos}",
    )
    ctx.check(
        "tail rotor sits aft on tail boom",
        tail_rotor_pos is not None
        and fuselage_pos is not None
        and tail_rotor_pos[1] < fuselage_pos[1] - 5.5
        and tail_rotor_pos[2] > fuselage_pos[2] + 1.4,
        details=f"fuselage={fuselage_pos}, tail_rotor={tail_rotor_pos}",
    )

    return ctx.report()


object_model = build_object_model()
