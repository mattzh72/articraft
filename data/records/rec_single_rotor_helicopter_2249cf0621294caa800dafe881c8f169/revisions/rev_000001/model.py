from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _xz_section(
    y_pos: float,
    width: float,
    height: float,
    radius: float,
    *,
    center_x: float = 0.0,
    center_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x_val, y_pos, center_z + z_val)
        for x_val, z_val in rounded_rect_profile(width, height, radius)
    ]


def _blade_section(
    x_pos: float,
    chord: float,
    thickness: float,
    *,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_t = 0.5 * thickness
    return [
        (x_pos, center_y - 0.48 * chord, center_z + 0.02 * half_t),
        (x_pos, center_y - 0.14 * chord, center_z + 1.00 * half_t),
        (x_pos, center_y + 0.50 * chord, center_z + 0.14 * half_t),
        (x_pos, center_y + 0.16 * chord, center_z - 0.92 * half_t),
    ]


def _span(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="agricultural_helicopter")

    body_paint = model.material("body_paint", rgba=(0.94, 0.77, 0.14, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.20, 0.25, 0.29, 0.62))
    frame_gray = model.material("frame_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.34, 0.36, 0.39, 1.0))

    airframe = model.part("airframe")

    cockpit_shell = section_loft(
        [
            _xz_section(1.85, 0.18, 0.46, 0.06, center_z=1.12),
            _xz_section(1.35, 0.70, 1.24, 0.18, center_z=1.23),
            _xz_section(0.90, 0.86, 1.42, 0.20, center_z=1.30),
            _xz_section(0.55, 1.00, 1.34, 0.21, center_z=1.32),
        ]
    )
    tank_shell = section_loft(
        [
            _xz_section(0.55, 1.18, 1.38, 0.24, center_z=1.30),
            _xz_section(0.10, 1.72, 1.56, 0.32, center_z=1.33),
            _xz_section(-0.55, 1.78, 1.52, 0.30, center_z=1.30),
            _xz_section(-1.10, 1.48, 1.22, 0.24, center_z=1.22),
        ]
    )
    tail_boom = section_loft(
        [
            _xz_section(-1.02, 0.44, 0.34, 0.09, center_z=1.18),
            _xz_section(-2.05, 0.30, 0.24, 0.07, center_z=1.12),
            _xz_section(-3.25, 0.22, 0.18, 0.05, center_z=1.06),
            _xz_section(-4.45, 0.15, 0.14, 0.04, center_z=1.12),
        ]
    )
    belly_fairing = BoxGeometry((0.88, 1.35, 0.28)).translate(0.0, 0.18, 0.73)
    mast_column = CylinderGeometry(radius=0.11, height=0.42).translate(0.0, 0.05, 2.30)
    fin = BoxGeometry((0.10, 0.54, 0.92)).translate(0.0, -4.28, 1.57)
    tailplane = BoxGeometry((0.78, 0.20, 0.05)).translate(0.0, -3.82, 1.25)

    housing_floor = BoxGeometry((0.92, 0.56, 0.06)).translate(0.0, -0.08, 1.74)
    housing_roof = BoxGeometry((1.00, 0.66, 0.10)).translate(0.0, -0.08, 2.24)
    housing_left = BoxGeometry((0.08, 0.56, 0.52)).translate(-0.46, -0.08, 1.98)
    housing_front = BoxGeometry((1.00, 0.08, 0.52)).translate(0.0, 0.16, 1.98)
    housing_rear = BoxGeometry((1.00, 0.08, 0.52)).translate(0.0, -0.32, 1.98)

    body_shell = _merge_geometries(
        cockpit_shell,
        tank_shell,
        tail_boom,
        belly_fairing,
        mast_column,
        fin,
    )
    airframe.visual(
        _save_mesh("airframe_shell", body_shell),
        material=body_paint,
        name="body_shell",
    )
    airframe.visual(
        Box((0.16, 0.34, 0.22)),
        origin=Origin(xyz=(0.05, -4.37, 1.17)),
        material=body_paint,
        name="tail_gearbox",
    )
    airframe.visual(
        Box((0.78, 0.20, 0.08)),
        origin=Origin(xyz=(0.0, -3.82, 1.18)),
        material=body_paint,
        name="tailplane",
    )
    airframe.visual(
        Box((0.92, 0.56, 0.06)),
        origin=Origin(xyz=(0.0, -0.08, 1.74)),
        material=body_paint,
        name="housing_floor",
    )
    airframe.visual(
        Box((1.00, 0.66, 0.10)),
        origin=Origin(xyz=(0.0, -0.08, 2.24)),
        material=body_paint,
        name="housing_roof",
    )
    airframe.visual(
        Box((0.08, 0.56, 0.52)),
        origin=Origin(xyz=(-0.46, -0.08, 1.98)),
        material=body_paint,
        name="housing_left",
    )
    airframe.visual(
        Box((1.00, 0.08, 0.52)),
        origin=Origin(xyz=(0.0, 0.16, 1.98)),
        material=body_paint,
        name="housing_front",
    )
    airframe.visual(
        Box((1.00, 0.08, 0.52)),
        origin=Origin(xyz=(0.0, -0.32, 1.98)),
        material=body_paint,
        name="housing_rear",
    )

    canopy = section_loft(
        [
            _xz_section(1.55, 0.40, 0.66, 0.12, center_z=1.46),
            _xz_section(1.18, 0.58, 0.88, 0.15, center_z=1.50),
            _xz_section(0.82, 0.64, 0.80, 0.14, center_z=1.52),
        ]
    )
    airframe.visual(
        _save_mesh("canopy_glass", canopy),
        material=canopy_tint,
        name="canopy_glass",
    )

    skid_right = tube_from_spline_points(
        [
            (0.97, 1.20, 0.21),
            (0.98, 0.85, 0.16),
            (0.99, 0.20, 0.14),
            (0.97, -0.95, 0.15),
            (0.95, -1.55, 0.20),
        ],
        radius=0.05,
        samples_per_segment=16,
        radial_segments=18,
    )
    skid_left = tube_from_spline_points(
        [
            (-0.97, 1.20, 0.21),
            (-0.98, 0.85, 0.16),
            (-0.99, 0.20, 0.14),
            (-0.97, -0.95, 0.15),
            (-0.95, -1.55, 0.20),
        ],
        radius=0.05,
        samples_per_segment=16,
        radial_segments=18,
    )
    front_cross = CylinderGeometry(radius=0.042, height=1.64).rotate_y(pi / 2.0).translate(
        0.0,
        0.55,
        0.60,
    )
    rear_cross = CylinderGeometry(radius=0.042, height=1.60).rotate_y(pi / 2.0).translate(
        0.0,
        -0.62,
        0.64,
    )
    gear_struts = [
        tube_from_spline_points(
            [(0.80, 0.55, 0.60), (0.96, 0.78, 0.18)],
            radius=0.035,
            samples_per_segment=6,
            radial_segments=14,
        ),
        tube_from_spline_points(
            [(-0.80, 0.55, 0.60), (-0.96, 0.78, 0.18)],
            radius=0.035,
            samples_per_segment=6,
            radial_segments=14,
        ),
        tube_from_spline_points(
            [(0.78, -0.62, 0.64), (0.94, -0.86, 0.17)],
            radius=0.035,
            samples_per_segment=6,
            radial_segments=14,
        ),
        tube_from_spline_points(
            [(-0.78, -0.62, 0.64), (-0.94, -0.86, 0.17)],
            radius=0.035,
            samples_per_segment=6,
            radial_segments=14,
        ),
    ]
    landing_gear = _merge_geometries(
        skid_right,
        skid_left,
        front_cross,
        rear_cross,
        *gear_struts,
    )
    airframe.visual(
        _save_mesh("landing_gear", landing_gear),
        material=frame_gray,
        name="landing_gear",
    )

    panel_frame = _merge_geometries(
        BoxGeometry((0.32, 0.10, 0.38)).translate(0.78, 0.05, 2.00),
        BoxGeometry((0.32, 0.10, 0.38)).translate(0.78, -0.39, 2.00),
        BoxGeometry((0.32, 0.34, 0.08)).translate(0.78, -0.17, 2.23),
        BoxGeometry((0.32, 0.34, 0.08)).translate(0.78, -0.17, 1.77),
    )
    airframe.visual(
        _save_mesh("panel_frame", panel_frame),
        material=body_paint,
        name="panel_frame",
    )

    airframe.visual(
        Box((0.60, 0.34, 0.24)),
        origin=Origin(xyz=(0.0, -0.10, 1.88)),
        material=dark_gray,
        name="engine_core",
    )
    airframe.visual(
        Box((0.50, 0.46, 0.10)),
        origin=Origin(xyz=(0.0, -0.62, 2.09)),
        material=body_paint,
        name="lid_seat",
    )
    airframe.visual(
        Cylinder(radius=0.14, length=0.06),
        origin=Origin(xyz=(0.0, 0.05, 2.54)),
        material=dark_gray,
        name="mast_cap",
    )
    airframe.visual(
        Cylinder(radius=0.04, length=0.10),
        origin=Origin(xyz=(0.13, -4.55, 1.18), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="tail_stub",
    )

    main_rotor = model.part("main_rotor")
    main_hub = _merge_geometries(
        CylinderGeometry(radius=0.20, height=0.08).translate(0.0, 0.0, 0.05),
        CylinderGeometry(radius=0.09, height=0.10).translate(0.0, 0.0, -0.02),
    )
    blade_geom = section_loft(
        [
            _blade_section(0.16, 0.34, 0.050, center_z=0.00),
            _blade_section(1.70, 0.24, 0.026, center_y=0.05, center_z=-0.01),
            _blade_section(3.75, 0.11, 0.010, center_y=0.12, center_z=-0.02),
        ]
    )
    main_blades = MeshGeometry()
    for blade_index in range(3):
        main_blades.merge(blade_geom.copy().rotate_z(blade_index * 2.0 * pi / 3.0))
    main_rotor.visual(
        _save_mesh("main_rotor_hub", main_hub),
        material=dark_gray,
        name="main_hub",
    )
    main_rotor.visual(
        _save_mesh("main_rotor_blades", main_blades),
        material=rotor_gray,
        name="main_blades",
    )

    tail_rotor = model.part("tail_rotor")
    tail_hub = CylinderGeometry(radius=0.07, height=0.10).rotate_y(pi / 2.0)
    tail_blade_single = _merge_geometries(
        BoxGeometry((0.02, 0.10, 0.62)).translate(0.0, 0.0, 0.31),
        BoxGeometry((0.02, 0.06, 0.16)).translate(0.0, 0.0, 0.62),
    )
    tail_blades = _merge_geometries(
        tail_blade_single,
        tail_blade_single.copy().rotate_x(pi),
    )
    tail_rotor.visual(
        _save_mesh("tail_rotor_hub", tail_hub),
        material=dark_gray,
        name="tail_hub",
    )
    tail_rotor.visual(
        _save_mesh("tail_rotor_blades", tail_blades),
        material=rotor_gray,
        name="tail_blades",
    )

    fill_lid = model.part("fill_lid")
    fill_lid.visual(
        Box((0.46, 0.40, 0.035)),
        origin=Origin(xyz=(0.0, -0.20, 0.0175)),
        material=body_paint,
        name="lid_plate",
    )
    fill_lid.visual(
        Box((0.18, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, -0.18, 0.055)),
        material=body_paint,
        name="lid_rise",
    )
    fill_lid.visual(
        Box((0.05, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, -0.18, 0.085)),
        material=dark_gray,
        name="lid_handle",
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.03, 0.34, 0.38)),
        origin=Origin(xyz=(0.015, -0.17, 0.0)),
        material=body_paint,
        name="panel_skin",
    )
    service_panel.visual(
        Box((0.05, 0.12, 0.09)),
        origin=Origin(xyz=(0.025, -0.14, 0.0)),
        material=body_paint,
        name="panel_stiffener",
    )
    service_panel.visual(
        Box((0.05, 0.03, 0.10)),
        origin=Origin(xyz=(0.045, -0.23, 0.0)),
        material=dark_gray,
        name="panel_handle",
    )

    model.articulation(
        "mast_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.05, 2.64)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=40.0),
    )
    model.articulation(
        "tail_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(0.23, -4.55, 1.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=60.0),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=fill_lid,
        origin=Origin(xyz=(0.0, -0.42, 2.14)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=service_panel,
        origin=Origin(xyz=(0.94, 0.00, 2.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    fill_lid = object_model.get_part("fill_lid")
    service_panel = object_model.get_part("service_panel")
    tail_rotor = object_model.get_part("tail_rotor")

    mast_spin = object_model.get_articulation("mast_spin")
    tail_spin = object_model.get_articulation("tail_spin")
    lid_hinge = object_model.get_articulation("lid_hinge")
    panel_hinge = object_model.get_articulation("panel_hinge")

    ctx.check(
        "main rotor uses a vertical continuous mast joint",
        mast_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(mast_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={mast_spin.articulation_type}, axis={mast_spin.axis}",
    )
    ctx.check(
        "tail rotor uses a transverse continuous tail joint",
        tail_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(tail_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={tail_spin.articulation_type}, axis={tail_spin.axis}",
    )
    ctx.check(
        "fill lid hinge is a top-mounted revolute joint",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )
    ctx.check(
        "service panel hinge is a side-mounted revolute joint",
        panel_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(panel_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={panel_hinge.articulation_type}, axis={panel_hinge.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, panel_hinge: 0.0}):
        ctx.expect_gap(
            fill_lid,
            airframe,
            axis="z",
            positive_elem="lid_plate",
            negative_elem="lid_seat",
            max_gap=0.003,
            max_penetration=0.001,
            name="fill lid sits down on the tank collar",
        )
        ctx.expect_overlap(
            fill_lid,
            airframe,
            axes="xy",
            elem_a="lid_plate",
            elem_b="lid_seat",
            min_overlap=0.32,
            name="fill lid covers the tank opening",
        )
        ctx.expect_gap(
            service_panel,
            airframe,
            axis="x",
            positive_elem="panel_skin",
            negative_elem="panel_frame",
            max_gap=0.002,
            max_penetration=0.001,
            name="service panel closes flush against the engine bay frame",
        )
        ctx.expect_overlap(
            service_panel,
            airframe,
            axes="yz",
            elem_a="panel_skin",
            elem_b="panel_frame",
            min_overlap=0.30,
            name="service panel covers the engine bay opening",
        )

        lid_closed = ctx.part_element_world_aabb(fill_lid, elem="lid_plate")
        panel_closed = ctx.part_element_world_aabb(service_panel, elem="panel_skin")
        tail_rest = ctx.part_element_world_aabb(tail_rotor, elem="tail_blades")

    with ctx.pose({lid_hinge: 1.10, panel_hinge: 1.00, tail_spin: pi / 2.0}):
        lid_open = ctx.part_element_world_aabb(fill_lid, elem="lid_plate")
        panel_open = ctx.part_element_world_aabb(service_panel, elem="panel_skin")
        tail_turned = ctx.part_element_world_aabb(tail_rotor, elem="tail_blades")

    ctx.check(
        "fill lid opens upward behind the mast",
        lid_closed is not None and lid_open is not None and lid_open[1][2] > lid_closed[1][2] + 0.22,
        details=f"closed={lid_closed}, open={lid_open}",
    )
    ctx.check(
        "service panel swings outward from the engine bay",
        panel_closed is not None
        and panel_open is not None
        and panel_open[1][0] > panel_closed[1][0] + 0.18,
        details=f"closed={panel_closed}, open={panel_open}",
    )
    ctx.check(
        "tail rotor quarter turn changes blade orientation",
        _span(tail_rest, 2) is not None
        and _span(tail_rest, 1) is not None
        and _span(tail_turned, 2) is not None
        and _span(tail_turned, 1) is not None
        and _span(tail_rest, 2) > _span(tail_rest, 1) + 0.30
        and _span(tail_turned, 1) > _span(tail_turned, 2) + 0.30,
        details=f"rest={tail_rest}, turned={tail_turned}",
    )

    return ctx.report()


object_model = build_object_model()
