from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

SHELL_OUTER_PROFILE = [
    (2.10, 0.02),
    (2.10, 0.92),
    (2.04, 1.20),
    (1.88, 1.55),
    (1.60, 1.95),
    (1.22, 2.30),
    (0.74, 2.60),
    (0.32, 2.78),
    (0.08, 2.84),
]
SHELL_INNER_PROFILE = [
    (2.02, 0.06),
    (2.02, 0.90),
    (1.96, 1.18),
    (1.80, 1.52),
    (1.54, 1.90),
    (1.18, 2.22),
    (0.72, 2.52),
    (0.30, 2.72),
    (0.03, 2.80),
]
RIB_ANGLES = (
    -2.45,
    -1.95,
    -1.35,
    -0.78,
    0.78,
    1.35,
    1.95,
    2.45,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float = 0.0,
    radial_segments: int = 72,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _radius_at_z(profile: list[tuple[float, float]], z_value: float) -> float:
    if z_value <= profile[0][1]:
        return profile[0][0]
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        if z_value <= z1:
            if abs(z1 - z0) < 1e-9:
                return r1
            t = (z_value - z0) / (z1 - z0)
            return r0 + (r1 - r0) * t
    return profile[-1][0]


def _shell_point(z_value: float, y_value: float, *, offset: float = 0.0) -> tuple[float, float, float]:
    radius = _radius_at_z(SHELL_OUTER_PROFILE, z_value) + offset
    x_value = math.sqrt(max(radius * radius - y_value * y_value, 1e-6))
    return (x_value, y_value, z_value)


def _section_loop(
    *,
    width: float,
    depth: float,
    corner_radius: float,
    x_center: float,
    z_value: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_value + x_center, y_value, z_value)
        for x_value, y_value in rounded_rect_profile(
            depth,
            width,
            corner_radius,
            corner_segments=6,
        )
    ]


def _build_dome_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        SHELL_OUTER_PROFILE,
        SHELL_INNER_PROFILE,
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    lower_slit_cut = BoxGeometry((3.20, 0.74, 1.95)).translate(1.08, 0.0, 1.35)
    crown_slit_cut = BoxGeometry((1.30, 0.62, 0.86)).translate(0.96, 0.0, 3.08)
    slit_cut = boolean_union(lower_slit_cut, crown_slit_cut)
    return boolean_difference(shell, slit_cut)


def _build_meridian_rib_mesh() -> MeshGeometry:
    rib_points = [
        _shell_point(0.96, 0.0, offset=0.012),
        _shell_point(1.18, 0.0, offset=0.012),
        _shell_point(1.46, 0.0, offset=0.012),
        _shell_point(1.78, 0.0, offset=0.012),
        _shell_point(2.08, 0.0, offset=0.010),
        _shell_point(2.38, 0.0, offset=0.010),
        _shell_point(2.62, 0.0, offset=0.008),
    ]
    return tube_from_spline_points(
        rib_points,
        radius=0.024,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )


def _build_slit_edge_mesh(sign: float) -> MeshGeometry:
    edge_points = [
        _shell_point(0.34, sign * 0.40, offset=0.012),
        _shell_point(0.82, sign * 0.40, offset=0.012),
        _shell_point(1.34, sign * 0.40, offset=0.012),
        _shell_point(1.86, sign * 0.38, offset=0.012),
        _shell_point(2.22, sign * 0.34, offset=0.012),
        _shell_point(2.48, sign * 0.30, offset=0.010),
        _shell_point(2.66, sign * 0.28, offset=0.010),
    ]
    return tube_from_spline_points(
        edge_points,
        radius=0.028,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )


def _build_main_shutter_panel_mesh() -> MeshGeometry:
    panel = repair_loft(
        section_loft(
            [
                _section_loop(width=0.60, depth=0.14, corner_radius=0.020, x_center=-0.05, z_value=-0.04),
                _section_loop(width=0.68, depth=0.18, corner_radius=0.024, x_center=0.18, z_value=-0.55),
                _section_loop(width=0.74, depth=0.22, corner_radius=0.026, x_center=0.46, z_value=-1.15),
                _section_loop(width=0.74, depth=0.22, corner_radius=0.026, x_center=0.76, z_value=-1.75),
                _section_loop(width=0.68, depth=0.18, corner_radius=0.022, x_center=0.90, z_value=-2.05),
            ]
        )
    )
    panel.merge(BoxGeometry((0.16, 0.78, 0.10)).translate(0.02, 0.0, -0.04))
    panel.merge(BoxGeometry((0.10, 0.72, 0.08)).translate(0.22, 0.0, -0.62))
    panel.merge(BoxGeometry((0.10, 0.72, 0.08)).translate(0.48, 0.0, -1.25))
    panel.merge(BoxGeometry((0.10, 0.68, 0.08)).translate(0.74, 0.0, -1.88))
    panel.merge(BoxGeometry((0.10, 0.10, 1.86)).translate(0.66, -0.30, -1.08))
    panel.merge(BoxGeometry((0.10, 0.10, 1.86)).translate(0.66, 0.30, -1.08))
    panel.merge(BoxGeometry((0.08, 0.60, 0.08)).translate(0.88, 0.0, -2.03))
    return panel


def _build_crown_shutter_panel_mesh() -> MeshGeometry:
    panel = repair_loft(
        section_loft(
            [
                _section_loop(width=0.54, depth=0.12, corner_radius=0.018, x_center=-0.02, z_value=-0.02),
                _section_loop(width=0.60, depth=0.16, corner_radius=0.020, x_center=0.12, z_value=-0.20),
                _section_loop(width=0.64, depth=0.18, corner_radius=0.022, x_center=0.34, z_value=-0.42),
                _section_loop(width=0.60, depth=0.16, corner_radius=0.020, x_center=0.52, z_value=-0.66),
            ]
        )
    )
    panel.merge(BoxGeometry((0.12, 0.64, 0.08)).translate(0.02, 0.0, -0.03))
    panel.merge(BoxGeometry((0.08, 0.60, 0.06)).translate(0.18, 0.0, -0.26))
    panel.merge(BoxGeometry((0.08, 0.56, 0.06)).translate(0.40, 0.0, -0.52))
    panel.merge(BoxGeometry((0.08, 0.08, 0.54)).translate(0.34, -0.26, -0.34))
    panel.merge(BoxGeometry((0.08, 0.08, 0.54)).translate(0.34, 0.26, -0.34))
    return panel


def _aabb_center(bounds):
    return tuple((lower + upper) * 0.5 for lower, upper in zip(bounds[0], bounds[1]))


def _hinge_collar_geometry(outer_radius: float, inner_radius: float, length: float) -> MeshGeometry:
    return _ring_band(
        outer_radius=outer_radius,
        inner_radius=inner_radius,
        height=length,
        radial_segments=32,
    ).rotate_x(math.pi / 2.0)


def _build_main_hinge_support_mesh() -> MeshGeometry:
    support = CylinderGeometry(radius=0.045, height=0.84, radial_segments=28)
    support.rotate_x(math.pi / 2.0).translate(2.16, 0.0, 2.42)
    support.merge(BoxGeometry((0.22, 0.10, 0.24)).translate(1.98, 0.42, 2.40))
    support.merge(BoxGeometry((0.22, 0.10, 0.24)).translate(1.98, -0.42, 2.40))
    support.merge(BoxGeometry((0.26, 0.92, 0.10)).translate(1.92, 0.0, 2.30))
    support.merge(BoxGeometry((0.16, 0.16, 0.12)).translate(2.04, 0.42, 2.24))
    support.merge(BoxGeometry((0.16, 0.16, 0.12)).translate(2.04, -0.42, 2.24))
    support.merge(BoxGeometry((0.16, 0.12, 0.18)).translate(2.00, 0.52, 2.32))
    support.merge(BoxGeometry((0.16, 0.12, 0.18)).translate(2.00, -0.52, 2.32))
    return support


def _build_crown_hinge_support_mesh() -> MeshGeometry:
    support = CylinderGeometry(radius=0.040, height=0.62, radial_segments=24)
    support.rotate_x(math.pi / 2.0).translate(1.24, 0.0, 3.08)
    support.merge(BoxGeometry((0.18, 0.08, 0.12)).translate(1.10, 0.22, 3.08))
    support.merge(BoxGeometry((0.18, 0.08, 0.12)).translate(1.10, -0.22, 3.08))
    support.merge(BoxGeometry((0.26, 0.58, 0.08)).translate(1.02, 0.0, 3.00))
    support.merge(BoxGeometry((0.14, 0.10, 0.14)).translate(1.08, 0.34, 3.02))
    support.merge(BoxGeometry((0.14, 0.10, 0.14)).translate(1.08, -0.34, 3.02))
    return support


def _build_main_shutter_body_mesh() -> MeshGeometry:
    body = BoxGeometry((0.06, 0.80, 1.94)).translate(0.10, 0.0, -0.98)
    body.merge(BoxGeometry((0.14, 0.84, 0.12)).translate(0.06, 0.0, -0.04))
    body.merge(BoxGeometry((0.08, 0.72, 0.08)).translate(0.10, 0.0, -0.58))
    body.merge(BoxGeometry((0.08, 0.72, 0.08)).translate(0.12, 0.0, -1.08))
    body.merge(BoxGeometry((0.08, 0.68, 0.08)).translate(0.14, 0.0, -1.58))
    body.merge(BoxGeometry((0.08, 0.08, 1.80)).translate(0.14, 0.36, -1.00))
    body.merge(BoxGeometry((0.08, 0.08, 1.80)).translate(0.14, -0.36, -1.00))
    body.merge(_hinge_collar_geometry(0.070, 0.051, 0.16).translate(0.0, 0.34, 0.0))
    body.merge(_hinge_collar_geometry(0.070, 0.051, 0.16).translate(0.0, -0.34, 0.0))
    body.merge(BoxGeometry((0.34, 0.06, 0.08)).translate(0.10, 0.34, 0.0))
    body.merge(BoxGeometry((0.34, 0.06, 0.08)).translate(0.10, -0.34, 0.0))
    for z_value in (-0.44, -0.88, -1.32, -1.76):
        body.merge(
            CylinderGeometry(radius=0.013, height=0.030, radial_segments=18)
            .rotate_y(math.pi / 2.0)
            .translate(0.14, 0.34, z_value)
        )
        body.merge(
            CylinderGeometry(radius=0.013, height=0.030, radial_segments=18)
            .rotate_y(math.pi / 2.0)
            .translate(0.14, -0.34, z_value)
        )
    return body


def _build_main_shutter_hinge_mesh() -> MeshGeometry:
    return BoxGeometry((0.02, 0.02, 0.02)).translate(0.12, 0.0, -0.98)


def _build_crown_shutter_body_mesh() -> MeshGeometry:
    body = BoxGeometry((0.05, 0.56, 0.72)).translate(0.08, 0.0, -0.36)
    body.merge(BoxGeometry((0.10, 0.58, 0.10)).translate(0.04, 0.0, -0.02))
    body.merge(BoxGeometry((0.08, 0.48, 0.06)).translate(0.10, 0.0, -0.26))
    body.merge(BoxGeometry((0.06, 0.06, 0.62)).translate(0.12, 0.24, -0.36))
    body.merge(BoxGeometry((0.06, 0.06, 0.62)).translate(0.12, -0.24, -0.36))
    body.merge(_hinge_collar_geometry(0.060, 0.045, 0.14).translate(0.0, 0.18, 0.0))
    body.merge(_hinge_collar_geometry(0.060, 0.045, 0.14).translate(0.0, -0.18, 0.0))
    body.merge(BoxGeometry((0.22, 0.06, 0.06)).translate(0.08, 0.18, 0.0))
    body.merge(BoxGeometry((0.22, 0.06, 0.06)).translate(0.08, -0.18, 0.0))
    for z_value in (-0.18, -0.40):
        body.merge(
            CylinderGeometry(radius=0.011, height=0.024, radial_segments=16)
            .rotate_y(math.pi / 2.0)
            .translate(0.12, 0.24, z_value)
        )
        body.merge(
            CylinderGeometry(radius=0.011, height=0.024, radial_segments=16)
            .rotate_y(math.pi / 2.0)
            .translate(0.12, -0.24, z_value)
        )
    return body


def _build_crown_shutter_hinge_mesh() -> MeshGeometry:
    return BoxGeometry((0.02, 0.02, 0.02)).translate(0.08, 0.0, -0.36)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotating_observatory_dome", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.61, 0.62, 0.63, 1.0))
    dome_paint = model.material("dome_paint", rgba=(0.73, 0.76, 0.79, 1.0))
    utility_green = model.material("utility_green", rgba=(0.42, 0.48, 0.44, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    fastener_zinc = model.material("fastener_zinc", rgba=(0.72, 0.74, 0.77, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))

    dome_shell_mesh = _save_mesh(
        "dome_shell.obj",
        _merge_geometries(
            _build_dome_shell_mesh(),
            _build_main_hinge_support_mesh(),
            _build_crown_hinge_support_mesh(),
        ),
    )
    meridian_rib_mesh = _save_mesh("meridian_rib.obj", _build_meridian_rib_mesh())
    slit_edge_left_mesh = _save_mesh("slit_edge_left.obj", _build_slit_edge_mesh(-1.0))
    slit_edge_right_mesh = _save_mesh("slit_edge_right.obj", _build_slit_edge_mesh(1.0))
    support_ring_mesh = _save_mesh(
        "support_ring.obj",
        _ring_band(outer_radius=2.14, inner_radius=1.90, height=0.08, radial_segments=80),
    )
    bearing_collar_mesh = _save_mesh(
        "bearing_collar.obj",
        _ring_band(outer_radius=2.12, inner_radius=1.94, height=0.10, radial_segments=80),
    )
    drive_ring_mesh = _save_mesh(
        "drive_ring.obj",
        _ring_band(outer_radius=2.18, inner_radius=2.02, height=0.14, radial_segments=80),
    )
    skirt_band_mesh = _save_mesh(
        "skirt_band.obj",
        _ring_band(outer_radius=2.12, inner_radius=2.06, height=0.06, radial_segments=72),
    )
    mid_band_mesh = _save_mesh(
        "mid_band.obj",
        _ring_band(outer_radius=1.90, inner_radius=1.84, height=0.05, radial_segments=72),
    )
    upper_band_mesh = _save_mesh(
        "upper_band.obj",
        _ring_band(outer_radius=1.28, inner_radius=1.22, height=0.05, radial_segments=72),
    )
    main_shutter_body_mesh = _save_mesh("main_shutter_body.obj", _build_main_shutter_body_mesh())
    main_shutter_hinge_mesh = _save_mesh("main_shutter_hinge.obj", _build_main_shutter_hinge_mesh())
    crown_shutter_body_mesh = _save_mesh("crown_shutter_body.obj", _build_crown_shutter_body_mesh())
    crown_shutter_hinge_mesh = _save_mesh("crown_shutter_hinge.obj", _build_crown_shutter_hinge_mesh())

    base_curb = model.part("base_curb")
    base_curb.visual(
        Box((5.40, 5.40, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=concrete,
        name="foundation_slab",
    )
    base_curb.visual(
        Cylinder(radius=2.28, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=utility_green,
        name="curb_wall",
    )
    base_curb.visual(
        support_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        material=hardware_dark,
        name="roller_mount_ring",
    )
    base_curb.visual(
        _save_mesh(
            "outer_splash_skirt.obj",
            _ring_band(outer_radius=2.34, inner_radius=2.22, height=0.14, radial_segments=80),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.71)),
        material=hardware_dark,
        name="outer_splash_skirt",
    )
    base_curb.visual(
        Box((0.74, 0.46, 0.62)),
        origin=Origin(xyz=(-2.32, 0.0, 0.53)),
        material=utility_green,
        name="service_cabinet",
    )
    base_curb.visual(
        Box((0.24, 0.30, 0.08)),
        origin=Origin(xyz=(-1.98, 0.0, 0.79)),
        material=hardware_dark,
        name="cabinet_adapter_plinth",
    )
    for index in range(8):
        angle = index * (math.tau / 8.0)
        radial = (math.cos(angle), math.sin(angle))
        tangent_angle = angle + (math.pi / 2.0)
        base_curb.visual(
            Box((0.32, 0.20, 0.14)),
            origin=Origin(
                xyz=(1.95 * radial[0], 1.95 * radial[1], 0.74),
                rpy=(0.0, 0.0, angle),
            ),
            material=hardware_dark,
            name=f"bogie_frame_{index:02d}",
        )
        base_curb.visual(
            Cylinder(radius=0.045, length=0.18),
            origin=Origin(
                xyz=(2.06 * radial[0], 2.06 * radial[1], 0.825),
                rpy=(0.0, math.pi / 2.0, tangent_angle),
            ),
            material=fastener_zinc,
            name=f"support_roller_{index:02d}",
        )
        base_curb.visual(
            Cylinder(radius=0.017, length=0.022),
            origin=Origin(
                xyz=(1.95 * radial[0], 1.95 * radial[1], 0.785),
            ),
            material=fastener_zinc,
            name=f"track_bolt_{index:02d}",
        )
    base_curb.inertial = Inertial.from_geometry(
        Box((5.40, 5.40, 0.88)),
        mass=6800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
    )

    rotating_dome = model.part("rotating_dome")
    rotating_dome.visual(
        bearing_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=hardware_dark,
        name="bearing_collar",
    )
    rotating_dome.visual(
        drive_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=hardware_dark,
        name="drive_ring",
    )
    rotating_dome.visual(
        dome_shell_mesh,
        material=dome_paint,
        name="shell_skin",
    )
    rotating_dome.visual(
        skirt_band_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
        material=hardware_dark,
        name="lower_reinforcement_band",
    )
    rotating_dome.visual(
        mid_band_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.58)),
        material=hardware_dark,
        name="mid_reinforcement_band",
    )
    rotating_dome.visual(
        upper_band_mesh,
        origin=Origin(xyz=(0.0, 0.0, 2.25)),
        material=hardware_dark,
        name="upper_reinforcement_band",
    )
    rotating_dome.visual(slit_edge_left_mesh, material=hardware_dark, name="slit_edge_left")
    rotating_dome.visual(slit_edge_right_mesh, material=hardware_dark, name="slit_edge_right")
    rotating_dome.visual(
        Box((0.08, 0.86, 0.08)),
        origin=Origin(xyz=(2.05, 0.0, 0.28)),
        material=hardware_dark,
        name="slit_sill",
    )
    for rib_index, angle in enumerate(RIB_ANGLES):
        rotating_dome.visual(
            meridian_rib_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=hardware_dark,
            name=f"meridian_rib_{rib_index:02d}",
        )
    for index in range(12):
        angle = index * (math.tau / 12.0)
        rotating_dome.visual(
            Cylinder(radius=0.016, length=0.024),
            origin=Origin(
                xyz=(2.10 * math.cos(angle), 2.10 * math.sin(angle), 0.22),
            ),
            material=fastener_zinc,
            name=f"drive_ring_bolt_{index:02d}",
        )
    rotating_dome.inertial = Inertial.from_geometry(
        Cylinder(radius=2.18, length=2.90),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
    )

    main_shutter = model.part("main_shutter")
    main_shutter.visual(
        main_shutter_body_mesh,
        material=dome_paint,
        name="shutter_body",
    )
    main_shutter.inertial = Inertial.from_geometry(
        Box((1.02, 0.72, 1.98)),
        mass=420.0,
        origin=Origin(xyz=(0.82, 0.0, -1.06)),
    )

    crown_shutter = model.part("crown_shutter")
    crown_shutter.visual(
        crown_shutter_body_mesh,
        material=dome_paint,
        name="shutter_body",
    )
    crown_shutter.inertial = Inertial.from_geometry(
        Box((0.32, 0.54, 0.58)),
        mass=130.0,
        origin=Origin(xyz=(0.22, 0.0, -0.16)),
    )

    model.articulation(
        "dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_curb,
        child=rotating_dome,
        origin=Origin(xyz=(0.0, 0.0, 0.87)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.45),
    )
    model.articulation(
        "main_shutter_hinge",
        ArticulationType.REVOLUTE,
        parent=rotating_dome,
        child=main_shutter,
        origin=Origin(xyz=(2.16, 0.0, 2.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.50,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "crown_shutter_hinge",
        ArticulationType.REVOLUTE,
        parent=rotating_dome,
        child=crown_shutter,
        origin=Origin(xyz=(1.24, 0.0, 3.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.60,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_curb = object_model.get_part("base_curb")
    rotating_dome = object_model.get_part("rotating_dome")
    main_shutter = object_model.get_part("main_shutter")
    crown_shutter = object_model.get_part("crown_shutter")

    dome_rotation = object_model.get_articulation("dome_rotation")
    main_shutter_hinge = object_model.get_articulation("main_shutter_hinge")
    crown_shutter_hinge = object_model.get_articulation("crown_shutter_hinge")

    bearing_collar = rotating_dome.get_visual("bearing_collar")
    shell_skin = rotating_dome.get_visual("shell_skin")
    support_roller_0 = base_curb.get_visual("support_roller_00")
    main_body = main_shutter.get_visual("shutter_body")
    crown_body = crown_shutter.get_visual("shutter_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        rotating_dome,
        main_shutter,
        elem_a=shell_skin,
        elem_b=main_body,
        reason="The main shutter seats into the slit rebate and wraps around its integrated hinge support at the dome crown.",
    )
    ctx.allow_overlap(
        rotating_dome,
        crown_shutter,
        elem_a=shell_skin,
        elem_b=crown_body,
        reason="The crown shutter nests into the crown aperture and shares the integrated support cradle at rest.",
    )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(rotating_dome, base_curb, axes="xy", max_dist=0.001)
    ctx.expect_contact(base_curb, rotating_dome, elem_a=support_roller_0, elem_b=bearing_collar)
    ctx.expect_overlap(
        base_curb,
        rotating_dome,
        axes="xy",
        elem_a=support_roller_0,
        elem_b=bearing_collar,
        min_overlap=0.08,
    )
    ctx.expect_overlap(
        rotating_dome,
        main_shutter,
        axes="yz",
        elem_a=shell_skin,
        elem_b=main_body,
        min_overlap=0.08,
    )
    ctx.expect_overlap(
        rotating_dome,
        crown_shutter,
        axes="yz",
        elem_a=shell_skin,
        elem_b=crown_body,
        min_overlap=0.08,
    )

    dome_bounds = ctx.part_world_aabb(rotating_dome)
    main_panel_bounds = ctx.part_element_world_aabb(main_shutter, elem=main_body)
    crown_panel_bounds = ctx.part_element_world_aabb(crown_shutter, elem=crown_body)
    dome_ok = (
        dome_bounds is not None
        and 4.0 <= (dome_bounds[1][0] - dome_bounds[0][0]) <= 4.6
        and 3.5 <= dome_bounds[1][2] <= 4.05
    )
    ctx.check(
        "dome_proportions_read_as_observatory_shell",
        dome_ok,
        details=f"dome_bounds={dome_bounds}",
    )
    main_cover_ok = (
        main_panel_bounds is not None
        and main_panel_bounds[0][2] < 1.45
        and main_panel_bounds[1][2] > 3.0
        and main_panel_bounds[0][1] < -0.36
        and main_panel_bounds[1][1] > 0.36
    )
    ctx.check(
        "main_shutter_covers_lower_slit",
        main_cover_ok,
        details=f"main_panel_bounds={main_panel_bounds}",
    )
    crown_cover_ok = (
        crown_panel_bounds is not None
        and crown_panel_bounds[0][2] < 3.35
        and crown_panel_bounds[1][2] > 3.70
        and crown_panel_bounds[0][1] < -0.24
        and crown_panel_bounds[1][1] > 0.24
    )
    ctx.check(
        "crown_shutter_covers_upper_slit",
        crown_cover_ok,
        details=f"crown_panel_bounds={crown_panel_bounds}",
    )

    main_hinge_rest = ctx.part_world_position(main_shutter)
    with ctx.pose({dome_rotation: math.pi / 2.0}):
        main_hinge_quarter = ctx.part_world_position(main_shutter)
        dome_rotation_ok = (
            main_hinge_rest is not None
            and main_hinge_quarter is not None
            and abs(main_hinge_quarter[0] + main_hinge_rest[1]) < 0.08
            and abs(main_hinge_quarter[1] - main_hinge_rest[0]) < 0.08
        )
        ctx.check(
            "dome_rotation_moves_shell_about_center",
            dome_rotation_ok,
            details=f"rest={main_hinge_rest}, quarter_turn={main_hinge_quarter}",
        )
        ctx.expect_contact(base_curb, rotating_dome, elem_a=support_roller_0, elem_b=bearing_collar)

    main_rest_bounds = ctx.part_element_world_aabb(main_shutter, elem=main_body)
    with ctx.pose({main_shutter_hinge: 1.28}):
        main_open_bounds = ctx.part_element_world_aabb(main_shutter, elem=main_body)
        main_open_center = None if main_open_bounds is None else _aabb_center(main_open_bounds)
        main_rest_center = None if main_rest_bounds is None else _aabb_center(main_rest_bounds)
        main_open_ok = (
            main_rest_center is not None
            and main_open_center is not None
            and main_open_center[0] < main_rest_center[0] - 0.70
            and main_open_bounds[1][2] > main_rest_bounds[1][2] - 0.10
        )
        ctx.check(
            "main_shutter_opens_back_over_crown",
            main_open_ok,
            details=f"rest_center={main_rest_center}, open_center={main_open_center}",
        )

    crown_rest_bounds = ctx.part_element_world_aabb(crown_shutter, elem=crown_body)
    with ctx.pose({crown_shutter_hinge: 1.00}):
        crown_open_bounds = ctx.part_element_world_aabb(crown_shutter, elem=crown_body)
        crown_open_center = None if crown_open_bounds is None else _aabb_center(crown_open_bounds)
        crown_rest_center = None if crown_rest_bounds is None else _aabb_center(crown_rest_bounds)
        crown_open_ok = (
            crown_rest_center is not None
            and crown_open_center is not None
            and crown_open_center[0] < crown_rest_center[0] - 0.10
            and crown_open_bounds[1][2] > crown_rest_bounds[1][2] - 0.08
        )
        ctx.check(
            "crown_shutter_unlatches_and_rolls_back",
            crown_open_ok,
            details=f"rest_center={crown_rest_center}, open_center={crown_open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
