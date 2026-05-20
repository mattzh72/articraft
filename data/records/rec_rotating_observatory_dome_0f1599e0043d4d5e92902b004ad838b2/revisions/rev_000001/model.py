from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    mesh_from_geometry,
    repair_loft,
    section_loft,
    tube_from_spline_points,
)


DOME_OUTER_RADIUS = 2.02
DOME_INNER_RADIUS = 1.94
TRACK_TOP_Z = 0.50
WEAR_RING_TOP_Z = 0.64
APERTURE_INNER_HALF_WIDTH = 0.44
RAIL_CENTER_X = 1.96
RAIL_HALF_SPAN_Y = 0.58
RAIL_HEIGHT = 2.68
RAIL_BOTTOM_Z = 0.64
RAIL_TOP_Z = RAIL_BOTTOM_Z + RAIL_HEIGHT


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _orient(geometry: MeshGeometry, *, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> MeshGeometry:
    rx, ry, rz = rpy
    if rx:
        geometry.rotate_x(rx)
    if ry:
        geometry.rotate_y(ry)
    if rz:
        geometry.rotate_z(rz)
    if xyz != (0.0, 0.0, 0.0):
        geometry.translate(*xyz)
    return geometry


def _box_geom(size, *, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> MeshGeometry:
    return _orient(BoxGeometry(size), xyz=xyz, rpy=rpy)


def _cyl_geom(
    radius: float,
    height: float,
    *,
    xyz=(0.0, 0.0, 0.0),
    rpy=(0.0, 0.0, 0.0),
    radial_segments: int = 28,
) -> MeshGeometry:
    return _orient(
        CylinderGeometry(radius=radius, height=height, radial_segments=radial_segments),
        xyz=xyz,
        rpy=rpy,
    )


def _arc_angles(start_angle: float, end_angle: float, samples: int) -> list[float]:
    return [
        start_angle + ((end_angle - start_angle) * index / (samples - 1))
        for index in range(samples)
    ]


def _sector_loop(
    outer_radius: float,
    inner_radius: float,
    z_pos: float,
    start_angle: float,
    end_angle: float,
    *,
    arc_samples: int = 26,
) -> list[tuple[float, float, float]]:
    outer = [
        (outer_radius * math.cos(angle), outer_radius * math.sin(angle), z_pos)
        for angle in _arc_angles(start_angle, end_angle, arc_samples)
    ]
    inner = [
        (inner_radius * math.cos(angle), inner_radius * math.sin(angle), z_pos)
        for angle in reversed(_arc_angles(start_angle, end_angle, arc_samples))
    ]
    return outer + inner


def _shell_sector(
    *,
    start_angle: float,
    end_angle: float,
    sections: list[tuple[float, float, float]],
    arc_samples: int = 26,
) -> MeshGeometry:
    loops = [
        _sector_loop(outer_radius, inner_radius, z_pos, start_angle, end_angle, arc_samples=arc_samples)
        for z_pos, outer_radius, inner_radius in sections
    ]
    return repair_loft(section_loft(loops), repair="mesh")


def _rotation_bogie(angle: float) -> MeshGeometry:
    bogie = MeshGeometry()
    bogie.merge(
        _box_geom(
            (0.16, 0.30, 0.12),
            xyz=(2.08, 0.0, 0.44),
        )
    )
    bogie.merge(
        _box_geom(
            (0.12, 0.18, 0.12),
            xyz=(2.19, 0.0, 0.56),
        )
    )
    bogie.merge(
        _cyl_geom(
            0.04,
            0.22,
            xyz=(2.10, 0.0, 0.56),
            rpy=(math.pi / 2.0, 0.0, 0.0),
            radial_segments=24,
        )
    )
    bogie.rotate_z(angle)
    return bogie


def _service_hatch(angle: float, *, radius: float, z_center: float) -> MeshGeometry:
    hatch = MeshGeometry()
    hatch.merge(_box_geom((0.022, 0.24, 0.34), xyz=(radius, 0.0, z_center)))
    hatch.merge(_box_geom((0.028, 0.28, 0.38), xyz=(radius - 0.008, 0.0, z_center)))
    hatch.merge(_box_geom((0.05, 0.02, 0.10), xyz=(radius + 0.016, 0.0, z_center)))
    hatch.rotate_z(angle)
    return hatch


def _adapter_plate(y_pos: float, z_center: float) -> MeshGeometry:
    adapter = MeshGeometry()
    adapter.merge(_box_geom((0.18, 0.14, 0.05), xyz=(1.88, y_pos, z_center)))
    for y_offset in (-0.026, 0.026):
        for z_offset in (-0.012, 0.012):
            adapter.merge(
                _cyl_geom(
                    0.018,
                    0.02,
                    xyz=(1.96, y_pos + y_offset, z_center + z_offset),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                    radial_segments=18,
                )
            )
    return adapter


def _build_base_mesh() -> MeshGeometry:
    curb = LatheGeometry.from_shell_profiles(
        [
            (2.34, 0.00),
            (2.34, 0.12),
            (2.26, 0.24),
            (2.17, 0.40),
            (2.10, TRACK_TOP_Z),
        ],
        [
            (1.44, 0.00),
            (1.48, 0.18),
            (1.56, 0.34),
            (1.62, TRACK_TOP_Z),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    rotation_track = LatheGeometry.from_shell_profiles(
        [
            (2.06, 0.44),
            (2.06, TRACK_TOP_Z),
        ],
        [
            (1.90, 0.44),
            (1.90, TRACK_TOP_Z),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    rain_skirt = LatheGeometry.from_shell_profiles(
        [
            (2.18, 0.34),
            (2.18, 0.40),
        ],
        [
            (2.07, 0.34),
            (2.07, 0.40),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    drive_box = _box_geom((0.42, 0.62, 0.28), xyz=(2.18, 0.0, 0.22))
    drive_box.rotate_z(-math.radians(16.0))
    geometries = [
        curb,
        rotation_track,
        rain_skirt,
        drive_box,
        _service_hatch(math.radians(52.0), radius=2.16, z_center=0.24),
        _service_hatch(-math.radians(56.0), radius=2.16, z_center=0.24),
        _rotation_bogie(0.0),
        _rotation_bogie(math.pi / 2.0),
        _rotation_bogie(math.pi),
        _rotation_bogie(3.0 * math.pi / 2.0),
    ]
    return _merge_geometries(geometries)


def _build_dome_mesh() -> MeshGeometry:
    shell_sections = [
        (0.62, DOME_OUTER_RADIUS, DOME_INNER_RADIUS),
        (0.98, 2.02, 1.94),
        (1.38, 1.96, 1.88),
        (1.78, 1.80, 1.72),
        (2.12, 1.50, 1.42),
        (2.40, 1.05, 0.98),
        (2.58, 0.48, 0.40),
    ]
    slit_angle = 0.33
    left_shell = _shell_sector(
        start_angle=slit_angle,
        end_angle=math.pi,
        sections=shell_sections,
        arc_samples=26,
    )
    right_shell = _shell_sector(
        start_angle=math.pi,
        end_angle=(2.0 * math.pi) - slit_angle,
        sections=shell_sections,
        arc_samples=26,
    )
    wear_ring = LatheGeometry.from_shell_profiles(
        [
            (2.02, TRACK_TOP_Z),
            (2.02, WEAR_RING_TOP_Z),
        ],
        [
            (1.90, TRACK_TOP_Z),
            (1.90, WEAR_RING_TOP_Z),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    left_jamb = _box_geom((0.20, 0.18, 1.94), xyz=(1.84, 0.53, 1.61))
    right_jamb = _box_geom((0.20, 0.18, 1.94), xyz=(1.84, -0.53, 1.61))
    crown_bridge = _box_geom((0.24, 1.26, 0.20), xyz=(1.78, 0.0, 2.66))
    rail_top_bridge = _box_geom((0.16, 1.34, 0.16), xyz=(1.88, 0.0, 3.26))
    left_rail = _box_geom((0.04, 0.08, RAIL_HEIGHT), xyz=(RAIL_CENTER_X, RAIL_HALF_SPAN_Y, 1.98))
    right_rail = _box_geom((0.04, 0.08, RAIL_HEIGHT), xyz=(RAIL_CENTER_X, -RAIL_HALF_SPAN_Y, 1.98))
    rear_rib = tube_from_spline_points(
        [
            (-2.00, 0.0, 0.70),
            (-1.86, 0.0, 1.52),
            (-1.16, 0.0, 2.24),
            (-0.38, 0.0, 2.58),
        ],
        radius=0.05,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    left_slit_rib = tube_from_spline_points(
        [
            (1.72, APERTURE_INNER_HALF_WIDTH, 0.68),
            (1.76, APERTURE_INNER_HALF_WIDTH, 1.40),
            (1.70, APERTURE_INNER_HALF_WIDTH, 2.10),
            (1.58, 0.38, 2.62),
        ],
        radius=0.032,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    right_slit_rib = left_slit_rib.copy().scale(1.0, -1.0, 1.0)
    dome = _merge_geometries(
        [
            left_shell,
            right_shell,
            wear_ring,
            left_jamb,
            right_jamb,
            crown_bridge,
            rail_top_bridge,
            left_rail,
            right_rail,
            rear_rib,
            left_slit_rib,
            right_slit_rib,
            _adapter_plate(0.69, 0.88),
            _adapter_plate(-0.69, 0.88),
            _adapter_plate(0.69, 1.82),
            _adapter_plate(-0.69, 1.82),
            _service_hatch(math.pi - 0.76, radius=2.03, z_center=0.96),
            _service_hatch(-(math.pi - 0.76), radius=2.03, z_center=0.96),
        ]
    )
    return dome


def _shutter_mesh(*, bottom_z: float, height: float, name_bias: float) -> MeshGeometry:
    z_center = bottom_z + (height * 0.5)
    top_z = bottom_z + height
    shutter = MeshGeometry()
    shutter.merge(_box_geom((0.08, 0.84, height), xyz=(2.00, 0.0, z_center)))
    shutter.merge(_box_geom((0.06, 0.08, height), xyz=(2.03, 0.44, z_center)))
    shutter.merge(_box_geom((0.06, 0.08, height), xyz=(2.03, -0.44, z_center)))
    shutter.merge(_box_geom((0.10, 0.86, 0.08), xyz=(1.99, 0.0, bottom_z + 0.04)))
    shutter.merge(_box_geom((0.10, 0.86, 0.08), xyz=(1.99, 0.0, top_z - 0.04)))
    shutter.merge(_box_geom((0.08, 0.24, 0.06), xyz=(2.05, 0.0, z_center + name_bias)))
    for y_pos in (RAIL_HALF_SPAN_Y, -RAIL_HALF_SPAN_Y):
        for z_pos in (bottom_z + 0.14, top_z - 0.14):
            shutter.merge(_box_geom((0.06, 0.09, 0.12), xyz=(2.01, y_pos, z_pos)))
            connector_y = 0.52 if y_pos > 0.0 else -0.52
            shutter.merge(_box_geom((0.05, 0.14, 0.10), xyz=(2.03, connector_y, z_pos)))
    return shutter


def _build_lower_shutter_mesh() -> MeshGeometry:
    return _shutter_mesh(bottom_z=0.68, height=0.86, name_bias=0.10)


def _build_upper_shutter_mesh() -> MeshGeometry:
    upper = _shutter_mesh(bottom_z=1.54, height=1.06, name_bias=-0.10)
    upper.merge(_box_geom((0.10, 0.86, 0.10), xyz=(2.00, 0.0, 2.55)))
    return upper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_observatory_dome")

    aged_white = model.material("aged_white", rgba=(0.88, 0.89, 0.86, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.36, 0.39, 0.42, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.66, 0.70, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        _save_mesh("base_ring_structure", _build_base_mesh()),
        material=machinery_gray,
        name="base_structure",
    )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=2.34, length=TRACK_TOP_Z),
        mass=2600.0,
        origin=Origin(xyz=(0.0, 0.0, TRACK_TOP_Z * 0.5)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        _save_mesh("dome_shell_structure", _build_dome_mesh()),
        material=aged_white,
        name="dome_structure",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=2.02, length=2.78),
        mass=980.0,
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
    )

    lower_shutter = model.part("lower_shutter")
    lower_shutter.visual(
        _save_mesh("lower_shutter_structure", _build_lower_shutter_mesh()),
        material=dark_graphite,
        name="lower_shutter_structure",
    )
    lower_shutter.inertial = Inertial.from_geometry(
        Box((0.16, 0.90, 0.88)),
        mass=110.0,
        origin=Origin(xyz=(2.00, 0.0, 1.11)),
    )

    upper_shutter = model.part("upper_shutter")
    upper_shutter.visual(
        _save_mesh("upper_shutter_structure", _build_upper_shutter_mesh()),
        material=galvanized,
        name="upper_shutter_structure",
    )
    upper_shutter.inertial = Inertial.from_geometry(
        Box((0.20, 1.08, 1.10)),
        mass=138.0,
        origin=Origin(xyz=(1.99, 0.0, 2.07)),
    )

    model.articulation(
        "dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2400.0, velocity=0.18),
    )
    model.articulation(
        "lower_shutter_slide",
        ArticulationType.PRISMATIC,
        parent=dome_shell,
        child=lower_shutter,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.12, lower=0.0, upper=0.55),
    )
    model.articulation(
        "upper_shutter_slide",
        ArticulationType.PRISMATIC,
        parent=dome_shell,
        child=upper_shutter,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.12, lower=0.0, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_ring = object_model.get_part("base_ring")
    dome_shell = object_model.get_part("dome_shell")
    lower_shutter = object_model.get_part("lower_shutter")
    upper_shutter = object_model.get_part("upper_shutter")
    dome_rotation = object_model.get_articulation("dome_rotation")
    lower_slide = object_model.get_articulation("lower_shutter_slide")
    upper_slide = object_model.get_articulation("upper_shutter_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.expect_contact(dome_shell, base_ring, name="dome_is_supported_on_rotation_track")
    ctx.expect_overlap(dome_shell, base_ring, axes="xy", min_overlap=3.0, name="dome_sits_concentrically_on_base")
    ctx.expect_contact(lower_shutter, dome_shell, name="lower_shutter_is_carried_by_guide_rails")
    ctx.expect_contact(upper_shutter, dome_shell, name="upper_shutter_is_carried_by_guide_rails")

    with ctx.pose({dome_rotation: math.radians(38.0)}):
        ctx.expect_contact(dome_shell, base_ring, name="rotation_track_stays_seated_when_turned")

    with ctx.pose({lower_slide: 0.0, upper_slide: 0.0}):
        ctx.expect_gap(
            upper_shutter,
            lower_shutter,
            axis="z",
            min_gap=0.0,
            max_gap=0.02,
            name="closed_shutters_meet_without_overlap",
        )

    with ctx.pose({lower_slide: 0.55, upper_slide: 0.80}):
        ctx.expect_contact(lower_shutter, dome_shell, name="lower_shutter_remains_supported_when_open")
        ctx.expect_contact(upper_shutter, dome_shell, name="upper_shutter_remains_supported_when_open")
        ctx.expect_gap(
            upper_shutter,
            lower_shutter,
            axis="z",
            min_gap=0.20,
            max_gap=0.38,
            name="opening_sequence_creates_clear_observing_slit",
        )
        ctx.expect_origin_gap(
            upper_shutter,
            dome_shell,
            axis="z",
            min_gap=0.78,
            max_gap=0.82,
            name="upper_shutter_travels_upward_along_the_rails",
        )
        ctx.expect_origin_gap(
            lower_shutter,
            dome_shell,
            axis="z",
            min_gap=0.53,
            max_gap=0.57,
            name="lower_shutter_travels_upward_along_the_rails",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
