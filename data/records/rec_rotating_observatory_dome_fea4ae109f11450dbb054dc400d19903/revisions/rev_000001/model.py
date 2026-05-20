from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


OUTER_DOME_RADIUS = 2.45
SHELL_THICKNESS = 0.06
INNER_DOME_RADIUS = OUTER_DOME_RADIUS - SHELL_THICKNESS
DOME_CENTER_Z = -0.35
SLIT_HALF_ANGLE = math.radians(10.0)
SHELL_BASE_Z = 0.28
SHELL_TOP_Z = 2.08
SHUTTER_HINGE_Z = 1.36
SHUTTER_TOP_Z = 2.02
HINGE_OUTBOARD = 0.08


def _linspace(start: float, stop: float, samples: int) -> list[float]:
    if samples <= 1:
        return [start]
    return [start + ((stop - start) * index / (samples - 1)) for index in range(samples)]


def _radial_extent(radius: float, z_value: float, center_z: float) -> float:
    return math.sqrt(max(radius * radius - (z_value - center_z) * (z_value - center_z), 0.0))


def _spherical_meridian_loop(theta: float, z_values: list[float]) -> list[tuple[float, float, float]]:
    outer_points: list[tuple[float, float, float]] = []
    inner_points: list[tuple[float, float, float]] = []
    for z_value in z_values:
        outer_r = _radial_extent(OUTER_DOME_RADIUS, z_value, DOME_CENTER_Z)
        inner_r = _radial_extent(INNER_DOME_RADIUS, z_value, DOME_CENTER_Z)
        outer_points.append((outer_r * math.cos(theta), outer_r * math.sin(theta), z_value))
        inner_points.append((inner_r * math.cos(theta), inner_r * math.sin(theta), z_value))
    return outer_points + list(reversed(inner_points))


def _cylindrical_meridian_loop(
    theta: float,
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
) -> list[tuple[float, float, float]]:
    return [
        (outer_radius * math.cos(theta), outer_radius * math.sin(theta), z_min),
        (outer_radius * math.cos(theta), outer_radius * math.sin(theta), z_max),
        (inner_radius * math.cos(theta), inner_radius * math.sin(theta), z_max),
        (inner_radius * math.cos(theta), inner_radius * math.sin(theta), z_min),
    ]


def _build_spherical_shell_band(theta_start: float, theta_end: float):
    z_values = _linspace(SHELL_BASE_Z, SHELL_TOP_Z, 18)
    sections = [
        _spherical_meridian_loop(theta, z_values)
        for theta in _linspace(theta_start, theta_end, 17)
    ]
    return section_loft(sections)


def _build_rotating_curb(theta_start: float, theta_end: float):
    sections = [
        _cylindrical_meridian_loop(
            theta,
            outer_radius=2.37,
            inner_radius=2.31,
            z_min=0.0,
            z_max=0.32,
        )
        for theta in _linspace(theta_start, theta_end, 17)
    ]
    return section_loft(sections)


def _build_annular_ring(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _leaf_slice_loop(
    z_value: float,
    *,
    x_hinge: float,
    outboard_offset: float = 0.0,
    y_scale: float = 0.78,
    span_samples: int = 7,
) -> list[tuple[float, float, float]]:
    outer_r = _radial_extent(OUTER_DOME_RADIUS, z_value, DOME_CENTER_Z)
    inner_r = _radial_extent(INNER_DOME_RADIUS, z_value, DOME_CENTER_Z)
    outer_half_width = outer_r * math.sin(SLIT_HALF_ANGLE) * y_scale
    inner_half_width = inner_r * math.sin(SLIT_HALF_ANGLE) * (y_scale - 0.02)

    outer_points: list[tuple[float, float, float]] = []
    inner_points: list[tuple[float, float, float]] = []
    for fraction in _linspace(-1.0, 1.0, span_samples):
        y_value = outer_half_width * fraction
        x_value = math.sqrt(max(outer_r * outer_r - y_value * y_value, 0.0)) - x_hinge - outboard_offset
        outer_points.append((x_value, y_value, z_value - SHUTTER_HINGE_Z))
    for fraction in reversed(_linspace(-1.0, 1.0, span_samples)):
        y_value = inner_half_width * fraction
        x_value = math.sqrt(max(inner_r * inner_r - y_value * y_value, 0.0)) - x_hinge - outboard_offset
        inner_points.append((x_value, y_value, z_value - SHUTTER_HINGE_Z))
    return outer_points + inner_points


def _build_shutter_leaf():
    x_hinge = _radial_extent(OUTER_DOME_RADIUS, SHUTTER_HINGE_Z, DOME_CENTER_Z)
    z_values = _linspace(SHUTTER_HINGE_Z, SHUTTER_TOP_Z, 12)
    sections = [
        _leaf_slice_loop(z_value, x_hinge=x_hinge, outboard_offset=HINGE_OUTBOARD)
        for z_value in z_values
    ]
    return section_loft(sections)


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.70, 1.0))
    white_shell = model.material("white_shell", rgba=(0.93, 0.94, 0.95, 1.0))
    track_steel = model.material("track_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.18, 0.19, 0.21, 1.0))
    black_glass = model.material("black_glass", rgba=(0.08, 0.09, 0.10, 1.0))

    slit_theta_start = SLIT_HALF_ANGLE
    slit_theta_end = (2.0 * math.pi) - SLIT_HALF_ANGLE

    foundation = model.part("foundation")
    foundation.visual(
        Cylinder(radius=2.95, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=concrete,
        name="ground_slab",
    )
    foundation.visual(
        mesh_from_geometry(
            _build_annular_ring(
                outer_radius=2.58,
                inner_radius=2.28,
                z_min=0.24,
                z_max=0.92,
            ),
            "base_ring_wall",
        ),
        origin=Origin(),
        material=concrete,
        name="base_ring_wall",
    )
    foundation.visual(
        mesh_from_geometry(
            _build_annular_ring(
                outer_radius=2.64,
                inner_radius=2.34,
                z_min=0.92,
                z_max=1.08,
            ),
            "bearing_plinth",
        ),
        origin=Origin(),
        material=concrete,
        name="bearing_plinth",
    )
    foundation.visual(
        mesh_from_geometry(
            _build_annular_ring(
                outer_radius=2.46,
                inner_radius=2.30,
                z_min=1.06,
                z_max=1.14,
            ),
            "bearing_track",
        ),
        origin=Origin(),
        material=track_steel,
        name="bearing_track",
    )
    foundation.visual(
        mesh_from_geometry(
            _build_annular_ring(
                outer_radius=2.30,
                inner_radius=0.95,
                z_min=0.92,
                z_max=0.96,
            ),
            "observation_floor",
        ),
        origin=Origin(),
        material=dark_frame,
        name="observation_floor",
    )
    foundation.visual(
        Box((0.68, 0.18, 0.46)),
        origin=Origin(xyz=(2.24, 0.0, 0.47)),
        material=dark_frame,
        name="service_door_surround",
    )
    foundation.visual(
        Box((0.58, 0.08, 0.32)),
        origin=Origin(xyz=(2.27, 0.0, 0.40)),
        material=black_glass,
        name="service_door_recess",
    )
    foundation.inertial = Inertial.from_geometry(
        Box((5.9, 5.9, 1.14)),
        mass=12500.0,
        origin=Origin(xyz=(0.0, 0.0, 0.57)),
    )

    support_pier = model.part("support_pier")
    support_pier.visual(
        Cylinder(radius=0.30, length=1.54),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material=concrete,
        name="main_pier",
    )
    support_pier.visual(
        Cylinder(radius=0.38, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
        material=dark_frame,
        name="instrument_pedestal",
    )
    support_pier.inertial = Inertial.from_geometry(
        Cylinder(radius=0.32, length=1.70),
        mass=1600.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        mesh_from_geometry(_build_rotating_curb(slit_theta_start, slit_theta_end), "rotating_curb"),
        material=white_shell,
        name="rotating_curb",
    )
    dome_shell.visual(
        mesh_from_geometry(
            _build_spherical_shell_band(slit_theta_start, slit_theta_end),
            "observatory_shell",
        ),
        material=white_shell,
        name="main_shell",
    )
    dome_shell.visual(
        Cylinder(radius=0.05, length=0.06),
        origin=Origin(
            xyz=(
                _radial_extent(OUTER_DOME_RADIUS, SHUTTER_HINGE_Z, DOME_CENTER_Z) + HINGE_OUTBOARD,
                -0.36,
                SHUTTER_HINGE_Z,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_frame,
        name="hinge_lug_left",
    )
    dome_shell.visual(
        Cylinder(radius=0.05, length=0.06),
        origin=Origin(
            xyz=(
                _radial_extent(OUTER_DOME_RADIUS, SHUTTER_HINGE_Z, DOME_CENTER_Z) + HINGE_OUTBOARD,
                0.36,
                SHUTTER_HINGE_Z,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_frame,
        name="hinge_lug_right",
    )
    dome_shell.visual(
        Box((0.08, 0.10, 0.16)),
        origin=Origin(
            xyz=(
                _radial_extent(OUTER_DOME_RADIUS, SHUTTER_HINGE_Z, DOME_CENTER_Z) - 0.01,
                -0.32,
                SHUTTER_HINGE_Z,
            )
        ),
        material=dark_frame,
        name="hinge_bracket_left",
    )
    dome_shell.visual(
        Box((0.08, 0.10, 0.16)),
        origin=Origin(
            xyz=(
                _radial_extent(OUTER_DOME_RADIUS, SHUTTER_HINGE_Z, DOME_CENTER_Z) - 0.01,
                0.32,
                SHUTTER_HINGE_Z,
            )
        ),
        material=dark_frame,
        name="hinge_bracket_right",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Box((4.9, 4.9, 2.18)),
        mass=2300.0,
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        Cylinder(radius=0.045, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_frame,
        name="hinge_barrel",
    )
    shutter_leaf.visual(
        Box((0.05, 0.42, 0.08)),
        origin=Origin(xyz=(-0.065, 0.0, 0.04)),
        material=dark_frame,
        name="hinge_flange",
    )
    shutter_leaf.visual(
        mesh_from_geometry(_build_shutter_leaf(), "shutter_leaf"),
        material=white_shell,
        name="leaf_shell",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 0.74)),
        mass=120.0,
        origin=Origin(xyz=(-0.12, 0.0, 0.33)),
    )

    model.articulation(
        "foundation_to_support_pier",
        ArticulationType.FIXED,
        parent=foundation,
        child=support_pier,
        origin=Origin(xyz=(-0.46, 0.24, 0.24)),
    )
    model.articulation(
        "dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=foundation,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35000.0, velocity=0.35),
    )
    model.articulation(
        "slit_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(
            xyz=(
                _radial_extent(OUTER_DOME_RADIUS, SHUTTER_HINGE_Z, DOME_CENTER_Z) + HINGE_OUTBOARD,
                0.0,
                SHUTTER_HINGE_Z,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(84.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foundation = object_model.get_part("foundation")
    support_pier = object_model.get_part("support_pier")
    dome_shell = object_model.get_part("dome_shell")
    shutter_leaf = object_model.get_part("shutter_leaf")
    dome_rotation = object_model.get_articulation("dome_rotation")
    slit_shutter = object_model.get_articulation("slit_shutter")

    ctx.expect_origin_distance(
        dome_shell,
        support_pier,
        axes="xy",
        min_dist=0.48,
        name="rotating dome axis stays offset from support pier",
    )

    ctx.expect_gap(
        dome_shell,
        foundation,
        axis="z",
        min_gap=0.0,
        max_gap=0.02,
        positive_elem="rotating_curb",
        negative_elem="bearing_track",
        name="rotating curb sits on the bearing track",
    )

    with ctx.pose({dome_rotation: math.pi / 2.0}):
        ctx.expect_origin_distance(
            dome_shell,
            support_pier,
            axes="xy",
            min_dist=0.48,
            name="offset pier remains clear after dome rotation",
        )

    closed_aabb = ctx.part_element_world_aabb(shutter_leaf, elem="leaf_shell")
    with ctx.pose({slit_shutter: math.radians(78.0)}):
        open_aabb = ctx.part_element_world_aabb(shutter_leaf, elem="leaf_shell")

    closed_center = _aabb_center(closed_aabb)
    open_center = _aabb_center(open_aabb)
    ctx.check(
        "shutter opens outward and upward",
        closed_center is not None
        and open_center is not None
        and open_center[0] > closed_center[0] + 0.12
        and open_center[2] > closed_center[2] + 0.10,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
