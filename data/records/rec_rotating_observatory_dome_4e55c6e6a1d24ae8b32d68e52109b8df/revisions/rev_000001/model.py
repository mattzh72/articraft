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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


DOME_OUTER_RADIUS = 2.08
DOME_INNER_RADIUS = 2.00
DOME_SPHERE_CENTER_Z = 0.30
SLIT_THETA_TOP = 0.30
SLIT_THETA_BOTTOM = 1.18
SLIT_PHI_HALF = 0.22
HINGE_BARREL_RADIUS = 0.03
HINGE_BARREL_LENGTH = 0.14


def _linspace(start: float, stop: float, count: int) -> list[float]:
    if count <= 1:
        return [start]
    step = (stop - start) / float(count - 1)
    return [start + step * i for i in range(count)]


def _merge_meshes(*meshes: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for mesh in meshes:
        merged.merge(mesh)
    return merged


def _surface_from_grid(
    rows: list[list[tuple[float, float, float]]],
    *,
    reverse: bool = False,
) -> MeshGeometry:
    mesh = MeshGeometry()
    indices: list[list[int]] = []
    for row in rows:
        indices.append([mesh.add_vertex(*point) for point in row])

    for row_index in range(len(indices) - 1):
        for col_index in range(len(indices[row_index]) - 1):
            a = indices[row_index][col_index]
            b = indices[row_index][col_index + 1]
            c = indices[row_index + 1][col_index]
            d = indices[row_index + 1][col_index + 1]
            if reverse:
                mesh.add_face(a, c, b)
                mesh.add_face(b, c, d)
            else:
                mesh.add_face(a, b, c)
                mesh.add_face(b, d, c)
    return mesh


def _sphere_point(radius: float, theta: float, phi: float, z_center: float) -> tuple[float, float, float]:
    sin_theta = math.sin(theta)
    return (
        radius * sin_theta * math.cos(phi),
        radius * sin_theta * math.sin(phi),
        z_center + radius * math.cos(theta),
    )


def _grid_surface(
    u_values: list[float],
    v_values: list[float],
    point_fn,
    *,
    reverse: bool = False,
) -> MeshGeometry:
    rows = []
    for u in u_values:
        rows.append([point_fn(u, v) for v in v_values])
    return _surface_from_grid(rows, reverse=reverse)


def _lathe_ring_band(
    radius_inner: float,
    radius_outer: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    return LatheGeometry(
        [
            (radius_inner, z_min),
            (radius_outer, z_min),
            (radius_outer, z_max),
            (radius_inner, z_max),
        ],
        segments=segments,
    )


def _build_dome_shell_mesh() -> MeshGeometry:
    outer_radius = DOME_OUTER_RADIUS
    inner_radius = DOME_INNER_RADIUS
    sphere_center_z = DOME_SPHERE_CENTER_Z
    theta_top = SLIT_THETA_TOP
    theta_bottom = SLIT_THETA_BOTTOM
    theta_base = math.pi / 2.0
    phi_half = SLIT_PHI_HALF

    theta_top_band = _linspace(0.0, theta_top, 10)
    theta_mid_band = _linspace(theta_top, theta_bottom, 20)
    theta_lower_band = _linspace(theta_bottom, theta_base, 10)
    theta_side = _linspace(theta_top, theta_bottom, 20)
    phi_full = _linspace(0.0, math.tau, 73)
    phi_rear = _linspace(phi_half, math.tau - phi_half, 69)
    phi_opening = _linspace(-phi_half, phi_half, 19)
    radii = [outer_radius, inner_radius]

    outer_top = _grid_surface(
        theta_top_band,
        phi_full,
        lambda theta, phi: _sphere_point(outer_radius, theta, phi, sphere_center_z),
    )
    outer_mid = _grid_surface(
        theta_mid_band,
        phi_rear,
        lambda theta, phi: _sphere_point(outer_radius, theta, phi, sphere_center_z),
    )
    outer_lower = _grid_surface(
        theta_lower_band,
        phi_full,
        lambda theta, phi: _sphere_point(outer_radius, theta, phi, sphere_center_z),
    )

    inner_top = _grid_surface(
        theta_top_band,
        phi_full,
        lambda theta, phi: _sphere_point(inner_radius, theta, phi, sphere_center_z),
        reverse=True,
    )
    inner_mid = _grid_surface(
        theta_mid_band,
        phi_rear,
        lambda theta, phi: _sphere_point(inner_radius, theta, phi, sphere_center_z),
        reverse=True,
    )
    inner_lower = _grid_surface(
        theta_lower_band,
        phi_full,
        lambda theta, phi: _sphere_point(inner_radius, theta, phi, sphere_center_z),
        reverse=True,
    )

    opening_top_wall = _grid_surface(
        radii,
        phi_opening,
        lambda radius, phi: _sphere_point(radius, theta_top, phi, sphere_center_z),
    )
    opening_bottom_wall = _grid_surface(
        radii,
        phi_opening,
        lambda radius, phi: _sphere_point(radius, theta_bottom, phi, sphere_center_z),
        reverse=True,
    )
    opening_right_wall = _grid_surface(
        radii,
        theta_side,
        lambda radius, theta: _sphere_point(radius, theta, phi_half, sphere_center_z),
    )
    opening_left_wall = _grid_surface(
        radii,
        theta_side,
        lambda radius, theta: _sphere_point(radius, theta, -phi_half, sphere_center_z),
        reverse=True,
    )

    return _merge_meshes(
        outer_top,
        outer_mid,
        outer_lower,
        inner_top,
        inner_mid,
        inner_lower,
        opening_top_wall,
        opening_bottom_wall,
        opening_right_wall,
        opening_left_wall,
    )


def _build_shutter_leaf_mesh() -> MeshGeometry:
    sphere_center_z = DOME_SPHERE_CENTER_Z
    theta_top = SLIT_THETA_TOP
    theta_bottom = 0.88
    phi_half_top = 0.09
    phi_half_bottom = 0.19
    inner_radius = DOME_OUTER_RADIUS + 0.015
    outer_radius = inner_radius + 0.05

    theta_panel = _linspace(theta_top, theta_bottom, 30)
    span_samples = _linspace(-1.0, 1.0, 21)
    radius_samples = [inner_radius, outer_radius]

    def phi_limit(theta: float) -> float:
        progress = (theta - theta_top) / (theta_bottom - theta_top)
        return phi_half_top + (phi_half_bottom - phi_half_top) * (progress**0.9)

    def shutter_point(radius: float, theta: float, span: float) -> tuple[float, float, float]:
        return _sphere_point(radius, theta, span * phi_limit(theta), sphere_center_z)

    outer_skin = _grid_surface(
        theta_panel,
        span_samples,
        lambda theta, span: shutter_point(outer_radius, theta, span),
    )
    inner_skin = _grid_surface(
        theta_panel,
        span_samples,
        lambda theta, span: shutter_point(inner_radius, theta, span),
        reverse=True,
    )
    top_cap = _grid_surface(
        radius_samples,
        span_samples,
        lambda radius, span: shutter_point(radius, theta_top, span),
    )
    bottom_cap = _grid_surface(
        radius_samples,
        span_samples,
        lambda radius, span: shutter_point(radius, theta_bottom, span),
        reverse=True,
    )
    right_side = _grid_surface(
        radius_samples,
        theta_panel,
        lambda radius, theta: _sphere_point(radius, theta, phi_limit(theta), sphere_center_z),
    )
    left_side = _grid_surface(
        radius_samples,
        theta_panel,
        lambda radius, theta: _sphere_point(radius, theta, -phi_limit(theta), sphere_center_z),
        reverse=True,
    )
    leaf_mesh = _merge_meshes(outer_skin, inner_skin, top_cap, bottom_cap, right_side, left_side)
    barrel_center = _sphere_point(
        DOME_OUTER_RADIUS + HINGE_BARREL_RADIUS,
        theta_top,
        0.0,
        sphere_center_z,
    )
    leaf_mesh.translate(-barrel_center[0], -barrel_center[1], -barrel_center[2])
    return leaf_mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.66, 1.0))
    rail_gray = model.material("rail_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    dome_white = model.material("dome_white", rgba=(0.90, 0.92, 0.94, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        Cylinder(radius=2.70, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="foundation_plinth",
    )
    base_ring.visual(
        mesh_from_geometry(_lathe_ring_band(1.48, 2.46, 0.18, 0.54), "base_annulus"),
        material=concrete,
        name="support_annulus",
    )
    base_ring.visual(
        mesh_from_geometry(_lathe_ring_band(1.94, 2.24, 0.54, 0.60), "support_track"),
        material=rail_gray,
        name="support_track",
    )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=2.70, length=0.60),
        mass=7800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        mesh_from_geometry(_lathe_ring_band(2.00, 2.08, 0.00, 0.30), "rotating_skirt"),
        material=dome_white,
        name="rotating_skirt",
    )
    dome_shell.visual(
        mesh_from_geometry(_build_dome_shell_mesh(), "dome_shell"),
        material=dome_white,
        name="dome_shell",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Box((4.16, 4.16, 2.38)),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.19)),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_gray,
        name="hinge_barrel",
    )
    shutter_leaf.visual(
        mesh_from_geometry(_build_shutter_leaf_mesh(), "shutter_leaf"),
        material=dome_white,
        name="shutter_panel",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((1.10, 0.50, 0.88)),
        mass=160.0,
        origin=Origin(xyz=(0.50, 0.0, -0.31)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.6),
    )
    model.articulation(
        "shutter_hinge",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(
            xyz=(
                (DOME_OUTER_RADIUS + HINGE_BARREL_RADIUS) * math.sin(SLIT_THETA_TOP),
                0.0,
                DOME_SPHERE_CENTER_Z
                + (DOME_OUTER_RADIUS + HINGE_BARREL_RADIUS) * math.cos(SLIT_THETA_TOP),
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.8,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base_ring = object_model.get_part("base_ring")
    dome_shell = object_model.get_part("dome_shell")
    shutter_leaf = object_model.get_part("shutter_leaf")
    azimuth = object_model.get_articulation("azimuth_rotation")
    shutter_hinge = object_model.get_articulation("shutter_hinge")

    ctx.expect_origin_distance(
        base_ring,
        dome_shell,
        axes="xy",
        max_dist=1e-6,
        name="dome stays concentric on the support ring",
    )
    ctx.expect_gap(
        dome_shell,
        base_ring,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="rotating shell rides just above the support ring",
    )
    ctx.expect_overlap(
        dome_shell,
        base_ring,
        axes="xy",
        min_overlap=3.9,
        name="support ring footprint sits under the dome shell",
    )

    with ctx.pose({azimuth: 1.1}):
        ctx.expect_gap(
            dome_shell,
            base_ring,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            name="azimuth rotation preserves shell support height",
        )

    ctx.expect_contact(
        shutter_leaf,
        dome_shell,
        name="closed shutter is seated on the slit frame",
    )

    def element_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    shutter_rest = element_center(ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel"))
    with ctx.pose({shutter_hinge: shutter_hinge.motion_limits.upper}):
        shutter_open = element_center(ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel"))

    ctx.check(
        "shutter opens upward from the crown slit",
        shutter_rest is not None
        and shutter_open is not None
        and shutter_open[2] > shutter_rest[2] + 0.18
        and abs(shutter_open[1] - shutter_rest[1]) < 0.02,
        details=f"rest={shutter_rest}, open={shutter_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
