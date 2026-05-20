from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _linspace(start: float, stop: float, count: int) -> list[float]:
    if count <= 1:
        return [start]
    step = (stop - start) / float(count - 1)
    return [start + step * i for i in range(count)]


def _sphere_point(
    radius: float, theta: float, phi: float, skirt_height: float
) -> tuple[float, float, float]:
    ring_radius = radius * math.sin(theta)
    return (
        ring_radius * math.cos(phi),
        ring_radius * math.sin(phi),
        skirt_height + radius * math.cos(theta),
    )


def _cylinder_point(radius: float, phi: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(phi), radius * math.sin(phi), z)


def _add_grid(
    geom: MeshGeometry, rows: list[list[tuple[float, float, float]]], *, flip: bool = False
) -> None:
    if len(rows) < 2 or len(rows[0]) < 2:
        return
    indices = [[geom.add_vertex(*point) for point in row] for row in rows]
    for row_index in range(len(indices) - 1):
        for col_index in range(len(indices[0]) - 1):
            a = indices[row_index][col_index]
            b = indices[row_index + 1][col_index]
            c = indices[row_index + 1][col_index + 1]
            d = indices[row_index][col_index + 1]
            if flip:
                geom.add_face(a, c, b)
                geom.add_face(a, d, c)
            else:
                geom.add_face(a, b, c)
                geom.add_face(a, c, d)


def _translated(
    points: list[tuple[float, float, float]], dx: float, dy: float, dz: float
) -> list[tuple[float, float, float]]:
    return [(x + dx, y + dy, z + dz) for x, y, z in points]


def _curved_shell_sector(
    *,
    radius_outer: float,
    thickness: float,
    skirt_height: float,
    phi_start: float,
    phi_end: float,
    theta_top: float,
    theta_bottom: float,
    z_bottom: float,
    phi_samples: int,
    theta_samples: int,
    skirt_samples: int,
) -> MeshGeometry:
    radius_inner = radius_outer - thickness
    phis = _linspace(phi_start, phi_end, phi_samples)
    thetas = _linspace(theta_top, theta_bottom, theta_samples)
    zs = _linspace(z_bottom, skirt_height, skirt_samples)

    geom = MeshGeometry(vertices=[], faces=[])

    outer_sphere = [
        [_sphere_point(radius_outer, theta, phi, skirt_height) for phi in phis] for theta in thetas
    ]
    inner_sphere = [
        [_sphere_point(radius_inner, theta, phi, skirt_height) for phi in phis] for theta in thetas
    ]
    _add_grid(geom, outer_sphere, flip=False)
    _add_grid(geom, inner_sphere, flip=True)

    top_band = [
        [_sphere_point(radius_outer, theta_top, phi, skirt_height) for phi in phis],
        [_sphere_point(radius_inner, theta_top, phi, skirt_height) for phi in phis],
    ]
    _add_grid(geom, top_band, flip=True)

    if z_bottom < skirt_height - 1e-6:
        outer_skirt = [[_cylinder_point(radius_outer, phi, z) for phi in phis] for z in zs]
        inner_skirt = [[_cylinder_point(radius_inner, phi, z) for phi in phis] for z in zs]
        _add_grid(geom, outer_skirt, flip=True)
        _add_grid(geom, inner_skirt, flip=False)

        bottom_band = [
            [_cylinder_point(radius_outer, phi, z_bottom) for phi in phis],
            [_cylinder_point(radius_inner, phi, z_bottom) for phi in phis],
        ]
        _add_grid(geom, bottom_band, flip=False)

        side_lower_left = [
            [
                _cylinder_point(radius_outer, phi_start, z),
                _cylinder_point(radius_inner, phi_start, z),
            ]
            for z in zs
        ]
        side_lower_right = [
            [_cylinder_point(radius_outer, phi_end, z), _cylinder_point(radius_inner, phi_end, z)]
            for z in zs
        ]
        _add_grid(geom, side_lower_left, flip=True)
        _add_grid(geom, side_lower_right, flip=False)

    side_upper_left = [
        [
            _sphere_point(radius_outer, theta, phi_start, skirt_height),
            _sphere_point(radius_inner, theta, phi_start, skirt_height),
        ]
        for theta in thetas
    ]
    side_upper_right = [
        [
            _sphere_point(radius_outer, theta, phi_end, skirt_height),
            _sphere_point(radius_inner, theta, phi_end, skirt_height),
        ]
        for theta in thetas
    ]
    _add_grid(geom, side_upper_left, flip=True)
    _add_grid(geom, side_upper_right, flip=False)

    return geom


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _edge_path(
    *,
    phi: float,
    radius: float,
    skirt_height: float,
    theta_top: float,
    z_bottom: float,
    proud: float = 0.0,
) -> list[tuple[float, float, float]]:
    points = [
        _sphere_point(radius + proud, theta, phi, skirt_height)
        for theta in _linspace(theta_top, math.pi / 2.0, 7)
    ]
    for z in _linspace(skirt_height - 0.01, z_bottom, 5)[1:]:
        points.append(_cylinder_point(radius + proud, phi, z))
    return points


def _material_library(model: ArticulatedObject) -> dict[str, Material]:
    materials = {
        "painted_shell": Material(name="painted_shell", rgba=(0.94, 0.95, 0.96, 1.0)),
        "concrete": Material(name="concrete", rgba=(0.70, 0.71, 0.73, 1.0)),
        "dark_metal": Material(name="dark_metal", rgba=(0.18, 0.20, 0.23, 1.0)),
        "steel": Material(name="steel", rgba=(0.55, 0.58, 0.62, 1.0)),
        "rubber": Material(name="rubber", rgba=(0.08, 0.08, 0.09, 1.0)),
        "glass": Material(name="glass", rgba=(0.20, 0.29, 0.34, 0.45)),
    }
    model.materials.extend(materials.values())
    return materials


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome", assets=ASSETS)
    materials = _material_library(model)

    foundation_size = (2.08, 2.08, 0.16)
    plinth_radius = 0.74
    plinth_height = 0.12
    drum_radius = 0.66
    drum_height = 0.74
    cornice_radius = 0.72
    cornice_height = 0.08
    rail_radius = 0.77
    rail_height = 0.05
    roof_cap_radius = 0.12
    roof_cap_height = 0.08
    dome_origin_z = 1.15
    dome_running_clearance = 0.008

    dome_outer_radius = 0.72
    dome_thickness = 0.026
    dome_skirt_height = 0.16
    shell_theta_top = 0.16
    shell_theta_bottom = math.pi / 2.0
    slit_half_angle = 0.18
    shutter_half_angle = 0.145
    shell_phi_start = math.pi / 2.0 + slit_half_angle
    shell_phi_end = shell_phi_start + (2.0 * math.pi - 2.0 * slit_half_angle)
    shutter_phi_start = math.pi / 2.0 - shutter_half_angle
    shutter_phi_end = math.pi / 2.0 + shutter_half_angle
    hinge_z = dome_skirt_height + dome_outer_radius * math.cos(shell_theta_top) - 0.01
    slit_bottom_z = 0.03

    base = model.part("base")
    base.visual(
        Box(foundation_size),
        origin=Origin(xyz=(0.0, 0.0, foundation_size[2] / 2.0)),
        material=materials["concrete"],
    )
    base.visual(
        Cylinder(radius=plinth_radius, length=plinth_height),
        origin=Origin(xyz=(0.0, 0.0, foundation_size[2] + plinth_height / 2.0)),
        material=materials["concrete"],
    )
    base.visual(
        Cylinder(radius=drum_radius, length=drum_height),
        origin=Origin(xyz=(0.0, 0.0, foundation_size[2] + plinth_height + drum_height / 2.0)),
        material=materials["painted_shell"],
    )
    base.visual(
        Cylinder(radius=cornice_radius, length=cornice_height),
        origin=Origin(
            xyz=(0.0, 0.0, foundation_size[2] + plinth_height + drum_height + cornice_height / 2.0)
        ),
        material=materials["painted_shell"],
    )
    base.visual(
        Cylinder(radius=rail_radius, length=rail_height),
        origin=Origin(xyz=(0.0, 0.0, dome_origin_z - dome_running_clearance - rail_height / 2.0)),
        material=materials["dark_metal"],
    )
    base.visual(
        Cylinder(radius=roof_cap_radius, length=roof_cap_height),
        origin=Origin(
            xyz=(0.0, 0.0, dome_origin_z - dome_running_clearance - roof_cap_height / 2.0)
        ),
        material=materials["steel"],
    )
    base.visual(
        Box((0.42, 0.28, 0.64)),
        origin=Origin(xyz=(0.0, -0.78, 0.44)),
        material=materials["painted_shell"],
    )
    base.visual(
        Box((0.22, 0.07, 0.44)),
        origin=Origin(xyz=(0.0, -0.90, 0.34)),
        material=materials["dark_metal"],
    )
    base.visual(
        Box((0.08, 0.035, 0.22)),
        origin=Origin(xyz=(-0.13, -0.885, 0.52)),
        material=materials["glass"],
    )
    base.visual(
        Box((0.08, 0.035, 0.22)),
        origin=Origin(xyz=(0.13, -0.885, 0.52)),
        material=materials["glass"],
    )
    base.visual(
        Box((0.38, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, -0.66, 0.76)),
        material=materials["steel"],
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.82, length=dome_origin_z),
        mass=650.0,
        origin=Origin(xyz=(0.0, 0.0, dome_origin_z / 2.0)),
    )

    dome = model.part("dome")
    shell_mesh = _curved_shell_sector(
        radius_outer=dome_outer_radius,
        thickness=dome_thickness,
        skirt_height=dome_skirt_height,
        phi_start=shell_phi_start,
        phi_end=shell_phi_end,
        theta_top=shell_theta_top,
        theta_bottom=shell_theta_bottom,
        z_bottom=0.0,
        phi_samples=72,
        theta_samples=20,
        skirt_samples=6,
    )
    dome.visual(_mesh("dome_shell.obj", shell_mesh), material=materials["painted_shell"])
    dome.visual(
        Cylinder(radius=dome_outer_radius - 0.02, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=materials["dark_metal"],
    )
    dome.visual(
        Cylinder(radius=0.11, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=materials["steel"],
    )
    dome.visual(
        Cylinder(radius=0.018, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["steel"],
    )
    left_rail = tube_from_spline_points(
        _edge_path(
            phi=math.pi / 2.0 - slit_half_angle,
            radius=dome_outer_radius,
            skirt_height=dome_skirt_height,
            theta_top=shell_theta_top,
            z_bottom=slit_bottom_z,
            proud=0.006,
        ),
        radius=0.012,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    right_rail = tube_from_spline_points(
        _edge_path(
            phi=math.pi / 2.0 + slit_half_angle,
            radius=dome_outer_radius,
            skirt_height=dome_skirt_height,
            theta_top=shell_theta_top,
            z_bottom=slit_bottom_z,
            proud=0.006,
        ),
        radius=0.012,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    sill_y = dome_outer_radius * math.cos(slit_half_angle)
    sill = tube_from_spline_points(
        [
            (-0.12, sill_y - 0.01, slit_bottom_z + 0.02),
            (0.0, sill_y, slit_bottom_z + 0.015),
            (0.12, sill_y - 0.01, slit_bottom_z + 0.02),
        ],
        radius=0.012,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    dome.visual(_mesh("dome_left_rail.obj", left_rail), material=materials["steel"])
    dome.visual(_mesh("dome_right_rail.obj", right_rail), material=materials["steel"])
    dome.visual(_mesh("dome_sill.obj", sill), material=materials["steel"])
    dome.visual(
        Box((0.14, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.34, 0.57)),
        material=materials["dark_metal"],
    )
    dome.visual(
        Box((0.18, 0.06, 0.02)),
        origin=Origin(xyz=(0.0, -0.34, 0.62)),
        material=materials["rubber"],
    )
    dome.inertial = Inertial.from_geometry(
        Cylinder(radius=0.75, length=0.92),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    shutter = model.part("shutter")
    shutter_shell = _curved_shell_sector(
        radius_outer=dome_outer_radius - 0.008,
        thickness=0.022,
        skirt_height=dome_skirt_height,
        phi_start=shutter_phi_start,
        phi_end=shutter_phi_end,
        theta_top=shell_theta_top,
        theta_bottom=shell_theta_bottom,
        z_bottom=slit_bottom_z,
        phi_samples=20,
        theta_samples=18,
        skirt_samples=6,
    )
    hinge_offset = (0.0, 0.0, -hinge_z)
    shutter_points_left = _translated(
        _edge_path(
            phi=math.pi / 2.0 - shutter_half_angle,
            radius=dome_outer_radius - 0.004,
            skirt_height=dome_skirt_height,
            theta_top=shell_theta_top,
            z_bottom=slit_bottom_z,
            proud=0.004,
        ),
        *hinge_offset,
    )
    shutter_points_right = _translated(
        _edge_path(
            phi=math.pi / 2.0 + shutter_half_angle,
            radius=dome_outer_radius - 0.004,
            skirt_height=dome_skirt_height,
            theta_top=shell_theta_top,
            z_bottom=slit_bottom_z,
            proud=0.004,
        ),
        *hinge_offset,
    )
    shutter.visual(
        _mesh(
            "shutter_shell.obj",
            MeshGeometry(
                vertices=[(x, y, z - hinge_z) for x, y, z in shutter_shell.vertices],
                faces=list(shutter_shell.faces),
            ),
        ),
        material=materials["painted_shell"],
    )
    shutter.visual(
        Cylinder(radius=0.018, length=0.23),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["steel"],
    )
    shutter.visual(
        _mesh(
            "shutter_left_support.obj",
            tube_from_spline_points(
                [
                    (-0.095, 0.0, 0.0),
                    (-0.09, 0.18, -0.07),
                    (-0.08, 0.42, -0.15),
                    (-0.075, 0.63, -0.21),
                ],
                radius=0.009,
                samples_per_segment=12,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=materials["steel"],
    )
    shutter.visual(
        _mesh(
            "shutter_right_support.obj",
            tube_from_spline_points(
                [
                    (0.095, 0.0, 0.0),
                    (0.09, 0.18, -0.07),
                    (0.08, 0.42, -0.15),
                    (0.075, 0.63, -0.21),
                ],
                radius=0.009,
                samples_per_segment=12,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=materials["steel"],
    )
    shutter.visual(
        _mesh(
            "shutter_left_rib.obj",
            tube_from_spline_points(
                shutter_points_left,
                radius=0.010,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=materials["steel"],
    )
    shutter.visual(
        _mesh(
            "shutter_right_rib.obj",
            tube_from_spline_points(
                shutter_points_right,
                radius=0.010,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=materials["steel"],
    )
    lower_crossbar_y = sill_y - 0.02
    shutter.visual(
        _mesh(
            "shutter_crossbar.obj",
            tube_from_spline_points(
                [
                    (-0.10, lower_crossbar_y, slit_bottom_z + 0.05 - hinge_z),
                    (0.0, lower_crossbar_y + 0.01, slit_bottom_z + 0.055 - hinge_z),
                    (0.10, lower_crossbar_y, slit_bottom_z + 0.05 - hinge_z),
                ],
                radius=0.010,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=materials["steel"],
    )
    shutter.visual(
        Box((0.05, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, lower_crossbar_y + 0.015, slit_bottom_z + 0.02 - hinge_z)),
        material=materials["dark_metal"],
    )
    shutter.inertial = Inertial.from_geometry(
        Box((0.30, 0.24, 0.82)),
        mass=35.0,
        origin=Origin(xyz=(0.0, 0.18, -0.34)),
    )

    model.articulation(
        "base_to_dome",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="dome",
        origin=Origin(xyz=(0.0, 0.0, dome_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35),
    )
    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent="dome",
        child="shutter",
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "dome",
        "shutter",
        reason="The shutter closes flush against the slit frame, and generated collision hulls conservatively thicken the thin shell surfaces.",
    )
    ctx.allow_overlap(
        "base",
        "dome",
        reason="The rotating dome skirt rides very close to the circular support ring and central drive cap; generated collision hulls conservatively report slight interpenetration in this intended near-contact interface.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.005, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("dome", "base", axes="xy", max_dist=0.02)
    ctx.expect_aabb_overlap("dome", "base", axes="xy", min_overlap=1.20)
    ctx.expect_aabb_gap("dome", "base", axis="z", max_gap=0.02, max_penetration=0.01)
    ctx.expect_aabb_overlap("shutter", "dome", axes="xy", min_overlap=0.18)
    ctx.expect_joint_motion_axis(
        "dome_to_shutter", "shutter", world_axis="z", direction="positive", min_delta=0.06
    )

    with ctx.pose({"base_to_dome": math.pi / 2.0}):
        ctx.expect_origin_distance("dome", "base", axes="xy", max_dist=0.02)
        ctx.expect_aabb_overlap("dome", "base", axes="xy", min_overlap=1.20)
        ctx.expect_aabb_gap("dome", "base", axis="z", max_gap=0.02, max_penetration=0.01)

    with ctx.pose({"dome_to_shutter": 1.05}):
        ctx.expect_aabb_overlap("shutter", "dome", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_overlap("shutter", "base", axes="xy", min_overlap=0.18)

    with ctx.pose({"base_to_dome": 2.10, "dome_to_shutter": 1.05}):
        ctx.expect_origin_distance("dome", "base", axes="xy", max_dist=0.02)
        ctx.expect_aabb_overlap("dome", "base", axes="xy", min_overlap=1.20)
        ctx.expect_aabb_overlap("shutter", "dome", axes="xy", min_overlap=0.10)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
