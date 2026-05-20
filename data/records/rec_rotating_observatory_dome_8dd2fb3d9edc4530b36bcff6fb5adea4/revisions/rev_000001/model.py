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
    MeshGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _angle_delta(angle: float, reference: float) -> float:
    return (angle - reference + math.pi) % (2.0 * math.pi) - math.pi


def _radius_at_z(profile: list[tuple[float, float]], z: float) -> float:
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + (r1 - r0) * t
    if z <= profile[0][1]:
        return profile[0][0]
    return profile[-1][0]


def _slit_half_angle(
    z: float,
    *,
    z_bottom: float,
    z_top: float,
    lower_half_angle: float,
    upper_half_angle: float,
) -> float:
    if z <= z_bottom:
        return lower_half_angle
    if z >= z_top:
        return upper_half_angle
    t = (z - z_bottom) / (z_top - z_bottom)
    return lower_half_angle + (upper_half_angle - lower_half_angle) * t


def _filter_slit_region(
    geometry: MeshGeometry,
    *,
    keep_inside_slit: bool,
    center_angle: float,
    z_bottom: float,
    z_top: float,
    lower_half_angle: float,
    upper_half_angle: float,
) -> MeshGeometry:
    kept_faces: list[tuple[int, int, int]] = []
    for face in geometry.faces:
        vertices = [geometry.vertices[index] for index in face]
        centroid_x = sum(vertex[0] for vertex in vertices) / 3.0
        centroid_y = sum(vertex[1] for vertex in vertices) / 3.0
        centroid_z = sum(vertex[2] for vertex in vertices) / 3.0
        centroid_angle = math.atan2(centroid_y, centroid_x)
        centroid_half_angle = _slit_half_angle(
            centroid_z,
            z_bottom=z_bottom,
            z_top=z_top,
            lower_half_angle=lower_half_angle,
            upper_half_angle=upper_half_angle,
        )
        inside_slit = (
            z_bottom <= centroid_z <= z_top
            and abs(_angle_delta(centroid_angle, center_angle)) <= centroid_half_angle
        )
        if not inside_slit:
            for vertex_x, vertex_y, vertex_z in vertices:
                vertex_half_angle = _slit_half_angle(
                    vertex_z,
                    z_bottom=z_bottom,
                    z_top=z_top,
                    lower_half_angle=lower_half_angle,
                    upper_half_angle=upper_half_angle,
                )
                if (
                    z_bottom <= vertex_z <= z_top
                    and abs(_angle_delta(math.atan2(vertex_y, vertex_x), center_angle))
                    <= vertex_half_angle
                ):
                    inside_slit = True
                    break
        if inside_slit == keep_inside_slit:
            kept_faces.append(face)

    used_indices = sorted({index for face in kept_faces for index in face})
    index_map = {old_index: new_index for new_index, old_index in enumerate(used_indices)}
    return MeshGeometry(
        vertices=[geometry.vertices[index] for index in used_indices],
        faces=[tuple(index_map[index] for index in face) for face in kept_faces],
    )


def _add_quad(
    geometry: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _build_shutter_leaf_geometry(
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    center_angle: float,
    z_bottom: float,
    z_top: float,
    lower_half_angle: float,
    upper_half_angle: float,
    angle_clearance: float,
    outer_clearance: float,
    inner_clearance: float,
    hinge_y: float,
    hinge_z: float,
    z_samples: int = 10,
    angle_samples: int = 12,
) -> MeshGeometry:
    geometry = MeshGeometry()
    outer_rows: list[list[int]] = []
    inner_rows: list[list[int]] = []

    for row in range(z_samples):
        t_z = row / (z_samples - 1)
        z = z_bottom + (z_top - z_bottom) * t_z
        half_angle = (
            _slit_half_angle(
                z,
                z_bottom=z_bottom,
                z_top=z_top,
                lower_half_angle=lower_half_angle,
                upper_half_angle=upper_half_angle,
            )
            - angle_clearance
        )
        outer_radius = _radius_at_z(outer_profile, z) - outer_clearance
        inner_radius = _radius_at_z(inner_profile, z) + inner_clearance
        row_outer: list[int] = []
        row_inner: list[int] = []
        for col in range(angle_samples):
            t_a = col / (angle_samples - 1)
            angle = center_angle - half_angle + (2.0 * half_angle * t_a)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            row_outer.append(
                geometry.add_vertex(
                    outer_radius * cos_a,
                    outer_radius * sin_a - hinge_y,
                    z - hinge_z,
                )
            )
            row_inner.append(
                geometry.add_vertex(
                    inner_radius * cos_a,
                    inner_radius * sin_a - hinge_y,
                    z - hinge_z,
                )
            )
        outer_rows.append(row_outer)
        inner_rows.append(row_inner)

    for row in range(z_samples - 1):
        for col in range(angle_samples - 1):
            _add_quad(
                geometry,
                outer_rows[row][col],
                outer_rows[row][col + 1],
                outer_rows[row + 1][col + 1],
                outer_rows[row + 1][col],
            )
            _add_quad(
                geometry,
                inner_rows[row + 1][col],
                inner_rows[row + 1][col + 1],
                inner_rows[row][col + 1],
                inner_rows[row][col],
            )

    for row in range(z_samples - 1):
        _add_quad(
            geometry,
            outer_rows[row][0],
            outer_rows[row + 1][0],
            inner_rows[row + 1][0],
            inner_rows[row][0],
        )
        _add_quad(
            geometry,
            outer_rows[row + 1][-1],
            outer_rows[row][-1],
            inner_rows[row][-1],
            inner_rows[row + 1][-1],
        )

    for col in range(angle_samples - 1):
        _add_quad(
            geometry,
            outer_rows[0][col + 1],
            outer_rows[0][col],
            inner_rows[0][col],
            inner_rows[0][col + 1],
        )
        _add_quad(
            geometry,
            outer_rows[-1][col],
            outer_rows[-1][col + 1],
            inner_rows[-1][col + 1],
            inner_rows[-1][col],
        )

    return geometry


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.70, 0.71, 0.73, 1.0))
    off_white = model.material("off_white", rgba=(0.92, 0.93, 0.95, 1.0))
    dome_gray = model.material("dome_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.36, 0.39, 1.0))

    base = model.part("base")
    base.visual(
        Box((6.4, 6.4, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=concrete,
        name="foundation_slab",
    )
    base.visual(
        Cylinder(radius=2.95, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=off_white,
        name="base_ring",
    )
    base.visual(
        Cylinder(radius=1.10, length=2.25),
        origin=Origin(xyz=(0.0, 0.0, 2.085)),
        material=off_white,
        name="support_drum",
    )
    base.visual(
        Cylinder(radius=2.25, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 3.35)),
        material=steel,
        name="support_deck",
    )
    base.visual(
        Cylinder(radius=2.52, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 3.57)),
        material=steel,
        name="bearing_track",
    )
    base.inertial = Inertial.from_geometry(
        Box((6.4, 6.4, 3.65)),
        mass=22000.0,
        origin=Origin(xyz=(0.0, 0.0, 1.825)),
    )

    dome_shell = model.part("dome_shell")
    dome_ring_z = 0.18
    dome_radius = 2.60
    slit_center_angle = math.pi / 2.0
    slit_bottom_z = 1.98
    slit_top_z = 3.12
    slit_half_angle_bottom = 0.19
    slit_half_angle_top = 0.09
    shell_outer = [
        (2.50, dome_ring_z),
        (2.58, 0.42),
        (2.66, 0.92),
        (2.48, 1.55),
        (2.05, 2.22),
        (1.45, 2.74),
        (0.82, 3.02),
        (0.52, 3.18),
        (0.30, 3.24),
    ]
    shell_inner = [
        (2.38, dome_ring_z + 0.02),
        (2.45, 0.42),
        (2.52, 0.90),
        (2.35, 1.48),
        (1.95, 2.12),
        (1.38, 2.61),
        (0.77, 2.88),
        (0.46, 3.02),
        (0.20, 3.08),
    ]
    full_shell_geometry = LatheGeometry.from_shell_profiles(
        shell_outer,
        shell_inner,
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    shell_geometry = _filter_slit_region(
        full_shell_geometry,
        keep_inside_slit=False,
        center_angle=slit_center_angle,
        z_bottom=slit_bottom_z,
        z_top=slit_top_z,
        lower_half_angle=slit_half_angle_bottom,
        upper_half_angle=slit_half_angle_top,
    )
    dome_mesh = mesh_from_geometry(shell_geometry, "dome_shell_opening")
    dome_shell.visual(
        Cylinder(radius=2.50, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=steel,
        name="rotation_skirt",
    )
    dome_shell.visual(
        dome_mesh,
        material=dome_gray,
        name="shell_body",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=dome_radius, length=3.50),
        mass=4800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.75)),
    )

    model.articulation(
        "base_to_dome",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 3.65)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.35),
    )

    shutter_leaf = model.part("shutter_leaf")
    hinge_z = 2.04
    hinge_surface_y = _radius_at_z(shell_outer, hinge_z)
    hinge_barrel_radius = 0.04
    hinge_barrel_length = 0.76
    hinge_axis_y = hinge_surface_y + hinge_barrel_radius
    hinge_cheek_size = (0.08, 0.11, 0.14)
    hinge_cheek_offset_x = (hinge_barrel_length + hinge_cheek_size[0]) / 2.0
    hinge_cheek_center_y = hinge_axis_y - (hinge_cheek_size[1] / 2.0)
    hinge_cheek_center_z = hinge_z + 0.02
    dome_shell.visual(
        Box(hinge_cheek_size),
        origin=Origin(
            xyz=(-hinge_cheek_offset_x, hinge_cheek_center_y, hinge_cheek_center_z)
        ),
        material=steel,
        name="left_hinge_cheek",
    )
    dome_shell.visual(
        Box(hinge_cheek_size),
        origin=Origin(
            xyz=(hinge_cheek_offset_x, hinge_cheek_center_y, hinge_cheek_center_z)
        ),
        material=steel,
        name="right_hinge_cheek",
    )
    shutter_geometry = _build_shutter_leaf_geometry(
        outer_profile=shell_outer,
        inner_profile=shell_inner,
        center_angle=slit_center_angle,
        z_bottom=hinge_z,
        z_top=3.06,
        lower_half_angle=0.155,
        upper_half_angle=0.070,
        angle_clearance=0.012,
        outer_clearance=0.018,
        inner_clearance=0.018,
        hinge_y=hinge_axis_y,
        hinge_z=hinge_z,
    )
    shutter_mesh = mesh_from_geometry(shutter_geometry, "dome_shutter_leaf")
    shutter_leaf.visual(
        shutter_mesh,
        material=off_white,
        name="shutter_panel",
    )
    shutter_leaf.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shutter_hinge_barrel",
    )
    shutter_leaf.visual(
        Box((0.72, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, -0.035, 0.012)),
        material=steel,
        name="shutter_lower_frame",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((0.90, 0.28, 1.18)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.02, 0.56)),
    )

    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.75,
            lower=0.0,
            upper=1.25,
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

    base = object_model.get_part("base")
    dome_shell = object_model.get_part("dome_shell")
    dome_joint = object_model.get_articulation("base_to_dome")
    shutter_leaf = object_model.get_part("shutter_leaf")
    shutter_joint = object_model.get_articulation("dome_to_shutter")

    ctx.expect_gap(
        dome_shell,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.03,
        positive_elem="rotation_skirt",
        negative_elem="bearing_track",
        name="dome skirt sits on the elevated bearing track",
    )

    with ctx.pose({dome_joint: math.pi / 2.0}):
        ctx.expect_gap(
            dome_shell,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.03,
            positive_elem="rotation_skirt",
            negative_elem="bearing_track",
            name="dome stays supported while rotated",
        )

    closed_shutter_aabb = ctx.part_world_aabb(shutter_leaf)
    with ctx.pose({shutter_joint: 1.0}):
        open_shutter_aabb = ctx.part_world_aabb(shutter_leaf)

    ctx.check(
        "shutter leaf swings outward when opened",
        closed_shutter_aabb is not None
        and open_shutter_aabb is not None
        and open_shutter_aabb[1][1] > closed_shutter_aabb[1][1] + 0.20,
        details=f"closed={closed_shutter_aabb}, open={open_shutter_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
