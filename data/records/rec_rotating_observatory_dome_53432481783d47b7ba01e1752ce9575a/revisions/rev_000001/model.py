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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _vertex(
    geom: MeshGeometry,
    cache: dict[tuple[float, float, float], int],
    xyz: tuple[float, float, float],
) -> int:
    key = (round(xyz[0], 6), round(xyz[1], 6), round(xyz[2], 6))
    if key not in cache:
        cache[key] = geom.add_vertex(*xyz)
    return cache[key]


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _append_shell_band(
    geom: MeshGeometry,
    outer: list[list[int]],
    inner: list[list[int]],
    mask: list[list[bool]],
    *,
    wrap_v: bool,
    close_u_min: bool,
    close_u_max: bool,
) -> None:
    row_count = len(outer)
    col_count = len(outer[0])
    cell_cols = col_count if wrap_v else col_count - 1

    for i in range(row_count - 1):
        for j in range(cell_cols):
            if not mask[i][j]:
                continue
            j1 = (j + 1) % col_count if wrap_v else j + 1
            _add_quad(geom, outer[i][j], outer[i + 1][j], outer[i + 1][j1], outer[i][j1])
            _add_quad(geom, inner[i][j], inner[i][j1], inner[i + 1][j1], inner[i + 1][j])

            left_included = mask[i][(j - 1) % cell_cols] if wrap_v else (j > 0 and mask[i][j - 1])
            if not left_included:
                _add_quad(geom, outer[i][j], outer[i + 1][j], inner[i + 1][j], inner[i][j])

            right_included = mask[i][(j + 1) % cell_cols] if wrap_v else (j + 1 < cell_cols and mask[i][j + 1])
            if not right_included:
                _add_quad(geom, outer[i][j1], inner[i][j1], inner[i + 1][j1], outer[i + 1][j1])

            if i == 0:
                if close_u_min:
                    _add_quad(geom, outer[i][j], inner[i][j], inner[i][j1], outer[i][j1])
            elif not mask[i - 1][j]:
                _add_quad(geom, outer[i][j], inner[i][j], inner[i][j1], outer[i][j1])

            if i == row_count - 2:
                if close_u_max:
                    _add_quad(
                        geom,
                        outer[i + 1][j],
                        outer[i + 1][j1],
                        inner[i + 1][j1],
                        inner[i + 1][j],
                    )
            elif not mask[i + 1][j]:
                _add_quad(
                    geom,
                    outer[i + 1][j],
                    outer[i + 1][j1],
                    inner[i + 1][j1],
                    inner[i + 1][j],
                )


def _slit_void(
    x: float,
    y: float,
    z: float,
    *,
    skirt_h: float,
    crown_z: float,
    hinge_z: float,
) -> bool:
    if z < 0.30 or x <= 0.0:
        return False
    if z <= hinge_z:
        return abs(y) < 0.36 and x > (1.16 if z < skirt_h else 0.10)
    t = min(1.0, max(0.0, (z - hinge_z) / (crown_z - hinge_z)))
    half_width = 0.50 - 0.34 * t
    front_gate = 0.05 - 0.35 * t
    return abs(y) < half_width and x > front_gate


def _build_dome_shell_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    cache: dict[tuple[float, float, float], int] = {}

    outer_radius = 2.26
    shell_thickness = 0.06
    inner_radius = outer_radius - shell_thickness
    skirt_h = 0.46
    hinge_z = 1.86
    crown_z = skirt_h + outer_radius

    phi_count = 88
    dome_rows = 24
    skirt_rows = 9
    phis = [(-math.pi) + (2.0 * math.pi * j / phi_count) for j in range(phi_count)]
    thetas = [(0.5 * math.pi) * i / dome_rows for i in range(dome_rows + 1)]
    skirt_zs = [skirt_h * i / skirt_rows for i in range(skirt_rows + 1)]

    dome_outer = [
        [
            _vertex(
                geom,
                cache,
                (
                    outer_radius * math.sin(theta) * math.cos(phi),
                    outer_radius * math.sin(theta) * math.sin(phi),
                    skirt_h + outer_radius * math.cos(theta),
                ),
            )
            for phi in phis
        ]
        for theta in thetas
    ]
    dome_inner = [
        [
            _vertex(
                geom,
                cache,
                (
                    inner_radius * math.sin(theta) * math.cos(phi),
                    inner_radius * math.sin(theta) * math.sin(phi),
                    skirt_h + inner_radius * math.cos(theta),
                ),
            )
            for phi in phis
        ]
        for theta in thetas
    ]
    dome_mask = []
    for i in range(dome_rows):
        row = []
        theta_mid = 0.5 * (thetas[i] + thetas[i + 1])
        for j in range(phi_count):
            phi_mid = phis[j] + (math.pi / phi_count)
            x_mid = outer_radius * math.sin(theta_mid) * math.cos(phi_mid)
            y_mid = outer_radius * math.sin(theta_mid) * math.sin(phi_mid)
            z_mid = skirt_h + outer_radius * math.cos(theta_mid)
            row.append(
                not _slit_void(
                    x_mid,
                    y_mid,
                    z_mid,
                    skirt_h=skirt_h,
                    crown_z=crown_z,
                    hinge_z=hinge_z,
                )
            )
        dome_mask.append(row)

    _append_shell_band(
        geom,
        dome_outer,
        dome_inner,
        dome_mask,
        wrap_v=True,
        close_u_min=True,
        close_u_max=False,
    )

    skirt_outer = [
        [
            _vertex(
                geom,
                cache,
                (
                    outer_radius * math.cos(phi),
                    outer_radius * math.sin(phi),
                    z,
                ),
            )
            for phi in phis
        ]
        for z in skirt_zs
    ]
    skirt_inner = [
        [
            _vertex(
                geom,
                cache,
                (
                    inner_radius * math.cos(phi),
                    inner_radius * math.sin(phi),
                    z,
                ),
            )
            for phi in phis
        ]
        for z in skirt_zs
    ]
    skirt_mask = []
    for i in range(skirt_rows):
        row = []
        z_mid = 0.5 * (skirt_zs[i] + skirt_zs[i + 1])
        for j in range(phi_count):
            phi_mid = phis[j] + (math.pi / phi_count)
            x_mid = outer_radius * math.cos(phi_mid)
            y_mid = outer_radius * math.sin(phi_mid)
            row.append(
                not _slit_void(
                    x_mid,
                    y_mid,
                    z_mid,
                    skirt_h=skirt_h,
                    crown_z=crown_z,
                    hinge_z=hinge_z,
                )
            )
        skirt_mask.append(row)

    _append_shell_band(
        geom,
        skirt_outer,
        skirt_inner,
        skirt_mask,
        wrap_v=True,
        close_u_min=False,
        close_u_max=False,
    )

    return geom


def _build_shutter_leaf_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    cache: dict[tuple[float, float, float], int] = {}

    width = 0.60
    outer_radius = 1.12
    thickness = 0.045
    inner_radius = outer_radius - thickness
    arc_extent = 0.72
    arc_rows = 16
    width_cols = 12

    arc_samples = [arc_extent * i / arc_rows for i in range(arc_rows + 1)]
    y_samples = [(-width * 0.5) + width * i / width_cols for i in range(width_cols + 1)]

    outer = [
        [
            _vertex(
                geom,
                cache,
                (
                    outer_radius * math.sin(a),
                    y,
                    0.02 + outer_radius * (1.0 - math.cos(a)),
                ),
            )
            for y in y_samples
        ]
        for a in arc_samples
    ]
    inner = [
        [
            _vertex(
                geom,
                cache,
                (
                    inner_radius * math.sin(a),
                    y,
                    0.02 + inner_radius * (1.0 - math.cos(a)),
                ),
            )
            for y in y_samples
        ]
        for a in arc_samples
    ]
    full_mask = [[True for _ in range(width_cols)] for _ in range(arc_rows)]
    _append_shell_band(
        geom,
        outer,
        inner,
        full_mask,
        wrap_v=False,
        close_u_min=True,
        close_u_max=True,
    )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.73, 0.74, 0.75, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    white_shell = model.material("white_shell", rgba=(0.92, 0.94, 0.96, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.19, 1.0))
    glass = model.material("glass", rgba=(0.38, 0.49, 0.58, 0.40))
    shadow = model.material("shadow", rgba=(0.08, 0.09, 0.10, 1.0))

    dome_shell_mesh = mesh_from_geometry(_build_dome_shell_geometry(), "dome_shell_skin")
    shutter_leaf_mesh = mesh_from_geometry(_build_shutter_leaf_geometry(), "slit_shutter_leaf")

    support_base = model.part("support_base")
    support_base.visual(
        Cylinder(radius=3.10, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=concrete,
        name="lower_drum",
    )
    support_base.visual(
        Cylinder(radius=2.78, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material=dark_trim,
        name="bearing_ring",
    )
    support_base.visual(
        Cylinder(radius=2.96, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.22)),
        material=steel,
        name="service_ledge",
    )
    support_base.visual(
        Box((1.10, 0.28, 0.92)),
        origin=Origin(xyz=(2.52, 0.0, 0.46)),
        material=glass,
        name="entry_window_band",
    )
    support_base.inertial = Inertial.from_geometry(
        Cylinder(radius=3.10, length=1.62),
        mass=6200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        Cylinder(radius=2.44, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_trim,
        name="rotation_track",
    )
    dome_shell.visual(
        dome_shell_mesh,
        material=white_shell,
        name="shell_skin",
    )
    dome_shell.visual(
        Box((0.16, 0.16, 1.58)),
        origin=Origin(xyz=(2.09, 0.45, 1.13)),
        material=dark_trim,
        name="slit_jamb_left",
    )
    dome_shell.visual(
        Box((0.16, 0.16, 1.58)),
        origin=Origin(xyz=(2.09, -0.45, 1.13)),
        material=dark_trim,
        name="slit_jamb_right",
    )
    dome_shell.visual(
        Box((0.16, 0.16, 0.12)),
        origin=Origin(xyz=(2.17, 0.39, 1.88)),
        material=steel,
        name="hinge_mount_left",
    )
    dome_shell.visual(
        Box((0.16, 0.16, 0.12)),
        origin=Origin(xyz=(2.17, -0.39, 1.88)),
        material=steel,
        name="hinge_mount_right",
    )
    dome_shell.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(2.30, 0.39, 1.88), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_knuckle_left",
    )
    dome_shell.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(2.30, -0.39, 1.88), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_knuckle_right",
    )
    dome_shell.visual(
        Box((0.24, 1.06, 0.10)),
        origin=Origin(xyz=(2.08, 0.0, 0.34)),
        material=shadow,
        name="slit_sill",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=2.43, length=2.60),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        shutter_leaf_mesh,
        material=white_shell,
        name="leaf_skin",
    )
    shutter_leaf.visual(
        Cylinder(radius=0.05, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="leaf_hinge_bar",
    )
    shutter_leaf.visual(
        Box((0.08, 0.62, 0.06)),
        origin=Origin(xyz=(-0.06, 0.0, -0.05)),
        material=steel,
        name="leaf_frame_root",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((1.00, 0.62, 0.36)),
        mass=120.0,
        origin=Origin(xyz=(0.50, 0.0, 0.16)),
    )

    model.articulation(
        "dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=support_base,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 1.48)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22000.0, velocity=0.45),
    )
    model.articulation(
        "slit_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(xyz=(2.30, 0.0, 1.88), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.70,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_base = object_model.get_part("support_base")
    dome_shell = object_model.get_part("dome_shell")
    dome_rotation = object_model.get_articulation("dome_rotation")
    slit_shutter = object_model.get_articulation("slit_shutter")

    base_aabb = ctx.part_world_aabb(support_base)
    dome_aabb = ctx.part_world_aabb(dome_shell)
    ctx.check(
        "fixed support is broader than rotating dome",
        base_aabb is not None
        and dome_aabb is not None
        and (base_aabb[1][0] - base_aabb[0][0]) > (dome_aabb[1][0] - dome_aabb[0][0]) + 0.8,
        details=f"support={base_aabb}, dome={dome_aabb}",
    )

    closed_leaf = ctx.part_element_world_aabb("shutter_leaf", elem="leaf_skin")
    with ctx.pose({slit_shutter: slit_shutter.motion_limits.upper}):
        open_leaf = ctx.part_element_world_aabb("shutter_leaf", elem="leaf_skin")

    closed_center = (
        None
        if closed_leaf is None
        else (
            0.5 * (closed_leaf[0][0] + closed_leaf[1][0]),
            0.5 * (closed_leaf[0][1] + closed_leaf[1][1]),
            0.5 * (closed_leaf[0][2] + closed_leaf[1][2]),
        )
    )
    open_center = (
        None
        if open_leaf is None
        else (
            0.5 * (open_leaf[0][0] + open_leaf[1][0]),
            0.5 * (open_leaf[0][1] + open_leaf[1][1]),
            0.5 * (open_leaf[0][2] + open_leaf[1][2]),
        )
    )
    ctx.check(
        "shutter opens upward over the crown",
        closed_center is not None
        and open_center is not None
        and open_center[2] > closed_center[2] + 0.22
        and open_center[0] > closed_center[0] + 0.25,
        details=f"closed={closed_center}, open={open_center}",
    )

    closed_shell = ctx.part_world_position(dome_shell)
    with ctx.pose({dome_rotation: 1.1}):
        rotated_shell = ctx.part_world_position(dome_shell)
    ctx.check(
        "dome shell rotates about the vertical axis",
        closed_shell is not None
        and rotated_shell is not None
        and abs(rotated_shell[2] - closed_shell[2]) < 1e-6
        and abs(rotated_shell[0]) < 1e-6
        and abs(rotated_shell[1]) < 1e-6,
        details=f"closed={closed_shell}, rotated={rotated_shell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
