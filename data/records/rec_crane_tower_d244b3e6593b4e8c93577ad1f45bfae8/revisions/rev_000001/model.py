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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


Vec3 = tuple[float, float, float]


def _midpoint(a: Vec3, b: Vec3) -> Vec3:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: Vec3, b: Vec3) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a: Vec3, b: Vec3, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_square_lattice(
    part,
    *,
    width: float,
    bottom_z: float,
    top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
    ladder_material,
    ladder_side: str = "+x",
) -> None:
    half = width * 0.5
    corners = [
        (half, half),
        (half, -half),
        (-half, -half),
        (-half, half),
    ]
    levels = [bottom_z + (top_z - bottom_z) * i / panels for i in range(panels + 1)]

    for x, y in corners:
        _add_member(part, (x, y, bottom_z), (x, y, top_z), chord_radius, material)

    for z in levels:
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_member(part, (x0, y0, z), (x1, y1, z), brace_radius, material)

    for i in range(panels):
        z0 = levels[i]
        z1 = levels[i + 1]
        for j in range(4):
            x0, y0 = corners[j]
            x1, y1 = corners[(j + 1) % 4]
            _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
            _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)

    ladder_x = half + 0.08 if ladder_side == "+x" else -(half + 0.08)
    ladder_bottom = bottom_z + 0.35
    ladder_top = top_z - 0.25
    rail_y = 0.23
    _add_member(
        part,
        (ladder_x, -rail_y, ladder_bottom),
        (ladder_x, -rail_y, ladder_top),
        brace_radius * 0.75,
        ladder_material,
    )
    _add_member(
        part,
        (ladder_x, rail_y, ladder_bottom),
        (ladder_x, rail_y, ladder_top),
        brace_radius * 0.75,
        ladder_material,
    )
    rung_count = max(8, int((ladder_top - ladder_bottom) / 0.35))
    for i in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * i / rung_count
        _add_member(part, (ladder_x, -rail_y, z), (ladder_x, rail_y, z), brace_radius * 0.55, ladder_material)

    face_x = half if ladder_side == "+x" else -half
    for z in levels:
        _add_member(part, (face_x, -rail_y, z), (ladder_x, -rail_y, z), brace_radius * 0.42, ladder_material)
        _add_member(part, (face_x, rail_y, z), (ladder_x, rail_y, z), brace_radius * 0.42, ladder_material)


def _add_outer_climbing_frame(
    part,
    *,
    mast_half: float,
    outer_half: float,
    bottom_z: float,
    top_z: float,
    material,
    hydraulic_material,
    steel_material,
) -> None:
    corners = [
        (outer_half, outer_half),
        (outer_half, -outer_half),
        (-outer_half, -outer_half),
        (-outer_half, outer_half),
    ]
    levels = [bottom_z, bottom_z + 0.85, top_z]

    for x, y in corners:
        _add_member(part, (x, y, bottom_z), (x, y, top_z), 0.07, material)

    for z in levels:
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_member(part, (x0, y0, z), (x1, y1, z), 0.045, material)

    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 4]
        _add_member(part, (x0, y0, bottom_z), (x1, y1, top_z), 0.032, material)
        _add_member(part, (x1, y1, bottom_z), (x0, y0, top_z), 0.032, material)

    tie_zs = (bottom_z + 0.20, bottom_z + 0.95)
    mast_corners = [
        (mast_half, mast_half),
        (mast_half, -mast_half),
        (-mast_half, -mast_half),
        (-mast_half, mast_half),
    ]
    for z in tie_zs:
        for (mx, my), (ox, oy) in zip(mast_corners, corners):
            _add_member(part, (mx, my, z), (ox, oy, z), 0.04, steel_material)

    cylinder_centers = [
        (outer_half - 0.18, 0.0),
        (-(outer_half - 0.18), 0.0),
        (0.0, outer_half - 0.18),
        (0.0, -(outer_half - 0.18)),
    ]
    cyl_bottom = bottom_z + 0.25
    cyl_top = top_z - 0.25
    rod_bottom = bottom_z + 0.85
    rod_top = top_z - 0.08
    for x, y in cylinder_centers:
        part.visual(
            Cylinder(radius=0.09, length=cyl_top - cyl_bottom),
            origin=Origin(xyz=(x, y, (cyl_bottom + cyl_top) * 0.5)),
            material=hydraulic_material,
        )
        part.visual(
            Cylinder(radius=0.05, length=rod_top - rod_bottom),
            origin=Origin(xyz=(x, y, (rod_bottom + rod_top) * 0.5)),
            material=steel_material,
        )
        if abs(x) > 0.0:
            sign_x = 1.0 if x > 0.0 else -1.0
            _add_member(
                part,
                (sign_x * outer_half, outer_half, cyl_bottom),
                (x, y, cyl_bottom),
                0.022,
                steel_material,
            )
            _add_member(
                part,
                (sign_x * outer_half, -outer_half, cyl_bottom),
                (x, y, cyl_bottom),
                0.022,
                steel_material,
            )
            _add_member(
                part,
                (sign_x * outer_half, outer_half, cyl_top),
                (x, y, cyl_top),
                0.022,
                steel_material,
            )
            _add_member(
                part,
                (sign_x * outer_half, -outer_half, cyl_top),
                (x, y, cyl_top),
                0.022,
                steel_material,
            )
        else:
            sign_y = 1.0 if y > 0.0 else -1.0
            _add_member(
                part,
                (outer_half, sign_y * outer_half, cyl_bottom),
                (x, y, cyl_bottom),
                0.022,
                steel_material,
            )
            _add_member(
                part,
                (-outer_half, sign_y * outer_half, cyl_bottom),
                (x, y, cyl_bottom),
                0.022,
                steel_material,
            )
            _add_member(
                part,
                (outer_half, sign_y * outer_half, cyl_top),
                (x, y, cyl_top),
                0.022,
                steel_material,
            )
            _add_member(
                part,
                (-outer_half, sign_y * outer_half, cyl_top),
                (x, y, cyl_top),
                0.022,
                steel_material,
            )

    guide_z = top_z - 0.52
    guide_height = 0.72
    guide_inner_face_x = 0.814
    guide_outer_face_x = 1.18
    guide_depth_y = 0.30
    for guide_y in (-0.23, 0.23):
        part.visual(
            Box((guide_outer_face_x - guide_inner_face_x, guide_depth_y, guide_height)),
            origin=Origin(
                xyz=(
                    (guide_inner_face_x + guide_outer_face_x) * 0.5,
                    guide_y,
                    guide_z,
                )
            ),
            material=steel_material,
        )


def _add_triangular_truss(
    part,
    *,
    x_start: float,
    x_end: float,
    bottom_z: float,
    half_width: float,
    root_top_z: float,
    tip_top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
) -> dict[str, list[Vec3]]:
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    span = x_end - x_start

    def top_z(x: float) -> float:
        t = 0.0 if abs(span) < 1e-9 else (x - x_start) / span
        return root_top_z + (tip_top_z - root_top_z) * t

    lower_left = [(x, -half_width, bottom_z) for x in xs]
    lower_right = [(x, half_width, bottom_z) for x in xs]
    upper = [(x, 0.0, top_z(x)) for x in xs]

    for i in range(panels):
        _add_member(part, lower_left[i], lower_left[i + 1], chord_radius, material)
        _add_member(part, lower_right[i], lower_right[i + 1], chord_radius, material)
        _add_member(part, upper[i], upper[i + 1], chord_radius, material)

    for i in range(panels + 1):
        _add_member(part, lower_left[i], lower_right[i], brace_radius, material)
        _add_member(part, lower_left[i], upper[i], brace_radius, material)
        _add_member(part, lower_right[i], upper[i], brace_radius, material)

    for i in range(panels):
        if i % 2 == 0:
            _add_member(part, lower_left[i], upper[i + 1], brace_radius, material)
            _add_member(part, lower_right[i], upper[i + 1], brace_radius, material)
        else:
            _add_member(part, upper[i], lower_left[i + 1], brace_radius, material)
            _add_member(part, upper[i], lower_right[i + 1], brace_radius, material)

    return {"lower_left": lower_left, "lower_right": lower_right, "upper": upper}


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.10, 0.0, -0.28),
            (0.18, 0.0, -0.42),
            (0.20, 0.0, -0.60),
            (0.10, 0.0, -0.76),
            (-0.07, 0.0, -0.82),
            (-0.20, 0.0, -0.70),
            (-0.16, 0.0, -0.52),
        ],
        radius=0.035,
        samples_per_segment=20,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "climbing_tower_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="climbing_frame_tower_crane")

    tower_yellow = model.material("tower_yellow", rgba=(0.93, 0.77, 0.12, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.23, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    hydraulic_red = model.material("hydraulic_red", rgba=(0.70, 0.12, 0.10, 1.0))
    ballast = model.material("ballast", rgba=(0.47, 0.47, 0.46, 1.0))
    cable = model.material("cable", rgba=(0.12, 0.12, 0.13, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.68, 0.82, 0.90, 0.35))
    safety_red = model.material("safety_red", rgba=(0.78, 0.15, 0.10, 1.0))

    hook_mesh = _build_hook_mesh()

    lower_mast = model.part("lower_mast")
    lower_mast.visual(
        Box((6.0, 6.0, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        material=concrete,
        name="foundation",
    )
    lower_mast.visual(
        Box((2.7, 2.7, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 1.275)),
        material=concrete,
    )
    lower_mast.visual(
        Box((1.9, 1.9, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 1.675)),
        material=dark_grey,
    )
    for sx in (-0.58, 0.58):
        for sy in (-0.58, 0.58):
            lower_mast.visual(
                Cylinder(radius=0.08, length=0.55),
                origin=Origin(xyz=(sx, sy, 1.275)),
                material=steel,
            )

    _add_square_lattice(
        lower_mast,
        width=1.60,
        bottom_z=1.80,
        top_z=9.40,
        panels=6,
        chord_radius=0.06,
        brace_radius=0.035,
        material=tower_yellow,
        ladder_material=steel,
    )
    lower_mast.visual(
        Box((1.70, 1.70, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 9.36)),
        material=dark_grey,
    )
    _add_outer_climbing_frame(
        lower_mast,
        mast_half=0.80,
        outer_half=1.25,
        bottom_z=8.45,
        top_z=10.65,
        material=tower_yellow,
        hydraulic_material=hydraulic_red,
        steel_material=steel,
    )
    lower_mast.inertial = Inertial.from_geometry(
        Box((6.0, 6.0, 10.8)),
        mass=54000.0,
        origin=Origin(xyz=(0.0, 0.0, 5.40)),
    )

    upper_mast = model.part("upper_mast")
    _add_square_lattice(
        upper_mast,
        width=1.42,
        bottom_z=0.0,
        top_z=6.80,
        panels=5,
        chord_radius=0.055,
        brace_radius=0.032,
        material=tower_yellow,
        ladder_material=steel,
    )
    upper_mast.visual(
        Box((1.55, 1.55, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 6.87)),
        material=dark_grey,
        name="mast_head_cap",
    )
    upper_mast.inertial = Inertial.from_geometry(
        Box((1.55, 1.55, 6.90)),
        mass=9500.0,
        origin=Origin(xyz=(0.0, 0.0, 3.45)),
    )

    jib_assembly = model.part("jib_assembly")
    jib_assembly.visual(
        Cylinder(radius=0.78, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_grey,
        name="slew_ring",
    )
    jib_assembly.visual(
        Box((3.20, 2.10, 0.22)),
        origin=Origin(xyz=(0.10, 0.0, 0.29)),
        material=dark_grey,
        name="machinery_deck",
    )
    jib_assembly.visual(
        Box((1.15, 1.30, 0.95)),
        origin=Origin(xyz=(-0.65, 0.0, 0.79)),
        material=dark_grey,
        name="hoist_house",
    )
    jib_assembly.visual(
        Box((0.90, 0.85, 1.30)),
        origin=Origin(xyz=(1.05, -1.00, 0.90)),
        material=dark_grey,
        name="cab_body",
    )
    jib_assembly.visual(
        Box((0.82, 0.78, 0.98)),
        origin=Origin(xyz=(1.08, -1.00, 0.95)),
        material=cab_glass,
        name="cab_glass",
    )

    jib = _add_triangular_truss(
        jib_assembly,
        x_start=1.80,
        x_end=18.80,
        bottom_z=0.82,
        half_width=0.50,
        root_top_z=3.10,
        tip_top_z=1.55,
        panels=10,
        chord_radius=0.065,
        brace_radius=0.036,
        material=tower_yellow,
    )
    jib_assembly.visual(
        Box((15.60, 0.08, 0.08)),
        origin=Origin(xyz=(10.10, -0.22, 0.72)),
        material=steel,
        name="jib_track_left",
    )
    jib_assembly.visual(
        Box((15.60, 0.08, 0.08)),
        origin=Origin(xyz=(10.10, 0.22, 0.72)),
        material=steel,
        name="jib_track_right",
    )
    for left_node, right_node in zip(jib["lower_left"][1:-1], jib["lower_right"][1:-1]):
        track_z = 0.76
        _add_member(
            jib_assembly,
            left_node,
            (left_node[0], -0.22, track_z),
            0.018,
            steel,
        )
        _add_member(
            jib_assembly,
            right_node,
            (right_node[0], 0.22, track_z),
            0.018,
            steel,
        )

    counter_jib = _add_triangular_truss(
        jib_assembly,
        x_start=-1.20,
        x_end=-6.80,
        bottom_z=0.82,
        half_width=0.42,
        root_top_z=2.80,
        tip_top_z=1.45,
        panels=4,
        chord_radius=0.060,
        brace_radius=0.034,
        material=tower_yellow,
    )
    jib_assembly.visual(
        Box((0.90, 1.00, 0.28)),
        origin=Origin(xyz=(-4.60, 0.0, 1.02)),
        material=ballast,
    )
    jib_assembly.visual(
        Box((0.90, 1.00, 0.28)),
        origin=Origin(xyz=(-5.55, 0.0, 1.02)),
        material=ballast,
    )
    jib_assembly.visual(
        Box((0.70, 0.90, 0.24)),
        origin=Origin(xyz=(-6.25, 0.0, 0.98)),
        material=ballast,
    )

    apex = (0.35, 0.0, 4.40)
    _add_member(jib_assembly, (-0.45, -0.55, 0.32), apex, 0.08, tower_yellow)
    _add_member(jib_assembly, (-0.45, 0.55, 0.32), apex, 0.08, tower_yellow)
    _add_member(jib_assembly, (0.80, -0.30, 3.15), (0.80, 0.30, 3.15), 0.045, tower_yellow)
    _add_member(jib_assembly, (0.80, -0.30, 3.15), apex, 0.028, tower_yellow)
    _add_member(jib_assembly, (0.80, 0.30, 3.15), apex, 0.028, tower_yellow)
    jib_assembly.visual(
        Box((0.35, 0.35, 0.30)),
        origin=Origin(xyz=apex),
        material=tower_yellow,
    )
    _add_member(jib_assembly, apex, jib["upper"][5], 0.022, cable)
    _add_member(jib_assembly, apex, jib["upper"][-1], 0.020, cable)
    _add_member(jib_assembly, apex, counter_jib["upper"][-1], 0.022, cable)

    jib_assembly.inertial = Inertial.from_geometry(
        Box((26.0, 2.6, 4.8)),
        mass=17000.0,
        origin=Origin(xyz=(6.5, 0.0, 1.45)),
    )

    trolley_block = model.part("trolley_block")
    trolley_block.visual(
        Box((0.95, 0.60, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, -0.27)),
        material=dark_grey,
        name="carriage_body",
    )
    trolley_block.visual(
        Box((0.72, 0.46, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=steel,
        name="saddle",
    )
    trolley_block.visual(
        Box((0.18, 0.08, 0.16)),
        origin=Origin(xyz=(-0.24, -0.28, -0.16)),
        material=steel,
    )
    trolley_block.visual(
        Box((0.18, 0.08, 0.16)),
        origin=Origin(xyz=(-0.24, 0.28, -0.16)),
        material=steel,
    )
    trolley_block.visual(
        Box((0.18, 0.08, 0.16)),
        origin=Origin(xyz=(0.24, -0.28, -0.16)),
        material=steel,
    )
    trolley_block.visual(
        Box((0.18, 0.08, 0.16)),
        origin=Origin(xyz=(0.24, 0.28, -0.16)),
        material=steel,
    )
    trolley_block.visual(
        Cylinder(radius=0.12, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, -0.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hoist_drum",
    )
    trolley_block.visual(
        Cylinder(radius=0.018, length=2.65),
        origin=Origin(xyz=(-0.08, 0.0, -1.46)),
        material=cable,
    )
    trolley_block.visual(
        Cylinder(radius=0.018, length=2.65),
        origin=Origin(xyz=(0.08, 0.0, -1.46)),
        material=cable,
    )
    trolley_block.visual(
        Box((0.42, 0.28, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, -2.83)),
        material=tower_yellow,
        name="hook_block_body",
    )
    trolley_block.visual(
        Cylinder(radius=0.05, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, -2.61), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    trolley_block.visual(
        Cylinder(radius=0.05, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, -3.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    trolley_block.visual(
        hook_mesh,
        origin=Origin(xyz=(0.0, 0.0, -2.58)),
        material=safety_red,
        name="hook",
    )
    trolley_block.inertial = Inertial.from_geometry(
        Box((1.0, 0.7, 3.7)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, -1.85)),
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 9.48)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400000.0, velocity=0.18, lower=0.0, upper=4.0),
    )
    model.articulation(
        "jib_slew",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=jib_assembly,
        origin=Origin(xyz=(0.0, 0.0, 6.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180000.0, velocity=0.22, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=jib_assembly,
        child=trolley_block,
        origin=Origin(xyz=(2.40, 0.0, 0.68)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70000.0, velocity=1.1, lower=0.0, upper=14.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_mast = object_model.get_part("lower_mast")
    upper_mast = object_model.get_part("upper_mast")
    jib_assembly = object_model.get_part("jib_assembly")
    trolley_block = object_model.get_part("trolley_block")

    mast_extension = object_model.get_articulation("mast_extension")
    jib_slew = object_model.get_articulation("jib_slew")
    trolley_travel = object_model.get_articulation("trolley_travel")

    jib_track_left = jib_assembly.get_visual("jib_track_left")
    jib_track_right = jib_assembly.get_visual("jib_track_right")

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

    part_names = {part.name for part in object_model.parts}
    articulation_names = {articulation.name for articulation in object_model.articulations}
    ctx.check(
        "core_parts_present",
        {"lower_mast", "upper_mast", "jib_assembly", "trolley_block"}.issubset(part_names),
        f"present parts: {sorted(part_names)}",
    )
    ctx.check(
        "core_articulations_present",
        {"mast_extension", "jib_slew", "trolley_travel"}.issubset(articulation_names),
        f"present articulations: {sorted(articulation_names)}",
    )
    ctx.check(
        "mast_extension_axis_vertical",
        mast_extension.axis == (0.0, 0.0, 1.0)
        and mast_extension.motion_limits is not None
        and mast_extension.motion_limits.lower == 0.0
        and mast_extension.motion_limits.upper == 4.0,
        f"axis={mast_extension.axis}, limits={mast_extension.motion_limits}",
    )
    ctx.check(
        "jib_slew_axis_vertical",
        jib_slew.axis == (0.0, 0.0, 1.0)
        and jib_slew.motion_limits is not None
        and jib_slew.motion_limits.lower is not None
        and jib_slew.motion_limits.upper is not None
        and jib_slew.motion_limits.lower <= -3.0
        and jib_slew.motion_limits.upper >= 3.0,
        f"axis={jib_slew.axis}, limits={jib_slew.motion_limits}",
    )
    ctx.check(
        "trolley_travel_axis_along_jib",
        trolley_travel.axis == (1.0, 0.0, 0.0)
        and trolley_travel.motion_limits is not None
        and trolley_travel.motion_limits.upper is not None
        and trolley_travel.motion_limits.upper >= 14.0,
        f"axis={trolley_travel.axis}, limits={trolley_travel.motion_limits}",
    )
    ctx.check(
        "jib_tracks_present",
        jib_track_left.name == "jib_track_left" and jib_track_right.name == "jib_track_right",
        "expected both jib track visuals to be named and accessible",
    )

    ctx.expect_origin_distance(
        upper_mast,
        lower_mast,
        axes="xy",
        max_dist=0.02,
        name="mast_sections_share_centerline",
    )
    ctx.expect_overlap(
        upper_mast,
        lower_mast,
        axes="xy",
        min_overlap=1.20,
        name="upper_mast_stays_over_lower_mast_footprint",
    )
    ctx.expect_origin_distance(
        trolley_block,
        jib_assembly,
        axes="y",
        max_dist=0.02,
        name="trolley_stays_centered_under_jib",
    )

    with ctx.pose({mast_extension: 3.2}):
        ctx.expect_origin_gap(
            upper_mast,
            lower_mast,
            axis="z",
            min_gap=12.66,
            max_gap=12.70,
            name="mast_extension_raises_upper_mast",
        )
        ctx.expect_origin_distance(
            upper_mast,
            lower_mast,
            axes="xy",
            max_dist=0.02,
            name="mast_extension_remains_coaxial",
        )

    with ctx.pose({trolley_travel: 13.5}):
        ctx.expect_origin_distance(
            trolley_block,
            upper_mast,
            axes="xy",
            min_dist=15.7,
            max_dist=16.1,
            name="trolley_can_run_far_outboard",
        )
        ctx.expect_overlap(
            trolley_block,
            jib_assembly,
            axes="y",
            min_overlap=0.45,
            name="trolley_remains_within_jib_width_at_tip_pose",
        )

    with ctx.pose({jib_slew: 1.15, trolley_travel: 10.0}):
        ctx.expect_origin_distance(
            trolley_block,
            upper_mast,
            axes="xy",
            min_dist=12.2,
            max_dist=12.5,
            name="trolley_moves_with_slewed_jib",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
