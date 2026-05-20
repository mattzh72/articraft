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


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (x * c - y * s, x * s + y * c)


def _place_local(origin_xyz: Vec3, yaw: float, local_xyz: Vec3) -> Vec3:
    rx, ry = _rotate_xy(local_xyz[0], local_xyz[1], yaw)
    return (origin_xyz[0] + rx, origin_xyz[1] + ry, origin_xyz[2] + local_xyz[2])


def _add_socket(base_part, *, name_prefix: str, origin_xyz: Vec3, yaw: float, material) -> None:
    floor_center = _place_local(origin_xyz, yaw, (0.04, 0.0, -0.075))
    wall_left_center = _place_local(origin_xyz, yaw, (0.04, 0.075, 0.02))
    wall_right_center = _place_local(origin_xyz, yaw, (0.04, -0.075, 0.02))
    stop_center = _place_local(origin_xyz, yaw, (-0.415, 0.0, 0.02))

    base_part.visual(
        Box((0.88, 0.16, 0.03)),
        origin=Origin(xyz=floor_center, rpy=(0.0, 0.0, yaw)),
        material=material,
        name=f"{name_prefix}_floor",
    )
    base_part.visual(
        Box((0.88, 0.02, 0.16)),
        origin=Origin(xyz=wall_left_center, rpy=(0.0, 0.0, yaw)),
        material=material,
    )
    base_part.visual(
        Box((0.88, 0.02, 0.16)),
        origin=Origin(xyz=wall_right_center, rpy=(0.0, 0.0, yaw)),
        material=material,
    )
    base_part.visual(
        Box((0.03, 0.16, 0.16)),
        origin=Origin(xyz=stop_center, rpy=(0.0, 0.0, yaw)),
        material=material,
    )


def _add_square_lattice_mast(
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
        for index in range(4):
            x0, y0 = corners[index]
            x1, y1 = corners[(index + 1) % 4]
            _add_member(part, (x0, y0, z), (x1, y1, z), brace_radius, material)

    for index in range(panels):
        z0 = levels[index]
        z1 = levels[index + 1]
        for face in range(4):
            x0, y0 = corners[face]
            x1, y1 = corners[(face + 1) % 4]
            _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
            _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)

    ladder_x = half + 0.012
    rail_y = 0.030
    ladder_bottom = bottom_z + 0.12
    ladder_top = top_z - 0.16
    _add_member(
        part,
        (ladder_x, -rail_y, ladder_bottom),
        (ladder_x, -rail_y, ladder_top),
        0.006,
        ladder_material,
    )
    _add_member(
        part,
        (ladder_x, rail_y, ladder_bottom),
        (ladder_x, rail_y, ladder_top),
        0.006,
        ladder_material,
    )
    rung_count = 18
    for rung_index in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * rung_index / rung_count
        _add_member(part, (ladder_x, -rail_y, z), (ladder_x, rail_y, z), 0.0045, ladder_material)


def _add_box_truss(
    part,
    *,
    x_start: float,
    x_end: float,
    top_half_width: float,
    bottom_half_width: float,
    top_z_root: float,
    top_z_tip: float,
    bottom_z_root: float,
    bottom_z_tip: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
) -> dict[str, list[Vec3]]:
    xs = [x_start + (x_end - x_start) * index / panels for index in range(panels + 1)]
    span = x_end - x_start

    def _lerp(root_value: float, tip_value: float, x: float) -> float:
        t = 0.0 if abs(span) < 1e-9 else (x - x_start) / span
        return root_value + (tip_value - root_value) * t

    upper_left = [(x, top_half_width, _lerp(top_z_root, top_z_tip, x)) for x in xs]
    upper_right = [(x, -top_half_width, _lerp(top_z_root, top_z_tip, x)) for x in xs]
    lower_left = [(x, bottom_half_width, _lerp(bottom_z_root, bottom_z_tip, x)) for x in xs]
    lower_right = [(x, -bottom_half_width, _lerp(bottom_z_root, bottom_z_tip, x)) for x in xs]

    for index in range(panels):
        _add_member(part, upper_left[index], upper_left[index + 1], chord_radius, material)
        _add_member(part, upper_right[index], upper_right[index + 1], chord_radius, material)
        _add_member(part, lower_left[index], lower_left[index + 1], chord_radius, material)
        _add_member(part, lower_right[index], lower_right[index + 1], chord_radius, material)

    for index in range(panels + 1):
        _add_member(part, upper_left[index], upper_right[index], brace_radius, material)
        _add_member(part, lower_left[index], lower_right[index], brace_radius, material)
        _add_member(part, upper_left[index], lower_left[index], brace_radius, material)
        _add_member(part, upper_right[index], lower_right[index], brace_radius, material)

    for index in range(panels):
        _add_member(part, upper_left[index], lower_left[index + 1], brace_radius, material)
        _add_member(part, lower_left[index], upper_left[index + 1], brace_radius, material)
        _add_member(part, upper_right[index], lower_right[index + 1], brace_radius, material)
        _add_member(part, lower_right[index], upper_right[index + 1], brace_radius, material)
        _add_member(part, upper_left[index], upper_right[index + 1], brace_radius * 0.9, material)
        _add_member(part, upper_right[index], upper_left[index + 1], brace_radius * 0.9, material)
        _add_member(part, lower_left[index], lower_right[index + 1], brace_radius * 0.9, material)
        _add_member(part, lower_right[index], lower_left[index + 1], brace_radius * 0.9, material)

    return {
        "upper_left": upper_left,
        "upper_right": upper_right,
        "lower_left": lower_left,
        "lower_right": lower_right,
    }


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.032, 0.0, -0.125),
            (0.060, 0.0, -0.165),
            (0.065, 0.0, -0.215),
            (0.030, 0.0, -0.260),
            (-0.020, 0.0, -0.275),
            (-0.055, 0.0, -0.238),
            (-0.045, 0.0, -0.195),
        ],
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "mobile_tower_crane_hook")


def _add_outrigger(
    model: ArticulatedObject,
    *,
    parent,
    name: str,
    joint_name: str,
    joint_origin_xyz: Vec3,
    yaw: float,
    beam_material,
    steel_material,
    pad_material,
):
    outrigger = model.part(name)
    outrigger.visual(
        Box((1.40, 0.12, 0.12)),
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
        material=beam_material,
        name="beam",
    )
    outrigger.visual(
        Cylinder(radius=0.045, length=0.34),
        origin=Origin(xyz=(0.96, 0.0, -0.23)),
        material=steel_material,
        name="jack",
    )
    outrigger.visual(
        Box((0.24, 0.24, 0.03)),
        origin=Origin(xyz=(0.96, 0.0, -0.415)),
        material=pad_material,
        name="pad",
    )
    outrigger.inertial = Inertial.from_geometry(
        Box((1.42, 0.24, 0.46)),
        mass=3.0,
        origin=Origin(xyz=(0.46, 0.0, -0.16)),
    )

    articulation = model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=parent,
        child=outrigger,
        origin=Origin(xyz=joint_origin_xyz, rpy=(0.0, 0.0, yaw)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.52),
    )
    return outrigger, articulation


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.90, 0.73, 0.17, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.15, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    ballast = model.material("ballast", rgba=(0.48, 0.48, 0.47, 1.0))
    glass = model.material("glass", rgba=(0.65, 0.79, 0.86, 0.35))
    safety_red = model.material("safety_red", rgba=(0.76, 0.14, 0.10, 1.0))

    hook_mesh = _build_hook_mesh()

    carrier_base = model.part("carrier_base")
    carrier_base.visual(
        Box((1.00, 1.00, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=dark_grey,
        name="chassis",
    )
    carrier_base.visual(
        Box((0.46, 0.46, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        material=dark_grey,
        name="pedestal_core",
    )
    carrier_base.visual(
        Box((0.54, 0.54, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        material=steel,
        name="pedestal_cap",
    )
    carrier_base.visual(
        Box((0.32, 0.20, 0.14)),
        origin=Origin(xyz=(-0.12, -0.20, 0.55)),
        material=dark_grey,
    )
    carrier_base.visual(
        Box((0.28, 0.16, 0.10)),
        origin=Origin(xyz=(0.14, 0.18, 0.53)),
        material=dark_grey,
    )
    _add_socket(carrier_base, name_prefix="socket_east", origin_xyz=(0.50, 0.0, 0.24), yaw=0.0, material=steel)
    _add_socket(
        carrier_base,
        name_prefix="socket_west",
        origin_xyz=(-0.50, 0.0, 0.24),
        yaw=math.pi,
        material=steel,
    )
    _add_socket(
        carrier_base,
        name_prefix="socket_north",
        origin_xyz=(0.0, 0.50, 0.24),
        yaw=math.pi / 2.0,
        material=steel,
    )
    _add_socket(
        carrier_base,
        name_prefix="socket_south",
        origin_xyz=(0.0, -0.50, 0.24),
        yaw=-math.pi / 2.0,
        material=steel,
    )
    carrier_base.inertial = Inertial.from_geometry(
        Box((1.20, 1.20, 0.84)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
    )

    _add_outrigger(
        model,
        parent=carrier_base,
        name="outrigger_east",
        joint_name="base_to_outrigger_east",
        joint_origin_xyz=(0.50, 0.0, 0.24),
        yaw=0.0,
        beam_material=crane_yellow,
        steel_material=steel,
        pad_material=rubber,
    )
    _add_outrigger(
        model,
        parent=carrier_base,
        name="outrigger_west",
        joint_name="base_to_outrigger_west",
        joint_origin_xyz=(-0.50, 0.0, 0.24),
        yaw=math.pi,
        beam_material=crane_yellow,
        steel_material=steel,
        pad_material=rubber,
    )
    _add_outrigger(
        model,
        parent=carrier_base,
        name="outrigger_north",
        joint_name="base_to_outrigger_north",
        joint_origin_xyz=(0.0, 0.50, 0.24),
        yaw=math.pi / 2.0,
        beam_material=crane_yellow,
        steel_material=steel,
        pad_material=rubber,
    )
    _add_outrigger(
        model,
        parent=carrier_base,
        name="outrigger_south",
        joint_name="base_to_outrigger_south",
        joint_origin_xyz=(0.0, -0.50, 0.24),
        yaw=-math.pi / 2.0,
        beam_material=crane_yellow,
        steel_material=steel,
        pad_material=rubber,
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.42, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_grey,
        name="base_collar",
    )
    _add_square_lattice_mast(
        mast,
        width=0.36,
        bottom_z=0.08,
        top_z=4.88,
        panels=8,
        chord_radius=0.016,
        brace_radius=0.009,
        material=crane_yellow,
        ladder_material=steel,
    )
    mast.visual(
        Box((0.48, 0.48, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 4.94)),
        material=dark_grey,
        name="mast_head_cap",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.60, 0.60, 5.00)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 2.50)),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.FIXED,
        parent=carrier_base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
    )

    jib = model.part("jib")
    jib.visual(
        Box((1.00, 0.68, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_grey,
        name="deck_base",
    )
    jib.visual(
        Cylinder(radius=0.34, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel,
    )
    jib.visual(
        Box((0.78, 0.34, 0.34)),
        origin=Origin(xyz=(-0.54, 0.0, 0.24)),
        material=dark_grey,
    )
    jib.visual(
        Box((0.28, 0.22, 0.18)),
        origin=Origin(xyz=(0.34, -0.36, 0.24)),
        material=dark_grey,
        name="operator_cab_body",
    )
    jib.visual(
        Box((0.24, 0.18, 0.14)),
        origin=Origin(xyz=(0.37, -0.36, 0.25)),
        material=glass,
        name="operator_cab_glass",
    )
    jib.visual(
        Box((0.18, 0.20, 0.06)),
        origin=Origin(xyz=(0.30, -0.29, 0.13)),
        material=steel,
        name="operator_cab_support",
    )

    main_boom = _add_box_truss(
        jib,
        x_start=0.45,
        x_end=4.65,
        top_half_width=0.15,
        bottom_half_width=0.22,
        top_z_root=0.31,
        top_z_tip=0.31,
        bottom_z_root=0.14,
        bottom_z_tip=0.08,
        panels=10,
        chord_radius=0.016,
        brace_radius=0.009,
        material=crane_yellow,
    )
    jib.visual(
        Box((4.20, 0.06, 0.06)),
        origin=Origin(xyz=(2.55, 0.15, 0.31)),
        material=steel,
        name="left_rail",
    )
    jib.visual(
        Box((4.20, 0.06, 0.06)),
        origin=Origin(xyz=(2.55, -0.15, 0.31)),
        material=steel,
        name="right_rail",
    )
    jib.visual(
        Box((4.20, 0.04, 0.02)),
        origin=Origin(xyz=(2.55, 0.15, 0.28)),
        material=steel,
        name="left_catwalk",
    )
    jib.visual(
        Box((4.20, 0.04, 0.02)),
        origin=Origin(xyz=(2.55, -0.15, 0.28)),
        material=steel,
        name="right_catwalk",
    )
    jib.visual(
        Box((0.20, 0.28, 0.14)),
        origin=Origin(xyz=(4.75, 0.0, 0.23)),
        material=steel,
        name="boom_tip",
    )

    counter_boom = _add_box_truss(
        jib,
        x_start=-0.35,
        x_end=-1.75,
        top_half_width=0.13,
        bottom_half_width=0.19,
        top_z_root=0.31,
        top_z_tip=0.24,
        bottom_z_root=0.16,
        bottom_z_tip=0.10,
        panels=4,
        chord_radius=0.014,
        brace_radius=0.008,
        material=crane_yellow,
    )
    jib.visual(
        Box((1.44, 0.28, 0.02)),
        origin=Origin(xyz=(-1.05, 0.0, 0.26)),
        material=steel,
    )
    jib.visual(
        Box((0.28, 0.24, 0.18)),
        origin=Origin(xyz=(-1.45, 0.0, 0.23)),
        material=ballast,
    )
    jib.visual(
        Box((0.24, 0.22, 0.16)),
        origin=Origin(xyz=(-1.70, 0.0, 0.22)),
        material=ballast,
    )

    apex = (0.10, 0.0, 1.15)
    _add_member(jib, (-0.12, 0.16, 0.10), apex, 0.020, crane_yellow)
    _add_member(jib, (-0.12, -0.16, 0.10), apex, 0.020, crane_yellow)
    _add_member(jib, (0.027, -0.055, 0.80), (0.027, 0.055, 0.80), 0.012, crane_yellow)
    jib.visual(Box((0.06, 0.06, 0.05)), origin=Origin(xyz=apex), material=crane_yellow)

    _add_member(jib, apex, main_boom["upper_left"][-1], 0.006, cable)
    _add_member(jib, apex, main_boom["upper_right"][-1], 0.006, cable)
    _add_member(jib, apex, counter_boom["upper_left"][-1], 0.006, cable)
    _add_member(jib, apex, counter_boom["upper_right"][-1], 0.006, cable)

    jib.inertial = Inertial.from_geometry(
        Box((6.70, 1.00, 1.30)),
        mass=18.0,
        origin=Origin(xyz=(1.25, 0.0, 0.38)),
    )

    model.articulation(
        "mast_to_jib",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=jib,
        origin=Origin(xyz=(0.0, 0.0, 5.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.25,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.60, 0.42, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_grey,
        name="carriage",
    )
    trolley.visual(
        Box((0.30, 0.24, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=steel,
    )
    trolley.visual(
        Box((0.20, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.15, 0.04)),
        material=steel,
        name="left_shoe",
    )
    trolley.visual(
        Box((0.20, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.15, 0.04)),
        material=steel,
        name="right_shoe",
    )
    trolley.visual(
        Cylinder(radius=0.007, length=1.16),
        origin=Origin(xyz=(0.0, 0.07, -0.52)),
        material=cable,
    )
    trolley.visual(
        Cylinder(radius=0.007, length=1.16),
        origin=Origin(xyz=(0.0, -0.07, -0.52)),
        material=cable,
    )
    trolley.visual(
        Box((0.20, 0.16, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -1.13)),
        material=steel,
    )
    trolley.visual(
        Box((0.24, 0.16, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -1.24)),
        material=crane_yellow,
    )
    trolley.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -1.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    trolley.visual(
        Cylinder(radius=0.012, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -1.39)),
        material=steel,
    )
    trolley.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -1.34)),
        material=steel,
        name="hook_shank",
    )
    trolley.visual(hook_mesh, origin=Origin(xyz=(-0.032, 0.0, -1.255)), material=safety_red, name="hook")
    trolley.inertial = Inertial.from_geometry(
        Box((0.60, 0.42, 1.70)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.58)),
    )

    model.articulation(
        "jib_to_trolley",
        ArticulationType.PRISMATIC,
        parent=jib,
        child=trolley,
        origin=Origin(xyz=(1.05, 0.0, 0.34)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.80, lower=0.0, upper=3.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carrier_base = object_model.get_part("carrier_base")
    mast = object_model.get_part("mast")
    jib = object_model.get_part("jib")
    trolley = object_model.get_part("trolley")
    outrigger_east = object_model.get_part("outrigger_east")
    outrigger_west = object_model.get_part("outrigger_west")
    outrigger_north = object_model.get_part("outrigger_north")
    outrigger_south = object_model.get_part("outrigger_south")

    joint_east = object_model.get_articulation("base_to_outrigger_east")
    joint_west = object_model.get_articulation("base_to_outrigger_west")
    joint_north = object_model.get_articulation("base_to_outrigger_north")
    joint_south = object_model.get_articulation("base_to_outrigger_south")
    slew_joint = object_model.get_articulation("mast_to_jib")
    trolley_joint = object_model.get_articulation("jib_to_trolley")

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

    for part_name in (
        "carrier_base",
        "mast",
        "jib",
        "trolley",
        "outrigger_east",
        "outrigger_west",
        "outrigger_north",
        "outrigger_south",
    ):
        ctx.check(f"{part_name} exists", object_model.get_part(part_name) is not None)

    ctx.expect_contact(
        mast,
        carrier_base,
        elem_a="base_collar",
        elem_b="pedestal_cap",
        name="mast is seated on the pedestal cap",
    )
    ctx.expect_contact(
        jib,
        mast,
        elem_a="deck_base",
        elem_b="mast_head_cap",
        name="slewing deck is seated on the mast head",
    )
    ctx.expect_contact(
        trolley,
        jib,
        elem_a="left_shoe",
        elem_b="left_rail",
        name="trolley left shoe bears on left rail",
    )
    ctx.expect_contact(
        trolley,
        jib,
        elem_a="right_shoe",
        elem_b="right_rail",
        name="trolley right shoe bears on right rail",
    )

    ctx.expect_contact(
        outrigger_east,
        carrier_base,
        elem_a="beam",
        elem_b="socket_east_floor",
        name="east outrigger sits on its socket floor",
    )
    ctx.expect_contact(
        outrigger_west,
        carrier_base,
        elem_a="beam",
        elem_b="socket_west_floor",
        name="west outrigger sits on its socket floor",
    )
    ctx.expect_contact(
        outrigger_north,
        carrier_base,
        elem_a="beam",
        elem_b="socket_north_floor",
        name="north outrigger sits on its socket floor",
    )
    ctx.expect_contact(
        outrigger_south,
        carrier_base,
        elem_a="beam",
        elem_b="socket_south_floor",
        name="south outrigger sits on its socket floor",
    )
    ctx.expect_overlap(
        outrigger_east,
        carrier_base,
        axes="x",
        elem_a="beam",
        elem_b="socket_east_floor",
        min_overlap=0.35,
        name="east outrigger retains insertion at rest",
    )
    ctx.expect_overlap(
        outrigger_west,
        carrier_base,
        axes="x",
        elem_a="beam",
        elem_b="socket_west_floor",
        min_overlap=0.35,
        name="west outrigger retains insertion at rest",
    )
    ctx.expect_overlap(
        outrigger_north,
        carrier_base,
        axes="y",
        elem_a="beam",
        elem_b="socket_north_floor",
        min_overlap=0.35,
        name="north outrigger retains insertion at rest",
    )
    ctx.expect_overlap(
        outrigger_south,
        carrier_base,
        axes="y",
        elem_a="beam",
        elem_b="socket_south_floor",
        min_overlap=0.35,
        name="south outrigger retains insertion at rest",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    east_rest = ctx.part_world_position(outrigger_east)
    with ctx.pose({joint_east: 0.52}):
        east_extended = ctx.part_world_position(outrigger_east)
        ctx.expect_contact(
            outrigger_east,
            carrier_base,
            elem_a="beam",
            elem_b="socket_east_floor",
            name="east outrigger stays supported when extended",
        )
        ctx.expect_overlap(
            outrigger_east,
            carrier_base,
            axes="x",
            elem_a="beam",
            elem_b="socket_east_floor",
            min_overlap=0.18,
            name="east outrigger retains insertion when extended",
        )
    ctx.check(
        "east outrigger extends outward",
        east_rest is not None and east_extended is not None and east_extended[0] > east_rest[0] + 0.20,
        details=f"rest={east_rest}, extended={east_extended}",
    )

    west_rest = ctx.part_world_position(outrigger_west)
    with ctx.pose({joint_west: 0.52}):
        west_extended = ctx.part_world_position(outrigger_west)
        ctx.expect_contact(
            outrigger_west,
            carrier_base,
            elem_a="beam",
            elem_b="socket_west_floor",
            name="west outrigger stays supported when extended",
        )
        ctx.expect_overlap(
            outrigger_west,
            carrier_base,
            axes="x",
            elem_a="beam",
            elem_b="socket_west_floor",
            min_overlap=0.18,
            name="west outrigger retains insertion when extended",
        )
    ctx.check(
        "west outrigger extends outward",
        west_rest is not None and west_extended is not None and west_extended[0] < west_rest[0] - 0.20,
        details=f"rest={west_rest}, extended={west_extended}",
    )

    north_rest = ctx.part_world_position(outrigger_north)
    with ctx.pose({joint_north: 0.52}):
        north_extended = ctx.part_world_position(outrigger_north)
        ctx.expect_contact(
            outrigger_north,
            carrier_base,
            elem_a="beam",
            elem_b="socket_north_floor",
            name="north outrigger stays supported when extended",
        )
        ctx.expect_overlap(
            outrigger_north,
            carrier_base,
            axes="y",
            elem_a="beam",
            elem_b="socket_north_floor",
            min_overlap=0.18,
            name="north outrigger retains insertion when extended",
        )
    ctx.check(
        "north outrigger extends outward",
        north_rest is not None and north_extended is not None and north_extended[1] > north_rest[1] + 0.20,
        details=f"rest={north_rest}, extended={north_extended}",
    )

    south_rest = ctx.part_world_position(outrigger_south)
    with ctx.pose({joint_south: 0.52}):
        south_extended = ctx.part_world_position(outrigger_south)
        ctx.expect_contact(
            outrigger_south,
            carrier_base,
            elem_a="beam",
            elem_b="socket_south_floor",
            name="south outrigger stays supported when extended",
        )
        ctx.expect_overlap(
            outrigger_south,
            carrier_base,
            axes="y",
            elem_a="beam",
            elem_b="socket_south_floor",
            min_overlap=0.18,
            name="south outrigger retains insertion when extended",
        )
    ctx.check(
        "south outrigger extends outward",
        south_rest is not None and south_extended is not None and south_extended[1] < south_rest[1] - 0.20,
        details=f"rest={south_rest}, extended={south_extended}",
    )

    trolley_rest = ctx.part_world_position(trolley)
    with ctx.pose({trolley_joint: 3.15}):
        trolley_extended = ctx.part_world_position(trolley)
        ctx.expect_contact(
            trolley,
            jib,
            elem_a="left_shoe",
            elem_b="left_rail",
            name="trolley left shoe remains on the left rail at full reach",
        )
        ctx.expect_contact(
            trolley,
            jib,
            elem_a="right_shoe",
            elem_b="right_rail",
            name="trolley right shoe remains on the right rail at full reach",
        )
        ctx.expect_overlap(
            trolley,
            jib,
            axes="x",
            elem_a="left_shoe",
            elem_b="left_rail",
            min_overlap=0.14,
            name="trolley retains rail engagement at full reach",
        )
    ctx.check(
        "trolley travels outward along the jib",
        trolley_rest is not None and trolley_extended is not None and trolley_extended[0] > trolley_rest[0] + 2.5,
        details=f"rest={trolley_rest}, extended={trolley_extended}",
    )

    rest_tip_center = _center_from_aabb(ctx.part_element_world_aabb(jib, elem="boom_tip"))
    with ctx.pose({slew_joint: math.pi / 2.0}):
        swung_tip_center = _center_from_aabb(ctx.part_element_world_aabb(jib, elem="boom_tip"))
    ctx.check(
        "jib slews around the vertical mast axis",
        rest_tip_center is not None
        and swung_tip_center is not None
        and swung_tip_center[1] > rest_tip_center[1] + 3.5
        and abs(swung_tip_center[2] - rest_tip_center[2]) < 0.05,
        details=f"rest_tip={rest_tip_center}, swung_tip={swung_tip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
