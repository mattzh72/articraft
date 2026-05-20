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


SCALE = 20.0


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_square_mast(
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
        _add_member(part, (half, half, z0), (-half, -half, z1), brace_radius * 0.9, material)
        _add_member(part, (half, -half, z0), (-half, half, z1), brace_radius * 0.9, material)

    ladder_x = half + 0.004 * SCALE
    rail_y = 0.022 * SCALE
    ladder_bottom = bottom_z + 0.09 * SCALE
    ladder_top = top_z - 0.08 * SCALE
    _add_member(
        part,
        (ladder_x, -rail_y, ladder_bottom),
        (ladder_x, -rail_y, ladder_top),
        0.0035 * SCALE,
        ladder_material,
    )
    _add_member(
        part,
        (ladder_x, rail_y, ladder_bottom),
        (ladder_x, rail_y, ladder_top),
        0.0035 * SCALE,
        ladder_material,
    )
    rung_count = 34
    for i in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * i / rung_count
        _add_member(
            part,
            (ladder_x, -rail_y, z),
            (ladder_x, rail_y, z),
            0.0026 * SCALE,
            ladder_material,
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
) -> dict[str, list[tuple[float, float, float]]]:
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


def _build_hook_mesh(scale: float):
    hook_geom = tube_from_spline_points(
        [
            (0.018 * scale, 0.0, -0.125 * scale),
            (0.034 * scale, 0.0, -0.155 * scale),
            (0.036 * scale, 0.0, -0.192 * scale),
            (0.018 * scale, 0.0, -0.226 * scale),
            (-0.012 * scale, 0.0, -0.236 * scale),
            (-0.034 * scale, 0.0, -0.212 * scale),
            (-0.028 * scale, 0.0, -0.176 * scale),
        ],
        radius=0.0065 * scale,
        samples_per_segment=20,
        radial_segments=20,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "hammerhead_tower_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hammerhead_tower_crane")

    tower_yellow = model.material("tower_yellow", rgba=(0.90, 0.76, 0.12, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.16, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.67, 0.82, 0.90, 0.35))
    safety_red = model.material("safety_red", rgba=(0.76, 0.12, 0.08, 1.0))

    hook_mesh = _build_hook_mesh(SCALE)

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((0.60 * SCALE, 0.60 * SCALE, 0.10 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, 0.05 * SCALE)),
        material=concrete,
        name="foundation_slab",
    )
    tower_base.visual(
        Box((0.34 * SCALE, 0.34 * SCALE, 0.06 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, 0.13 * SCALE)),
        material=concrete,
        name="pedestal_block",
    )
    tower_base.visual(
        Box((0.22 * SCALE, 0.22 * SCALE, 0.04 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, 0.18 * SCALE)),
        material=dark_grey,
        name="mast_foot",
    )
    for sx in (-0.07 * SCALE, 0.07 * SCALE):
        for sy in (-0.07 * SCALE, 0.07 * SCALE):
            tower_base.visual(
                Cylinder(radius=0.010 * SCALE, length=0.08 * SCALE),
                origin=Origin(xyz=(sx, sy, 0.14 * SCALE)),
                material=steel,
            )
    _add_square_mast(
        tower_base,
        width=0.18 * SCALE,
        bottom_z=0.20 * SCALE,
        top_z=1.58 * SCALE,
        panels=12,
        chord_radius=0.0085 * SCALE,
        brace_radius=0.0055 * SCALE,
        material=tower_yellow,
        ladder_material=steel,
    )
    tower_base.visual(
        Cylinder(radius=0.15 * SCALE, length=0.02 * SCALE),
        origin=Origin(xyz=(0.0, 0.0, 1.59 * SCALE)),
        material=dark_grey,
        name="slew_bearing_base",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((0.60 * SCALE, 0.60 * SCALE, 1.65 * SCALE)),
        mass=80000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.825 * SCALE)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=0.16 * SCALE, length=0.03 * SCALE),
        origin=Origin(xyz=(0.0, 0.0, 0.015 * SCALE)),
        material=dark_grey,
        name="slew_ring",
    )
    upperworks.visual(
        Cylinder(radius=0.05 * SCALE, length=0.10 * SCALE),
        origin=Origin(xyz=(0.0, 0.0, 0.07 * SCALE)),
        material=tower_yellow,
        name="kingpost",
    )
    upperworks.visual(
        Box((0.42 * SCALE, 0.26 * SCALE, 0.05 * SCALE)),
        origin=Origin(xyz=(0.02 * SCALE, 0.0, 0.065 * SCALE)),
        material=dark_grey,
        name="machinery_platform",
    )
    upperworks.visual(
        Box((0.16 * SCALE, 0.12 * SCALE, 0.02 * SCALE)),
        origin=Origin(xyz=(0.24 * SCALE, 0.0, 0.105 * SCALE)),
        material=steel,
        name="front_service_deck",
    )
    upperworks.visual(
        Box((0.14 * SCALE, 0.10 * SCALE, 0.02 * SCALE)),
        origin=Origin(xyz=(0.32 * SCALE, 0.0, 0.118 * SCALE)),
        material=tower_yellow,
        name="jib_root_table",
    )
    upperworks.visual(
        Box((0.30 * SCALE, 0.14 * SCALE, 0.13 * SCALE)),
        origin=Origin(xyz=(-0.16 * SCALE, 0.0, 0.145 * SCALE)),
        material=dark_grey,
        name="machinery_house",
    )
    upperworks.visual(
        Box((0.14 * SCALE, 0.10 * SCALE, 0.10 * SCALE)),
        origin=Origin(xyz=(0.18 * SCALE, -0.12 * SCALE, 0.14 * SCALE)),
        material=dark_grey,
        name="cab_shell",
    )
    upperworks.visual(
        Box((0.125 * SCALE, 0.086 * SCALE, 0.084 * SCALE)),
        origin=Origin(xyz=(0.19 * SCALE, -0.12 * SCALE, 0.142 * SCALE)),
        material=cab_glass,
        name="cab_glazing",
    )

    jib = _add_triangular_truss(
        upperworks,
        x_start=0.18 * SCALE,
        x_end=1.92 * SCALE,
        bottom_z=0.115 * SCALE,
        half_width=0.08 * SCALE,
        root_top_z=0.29 * SCALE,
        tip_top_z=0.20 * SCALE,
        panels=10,
        chord_radius=0.009 * SCALE,
        brace_radius=0.0055 * SCALE,
        material=tower_yellow,
    )
    upperworks.visual(
        Box((1.72 * SCALE, 0.040 * SCALE, 0.010 * SCALE)),
        origin=Origin(xyz=(1.05 * SCALE, 0.0, 0.100 * SCALE)),
        material=steel,
        name="jib_walkway",
    )
    left_rail_a = (0.18 * SCALE, -0.052 * SCALE, 0.118 * SCALE)
    left_rail_b = (1.86 * SCALE, -0.052 * SCALE, 0.118 * SCALE)
    upperworks.visual(
        Cylinder(radius=0.005 * SCALE, length=_distance(left_rail_a, left_rail_b)),
        origin=Origin(
            xyz=_midpoint(left_rail_a, left_rail_b),
            rpy=_rpy_for_cylinder(left_rail_a, left_rail_b),
        ),
        material=steel,
        name="jib_rail_left",
    )
    right_rail_a = (0.18 * SCALE, 0.052 * SCALE, 0.118 * SCALE)
    right_rail_b = (1.86 * SCALE, 0.052 * SCALE, 0.118 * SCALE)
    upperworks.visual(
        Cylinder(radius=0.005 * SCALE, length=_distance(right_rail_a, right_rail_b)),
        origin=Origin(
            xyz=_midpoint(right_rail_a, right_rail_b),
            rpy=_rpy_for_cylinder(right_rail_a, right_rail_b),
        ),
        material=steel,
        name="jib_rail_right",
    )

    counter_jib = _add_triangular_truss(
        upperworks,
        x_start=-0.18 * SCALE,
        x_end=-0.92 * SCALE,
        bottom_z=0.115 * SCALE,
        half_width=0.05 * SCALE,
        root_top_z=0.30 * SCALE,
        tip_top_z=0.18 * SCALE,
        panels=4,
        chord_radius=0.0085 * SCALE,
        brace_radius=0.0052 * SCALE,
        material=tower_yellow,
    )
    upperworks.visual(
        Box((0.74 * SCALE, 0.10 * SCALE, 0.012 * SCALE)),
        origin=Origin(xyz=(-0.55 * SCALE, 0.0, 0.106 * SCALE)),
        material=steel,
        name="counter_jib_walkway",
    )
    _add_member(
        upperworks,
        (-0.22 * SCALE, -0.04 * SCALE, 0.155 * SCALE),
        (-0.22 * SCALE, 0.04 * SCALE, 0.155 * SCALE),
        0.035 * SCALE,
        steel,
    )
    _add_member(
        upperworks,
        (-0.31 * SCALE, -0.04 * SCALE, 0.155 * SCALE),
        (-0.31 * SCALE, 0.04 * SCALE, 0.155 * SCALE),
        0.028 * SCALE,
        steel,
    )
    upperworks.visual(
        Box((0.14 * SCALE, 0.12 * SCALE, 0.10 * SCALE)),
        origin=Origin(xyz=(-0.60 * SCALE, 0.0, 0.162 * SCALE)),
        material=concrete,
        name="counterweight_inner",
    )
    upperworks.visual(
        Box((0.14 * SCALE, 0.12 * SCALE, 0.10 * SCALE)),
        origin=Origin(xyz=(-0.74 * SCALE, 0.0, 0.162 * SCALE)),
        material=concrete,
        name="counterweight_mid",
    )
    upperworks.visual(
        Box((0.10 * SCALE, 0.10 * SCALE, 0.08 * SCALE)),
        origin=Origin(xyz=(-0.86 * SCALE, 0.0, 0.152 * SCALE)),
        material=concrete,
        name="counterweight_outer",
    )

    apex = (0.05 * SCALE, 0.0, 0.45 * SCALE)
    _add_member(
        upperworks,
        (-0.05 * SCALE, -0.06 * SCALE, 0.10 * SCALE),
        apex,
        0.011 * SCALE,
        tower_yellow,
    )
    _add_member(
        upperworks,
        (-0.05 * SCALE, 0.06 * SCALE, 0.10 * SCALE),
        apex,
        0.011 * SCALE,
        tower_yellow,
    )
    _add_member(
        upperworks,
        (0.005 * SCALE, -0.027 * SCALE, 0.2925 * SCALE),
        (0.005 * SCALE, 0.027 * SCALE, 0.2925 * SCALE),
        0.007 * SCALE,
        tower_yellow,
    )
    upperworks.visual(
        Box((0.05 * SCALE, 0.05 * SCALE, 0.04 * SCALE)),
        origin=Origin(xyz=apex),
        material=tower_yellow,
        name="apex_head",
    )

    _add_member(upperworks, apex, jib["upper"][6], 0.004 * SCALE, cable, name="jib_pendant_inner")
    _add_member(upperworks, apex, jib["upper"][-1], 0.004 * SCALE, cable, name="jib_pendant_outer")
    _add_member(
        upperworks,
        apex,
        counter_jib["upper"][-1],
        0.004 * SCALE,
        cable,
        name="counter_jib_pendant",
    )

    upperworks.inertial = Inertial.from_geometry(
        Box((2.90 * SCALE, 0.42 * SCALE, 0.55 * SCALE)),
        mass=35000.0,
        origin=Origin(xyz=(0.50 * SCALE, 0.0, 0.20 * SCALE)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.16 * SCALE, 0.096 * SCALE, 0.045 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, -0.055 * SCALE)),
        material=dark_grey,
        name="trolley_frame",
    )
    trolley.visual(
        Box((0.14 * SCALE, 0.012 * SCALE, 0.035 * SCALE)),
        origin=Origin(xyz=(0.0, -0.052 * SCALE, -0.0275 * SCALE)),
        material=steel,
        name="left_bogie_side",
    )
    trolley.visual(
        Box((0.14 * SCALE, 0.012 * SCALE, 0.035 * SCALE)),
        origin=Origin(xyz=(0.0, 0.052 * SCALE, -0.0275 * SCALE)),
        material=steel,
        name="right_bogie_side",
    )
    trolley.visual(
        Cylinder(radius=0.005 * SCALE, length=0.014 * SCALE),
        origin=Origin(
            xyz=(-0.05 * SCALE, -0.052 * SCALE, -0.01 * SCALE),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_grey,
        name="left_front_travel_wheel",
    )
    trolley.visual(
        Cylinder(radius=0.005 * SCALE, length=0.014 * SCALE),
        origin=Origin(
            xyz=(0.05 * SCALE, -0.052 * SCALE, -0.01 * SCALE),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_grey,
        name="left_rear_travel_wheel",
    )
    trolley.visual(
        Cylinder(radius=0.005 * SCALE, length=0.014 * SCALE),
        origin=Origin(
            xyz=(-0.05 * SCALE, 0.052 * SCALE, -0.01 * SCALE),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_grey,
        name="right_front_travel_wheel",
    )
    trolley.visual(
        Cylinder(radius=0.005 * SCALE, length=0.014 * SCALE),
        origin=Origin(
            xyz=(0.05 * SCALE, 0.052 * SCALE, -0.01 * SCALE),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_grey,
        name="right_rear_travel_wheel",
    )
    trolley.visual(
        Box((0.09 * SCALE, 0.08 * SCALE, 0.04 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, -0.045 * SCALE)),
        material=steel,
        name="hoist_winch",
    )
    trolley.visual(
        Box((0.10 * SCALE, 0.06 * SCALE, 0.03 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, -0.085 * SCALE)),
        material=tower_yellow,
        name="wheel_carriage",
    )
    trolley.visual(
        Box((0.11 * SCALE, 0.07 * SCALE, 0.045 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, -0.12 * SCALE)),
        material=steel,
        name="suspension_sheave_body",
    )
    trolley.visual(
        Cylinder(radius=0.018 * SCALE, length=0.11 * SCALE),
        origin=Origin(
            xyz=(0.0, 0.0, -0.045 * SCALE),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="winch_drum",
    )
    trolley.visual(
        Cylinder(radius=0.014 * SCALE, length=0.052 * SCALE),
        origin=Origin(
            xyz=(0.0, -0.016 * SCALE, -0.038 * SCALE),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_grey,
        name="sheave_left",
    )
    trolley.visual(
        Cylinder(radius=0.014 * SCALE, length=0.052 * SCALE),
        origin=Origin(
            xyz=(0.0, 0.016 * SCALE, -0.038 * SCALE),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_grey,
        name="sheave_right",
    )
    trolley.visual(
        Cylinder(radius=0.0028 * SCALE, length=1.03 * SCALE),
        origin=Origin(xyz=(0.0, -0.016 * SCALE, -0.55 * SCALE)),
        material=cable,
        name="hoist_line_left",
    )
    trolley.visual(
        Cylinder(radius=0.0028 * SCALE, length=1.03 * SCALE),
        origin=Origin(xyz=(0.0, 0.016 * SCALE, -0.55 * SCALE)),
        material=cable,
        name="hoist_line_right",
    )
    trolley.visual(
        Cylinder(radius=0.006 * SCALE, length=0.044 * SCALE),
        origin=Origin(
            xyz=(0.0, 0.0, -1.03 * SCALE),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="hook_spreader_pin",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.18 * SCALE, 0.14 * SCALE, 1.08 * SCALE)),
        mass=3000.0,
        origin=Origin(xyz=(0.0, 0.0, -0.48 * SCALE)),
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Box((0.05 * SCALE, 0.05 * SCALE, 0.03 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, -0.055 * SCALE)),
        material=tower_yellow,
        name="upper_block",
    )
    hook_block.visual(
        Box((0.07 * SCALE, 0.05 * SCALE, 0.11 * SCALE)),
        origin=Origin(xyz=(0.0, 0.0, -0.12 * SCALE)),
        material=tower_yellow,
        name="lower_block",
    )
    _add_member(
        hook_block,
        (0.0, -0.02 * SCALE, -0.165 * SCALE),
        (0.0, 0.02 * SCALE, -0.165 * SCALE),
        0.016 * SCALE,
        steel,
        name="hook_cross_pin",
    )
    hook_block.visual(hook_mesh, material=safety_red, name="hook")
    hook_block.inertial = Inertial.from_geometry(
        Box((0.08 * SCALE, 0.06 * SCALE, 0.24 * SCALE)),
        mass=2000.0,
        origin=Origin(xyz=(0.0, 0.0, -0.12 * SCALE)),
    )

    model.articulation(
        "slewing_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower_base,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 1.60 * SCALE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150000.0, velocity=0.35),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=upperworks,
        child=trolley,
        origin=Origin(xyz=(0.64 * SCALE, 0.0, 0.118 * SCALE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50000.0,
            velocity=1.2,
            lower=0.0,
            upper=1.08 * SCALE,
        ),
    )
    model.articulation(
        "hook_suspension",
        ArticulationType.FIXED,
        parent=trolley,
        child=hook_block,
        origin=Origin(xyz=(0.0, 0.0, -1.025 * SCALE)),
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

    tower_base = object_model.get_part("tower_base")
    upperworks = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    hook_block = object_model.get_part("hook_block")
    slewing = object_model.get_articulation("slewing_rotation")
    trolley_travel = object_model.get_articulation("trolley_travel")

    ctx.check(
        "core crane parts exist",
        all(part is not None for part in (tower_base, upperworks, trolley, hook_block)),
    )
    ctx.check(
        "slewing joint rotates continuously about mast top",
        slewing.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in slewing.axis) == (0.0, 0.0, 1.0)
        and slewing.motion_limits is not None
        and slewing.motion_limits.lower is None
        and slewing.motion_limits.upper is None,
        details=f"type={slewing.articulation_type}, axis={slewing.axis}, limits={slewing.motion_limits}",
    )
    ctx.check(
        "trolley joint slides along jib axis",
        trolley_travel.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in trolley_travel.axis) == (1.0, 0.0, 0.0)
        and trolley_travel.motion_limits is not None
        and trolley_travel.motion_limits.lower == 0.0
        and trolley_travel.motion_limits.upper is not None
        and trolley_travel.motion_limits.upper > 15.0,
        details=f"type={trolley_travel.articulation_type}, axis={trolley_travel.axis}, limits={trolley_travel.motion_limits}",
    )
    ctx.expect_gap(
        upperworks,
        tower_base,
        axis="z",
        max_penetration=0.001,
        max_gap=0.01 * SCALE,
        name="upperworks seat on the mast top bearing",
    )
    ctx.expect_origin_distance(
        trolley,
        hook_block,
        axes="xy",
        max_dist=0.01,
        name="hook block stays centered below the trolley",
    )
    ctx.expect_origin_gap(
        trolley,
        hook_block,
        axis="z",
        min_gap=15.0,
        name="hook block hangs well below the trolley",
    )
    ctx.expect_contact(
        trolley,
        upperworks,
        elem_a="left_front_travel_wheel",
        elem_b="jib_rail_left",
        contact_tol=0.001,
        name="left trolley wheel rides on the left jib rail",
    )
    ctx.expect_contact(
        trolley,
        upperworks,
        elem_a="right_front_travel_wheel",
        elem_b="jib_rail_right",
        contact_tol=0.001,
        name="right trolley wheel rides on the right jib rail",
    )

    rest_pos = ctx.part_world_position(trolley)
    with ctx.pose({trolley_travel: trolley_travel.motion_limits.upper}):
        extended_pos = ctx.part_world_position(trolley)
        ctx.expect_origin_distance(
            trolley,
            hook_block,
            axes="xy",
            max_dist=0.01,
            name="hook block remains vertically aligned when the trolley extends",
        )
        ctx.expect_origin_gap(
            trolley,
            hook_block,
            axis="z",
            min_gap=15.0,
            name="hook block remains suspended below the trolley at full reach",
        )

    with ctx.pose({slewing: math.pi / 2.0, trolley_travel: trolley_travel.motion_limits.upper}):
        quarter_turn_pos = ctx.part_world_position(trolley)

    ctx.check(
        "trolley can travel outward along the saddle jib",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 15.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "slewing rotates the jib and trolley around the mast",
        extended_pos is not None
        and quarter_turn_pos is not None
        and abs(quarter_turn_pos[0]) < 2.0
        and quarter_turn_pos[1] > extended_pos[0] - 2.0,
        details=f"extended={extended_pos}, quarter_turn={quarter_turn_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
