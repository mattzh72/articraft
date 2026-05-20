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
    TorusGeometry,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_boom_truss(
    part,
    *,
    x_start: float,
    x_end: float,
    root_half_width: float,
    tip_half_width: float,
    root_lower_z: float,
    tip_lower_z: float,
    root_upper_z: float,
    tip_upper_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
) -> dict[str, list[tuple[float, float, float]]]:
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    lower_left: list[tuple[float, float, float]] = []
    lower_right: list[tuple[float, float, float]] = []
    upper: list[tuple[float, float, float]] = []
    span = x_end - x_start

    for x in xs:
        t = 0.0 if abs(span) < 1e-9 else (x - x_start) / span
        half_width = root_half_width + (tip_half_width - root_half_width) * t
        lower_z = root_lower_z + (tip_lower_z - root_lower_z) * t
        upper_z = root_upper_z + (tip_upper_z - root_upper_z) * t
        lower_left.append((x, -half_width, lower_z))
        lower_right.append((x, half_width, lower_z))
        upper.append((x, 0.0, upper_z))

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


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return (
        0.5 * (min_corner[0] + max_corner[0]),
        0.5 * (min_corner[1] + max_corner[1]),
        0.5 * (min_corner[2] + max_corner[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="derrick_mast_crane")

    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.65, 1.0))
    crane_yellow = model.material("crane_yellow", rgba=(0.90, 0.74, 0.15, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.23, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.46, 0.36, 0.28, 1.0))

    slew_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.88, tube=0.065, radial_segments=18, tubular_segments=56),
        "slew_base_ring",
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((3.60, 3.60, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=concrete,
        name="foundation_pad",
    )
    pedestal_base.visual(
        Cylinder(radius=0.74, length=1.25),
        origin=Origin(xyz=(0.0, 0.0, 1.225)),
        material=dark_grey,
        name="pedestal_pier",
    )
    pedestal_base.visual(
        Cylinder(radius=1.02, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 1.975)),
        material=weathered_steel,
        name="slew_support_cap",
    )
    pedestal_base.visual(
        slew_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 2.01)),
        material=steel,
        name="slew_base_ring",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            pedestal_base.visual(
                Cylinder(radius=0.05, length=0.22),
                origin=Origin(xyz=(0.62 * x_sign, 0.62 * y_sign, 1.97)),
                material=steel,
            )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((3.60, 3.60, 2.20)),
        mass=14000.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    upperworks = model.part("slewing_upperworks")
    upperworks.visual(
        Cylinder(radius=0.78, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_grey,
        name="turntable_drum",
    )
    upperworks.visual(
        Box((1.95, 1.30, 0.14)),
        origin=Origin(xyz=(0.08, 0.0, 0.31)),
        material=dark_grey,
        name="service_platform",
    )
    upperworks.visual(
        Box((1.08, 0.92, 0.76)),
        origin=Origin(xyz=(-0.54, 0.0, 0.69)),
        material=dark_grey,
        name="machinery_house",
    )
    upperworks.visual(
        Box((0.72, 0.82, 0.56)),
        origin=Origin(xyz=(-1.12, 0.0, 0.59)),
        material=weathered_steel,
        name="counterweight_block",
    )
    upperworks.visual(
        Cylinder(radius=0.34, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=weathered_steel,
        name="mast_step_socket",
    )
    upperworks.visual(
        Cylinder(radius=0.24, length=5.75),
        origin=Origin(xyz=(0.0, 0.0, 3.115)),
        material=crane_yellow,
        name="mast_shell",
    )
    upperworks.visual(
        Cylinder(radius=0.30, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 5.81)),
        material=weathered_steel,
        name="mast_head_band",
    )
    upperworks.visual(
        Box((0.16, 0.40, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 6.01)),
        material=weathered_steel,
        name="mast_head_yoke",
    )
    upperworks.visual(
        Box((0.18, 0.08, 0.30)),
        origin=Origin(xyz=(0.30, -0.15, 6.03)),
        material=weathered_steel,
        name="mast_head_cheek_left",
    )
    upperworks.visual(
        Box((0.18, 0.08, 0.30)),
        origin=Origin(xyz=(0.30, 0.15, 6.03)),
        material=weathered_steel,
        name="mast_head_cheek_right",
    )
    _add_member(
        upperworks,
        (0.06, -0.15, 6.03),
        (0.22, -0.15, 6.03),
        0.035,
        weathered_steel,
    )
    _add_member(
        upperworks,
        (0.06, 0.15, 6.03),
        (0.22, 0.15, 6.03),
        0.035,
        weathered_steel,
    )
    _add_member(
        upperworks,
        (0.34, -0.28, 0.24),
        (0.0, -0.18, 1.65),
        0.07,
        crane_yellow,
        name="lower_brace_left",
    )
    _add_member(
        upperworks,
        (0.34, 0.28, 0.24),
        (0.0, 0.18, 1.65),
        0.07,
        crane_yellow,
        name="lower_brace_right",
    )
    _add_member(
        upperworks,
        (-0.52, -0.30, 0.24),
        (0.0, -0.14, 2.65),
        0.055,
        steel,
    )
    _add_member(
        upperworks,
        (-0.52, 0.30, 0.24),
        (0.0, 0.14, 2.65),
        0.055,
        steel,
    )
    upperworks.inertial = Inertial.from_geometry(
        Box((3.10, 1.40, 6.30)),
        mass=4500.0,
        origin=Origin(xyz=(-0.10, 0.0, 3.15)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.11, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=weathered_steel,
        name="boom_root_hub",
    )
    boom.visual(
        Box((0.28, 0.18, 0.18)),
        origin=Origin(xyz=(0.16, 0.0, -0.03)),
        material=weathered_steel,
        name="boom_root_knuckle",
    )
    _add_member(boom, (0.0, 0.0, 0.0), (0.25, -0.19, -0.13), 0.055, weathered_steel)
    _add_member(boom, (0.0, 0.0, 0.0), (0.25, 0.19, -0.13), 0.055, weathered_steel)
    _add_member(boom, (0.0, 0.0, 0.0), (0.25, 0.0, 0.25), 0.050, weathered_steel)
    boom_truss = _add_boom_truss(
        boom,
        x_start=0.25,
        x_end=6.20,
        root_half_width=0.19,
        tip_half_width=0.08,
        root_lower_z=-0.13,
        tip_lower_z=-0.05,
        root_upper_z=0.25,
        tip_upper_z=0.12,
        panels=7,
        chord_radius=0.05,
        brace_radius=0.028,
        material=crane_yellow,
    )
    boom.visual(
        Box((0.26, 0.18, 0.18)),
        origin=Origin(xyz=(6.28, 0.0, 0.07)),
        material=weathered_steel,
        name="boom_tip_head",
    )
    _add_member(
        boom,
        boom_truss["lower_left"][-1],
        (6.28, -0.08, 0.07),
        0.038,
        crane_yellow,
    )
    _add_member(
        boom,
        boom_truss["lower_right"][-1],
        (6.28, 0.08, 0.07),
        0.038,
        crane_yellow,
    )
    _add_member(
        boom,
        boom_truss["upper"][-1],
        (6.28, 0.0, 0.15),
        0.038,
        crane_yellow,
    )
    boom.visual(
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(xyz=(6.30, 0.0, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="boom_tip_sheave_pin",
    )
    boom.inertial = Inertial.from_geometry(
        Box((6.55, 0.48, 0.62)),
        mass=1800.0,
        origin=Origin(xyz=(3.28, 0.0, 0.03)),
    )

    model.articulation(
        "slew_bearing",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80000.0, velocity=0.4),
    )
    model.articulation(
        "boom_luff",
        ArticulationType.REVOLUTE,
        parent=upperworks,
        child=boom,
        origin=Origin(xyz=(0.36, 0.0, 6.03), rpy=(0.0, -0.92, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40000.0,
            velocity=0.35,
            lower=-0.28,
            upper=0.48,
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

    pedestal_base = object_model.get_part("pedestal_base")
    upperworks = object_model.get_part("slewing_upperworks")
    boom = object_model.get_part("boom")
    slew_bearing = object_model.get_articulation("slew_bearing")
    boom_luff = object_model.get_articulation("boom_luff")

    ctx.expect_contact(
        upperworks,
        pedestal_base,
        elem_a="turntable_drum",
        elem_b="slew_support_cap",
        name="turntable drum seats on the slew support cap",
    )
    ctx.expect_overlap(
        boom,
        upperworks,
        axes="yz",
        elem_a="boom_root_hub",
        elem_b="mast_head_yoke",
        min_overlap=0.20,
        name="boom hinge hub stays aligned with the mast head yoke",
    )

    rest_tip_aabb = ctx.part_element_world_aabb(boom, elem="boom_tip_head")
    with ctx.pose({boom_luff: boom_luff.motion_limits.lower}):
        lowered_tip_aabb = ctx.part_element_world_aabb(boom, elem="boom_tip_head")
    with ctx.pose({boom_luff: boom_luff.motion_limits.upper}):
        raised_tip_aabb = ctx.part_element_world_aabb(boom, elem="boom_tip_head")
    rest_tip_center = _aabb_center(rest_tip_aabb)
    lowered_tip_center = _aabb_center(lowered_tip_aabb)
    raised_tip_center = _aabb_center(raised_tip_aabb)
    ctx.check(
        "positive boom luff raises the boom tip",
        lowered_tip_center is not None
        and raised_tip_center is not None
        and raised_tip_center[2] > lowered_tip_center[2] + 2.2,
        details=(
            f"lowered_tip={lowered_tip_center}, "
            f"rest_tip={rest_tip_center}, raised_tip={raised_tip_center}"
        ),
    )

    with ctx.pose({slew_bearing: math.pi / 2.0}):
        slewed_tip_aabb = ctx.part_element_world_aabb(boom, elem="boom_tip_head")
    slewed_tip_center = _aabb_center(slewed_tip_aabb)
    ctx.check(
        "slew bearing swings the boom around the mast",
        rest_tip_center is not None
        and slewed_tip_center is not None
        and rest_tip_center[0] > 3.0
        and abs(rest_tip_center[1]) < 0.4
        and slewed_tip_center[1] > 3.0
        and abs(slewed_tip_center[0]) < 0.8,
        details=f"rest_tip={rest_tip_center}, slewed_tip={slewed_tip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
