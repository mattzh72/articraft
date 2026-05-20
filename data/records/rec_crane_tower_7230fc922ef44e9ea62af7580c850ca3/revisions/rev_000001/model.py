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
)


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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
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


def _round_mast_shell(name: str, *, outer_radius: float, inner_radius: float, length: float):
    shell = LatheGeometry.from_shell_profiles(
        [
            (outer_radius + 0.030, 0.00),
            (outer_radius + 0.030, 0.18),
            (outer_radius, 0.18),
            (outer_radius, length - 0.18),
            (outer_radius + 0.030, length - 0.18),
            (outer_radius + 0.030, length),
        ],
        [
            (inner_radius, 0.05),
            (inner_radius, length - 0.05),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell, name)


def _round_inner_mast(
    name: str,
    *,
    tube_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    guide_radius: float,
    guide_bands: tuple[tuple[float, float], ...],
):
    outer_profile = [(tube_radius, z_min)]
    for center_z, band_length in sorted(guide_bands):
        band_start = center_z - band_length * 0.5
        band_end = center_z + band_length * 0.5
        outer_profile.extend(
            [
                (tube_radius, band_start),
                (guide_radius, band_start),
                (guide_radius, band_end),
                (tube_radius, band_end),
            ]
        )
    outer_profile.append((tube_radius, z_max))

    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        [
            (inner_radius, z_min + 0.06),
            (inner_radius, z_max - 0.06),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_erecting_mini_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.90, 0.76, 0.16, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.24, 0.25, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    ballast = model.material("ballast", rgba=(0.54, 0.53, 0.50, 1.0))
    hydraulic = model.material("hydraulic", rgba=(0.13, 0.13, 0.14, 1.0))
    pad_grey = model.material("pad_grey", rgba=(0.44, 0.45, 0.47, 1.0))

    guide_rail_radius = 0.033
    guide_rail_offset = 0.168
    guide_rail_length = 3.06
    guide_rail_center_z = 2.01

    base = model.part("base")
    base.visual(
        Box((3.80, 0.28, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark_grey,
        name="outrigger_beam_x",
    )
    base.visual(
        Box((0.28, 3.80, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark_grey,
        name="outrigger_beam_y",
    )
    base.visual(
        Box((2.20, 2.20, 0.48)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=ballast,
        name="ballast_block",
    )
    base.visual(
        Cylinder(radius=0.42, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
        material=dark_grey,
        name="slew_pedestal",
    )
    for x, y in ((1.65, 0.0), (-1.65, 0.0), (0.0, 1.65), (0.0, -1.65)):
        base.visual(
            Cylinder(radius=0.055, length=0.16),
            origin=Origin(xyz=(x, y, 0.10)),
            material=steel,
        )
        base.visual(
            Cylinder(radius=0.18, length=0.04),
            origin=Origin(xyz=(x, y, 0.02)),
            material=pad_grey,
        )
    base.inertial = Inertial.from_geometry(
        Box((3.80, 3.80, 0.70)),
        mass=2800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=0.44, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_grey,
        name="slew_ring",
    )
    upperworks.visual(
        Box((1.60, 1.10, 0.20)),
        origin=Origin(xyz=(-0.25, 0.0, 0.20)),
        material=dark_grey,
        name="machinery_deck",
    )
    upperworks.visual(
        Box((0.92, 0.74, 0.42)),
        origin=Origin(xyz=(-0.78, 0.0, 0.51)),
        material=ballast,
        name="counterweight_pack",
    )
    upperworks.visual(
        Box((0.46, 0.42, 0.25)),
        origin=Origin(xyz=(0.48, 0.0, 0.26)),
        material=dark_grey,
        name="power_pack",
    )
    upperworks.visual(
        Cylinder(radius=0.28, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=crane_yellow,
        name="mast_base_collar",
    )
    upperworks.visual(
        Cylinder(radius=0.20, length=0.64),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=crane_yellow,
        name="outer_mast_lower_sleeve",
    )
    upperworks.visual(
        Cylinder(radius=guide_rail_radius, length=guide_rail_length),
        origin=Origin(xyz=(guide_rail_offset, 0.0, guide_rail_center_z)),
        material=crane_yellow,
        name="mast_guide_pos_x",
    )
    upperworks.visual(
        Cylinder(radius=guide_rail_radius, length=guide_rail_length),
        origin=Origin(xyz=(-guide_rail_offset, 0.0, guide_rail_center_z)),
        material=crane_yellow,
        name="mast_guide_neg_x",
    )
    upperworks.visual(
        Cylinder(radius=guide_rail_radius, length=guide_rail_length),
        origin=Origin(xyz=(0.0, guide_rail_offset, guide_rail_center_z)),
        material=crane_yellow,
        name="mast_guide_pos_y",
    )
    upperworks.visual(
        Cylinder(radius=guide_rail_radius, length=guide_rail_length),
        origin=Origin(xyz=(0.0, -guide_rail_offset, guide_rail_center_z)),
        material=crane_yellow,
        name="mast_guide_neg_y",
    )
    upperworks.inertial = Inertial.from_geometry(
        Box((1.80, 1.20, 3.70)),
        mass=900.0,
        origin=Origin(xyz=(-0.05, 0.0, 1.85)),
    )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        Cylinder(radius=0.135, length=6.00),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=crane_yellow,
        name="inner_mast_tube",
    )
    inner_mast.visual(
        Box((0.42, 0.30, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 3.44)),
        material=dark_grey,
        name="mast_head",
    )
    inner_mast.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, 6.30)),
        mass=480.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    jib = model.part("jib")
    jib.visual(
        Box((0.56, 0.30, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_grey,
        name="jib_center_carrier",
    )

    main_jib = _add_triangular_truss(
        jib,
        x_start=0.18,
        x_end=7.40,
        bottom_z=0.10,
        half_width=0.11,
        root_top_z=0.80,
        tip_top_z=0.34,
        panels=8,
        chord_radius=0.026,
        brace_radius=0.016,
        material=crane_yellow,
    )
    counter_jib = _add_triangular_truss(
        jib,
        x_start=-0.18,
        x_end=-2.25,
        bottom_z=0.10,
        half_width=0.10,
        root_top_z=0.74,
        tip_top_z=0.28,
        panels=3,
        chord_radius=0.024,
        brace_radius=0.015,
        material=crane_yellow,
    )
    jib.visual(
        Box((7.10, 0.16, 0.03)),
        origin=Origin(xyz=(3.75, 0.0, 0.08)),
        material=steel,
        name="main_jib_walkway",
    )
    jib.visual(
        Box((2.05, 0.14, 0.03)),
        origin=Origin(xyz=(-1.08, 0.0, 0.08)),
        material=steel,
        name="counter_jib_deck",
    )
    _add_member(jib, (-0.04, -0.10, 0.18), (0.22, 0.0, 1.22), 0.030, crane_yellow)
    _add_member(jib, (-0.04, 0.10, 0.18), (0.22, 0.0, 1.22), 0.030, crane_yellow)
    jib.visual(
        Box((0.14, 0.14, 0.14)),
        origin=Origin(xyz=(0.22, 0.0, 1.24)),
        material=crane_yellow,
        name="jib_apex",
    )
    jib.visual(
        Box((0.22, 0.30, 0.34)),
        origin=Origin(xyz=(7.48, 0.0, 0.24)),
        material=dark_grey,
        name="jib_tip_block",
    )
    jib.visual(
        Box((0.20, 0.26, 0.24)),
        origin=Origin(xyz=(-2.34, 0.0, 0.20)),
        material=dark_grey,
        name="counter_jib_tip_block",
    )
    _add_member(jib, (0.22, 0.0, 1.22), main_jib["upper"][5], 0.014, hydraulic)
    _add_member(jib, (0.22, 0.0, 1.22), main_jib["upper"][-1], 0.014, hydraulic)
    _add_member(jib, (0.22, 0.0, 1.22), counter_jib["upper"][-1], 0.014, hydraulic)
    jib.visual(
        Box((0.52, 0.56, 0.34)),
        origin=Origin(xyz=(-1.45, 0.0, 0.28)),
        material=ballast,
        name="rear_counterweight_left",
    )
    jib.visual(
        Box((0.52, 0.56, 0.34)),
        origin=Origin(xyz=(-1.98, 0.0, 0.28)),
        material=ballast,
        name="rear_counterweight_right",
    )
    jib.inertial = Inertial.from_geometry(
        Box((10.0, 1.20, 1.45)),
        mass=600.0,
        origin=Origin(xyz=(2.10, 0.0, 0.42)),
    )

    model.articulation(
        "slew_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15000.0, velocity=0.45),
    )
    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=upperworks,
        child=inner_mast,
        origin=Origin(xyz=(0.0, 0.0, 3.54)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.20, lower=0.0, upper=1.85),
    )
    model.articulation(
        "jib_luff",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=jib,
        origin=Origin(xyz=(0.16, 0.0, 3.58)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.35, lower=0.0, upper=1.00),
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
    upperworks = object_model.get_part("upperworks")
    inner_mast = object_model.get_part("inner_mast")
    jib = object_model.get_part("jib")

    slew = object_model.get_articulation("slew_rotation")
    mast_extension = object_model.get_articulation("mast_extension")
    jib_luff = object_model.get_articulation("jib_luff")

    mast_extension_upper = mast_extension.motion_limits.upper or 0.0
    jib_luff_upper = jib_luff.motion_limits.upper or 0.0

    with ctx.pose({slew: 0.0, mast_extension: 0.0, jib_luff: 0.0}):
        ctx.expect_contact(
            upperworks,
            base,
            elem_a="slew_ring",
            elem_b="slew_pedestal",
            name="slew ring seats on pedestal",
        )
        ctx.expect_contact(
            inner_mast,
            jib,
            elem_a="mast_head",
            elem_b="jib_center_carrier",
            name="jib hinge carrier seats on mast head",
        )
        ctx.expect_origin_distance(
            inner_mast,
            upperworks,
            axes="xy",
            max_dist=0.001,
            name="collapsed mast tube stays concentric with slewing centerline",
        )
        ctx.expect_overlap(
            inner_mast,
            upperworks,
            axes="z",
            elem_a="inner_mast_tube",
            elem_b="mast_guide_pos_x",
            min_overlap=0.95,
            name="collapsed mast tube remains deeply retained in guide frame",
        )
        ctx.expect_contact(
            inner_mast,
            upperworks,
            elem_a="inner_mast_tube",
            elem_b="mast_guide_pos_x",
            name="collapsed mast tube bears on front guide rail",
        )
        ctx.expect_contact(
            inner_mast,
            upperworks,
            elem_a="inner_mast_tube",
            elem_b="mast_guide_neg_x",
            name="collapsed mast tube bears on rear guide rail",
        )

    rest_mast_aabb = None
    extended_mast_aabb = None
    rest_tip_aabb = None
    luffed_tip_aabb = None
    slewed_tip_aabb = None

    with ctx.pose({slew: 0.0, mast_extension: 0.0, jib_luff: 0.0}):
        rest_mast_aabb = ctx.part_element_world_aabb(inner_mast, elem="inner_mast_tube")
        rest_tip_aabb = ctx.part_element_world_aabb(jib, elem="jib_tip_block")

    with ctx.pose({slew: 0.0, mast_extension: mast_extension_upper, jib_luff: 0.0}):
        ctx.expect_origin_distance(
            inner_mast,
            upperworks,
            axes="xy",
            max_dist=0.001,
            name="extended mast tube stays concentric with slewing centerline",
        )
        ctx.expect_overlap(
            inner_mast,
            upperworks,
            axes="z",
            elem_a="inner_mast_tube",
            elem_b="mast_guide_pos_x",
            min_overlap=0.75,
            name="extended mast tube retains insertion in guide frame",
        )
        ctx.expect_contact(
            inner_mast,
            upperworks,
            elem_a="inner_mast_tube",
            elem_b="mast_guide_pos_x",
            name="extended mast tube still bears on front guide rail",
        )
        extended_mast_aabb = ctx.part_element_world_aabb(inner_mast, elem="inner_mast_tube")

    with ctx.pose({slew: 0.0, mast_extension: 0.0, jib_luff: jib_luff_upper}):
        luffed_tip_aabb = ctx.part_element_world_aabb(jib, elem="jib_tip_block")

    with ctx.pose({slew: math.pi / 2.0, mast_extension: 0.0, jib_luff: 0.0}):
        slewed_tip_aabb = ctx.part_element_world_aabb(jib, elem="jib_tip_block")

    ctx.check(
        "mast extends upward",
        rest_mast_aabb is not None
        and extended_mast_aabb is not None
        and extended_mast_aabb[1][2] > rest_mast_aabb[1][2] + 1.6,
        details=f"rest={rest_mast_aabb}, extended={extended_mast_aabb}",
    )
    ctx.check(
        "jib luffs upward",
        rest_tip_aabb is not None
        and luffed_tip_aabb is not None
        and luffed_tip_aabb[1][2] > rest_tip_aabb[1][2] + 2.0,
        details=f"rest={rest_tip_aabb}, luffed={luffed_tip_aabb}",
    )

    rest_tip_center = None
    slewed_tip_center = None
    if rest_tip_aabb is not None:
        rest_tip_center = (
            (rest_tip_aabb[0][0] + rest_tip_aabb[1][0]) * 0.5,
            (rest_tip_aabb[0][1] + rest_tip_aabb[1][1]) * 0.5,
            (rest_tip_aabb[0][2] + rest_tip_aabb[1][2]) * 0.5,
        )
    if slewed_tip_aabb is not None:
        slewed_tip_center = (
            (slewed_tip_aabb[0][0] + slewed_tip_aabb[1][0]) * 0.5,
            (slewed_tip_aabb[0][1] + slewed_tip_aabb[1][1]) * 0.5,
            (slewed_tip_aabb[0][2] + slewed_tip_aabb[1][2]) * 0.5,
        )

    ctx.check(
        "slewing rotates jib around mast axis",
        rest_tip_center is not None
        and slewed_tip_center is not None
        and rest_tip_center[0] > 6.5
        and abs(rest_tip_center[1]) < 0.6
        and slewed_tip_center[1] > 6.5
        and abs(slewed_tip_center[0]) < 0.6,
        details=f"rest_center={rest_tip_center}, slewed_center={slewed_tip_center}",
    )

    ctx.check(
        "single rigid crane stack hangs from base root",
        base is not None and upperworks is not None and inner_mast is not None and jib is not None,
        details="Expected base, upperworks, inner_mast, and jib parts to resolve.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
