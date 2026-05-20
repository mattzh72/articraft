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
    Sphere,
    TestContext,
    TestReport,
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
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(horizontal, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _bell_shell_mesh(name: str, mouth_radius: float, height: float):
    wall = max(0.016, mouth_radius * 0.045)
    outer = [
        (mouth_radius, 0.0),
        (mouth_radius * 0.985, height * 0.05),
        (mouth_radius * 0.94, height * 0.12),
        (mouth_radius * 0.86, height * 0.30),
        (mouth_radius * 0.76, height * 0.56),
        (mouth_radius * 0.62, height * 0.79),
        (mouth_radius * 0.42, height * 0.94),
        (mouth_radius * 0.20, height),
    ]
    inner = [
        (mouth_radius - wall, 0.0),
        (mouth_radius * 0.89, height * 0.10),
        (mouth_radius * 0.80, height * 0.33),
        (mouth_radius * 0.69, height * 0.58),
        (mouth_radius * 0.54, height * 0.81),
        (mouth_radius * 0.33, height * 0.96),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    return mesh_from_geometry(shell, name)


def _interpolate_frame_half_span(
    lower: tuple[float, float, float],
    upper: tuple[float, float, float],
    z: float,
) -> tuple[float, float]:
    z0, hx0, hy0 = lower
    z1, hx1, hy1 = upper
    t = (z - z0) / (z1 - z0)
    return (hx0 + (hx1 - hx0) * t, hy0 + (hy1 - hy0) * t)


def _build_tower_frame(part, steel, dark_steel, concrete) -> dict[str, float]:
    levels = [
        (0.04, 1.95, 1.05),
        (1.50, 1.86, 1.00),
        (2.95, 1.77, 0.95),
        (4.15, 1.70, 0.90),
        (5.55, 1.62, 0.84),
    ]

    for sx in (-1.95, 1.95):
        for sy in (-1.05, 1.05):
            part.visual(
                Box((0.28, 0.24, 0.04)),
                origin=Origin(xyz=(sx, sy, 0.02)),
                material=concrete,
            )
            part.visual(
                Box((0.18, 0.14, 0.025)),
                origin=Origin(xyz=(sx, sy, 0.0525)),
                material=dark_steel,
            )

    corner_signs = [
        (1.0, 1.0),
        (1.0, -1.0),
        (-1.0, -1.0),
        (-1.0, 1.0),
    ]
    level_points: list[list[tuple[float, float, float]]] = []
    for z, hx, hy in levels:
        level_points.append([(sx * hx, sy * hy, z) for sx, sy in corner_signs])

    for idx in range(len(level_points) - 1):
        lower = level_points[idx]
        upper = level_points[idx + 1]
        for corner in range(4):
            _add_member(part, lower[corner], upper[corner], 0.055, steel)

    for ring in level_points:
        for i in range(4):
            _add_member(part, ring[i], ring[(i + 1) % 4], 0.040, steel)

    for idx in range(len(level_points) - 1):
        lower = level_points[idx]
        upper = level_points[idx + 1]
        if idx >= 2:
            continue
        faces = (
            (3, 0),
            (2, 1),
            (2, 3),
            (1, 0),
        )
        for a_idx, b_idx in faces:
            _add_member(part, lower[a_idx], upper[b_idx], 0.028, steel)
            _add_member(part, lower[b_idx], upper[a_idx], 0.028, steel)

    for z, hx, hy in levels[1:3]:
        _add_member(part, (-hx, 0.0, z), (hx, 0.0, z), 0.032, dark_steel)
        _add_member(part, (0.0, -hy, z), (0.0, hy, z), 0.032, dark_steel)

    bell_axis_z = 4.48
    hx_axis, hy_axis = _interpolate_frame_half_span(levels[3], levels[4], bell_axis_z)
    beam_length = hx_axis * 2.0
    beam_depth = 0.10
    beam_height = 0.12

    for beam_y, name in ((hy_axis, "front"), (-hy_axis, "rear")):
        part.visual(
            Box((beam_length, beam_depth, beam_height)),
            origin=Origin(xyz=(0.0, beam_y, bell_axis_z)),
            material=dark_steel,
            name=f"{name}_bell_beam",
        )

    top_z = levels[-1][0]
    top_hx = levels[-1][1]
    top_hy = levels[-1][2]
    part.visual(
        Box((top_hx * 2.0 - 0.18, 0.09, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, top_z - 0.06)),
        material=dark_steel,
    )
    part.visual(
        Box((0.09, top_hy * 2.0 - 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, top_z - 0.06)),
        material=dark_steel,
    )
    _add_member(part, (-top_hx, -top_hy, top_z), (top_hx, top_hy, top_z), 0.026, steel)
    _add_member(part, (-top_hx, top_hy, top_z), (top_hx, -top_hy, top_z), 0.026, steel)

    return {
        "bell_axis_z": bell_axis_z,
        "axle_length": (hy_axis - beam_depth * 0.5) * 2.0,
    }


def _add_bell_and_clapper(
    model: ArticulatedObject,
    *,
    label: str,
    x_offset: float,
    mouth_radius: float,
    shell_height: float,
    yoke_length: float,
    axle_length: float,
    swing_limits: tuple[float, float],
    bronze,
    steel,
    dark_steel,
) -> tuple[str, str]:
    bell_part_name = f"{label}_bell_yoke"
    clapper_part_name = f"{label}_clapper"
    bell_part = model.part(bell_part_name)

    beam_size_z = 0.10
    beam_center_z = -0.055
    saddle_center_z = -0.17
    saddle_size_z = 0.07
    shell_origin_z = -(shell_height + 0.22)
    pin_half = min(0.14, mouth_radius * 0.28)
    pin_z = -0.22 - shell_height * 0.16

    bell_part.visual(
        Cylinder(radius=0.030, length=axle_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    collar_y = axle_length * 0.5 - 0.16
    for y_sign, side_name in ((1.0, "front"), (-1.0, "rear")):
        bell_part.visual(
            Cylinder(radius=0.040, length=0.040),
            origin=Origin(
                xyz=(0.0, y_sign * collar_y, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"axle_collar_{side_name}",
        )

    bell_part.visual(
        Box((0.12, yoke_length, beam_size_z)),
        origin=Origin(xyz=(0.0, 0.0, beam_center_z)),
        material=steel,
        name="yoke_beam",
    )
    cheek_y = yoke_length * 0.5 - 0.08
    for y_sign, side_name in ((1.0, "front"), (-1.0, "rear")):
        bell_part.visual(
            Box((0.05, 0.10, 0.17)),
            origin=Origin(xyz=(0.0, y_sign * cheek_y, -0.085)),
            material=steel,
            name=f"yoke_cheek_{side_name}",
        )

    bell_part.visual(
        Box((0.18, 0.24, saddle_size_z)),
        origin=Origin(xyz=(0.0, 0.0, saddle_center_z)),
        material=steel,
        name="crown_saddle",
    )
    for y_sign, strap_name in ((1.0, "left"), (-1.0, "right")):
        bell_part.visual(
            Box((0.030, 0.09, 0.09)),
            origin=Origin(xyz=(0.0, y_sign * 0.095, -0.095)),
            material=steel,
            name=f"hanger_strap_{strap_name}",
        )

    bell_part.visual(
        Box((0.12, 0.18, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=bronze,
        name="crown_block",
    )
    bell_part.visual(
        _bell_shell_mesh(f"{label}_bell_shell", mouth_radius, shell_height),
        origin=Origin(xyz=(0.0, 0.0, shell_origin_z)),
        material=bronze,
        name="bell_shell",
    )

    lug_bottom = pin_z - 0.03
    lug_top = -0.205
    lug_height = lug_top - lug_bottom
    lug_center_z = (lug_top + lug_bottom) * 0.5
    for y_sign, lug_name in ((1.0, "front"), (-1.0, "rear")):
        bell_part.visual(
            Box((0.060, 0.060, lug_height)),
            origin=Origin(
                xyz=(0.0, y_sign * (pin_half + 0.03), lug_center_z),
            ),
            material=dark_steel,
            name=f"clapper_hanger_{lug_name}",
        )

    bell_part.inertial = Inertial.from_geometry(
        Box((mouth_radius * 2.2, axle_length, shell_height + 0.32)),
        mass=max(120.0, 680.0 * mouth_radius),
        origin=Origin(xyz=(0.0, 0.0, -(shell_height * 0.42 + 0.08))),
    )

    clapper_part = model.part(clapper_part_name)
    pin_radius = max(0.012, mouth_radius * 0.030)
    stem_radius = max(0.014, mouth_radius * 0.040)
    stem_length = shell_height * 0.64
    bulb_radius = mouth_radius * 0.17

    clapper_part.visual(
        Cylinder(radius=pin_radius, length=pin_half * 2.0),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pin",
    )
    clapper_part.visual(
        Cylinder(radius=stem_radius, length=stem_length),
        origin=Origin(xyz=(0.0, 0.0, -stem_length * 0.5)),
        material=dark_steel,
        name="clapper_stem",
    )
    neck_length = bulb_radius * 0.90
    clapper_part.visual(
        Cylinder(radius=bulb_radius * 0.70, length=neck_length),
        origin=Origin(xyz=(0.0, 0.0, -(stem_length + neck_length * 0.5))),
        material=dark_steel,
        name="clapper_neck",
    )
    clapper_part.visual(
        Sphere(radius=bulb_radius),
        origin=Origin(xyz=(0.0, 0.0, -(stem_length + bulb_radius * 1.25))),
        material=dark_steel,
        name="clapper_ball",
    )
    clapper_part.inertial = Inertial.from_geometry(
        Cylinder(radius=bulb_radius * 0.95, length=stem_length + bulb_radius * 1.7),
        mass=max(22.0, 95.0 * mouth_radius),
        origin=Origin(xyz=(0.0, 0.0, -(stem_length * 0.46))),
    )

    bell_joint_name = f"{label}_bell_swing"
    clapper_joint_name = f"{label}_clapper_swing"
    model.articulation(
        bell_joint_name,
        ArticulationType.REVOLUTE,
        parent="tower_frame",
        child=bell_part,
        origin=Origin(xyz=(x_offset, 0.0, FRAME_DATA["bell_axis_z"])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=1.2,
            lower=swing_limits[0],
            upper=swing_limits[1],
        ),
    )
    model.articulation(
        clapper_joint_name,
        ArticulationType.REVOLUTE,
        parent=bell_part,
        child=clapper_part,
        origin=Origin(xyz=(0.0, 0.0, pin_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=4.0,
            lower=-1.15,
            upper=1.15,
        ),
    )

    return bell_joint_name, clapper_joint_name


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carillon_bell_tower")

    galvanized_steel = model.material(
        "galvanized_steel", rgba=(0.62, 0.65, 0.68, 1.0)
    )
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    bell_bronze = model.material("bell_bronze", rgba=(0.64, 0.43, 0.18, 1.0))
    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.65, 1.0))

    tower_frame = model.part("tower_frame")
    tower_frame.inertial = Inertial.from_geometry(
        Box((4.20, 2.30, 5.65)),
        mass=3600.0,
        origin=Origin(xyz=(0.0, 0.0, 2.825)),
    )

    global FRAME_DATA
    FRAME_DATA = _build_tower_frame(
        tower_frame, galvanized_steel, dark_steel, concrete
    )

    _add_bell_and_clapper(
        model,
        label="medium",
        x_offset=-1.28,
        mouth_radius=0.41,
        shell_height=0.78,
        yoke_length=0.88,
        axle_length=FRAME_DATA["axle_length"],
        swing_limits=(-0.52, 0.52),
        bronze=bell_bronze,
        steel=galvanized_steel,
        dark_steel=dark_steel,
    )
    _add_bell_and_clapper(
        model,
        label="large",
        x_offset=0.0,
        mouth_radius=0.54,
        shell_height=1.02,
        yoke_length=1.05,
        axle_length=FRAME_DATA["axle_length"],
        swing_limits=(-0.46, 0.46),
        bronze=bell_bronze,
        steel=galvanized_steel,
        dark_steel=dark_steel,
    )
    _add_bell_and_clapper(
        model,
        label="small",
        x_offset=1.28,
        mouth_radius=0.31,
        shell_height=0.60,
        yoke_length=0.72,
        axle_length=FRAME_DATA["axle_length"],
        swing_limits=(-0.58, 0.58),
        bronze=bell_bronze,
        steel=galvanized_steel,
        dark_steel=dark_steel,
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    ctx = TestContext(object_model)
    frame = object_model.get_part("tower_frame")

    medium_bell = object_model.get_part("medium_bell_yoke")
    large_bell = object_model.get_part("large_bell_yoke")
    small_bell = object_model.get_part("small_bell_yoke")
    medium_clapper = object_model.get_part("medium_clapper")
    large_clapper = object_model.get_part("large_clapper")
    small_clapper = object_model.get_part("small_clapper")

    medium_bell_swing = object_model.get_articulation("medium_bell_swing")
    large_bell_swing = object_model.get_articulation("large_bell_swing")
    small_bell_swing = object_model.get_articulation("small_bell_swing")
    medium_clapper_swing = object_model.get_articulation("medium_clapper_swing")
    large_clapper_swing = object_model.get_articulation("large_clapper_swing")
    small_clapper_swing = object_model.get_articulation("small_clapper_swing")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for bell, clapper, prefix in (
        (medium_bell, medium_clapper, "medium"),
        (large_bell, large_clapper, "large"),
        (small_bell, small_clapper, "small"),
    ):
        ctx.expect_contact(
            frame,
            bell,
            elem_b="axle",
            name=f"{prefix}_axle_contacts_frame",
        )
        ctx.expect_contact(
            clapper,
            bell,
            elem_a="pin",
            name=f"{prefix}_clapper_pin_contacts_hanger",
        )
        ctx.expect_within(
            clapper,
            bell,
            axes="xy",
            inner_elem="clapper_ball",
            outer_elem="bell_shell",
            margin=0.0,
            name=f"{prefix}_clapper_stays_inside_bell_footprint",
        )

    for bell, joint, prefix, min_shift in (
        (medium_bell, medium_bell_swing, "medium", 0.10),
        (large_bell, large_bell_swing, "large", 0.12),
        (small_bell, small_bell_swing, "small", 0.08),
    ):
        rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
        assert rest_aabb is not None
        rest_center = _aabb_center(rest_aabb)
        with ctx.pose({joint: 0.24}):
            swung_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
            assert swung_aabb is not None
            swung_center = _aabb_center(swung_aabb)
            ctx.check(
                f"{prefix}_bell_swings_about_horizontal_axle",
                swung_center[0] < rest_center[0] - min_shift,
                details=(
                    f"Expected {prefix} bell shell center x to move negative by at least "
                    f"{min_shift:.3f} m, got {rest_center[0]:.3f} -> {swung_center[0]:.3f}."
                ),
            )
            ctx.expect_contact(
                frame,
                bell,
                elem_b="axle",
                name=f"{prefix}_axle_remains_seated_when_swung",
            )

    for clapper, joint, prefix, min_shift in (
        (medium_clapper, medium_clapper_swing, "medium", 0.08),
        (large_clapper, large_clapper_swing, "large", 0.10),
        (small_clapper, small_clapper_swing, "small", 0.06),
    ):
        rest_aabb = ctx.part_element_world_aabb(clapper, elem="clapper_ball")
        assert rest_aabb is not None
        rest_center = _aabb_center(rest_aabb)
        with ctx.pose({joint: 0.42}):
            swung_aabb = ctx.part_element_world_aabb(clapper, elem="clapper_ball")
            assert swung_aabb is not None
            swung_center = _aabb_center(swung_aabb)
            ctx.check(
                f"{prefix}_clapper_swings_on_secondary_pin",
                swung_center[0] < rest_center[0] - min_shift,
                details=(
                    f"Expected {prefix} clapper ball center x to move negative by at least "
                    f"{min_shift:.3f} m, got {rest_center[0]:.3f} -> {swung_center[0]:.3f}."
                ),
            )

    with ctx.pose(
        {
            medium_bell_swing: 0.18,
            large_bell_swing: -0.12,
            small_bell_swing: 0.18,
            medium_clapper_swing: -0.30,
            large_clapper_swing: 0.30,
            small_clapper_swing: -0.30,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="representative_swung_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
