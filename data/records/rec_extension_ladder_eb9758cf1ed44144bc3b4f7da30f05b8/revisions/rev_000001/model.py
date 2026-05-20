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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _circle_points(
    radius: float, center_y: float, z: float, *, samples: int = 14
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / samples),
            center_y + radius * math.sin(2.0 * math.pi * i / samples),
            z,
        )
        for i in range(samples)
    ]


def _point_on_hoop(
    radius: float, center_y: float, angle: float, z: float
) -> tuple[float, float, float]:
    return (radius * math.cos(angle), center_y + radius * math.sin(angle), z)


def _add_ladder_section(
    part,
    *,
    name_prefix: str,
    rail_half_span: float,
    rail_width: float,
    rail_depth: float,
    rail_height: float,
    rung_radius: float,
    rung_count: int,
    rung_bottom: float,
    rung_top: float,
    rail_material,
    rung_material,
) -> None:
    for side_name, x in (("left", -rail_half_span), ("right", rail_half_span)):
        part.visual(
            Box((rail_width, rail_depth, rail_height)),
            origin=Origin(xyz=(x, 0.0, rail_height * 0.5)),
            material=rail_material,
            name=f"{name_prefix}_{side_name}_rail",
        )

    for index in range(rung_count):
        z = rung_bottom if rung_count == 1 else rung_bottom + (rung_top - rung_bottom) * index / (rung_count - 1)
        part.visual(
            Cylinder(radius=rung_radius, length=rail_half_span * 2.0),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rung_material,
            name=f"{name_prefix}_rung_{index}",
        )


def _add_wall_mount(
    part,
    *,
    mount_name: str,
    rail_half_span: float,
    z: float,
    bracket_material,
    plate_material,
) -> None:
    for side_name, x in (("left", -rail_half_span), ("right", rail_half_span)):
        part.visual(
            Box((0.12, 0.03, 0.16)),
            origin=Origin(xyz=(x, -0.19, z)),
            material=plate_material,
            name=f"{mount_name}_{side_name}_wall_plate",
        )
        part.visual(
            Box((0.07, 0.21, 0.05)),
            origin=Origin(xyz=(x, -0.085, z)),
            material=bracket_material,
            name=f"{mount_name}_{side_name}_arm",
        )


def _add_guide_blocks(
    part,
    *,
    levels: tuple[float, ...],
    fly_rail_half_span: float,
    fly_rail_width: float,
    y_offset: float,
    material,
) -> None:
    clearance = 0.0
    guide_width = 0.010
    guide_depth = 0.064
    guide_height = 0.180
    outer_x = fly_rail_half_span + fly_rail_width * 0.5 + clearance + guide_width * 0.5

    for index, z in enumerate(levels):
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            part.visual(
                Box((guide_width, guide_depth, guide_height)),
                origin=Origin(
                    xyz=(side_sign * outer_x, y_offset - 0.026, z)
                ),
                material=material,
                name=f"guide_{side_name}_{index}_outer",
            )


def _add_safety_cage(
    part,
    *,
    hoop_mesh,
    hoop_levels: tuple[float, ...],
    hoop_radius: float,
    hoop_center_y: float,
    rail_half_span: float,
    cage_material,
) -> None:
    bracket_angle = 0.72
    strap_angles = (math.pi / 2.0, 1.03, math.pi - 1.03)

    for index, z in enumerate(hoop_levels):
        part.visual(
            hoop_mesh,
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=cage_material,
            name=f"cage_hoop_{index}",
        )
        left_attach = _point_on_hoop(hoop_radius, hoop_center_y, math.pi - bracket_angle, z)
        right_attach = _point_on_hoop(hoop_radius, hoop_center_y, bracket_angle, z)
        _add_member(
            part,
            (-rail_half_span, 0.0, z),
            left_attach,
            radius=0.012,
            material=cage_material,
            name=f"cage_left_bracket_{index}",
        )
        _add_member(
            part,
            (rail_half_span, 0.0, z),
            right_attach,
            radius=0.012,
            material=cage_material,
            name=f"cage_right_bracket_{index}",
        )

    bottom_z = hoop_levels[0]
    top_z = hoop_levels[-1]
    for strap_index, angle in enumerate(strap_angles):
        _add_member(
            part,
            _point_on_hoop(hoop_radius, hoop_center_y, angle, bottom_z),
            _point_on_hoop(hoop_radius, hoop_center_y, angle, top_z),
            radius=0.010,
            material=cage_material,
            name=f"cage_strap_{strap_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_cage_vertical_access_ladder")

    galvanized = model.material("galvanized", rgba=(0.71, 0.73, 0.76, 1.0))
    cage_yellow = model.material("cage_yellow", rgba=(0.90, 0.74, 0.16, 1.0))
    dark_mount = model.material("dark_mount", rgba=(0.28, 0.30, 0.33, 1.0))

    fixed_height = 4.35
    fixed_rail_half_span = 0.28
    fixed_rail_width = 0.055
    fixed_rail_depth = 0.028

    fly_height = 2.65
    fly_rail_half_span = 0.23
    fly_rail_width = 0.040
    fly_rail_depth = 0.024
    fly_y_offset = 0.070
    fly_base_z = 1.25

    hoop_radius = 0.34
    hoop_center_y = 0.24
    hoop_levels = (2.10, 2.62, 3.14, 3.66, 4.18)

    hoop_geometry = tube_from_spline_points(
        _circle_points(hoop_radius, hoop_center_y, 0.0, samples=14),
        radius=0.014,
        samples_per_segment=10,
        closed_spline=True,
        radial_segments=18,
        cap_ends=False,
        up_hint=(0.0, 0.0, 1.0),
    )
    hoop_mesh = mesh_from_geometry(hoop_geometry, "safety_cage_hoop")

    fixed_section = model.part("fixed_section")
    _add_ladder_section(
        fixed_section,
        name_prefix="fixed",
        rail_half_span=fixed_rail_half_span,
        rail_width=fixed_rail_width,
        rail_depth=fixed_rail_depth,
        rail_height=fixed_height,
        rung_radius=0.016,
        rung_count=14,
        rung_bottom=0.34,
        rung_top=4.02,
        rail_material=galvanized,
        rung_material=galvanized,
    )
    _add_wall_mount(
        fixed_section,
        mount_name="lower_mount",
        rail_half_span=fixed_rail_half_span,
        z=0.88,
        bracket_material=dark_mount,
        plate_material=dark_mount,
    )
    _add_wall_mount(
        fixed_section,
        mount_name="upper_mount",
        rail_half_span=fixed_rail_half_span,
        z=3.18,
        bracket_material=dark_mount,
        plate_material=dark_mount,
    )
    _add_guide_blocks(
        fixed_section,
        levels=(1.34, 3.28),
        fly_rail_half_span=fly_rail_half_span,
        fly_rail_width=fly_rail_width,
        y_offset=fly_y_offset,
        material=dark_mount,
    )
    _add_safety_cage(
        fixed_section,
        hoop_mesh=hoop_mesh,
        hoop_levels=hoop_levels,
        hoop_radius=hoop_radius,
        hoop_center_y=hoop_center_y,
        rail_half_span=fixed_rail_half_span,
        cage_material=cage_yellow,
    )
    fixed_section.inertial = Inertial.from_geometry(
        Box((0.76, 0.80, 4.40)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.10, 2.20)),
    )

    fly_section = model.part("fly_section")
    _add_ladder_section(
        fly_section,
        name_prefix="fly",
        rail_half_span=fly_rail_half_span,
        rail_width=fly_rail_width,
        rail_depth=fly_rail_depth,
        rail_height=fly_height,
        rung_radius=0.014,
        rung_count=9,
        rung_bottom=0.26,
        rung_top=2.38,
        rail_material=galvanized,
        rung_material=galvanized,
    )
    fly_section.visual(
        Box((0.42, 0.030, 0.06)),
        origin=Origin(xyz=(0.0, -0.010, 0.10)),
        material=dark_mount,
        name="fly_bottom_crosshead",
    )
    fly_section.visual(
        Box((0.42, 0.030, 0.06)),
        origin=Origin(xyz=(0.0, -0.010, fly_height - 0.10)),
        material=dark_mount,
        name="fly_top_crosshead",
    )
    fly_section.inertial = Inertial.from_geometry(
        Box((0.52, 0.12, fly_height)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, fly_height * 0.5)),
    )

    model.articulation(
        "fly_extension",
        ArticulationType.PRISMATIC,
        parent=fixed_section,
        child=fly_section,
        origin=Origin(xyz=(0.0, fly_y_offset, fly_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.45,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_section = object_model.get_part("fixed_section")
    fly_section = object_model.get_part("fly_section")
    fly_extension = object_model.get_articulation("fly_extension")

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

    lower_visual_names = {visual.name for visual in fixed_section.visuals if visual.name}
    fly_visual_names = {visual.name for visual in fly_section.visuals if visual.name}
    ctx.check(
        "safety cage hoops present",
        {"cage_hoop_0", "cage_hoop_4", "cage_strap_0"}.issubset(lower_visual_names),
        f"fixed_section visuals were {sorted(lower_visual_names)}",
    )
    ctx.check(
        "both ladder sections carry rails and rungs",
        {"fixed_left_rail", "fixed_rung_0"}.issubset(lower_visual_names)
        and {"fly_left_rail", "fly_rung_0"}.issubset(fly_visual_names),
        "Expected named rail and rung visuals on both fixed and fly sections.",
    )

    limits = fly_extension.motion_limits
    ctx.check(
        "fly section uses vertical prismatic travel",
        fly_extension.articulation_type == ArticulationType.PRISMATIC
        and tuple(fly_extension.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 1.40,
        f"Joint axis={fly_extension.axis}, type={fly_extension.articulation_type}, limits={limits}",
    )

    ctx.expect_origin_distance(
        fly_section,
        fixed_section,
        axes="x",
        max_dist=0.001,
        name="fly section centered on fixed stiles",
    )
    ctx.expect_origin_gap(
        fly_section,
        fixed_section,
        axis="y",
        min_gap=0.06,
        max_gap=0.08,
        name="fly section stands proud of fixed ladder plane",
    )
    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    rest_position = ctx.part_world_position(fly_section)
    ctx.check(
        "fly rest position resolves",
        rest_position is not None,
        "Could not resolve fly section world position in the rest pose.",
    )
    if rest_position is not None and limits is not None and limits.upper is not None:
        with ctx.pose({fly_extension: limits.upper}):
            extended_position = ctx.part_world_position(fly_section)
            ctx.check(
                "fly section extends upward through its full travel",
                extended_position is not None and extended_position[2] > rest_position[2] + 1.40,
                f"rest={rest_position}, extended={extended_position}",
            )
            ctx.expect_origin_gap(
                fly_section,
                fixed_section,
                axis="y",
                min_gap=0.06,
                max_gap=0.08,
                name="fly stays in the guide plane when extended",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
