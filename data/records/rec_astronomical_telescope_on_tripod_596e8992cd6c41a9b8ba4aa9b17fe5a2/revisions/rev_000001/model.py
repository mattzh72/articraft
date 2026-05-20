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
    tube_from_spline_points,
)


def _shell_tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 48,
):
    outer_profile = [
        (outer_radius, -length / 2.0),
        (outer_radius, length / 2.0),
    ]
    inner_profile = [
        (inner_radius, -length / 2.0),
        (inner_radius, length / 2.0),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (
        Origin(
            xyz=(
                (start[0] + end[0]) / 2.0,
                (start[1] + end[1]) / 2.0,
                (start[2] + end[2]) / 2.0,
            ),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _point_lerp(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    t: float,
) -> tuple[float, float, float]:
    return (
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
        start[2] + (end[2] - start[2]) * t,
    )


def _add_rod(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="binocular_telescope_mount")

    painted_white = model.material("painted_white", rgba=(0.93, 0.94, 0.96, 1.0))
    steel = model.material("steel", rgba=(0.28, 0.30, 0.33, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.12, 0.13, 0.15, 1.0))
    black = model.material("black", rgba=(0.05, 0.05, 0.06, 1.0))
    glass = model.material("glass", rgba=(0.35, 0.50, 0.62, 0.55))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    main_tube_mesh = _shell_tube_mesh(
        "main_tube_shell",
        outer_radius=0.055,
        inner_radius=0.051,
        length=0.49,
    )
    objective_cell_mesh = _shell_tube_mesh(
        "objective_cell_shell",
        outer_radius=0.062,
        inner_radius=0.054,
        length=0.06,
    )
    dew_shield_mesh = _shell_tube_mesh(
        "dew_shield_shell",
        outer_radius=0.062,
        inner_radius=0.058,
        length=0.17,
    )
    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 1.28)),
        material=steel,
        name="mount_plate",
    )
    tripod.visual(
        Cylinder(radius=0.17, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.19)),
        material=steel,
        name="crown_hub",
    )
    tripod.visual(
        Cylinder(radius=0.055, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material=steel,
        name="spreader_column",
    )
    tripod.visual(
        Cylinder(radius=0.065, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
        material=dark_anodized,
        name="spreader_hub",
    )
    for index, angle in enumerate(
        (math.radians(18.0), math.radians(138.0), math.radians(258.0))
    ):
        top = (0.13 * math.cos(angle), 0.13 * math.sin(angle), 1.12)
        foot = (0.58 * math.cos(angle), 0.58 * math.sin(angle), 0.03)
        mid = _point_lerp(top, foot, 0.48)

        _add_rod(
            tripod,
            f"leg_upper_{index}",
            top,
            mid,
            radius=0.032,
            material=steel,
        )
        _add_rod(
            tripod,
            f"leg_lower_{index}",
            mid,
            foot,
            radius=0.026,
            material=steel,
        )
        _add_rod(
            tripod,
            f"spreader_bar_{index}",
            (0.0, 0.0, 0.61),
            _point_lerp(top, foot, 0.39),
            radius=0.012,
            material=dark_anodized,
        )
        tripod.visual(
            Box((0.085, 0.050, 0.030)),
            origin=Origin(
                xyz=(0.60 * math.cos(angle), 0.60 * math.sin(angle), 0.015),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=f"foot_pad_{index}",
        )
    tripod.inertial = Inertial.from_geometry(
        Box((1.25, 1.25, 1.32)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
    )

    yoke = model.part("azimuth_head")
    yoke.visual(
        Cylinder(radius=0.16, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_anodized,
        name="azimuth_base",
    )
    yoke.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=steel,
        name="turntable_deck",
    )
    yoke.visual(
        Box((0.20, 0.18, 0.10)),
        origin=Origin(xyz=(0.04, 0.11, 0.15)),
        material=steel,
        name="arm_shoulder",
    )
    yoke.visual(
        Box((0.08, 0.10, 0.18)),
        origin=Origin(xyz=(0.15, 0.24, 0.16)),
        material=steel,
        name="arm_column",
    )
    yoke.visual(
        Box((0.10, 0.08, 0.10)),
        origin=Origin(xyz=(0.16, 0.29, 0.29)),
        material=steel,
        name="arm_head",
    )
    yoke.visual(
        Box((0.09, 0.02, 0.12)),
        origin=Origin(xyz=(0.16, 0.24, 0.35)),
        material=dark_anodized,
        name="altitude_bearing_pad",
    )
    yoke.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(-0.08, 0.00, 0.13)),
        material=steel,
        name="drive_box",
    )
    yoke.visual(
        Box((0.18, 0.08, 0.08)),
        origin=Origin(xyz=(0.08, 0.18, 0.09)),
        material=steel,
        name="arm_gusset",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.42, 0.40, 0.44)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.18, 0.20)),
    )

    bino = model.part("binocular_assembly")
    bino.visual(
        Cylinder(radius=0.04, length=0.05),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="altitude_trunnion",
    )
    bino.visual(
        Box((0.12, 0.045, 0.16)),
        origin=Origin(xyz=(0.04, -0.0425, 0.0)),
        material=steel,
        name="side_saddle",
    )
    bino.visual(
        Box((0.22, 0.24, 0.05)),
        origin=Origin(xyz=(0.02, -0.22, -0.08)),
        material=steel,
        name="rear_bridge",
    )
    bino.visual(
        Box((0.18, 0.24, 0.05)),
        origin=Origin(xyz=(0.28, -0.22, -0.08)),
        material=steel,
        name="front_bridge",
    )
    bino.visual(
        Box((0.36, 0.12, 0.04)),
        origin=Origin(xyz=(0.15, -0.22, -0.095)),
        material=dark_anodized,
        name="lower_bridge",
    )
    bino.visual(
        Box((0.26, 0.24, 0.025)),
        origin=Origin(xyz=(0.10, -0.22, 0.0675)),
        material=dark_anodized,
        name="top_bridge",
    )
    bino.visual(
        Box((0.10, 0.16, 0.14)),
        origin=Origin(xyz=(0.07, -0.13, -0.01)),
        material=steel,
        name="center_web",
    )
    _add_rod(
        bino,
        "handle_post_left",
        (0.01, -0.31, 0.08),
        (0.01, -0.31, 0.16),
        radius=0.008,
        material=dark_anodized,
    )
    _add_rod(
        bino,
        "handle_post_right",
        (0.19, -0.13, 0.08),
        (0.19, -0.13, 0.16),
        radius=0.008,
        material=dark_anodized,
    )
    _add_rod(
        bino,
        "carry_handle_bar",
        (0.01, -0.31, 0.16),
        (0.19, -0.13, 0.16),
        radius=0.008,
        material=dark_anodized,
    )

    tube_specs = (
        ("left", -0.31),
        ("right", -0.13),
    )
    for side, tube_y in tube_specs:
        bino.visual(
            main_tube_mesh,
            origin=Origin(xyz=(0.30, tube_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=painted_white,
            name=f"{side}_main_tube",
        )
        bino.visual(
            objective_cell_mesh,
            origin=Origin(xyz=(0.575, tube_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=painted_white,
            name=f"{side}_objective_cell",
        )
        bino.visual(
            dew_shield_mesh,
            origin=Origin(xyz=(0.69, tube_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"{side}_dew_shield",
        )
        bino.visual(
            Box((0.13, 0.09, 0.12)),
            origin=Origin(xyz=(-0.025, tube_y, 0.01)),
            material=painted_white,
            name=f"{side}_prism_housing",
        )
        bino.visual(
            Cylinder(radius=0.016, length=0.09),
            origin=Origin(xyz=(-0.01, tube_y, 0.12)),
            material=dark_anodized,
            name=f"{side}_eyepiece_barrel",
        )
        bino.visual(
            Cylinder(radius=0.020, length=0.025),
            origin=Origin(xyz=(-0.01, tube_y, 0.1775)),
            material=rubber,
            name=f"{side}_eyecup",
        )
    bino.visual(
        Box((0.05, 0.30, 0.04)),
        origin=Origin(xyz=(-0.02, -0.22, 0.08)),
        material=dark_anodized,
        name="eyepiece_bridge",
    )
    bino.inertial = Inertial.from_geometry(
        Box((0.82, 0.38, 0.28)),
        mass=13.0,
        origin=Origin(xyz=(0.18, -0.16, 0.01)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=bino,
        origin=Origin(xyz=(0.16, 0.205, 0.35)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=-0.35,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    yoke = object_model.get_part("azimuth_head")
    bino = object_model.get_part("binocular_assembly")
    azimuth = object_model.get_articulation("azimuth_rotation")
    altitude = object_model.get_articulation("altitude_axis")

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

    ctx.check(
        "part_count",
        len(object_model.parts) == 3,
        f"expected 3 authored parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "tube_visuals_present",
        len([v for v in bino.visuals if v.name and v.name.endswith("_main_tube")]) == 2,
        "expected two parallel main tube visuals",
    )
    ctx.check(
        "azimuth_axis_vertical",
        tuple(round(value, 3) for value in azimuth.axis) == (0.0, 0.0, 1.0),
        f"azimuth axis was {azimuth.axis}",
    )
    ctx.check(
        "altitude_axis_horizontal",
        tuple(round(value, 3) for value in altitude.axis) == (0.0, -1.0, 0.0),
        f"altitude axis was {altitude.axis}",
    )

    altitude_limits = altitude.motion_limits
    ctx.check(
        "altitude_limits_realistic",
        altitude_limits is not None
        and altitude_limits.lower is not None
        and altitude_limits.upper is not None
        and -0.5 <= altitude_limits.lower <= 0.0
        and 1.0 <= altitude_limits.upper <= 1.4,
        f"altitude limits were {altitude_limits}",
    )

    tripod_aabb = ctx.part_world_aabb(tripod)
    tripod_height = None
    if tripod_aabb is not None:
        tripod_height = tripod_aabb[1][2] - tripod_aabb[0][2]
    ctx.check(
        "tripod_height_realistic",
        tripod_height is not None and 1.20 <= tripod_height <= 1.40,
        f"tripod height was {tripod_height}",
    )

    bino_aabb = ctx.part_world_aabb(bino)
    bino_length = None
    bino_width = None
    if bino_aabb is not None:
        bino_length = bino_aabb[1][0] - bino_aabb[0][0]
        bino_width = bino_aabb[1][1] - bino_aabb[0][1]
    ctx.check(
        "binocular_scale_realistic",
        bino_length is not None
        and bino_width is not None
        and 0.70 <= bino_length <= 0.95
        and 0.28 <= bino_width <= 0.42,
        f"binocular length={bino_length}, width={bino_width}",
    )

    left_tube_aabb = ctx.part_element_world_aabb(bino, elem="left_main_tube")
    right_tube_aabb = ctx.part_element_world_aabb(bino, elem="right_main_tube")
    parallel_tubes = False
    if left_tube_aabb is not None and right_tube_aabb is not None:
        parallel_tubes = (
            abs(left_tube_aabb[0][2] - right_tube_aabb[0][2]) <= 0.002
            and abs(left_tube_aabb[1][2] - right_tube_aabb[1][2]) <= 0.002
            and abs(
                (left_tube_aabb[1][0] - left_tube_aabb[0][0])
                - (right_tube_aabb[1][0] - right_tube_aabb[0][0])
            )
            <= 0.002
        )
    ctx.check(
        "parallel_tubes_aligned",
        parallel_tubes,
        "left and right refractor tubes should share the same height and length",
    )

    with ctx.pose({azimuth: 0.0, altitude: 0.0}):
        ctx.expect_contact(
            yoke,
            tripod,
            elem_a="azimuth_base",
            elem_b="mount_plate",
            name="azimuth_bearing_contact",
        )
        ctx.expect_gap(
            yoke,
            tripod,
            axis="z",
            max_gap=0.0,
            max_penetration=0.0,
            positive_elem="azimuth_base",
            negative_elem="mount_plate",
            name="azimuth_bearing_seated",
        )
        ctx.expect_overlap(
            yoke,
            tripod,
            axes="xy",
            min_overlap=0.25,
            elem_a="azimuth_base",
            elem_b="mount_plate",
            name="azimuth_bearing_overlap",
        )
        ctx.expect_contact(
            bino,
            yoke,
            elem_a="altitude_trunnion",
            elem_b="altitude_bearing_pad",
            name="altitude_bearing_contact",
        )
        ctx.expect_gap(
            yoke,
            bino,
            axis="y",
            max_gap=0.0,
            max_penetration=0.0,
            positive_elem="altitude_bearing_pad",
            negative_elem="altitude_trunnion",
            name="altitude_bearing_seated",
        )
        ctx.expect_overlap(
            bino,
            yoke,
            axes="xz",
            min_overlap=0.08,
            elem_a="altitude_trunnion",
            elem_b="altitude_bearing_pad",
            name="altitude_bearing_overlap",
        )

    if altitude_limits is not None and altitude_limits.lower is not None and altitude_limits.upper is not None:
        with ctx.pose({altitude: altitude_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="altitude_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="altitude_lower_no_floating")
            ctx.expect_contact(
                bino,
                yoke,
                elem_a="altitude_trunnion",
                elem_b="altitude_bearing_pad",
                name="altitude_lower_contact",
            )
        with ctx.pose({altitude: altitude_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="altitude_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="altitude_upper_no_floating")
            ctx.expect_contact(
                bino,
                yoke,
                elem_a="altitude_trunnion",
                elem_b="altitude_bearing_pad",
                name="altitude_upper_contact",
            )

    with ctx.pose({azimuth: 1.2, altitude: 0.8}):
        ctx.fail_if_parts_overlap_in_current_pose(name="slew_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="slew_pose_no_floating")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=20,
        ignore_adjacent=False,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=12,
        name="sampled_pose_no_floating",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
