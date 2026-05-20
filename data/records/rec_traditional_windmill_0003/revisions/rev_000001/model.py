from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x_pos: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_center + z_pos)
        for z_pos, y_pos in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=8,
        )
    ]


def _build_tower_shell():
    outer_profile = [
        (2.72, 0.00),
        (2.66, 0.55),
        (2.54, 1.50),
        (2.38, 3.10),
        (2.18, 5.20),
        (2.02, 6.90),
        (1.92, 8.00),
    ]
    inner_profile = [
        (2.20, 0.18),
        (2.14, 0.70),
        (2.03, 1.60),
        (1.88, 3.10),
        (1.72, 5.20),
        (1.58, 6.90),
        (1.50, 7.92),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_cap_shell():
    return section_loft(
        [
            _yz_section(1.14, 1.08, 0.18, -1.50, 1.00),
            _yz_section(1.98, 1.58, 0.26, -0.92, 1.04),
            _yz_section(2.78, 2.04, 0.34, -0.16, 1.14),
            _yz_section(2.56, 1.98, 0.32, 0.36, 1.18),
            _yz_section(1.94, 1.62, 0.24, 0.70, 1.12),
            _yz_section(1.12, 1.04, 0.18, 0.84, 1.00),
        ]
    )


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _add_blade_lattice(
    rotor_part,
    *,
    angle: float,
    x_pos: float,
    material,
    prefix: str,
) -> None:
    span_start = 1.06
    span_length = 14.20
    spar_offset = 0.42
    spar_depth = 0.10

    for side, y_pos in (("port", spar_offset), ("starboard", -spar_offset)):
        rotor_part.visual(
            Box((0.08, spar_depth, span_length)),
            origin=Origin(
                xyz=(x_pos, y_pos, span_start + (span_length * 0.5)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_{side}_spar",
        )

    rotor_part.visual(
        Box((0.05, 0.08, 14.20)),
        origin=Origin(
            xyz=(x_pos, 0.0, span_start + (span_length * 0.5)),
            rpy=(angle, 0.0, 0.0),
        ),
        material=material,
        name=f"{prefix}_center_spar",
    )

    rung_positions = [1.34, 2.54, 3.76, 5.00, 6.26, 7.56, 8.90, 10.28, 11.68, 13.10, 14.54]
    for index, z_pos in enumerate(rung_positions):
        width = 1.00 - (0.038 * index)
        rotor_part.visual(
            Box((0.05, width, 0.08)),
            origin=Origin(
                xyz=(x_pos, 0.0, z_pos),
                rpy=(angle, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_rung_{index}",
        )

    slat_positions = [1.78, 3.06, 4.38, 5.74, 7.14, 8.58, 10.06, 11.58, 13.14, 14.72]
    for index, z_pos in enumerate(slat_positions):
        width = 0.90 - (0.033 * index)
        rotor_part.visual(
            Box((0.026, width, 0.020)),
            origin=Origin(
                xyz=(x_pos + 0.026, 0.0, z_pos),
                rpy=(angle, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_slat_{index}",
        )

    rotor_part.visual(
        Box((0.10, 0.96, 0.18)),
        origin=Origin(
            xyz=(x_pos, 0.0, 1.12),
            rpy=(angle, 0.0, 0.0),
        ),
        material=material,
        name=f"{prefix}_root_batten",
    )
    rotor_part.visual(
        Box((0.06, 0.88, 0.18)),
        origin=Origin(
            xyz=(x_pos, 0.0, 15.12),
            rpy=(angle, 0.0, 0.0),
        ),
        material=material,
        name=f"{prefix}_tip_batten",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill_premium", assets=ASSETS)

    tower_stone = model.material("tower_stone", rgba=(0.82, 0.80, 0.75, 1.0))
    lime_render = model.material("lime_render", rgba=(0.88, 0.86, 0.81, 1.0))
    weathered_oak = model.material("weathered_oak", rgba=(0.47, 0.35, 0.23, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.24, 0.18, 0.11, 1.0))
    iron_dark = model.material("iron_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    iron_satin = model.material("iron_satin", rgba=(0.46, 0.47, 0.48, 1.0))
    shadow = model.material("shadow", rgba=(0.08, 0.08, 0.09, 1.0))

    tower = model.part("tower")
    tower.visual(
        _save_mesh("tower_shell.obj", _build_tower_shell()),
        material=tower_stone,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.84, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=lime_render,
        name="stone_plinth",
    )
    tower.visual(
        Cylinder(radius=2.22, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 2.05)),
        material=lime_render,
        name="mid_band_lower",
    )
    tower.visual(
        Cylinder(radius=2.10, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 4.35)),
        material=lime_render,
        name="mid_band_middle",
    )
    tower.visual(
        Cylinder(radius=1.98, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 6.60)),
        material=lime_render,
        name="mid_band_upper",
    )
    tower.visual(
        Cylinder(radius=2.06, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 7.94)),
        material=lime_render,
        name="tower_cornice",
    )
    tower.visual(
        Cylinder(radius=2.02, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 8.10)),
        material=dark_oak,
        name="curb_ring",
    )
    tower.visual(
        Box((0.24, 1.18, 2.16)),
        origin=Origin(xyz=(0.0, -2.50, 1.14)),
        material=dark_oak,
        name="side_door_surround",
    )
    tower.visual(
        Box((0.05, 0.88, 1.78)),
        origin=Origin(xyz=(0.0, -2.57, 0.95)),
        material=shadow,
        name="side_door_recess",
    )
    tower.visual(
        Box((0.10, 0.70, 0.94)),
        origin=Origin(xyz=(0.0, 2.20, 4.48)),
        material=dark_oak,
        name="window_frame_lower",
    )
    tower.visual(
        Box((0.04, 0.48, 0.62)),
        origin=Origin(xyz=(0.0, 2.25, 4.48)),
        material=shadow,
        name="window_recess_lower",
    )
    tower.visual(
        Box((0.08, 0.56, 0.78)),
        origin=Origin(xyz=(0.0, 2.00, 6.48)),
        material=dark_oak,
        name="window_frame_upper",
    )
    tower.visual(
        Box((0.03, 0.38, 0.54)),
        origin=Origin(xyz=(0.0, 2.04, 6.48)),
        material=shadow,
        name="window_recess_upper",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.84, length=8.20),
        mass=68000.0,
        origin=Origin(xyz=(0.0, 0.0, 4.10)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=2.10, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_oak,
        name="cap_base_ring",
    )
    cap.visual(
        Cylinder(radius=2.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=iron_dark,
        name="cap_seam_band",
    )
    cap.visual(
        _save_mesh("cap_shell.obj", _build_cap_shell()),
        material=weathered_oak,
        name="cap_shell",
    )
    cap.visual(
        Box((2.44, 0.14, 0.12)),
        origin=Origin(xyz=(-0.10, 0.0, 2.16)),
        material=dark_oak,
        name="ridge_cap",
    )
    cap.visual(
        Box((0.34, 1.92, 1.02)),
        origin=Origin(xyz=(1.02, 0.0, 1.06)),
        material=dark_oak,
        name="front_windboard",
    )

    windshaft_rear_x = 1.30
    windshaft_front_x = 1.92
    shaft_axis_z = 1.15

    for prefix, x_pos in (("rear", windshaft_rear_x), ("front", windshaft_front_x)):
        cap.visual(
            Box((0.18, 0.38, 0.10)),
            origin=Origin(xyz=(x_pos, 0.0, shaft_axis_z + 0.15)),
            material=iron_dark,
            name=f"{prefix}_upper_bearing",
        )
        cap.visual(
            Box((0.18, 0.38, 0.10)),
            origin=Origin(xyz=(x_pos, 0.0, shaft_axis_z - 0.15)),
            material=iron_dark,
            name=f"{prefix}_lower_bearing",
        )
        for side, y_pos in (("left", 0.22), ("right", -0.22)):
            cap.visual(
                Box((0.12, 0.12, 0.56)),
                origin=Origin(xyz=(x_pos, y_pos, shaft_axis_z)),
                material=iron_satin,
                name=f"{prefix}_{side}_bearing_post",
            )

    cap.visual(
        Box((1.48, 0.10, 0.12)),
        origin=Origin(xyz=(1.20, 0.28, 1.48)),
        material=iron_satin,
        name="left_cheek_tie",
    )
    cap.visual(
        Box((1.48, 0.10, 0.12)),
        origin=Origin(xyz=(1.20, -0.28, 1.48)),
        material=iron_satin,
        name="right_cheek_tie",
    )
    cap.visual(
        Box((1.18, 0.10, 0.16)),
        origin=Origin(xyz=(1.10, 0.30, 0.80)),
        material=dark_oak,
        name="left_lower_frame",
    )
    cap.visual(
        Box((1.18, 0.10, 0.16)),
        origin=Origin(xyz=(1.10, -0.30, 0.80)),
        material=dark_oak,
        name="right_lower_frame",
    )

    front_left_brace = tube_from_spline_points(
        [(0.56, 0.84, 0.24), (1.18, 0.46, 0.66), (1.92, 0.22, 1.02)],
        radius=0.06,
        samples_per_segment=12,
        radial_segments=16,
    )
    rear_left_brace = tube_from_spline_points(
        [(0.02, 0.84, 0.22), (0.42, 0.46, 0.58), (1.30, 0.22, 1.02)],
        radius=0.055,
        samples_per_segment=12,
        radial_segments=16,
    )
    front_right_brace = tube_from_spline_points(
        _mirror_y([(0.56, 0.84, 0.24), (1.18, 0.46, 0.66), (1.92, 0.22, 1.02)]),
        radius=0.06,
        samples_per_segment=12,
        radial_segments=16,
    )
    rear_right_brace = tube_from_spline_points(
        _mirror_y([(0.02, 0.84, 0.22), (0.42, 0.46, 0.58), (1.30, 0.22, 1.02)]),
        radius=0.055,
        samples_per_segment=12,
        radial_segments=16,
    )
    cap.visual(
        _save_mesh("front_left_frame_brace.obj", front_left_brace),
        material=dark_oak,
        name="front_left_frame_brace",
    )
    cap.visual(
        _save_mesh("front_right_frame_brace.obj", front_right_brace),
        material=dark_oak,
        name="front_right_frame_brace",
    )
    cap.visual(
        _save_mesh("rear_left_frame_brace.obj", rear_left_brace),
        material=dark_oak,
        name="rear_left_frame_brace",
    )
    cap.visual(
        _save_mesh("rear_right_frame_brace.obj", rear_right_brace),
        material=dark_oak,
        name="rear_right_frame_brace",
    )

    tailpole = tube_from_spline_points(
        [(-1.36, 0.0, 0.84), (-2.18, 0.0, 0.94), (-3.58, 0.0, 0.70)],
        radius=0.08,
        samples_per_segment=14,
        radial_segments=18,
    )
    tail_left = tube_from_spline_points(
        [(-1.12, 0.50, 0.72), (-2.18, 0.24, 0.86), (-3.10, 0.0, 0.74)],
        radius=0.06,
        samples_per_segment=12,
        radial_segments=16,
    )
    tail_right = tube_from_spline_points(
        _mirror_y([(-1.12, 0.50, 0.72), (-2.18, 0.24, 0.86), (-3.10, 0.0, 0.74)]),
        radius=0.06,
        samples_per_segment=12,
        radial_segments=16,
    )
    cap.visual(
        _save_mesh("tailpole_beam.obj", tailpole),
        material=dark_oak,
        name="tailpole_beam",
    )
    cap.visual(
        _save_mesh("tail_left_brace.obj", tail_left),
        material=weathered_oak,
        name="tail_left_brace",
    )
    cap.visual(
        _save_mesh("tail_right_brace.obj", tail_right),
        material=weathered_oak,
        name="tail_right_brace",
    )
    cap.visual(
        _save_mesh(
            "tail_handwheel.obj",
            TorusGeometry(
                radius=0.32,
                tube=0.035,
                radial_segments=16,
                tubular_segments=40,
            )
            .rotate_y(math.pi / 2.0)
            .translate(-3.62, 0.0, 0.70),
        ),
        material=iron_satin,
        name="tail_handwheel",
    )
    cap.visual(
        Cylinder(radius=0.05, length=0.24),
        origin=Origin(xyz=(-3.62, 0.0, 0.70), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_dark,
        name="tail_handwheel_hub",
    )
    cap.visual(
        Cylinder(radius=0.016, length=0.64),
        origin=Origin(xyz=(-3.62, 0.0, 0.70)),
        material=iron_dark,
        name="tail_handwheel_spoke_vertical",
    )
    cap.visual(
        Cylinder(radius=0.016, length=0.64),
        origin=Origin(xyz=(-3.62, 0.0, 0.70), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_dark,
        name="tail_handwheel_spoke_horizontal",
    )
    cap.visual(
        Cylinder(radius=0.02, length=0.48),
        origin=Origin(xyz=(0.16, 0.0, 2.40)),
        material=iron_dark,
        name="lightning_rod",
    )
    cap.inertial = Inertial.from_geometry(
        Box((7.40, 4.50, 2.90)),
        mass=7200.0,
        origin=Origin(xyz=(-0.25, 0.0, 1.18)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.10, length=1.46),
        origin=Origin(xyz=(0.73, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_dark,
        name="windshaft",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.16),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_satin,
        name="rear_collar",
    )
    rotor.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(1.42, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_satin,
        name="front_collar",
    )
    rotor.visual(
        Cylinder(radius=0.42, length=0.88),
        origin=Origin(xyz=(1.78, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_dark,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.52, length=0.16),
        origin=Origin(xyz=(2.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_satin,
        name="hub_faceplate",
    )
    rotor.visual(
        Box((0.28, 0.24, 2.90)),
        origin=Origin(xyz=(1.84, 0.0, 0.0)),
        material=weathered_oak,
        name="vertical_stock",
    )
    rotor.visual(
        Box((0.28, 2.90, 0.24)),
        origin=Origin(xyz=(1.84, 0.0, 0.0)),
        material=weathered_oak,
        name="horizontal_stock",
    )

    blade_x = 2.02
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _add_blade_lattice(
            rotor,
            angle=angle,
            x_pos=blade_x,
            material=weathered_oak,
            prefix=f"blade_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Box((3.40, 17.20, 17.20)),
        mass=4200.0,
        origin=Origin(xyz=(1.90, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 8.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.6),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(windshaft_rear_x, 0.0, shaft_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.6),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    tower_to_cap = object_model.get_articulation("tower_to_cap")
    cap_to_rotor = object_model.get_articulation("cap_to_rotor")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="cap_seated_on_curb_ring",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        min_overlap=3.40,
        name="cap_centered_over_tower",
    )
    ctx.expect_contact(
        rotor,
        cap,
        contact_tol=0.002,
        name="rotor_supported_by_bearing_blocks",
    )
    ctx.expect_origin_gap(
        cap,
        tower,
        axis="z",
        min_gap=8.10,
        max_gap=8.30,
        name="cap_positioned_at_tower_head_height",
    )

    with ctx.pose({tower_to_cap: math.pi / 3.0}):
        ctx.expect_gap(
            cap,
            tower,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            name="cap_stays_seated_when_yawed",
        )
        ctx.expect_contact(
            rotor,
            cap,
            contact_tol=0.002,
            name="rotor_bearings_hold_under_yaw",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="yaw_pose_no_overlaps")
        ctx.fail_if_isolated_parts(name="yaw_pose_no_floating")

    with ctx.pose({cap_to_rotor: math.pi / 4.0}):
        ctx.expect_contact(
            rotor,
            cap,
            contact_tol=0.002,
            name="rotor_bearings_hold_under_spin",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="spin_pose_no_overlaps")
        ctx.fail_if_isolated_parts(name="spin_pose_no_floating")

    with ctx.pose({tower_to_cap: -math.pi / 2.0, cap_to_rotor: math.pi / 4.0}):
        ctx.expect_contact(
            rotor,
            cap,
            contact_tol=0.002,
            name="combined_pose_rotor_supported",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlaps")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    rotor_aabb = ctx.part_world_aabb(rotor)
    tower_aabb = ctx.part_world_aabb(tower)
    cap_aabb = ctx.part_world_aabb(cap)
    if rotor_aabb is None or tower_aabb is None or cap_aabb is None:
        ctx.fail("world_aabb_available", "Expected world AABBs for tower, cap, and rotor.")
    else:
        rotor_span_y = rotor_aabb[1][1] - rotor_aabb[0][1]
        rotor_span_z = rotor_aabb[1][2] - rotor_aabb[0][2]
        total_height = max(tower_aabb[1][2], cap_aabb[1][2]) - min(
            tower_aabb[0][2],
            cap_aabb[0][2],
        )
        ctx.check(
            "rotor_span_reads_as_full_windmill_sails",
            rotor_span_y > 15.0 and rotor_span_z > 15.0,
            (
                f"Rotor span too small: y={rotor_span_y:.3f} m, "
                f"z={rotor_span_z:.3f} m"
            ),
        )
        ctx.check(
            "windmill_height_realistic",
            10.0 < total_height < 11.5,
            f"Total windmill height out of range: {total_height:.3f} m",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
