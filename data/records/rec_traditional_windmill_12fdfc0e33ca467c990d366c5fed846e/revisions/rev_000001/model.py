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
    rounded_rect_profile,
    section_loft,
)


def _rotate_point(
    point: tuple[float, float, float], rpy: tuple[float, float, float]
) -> tuple[float, float, float]:
    x, y, z = point
    roll, pitch, yaw = rpy

    cr = math.cos(roll)
    sr = math.sin(roll)
    y, z = (y * cr - z * sr, y * sr + z * cr)

    cp = math.cos(pitch)
    sp = math.sin(pitch)
    x, z = (x * cp + z * sp, -x * sp + z * cp)

    cy = math.cos(yaw)
    sy = math.sin(yaw)
    x, y = (x * cy - y * sy, x * sy + y * cy)

    return (x, y, z)


def _octagon_loop(
    radius_x: float, radius_y: float, z: float
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(8):
        angle = (2.0 * math.pi * index / 8.0) + (math.pi / 8.0)
        points.append((radius_x * math.cos(angle), radius_y * math.sin(angle), z))
    return points


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    corner_radius: float,
    base_z: float,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        height,
        corner_radius,
        corner_segments=corner_segments,
    )
    z_offset = base_z + (height * 0.5)
    return [(x, y, z_offset + z) for y, z in profile]


def _place_visual(
    part,
    geometry,
    *,
    anchor_xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
    local_xyz: tuple[float, float, float],
    material,
    name: str | None = None,
) -> None:
    dx, dy, dz = _rotate_point(local_xyz, rpy)
    part.visual(
        geometry,
        origin=Origin(
            xyz=(anchor_xyz[0] + dx, anchor_xyz[1] + dy, anchor_xyz[2] + dz),
            rpy=rpy,
        ),
        material=material,
        name=name,
    )


def _add_lattice_blade(
    part,
    *,
    prefix: str,
    anchor_xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
    frame_material,
    slat_material,
) -> None:
    _place_visual(
        part,
        Box((0.016, 0.018, 0.024)),
        anchor_xyz=anchor_xyz,
        rpy=rpy,
        local_xyz=(0.0, 0.009, 0.0),
        material=frame_material,
        name=f"{prefix}_root",
    )
    _place_visual(
        part,
        Box((0.008, 0.078, 0.006)),
        anchor_xyz=anchor_xyz,
        rpy=rpy,
        local_xyz=(0.0, 0.041, 0.022),
        material=frame_material,
    )
    _place_visual(
        part,
        Box((0.008, 0.078, 0.006)),
        anchor_xyz=anchor_xyz,
        rpy=rpy,
        local_xyz=(0.0, 0.041, -0.022),
        material=frame_material,
    )
    _place_visual(
        part,
        Box((0.008, 0.008, 0.050)),
        anchor_xyz=anchor_xyz,
        rpy=rpy,
        local_xyz=(0.0, 0.074, 0.0),
        material=frame_material,
        name=f"{prefix}_tip",
    )
    for slat_index, slat_y in enumerate((0.018, 0.034, 0.050, 0.066)):
        _place_visual(
            part,
            Box((0.004, 0.006, 0.044)),
            anchor_xyz=anchor_xyz,
            rpy=rpy,
            local_xyz=(0.0, slat_y, 0.0),
            material=slat_material,
            name=f"{prefix}_slat_{slat_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_traditional_windmill")

    stone = model.material("stone", rgba=(0.72, 0.71, 0.68, 1.0))
    timber_dark = model.material("timber_dark", rgba=(0.30, 0.22, 0.16, 1.0))
    cap_shingle = model.material("cap_shingle", rgba=(0.41, 0.22, 0.18, 1.0))
    iron = model.material("iron", rgba=(0.22, 0.24, 0.27, 1.0))
    blade_wood = model.material("blade_wood", rgba=(0.76, 0.72, 0.62, 1.0))
    weathered_trim = model.material("weathered_trim", rgba=(0.57, 0.49, 0.38, 1.0))

    tower = model.part("tower")
    tower_shell = mesh_from_geometry(
        section_loft(
            [
                _octagon_loop(0.067, 0.067, 0.025),
                _octagon_loop(0.056, 0.056, 0.185),
                _octagon_loop(0.046, 0.046, 0.335),
            ]
        ),
        "windmill_tower_shell",
    )
    tower.visual(
        Box((0.180, 0.180, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=stone,
        name="base_plinth",
    )
    tower.visual(tower_shell, material=stone, name="tower_shell")
    tower.visual(
        Cylinder(radius=0.071, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=weathered_trim,
        name="crown_seat",
    )
    tower.visual(
        Box((0.034, 0.010, 0.064)),
        origin=Origin(xyz=(0.060, 0.0, 0.052)),
        material=timber_dark,
        name="front_door",
    )
    tower.visual(
        Box((0.028, 0.010, 0.024)),
        origin=Origin(xyz=(0.054, 0.0, 0.148)),
        material=timber_dark,
        name="front_window",
    )
    tower.visual(
        Box((0.112, 0.010, 0.008)),
        origin=Origin(xyz=(0.042, 0.0, 0.210)),
        material=weathered_trim,
        name="front_gallery_lip",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.180, 0.180, 0.360)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    cap = model.part("cap")
    cap_shell = mesh_from_geometry(
        section_loft(
            [
                _yz_section(
                    -0.070,
                    width=0.100,
                    height=0.050,
                    corner_radius=0.016,
                    base_z=0.010,
                ),
                _yz_section(
                    -0.025,
                    width=0.132,
                    height=0.074,
                    corner_radius=0.020,
                    base_z=0.008,
                ),
                _yz_section(
                    0.015,
                    width=0.122,
                    height=0.070,
                    corner_radius=0.018,
                    base_z=0.010,
                ),
                _yz_section(
                    0.050,
                    width=0.090,
                    height=0.050,
                    corner_radius=0.014,
                    base_z=0.018,
                ),
            ]
        ),
        "windmill_cap_shell",
    )
    cap.visual(
        Cylinder(radius=0.076, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=weathered_trim,
        name="turntable_plate",
    )
    cap.visual(cap_shell, material=cap_shingle, name="cap_shell")
    cap.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.060, 0.030, 0.040)),
        material=timber_dark,
        name="left_strut",
    )
    cap.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.060, -0.030, 0.040)),
        material=timber_dark,
        name="right_strut",
    )
    cap.visual(
        Box((0.028, 0.018, 0.060)),
        origin=Origin(xyz=(0.089, 0.027, 0.050)),
        material=timber_dark,
        name="left_cheek",
    )
    cap.visual(
        Box((0.028, 0.018, 0.060)),
        origin=Origin(xyz=(0.089, -0.027, 0.050)),
        material=timber_dark,
        name="right_cheek",
    )
    cap.visual(
        Box((0.014, 0.008, 0.030)),
        origin=Origin(xyz=(0.095, 0.016, 0.055)),
        material=iron,
        name="left_bearing_seat",
    )
    cap.visual(
        Box((0.014, 0.008, 0.030)),
        origin=Origin(xyz=(0.095, -0.016, 0.055)),
        material=iron,
        name="right_bearing_seat",
    )
    cap.visual(
        Box((0.018, 0.078, 0.012)),
        origin=Origin(xyz=(0.092, 0.0, 0.081)),
        material=timber_dark,
        name="top_bridge",
    )
    cap.visual(
        Box((0.018, 0.070, 0.010)),
        origin=Origin(xyz=(0.092, 0.0, 0.030)),
        material=timber_dark,
        name="bottom_bridge",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.190, 0.150, 0.100)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="main_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="bearing_collar",
    )
    rotor.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(0.042, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_trim,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_nose",
    )
    rotor.visual(
        Box((0.016, 0.220, 0.020)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=weathered_trim,
        name="stock_horizontal",
    )
    rotor.visual(
        Box((0.016, 0.020, 0.220)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=weathered_trim,
        name="stock_vertical",
    )

    _add_lattice_blade(
        rotor,
        prefix="blade_right",
        anchor_xyz=(0.040, 0.094, 0.0),
        rpy=(0.0, 0.0, 0.0),
        frame_material=blade_wood,
        slat_material=weathered_trim,
    )
    _add_lattice_blade(
        rotor,
        prefix="blade_left",
        anchor_xyz=(0.040, -0.094, 0.0),
        rpy=(0.0, 0.0, math.pi),
        frame_material=blade_wood,
        slat_material=weathered_trim,
    )
    _add_lattice_blade(
        rotor,
        prefix="blade_top",
        anchor_xyz=(0.040, 0.0, 0.094),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        frame_material=blade_wood,
        slat_material=weathered_trim,
    )
    _add_lattice_blade(
        rotor,
        prefix="blade_bottom",
        anchor_xyz=(0.040, 0.0, -0.094),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        frame_material=blade_wood,
        slat_material=weathered_trim,
    )
    rotor.inertial = Inertial.from_geometry(
        Box((0.170, 0.350, 0.350)),
        mass=0.38,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.4),
    )
    model.articulation(
        "cap_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.095, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    cap_yaw = object_model.get_articulation("tower_to_cap_yaw")
    rotor_spin = object_model.get_articulation("cap_to_rotor_spin")

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

    tower_visuals = {visual.name for visual in tower.visuals if visual.name is not None}
    cap_visuals = {visual.name for visual in cap.visuals if visual.name is not None}
    rotor_visuals = {visual.name for visual in rotor.visuals if visual.name is not None}

    ctx.check(
        "tower_visuals_present",
        {"tower_shell", "crown_seat", "front_door"}.issubset(tower_visuals),
        f"tower visuals present: {sorted(tower_visuals)}",
    )
    ctx.check(
        "cap_support_visuals_present",
        {
            "turntable_plate",
            "left_cheek",
            "right_cheek",
            "left_bearing_seat",
            "right_bearing_seat",
            "top_bridge",
            "bottom_bridge",
        }.issubset(
            cap_visuals
        ),
        f"cap visuals present: {sorted(cap_visuals)}",
    )
    ctx.check(
        "rotor_blade_visuals_present",
        {"main_shaft", "bearing_collar", "hub_drum", "blade_top_tip", "blade_bottom_tip", "blade_left_tip", "blade_right_tip"}.issubset(
            rotor_visuals
        ),
        f"rotor visuals present: {sorted(rotor_visuals)}",
    )
    ctx.check(
        "cap_yaw_axis_vertical",
        tuple(cap_yaw.axis) == (0.0, 0.0, 1.0),
        f"expected vertical yaw axis, got {cap_yaw.axis}",
    )
    ctx.check(
        "rotor_spin_axis_horizontal",
        tuple(rotor_spin.axis) == (1.0, 0.0, 0.0),
        f"expected horizontal shaft axis, got {rotor_spin.axis}",
    )

    with ctx.pose({cap_yaw: 0.0, rotor_spin: 0.0}):
        ctx.expect_gap(
            cap,
            tower,
            axis="z",
            positive_elem="turntable_plate",
            negative_elem="crown_seat",
            max_gap=0.0005,
            max_penetration=1e-5,
            name="cap_turntable_seated_on_crown",
        )
        ctx.expect_gap(
            rotor,
            tower,
            axis="x",
            positive_elem="bearing_collar",
            negative_elem="tower_shell",
            min_gap=0.015,
            name="front_shaft_stage_clears_tower",
        )
        ctx.expect_gap(
            cap,
            rotor,
            axis="z",
            positive_elem="top_bridge",
            negative_elem="bearing_collar",
            min_gap=0.006,
            name="shaft_clears_top_bridge",
        )
        ctx.expect_gap(
            rotor,
            cap,
            axis="z",
            positive_elem="bearing_collar",
            negative_elem="bottom_bridge",
            min_gap=0.006,
            name="shaft_clears_bottom_bridge",
        )
        ctx.expect_contact(
            rotor,
            cap,
            elem_a="bearing_collar",
            elem_b="left_bearing_seat",
            name="collar_supported_by_left_bearing_seat",
        )
        ctx.expect_contact(
            rotor,
            cap,
            elem_a="bearing_collar",
            elem_b="right_bearing_seat",
            name="collar_supported_by_right_bearing_seat",
        )
        ctx.expect_gap(
            cap,
            rotor,
            axis="y",
            positive_elem="left_cheek",
            negative_elem="bearing_collar",
            min_gap=0.006,
            name="shaft_clears_left_cheek",
        )
        ctx.expect_gap(
            rotor,
            cap,
            axis="y",
            positive_elem="bearing_collar",
            negative_elem="right_cheek",
            min_gap=0.006,
            name="shaft_clears_right_cheek",
        )

    with ctx.pose({cap_yaw: math.pi / 2.0, rotor_spin: 0.0}):
        ctx.expect_origin_distance(
            cap,
            tower,
            axes="xy",
            max_dist=0.001,
            name="cap_rotates_about_centered_bearing",
        )
        ctx.expect_gap(
            rotor,
            tower,
            axis="y",
            positive_elem="bearing_collar",
            negative_elem="tower_shell",
            min_gap=0.015,
            name="side_pose_shaft_stage_clears_tower",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
