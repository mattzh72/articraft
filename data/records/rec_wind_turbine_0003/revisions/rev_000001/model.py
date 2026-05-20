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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    def _save_mesh(name: str, geometry: MeshGeometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
        merged = MeshGeometry()
        for geometry in geometries:
            merged.merge(geometry)
        return merged

    def _yz_section(
        x_pos: float,
        *,
        width: float,
        height: float,
        radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_val, z_val + z_center)
            for z_val, y_val in rounded_rect_profile(
                height,
                width,
                radius,
                corner_segments=8,
            )
        ]

    def _blade_loop(
        span_pos: float,
        *,
        chord: float,
        thickness: float,
        twist_deg: float,
        x_offset: float,
    ) -> list[tuple[float, float, float]]:
        base_points = [
            (0.50, 0.00),
            (0.20, 0.56),
            (-0.22, 0.64),
            (-0.46, 0.24),
            (-0.50, 0.00),
            (-0.42, -0.18),
            (-0.05, -0.30),
            (0.26, -0.16),
        ]
        twist = math.radians(twist_deg)
        cos_t = math.cos(twist)
        sin_t = math.sin(twist)
        loop: list[tuple[float, float, float]] = []
        for x_val, y_val in base_points:
            px = x_val * chord
            py = y_val * thickness
            rx = (cos_t * px) - (sin_t * py) + x_offset
            ry = (sin_t * px) + (cos_t * py)
            loop.append((rx, ry, span_pos))
        return loop

    def _ellipse_loop(
        x_pos: float,
        *,
        radius_y: float,
        radius_z: float,
        segments: int = 20,
    ) -> list[tuple[float, float, float]]:
        loop: list[tuple[float, float, float]] = []
        for index in range(segments):
            angle = (2.0 * math.pi * index) / segments
            loop.append((x_pos, radius_y * math.cos(angle), radius_z * math.sin(angle)))
        return loop

    def _build_tower_shell_mesh() -> MeshGeometry:
        outer_profile = [
            (2.45, 0.00),
            (2.40, 8.00),
            (2.18, 28.00),
            (1.93, 52.00),
            (1.64, 74.00),
            (1.48, 84.50),
            (1.40, 86.45),
        ]
        inner_profile = [
            (2.32, 0.00),
            (2.28, 8.00),
            (2.08, 28.00),
            (1.85, 52.00),
            (1.58, 74.00),
            (1.44, 84.50),
            (1.37, 86.45),
        ]
        return LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
        ).translate(0.0, 0.0, 1.2)

    def _build_nacelle_shell_mesh() -> MeshGeometry:
        sections = [
            _yz_section(-4.60, width=2.70, height=2.35, radius=0.58, z_center=2.15),
            _yz_section(-2.50, width=3.78, height=3.42, radius=0.76, z_center=2.34),
            _yz_section(0.00, width=4.70, height=4.45, radius=0.95, z_center=2.58),
            _yz_section(2.55, width=4.42, height=4.16, radius=0.90, z_center=2.62),
            _yz_section(4.35, width=2.96, height=3.02, radius=0.64, z_center=2.42),
        ]
        return repair_loft(section_loft(sections))

    def _build_hub_shell_mesh() -> MeshGeometry:
        sections = [
            _ellipse_loop(0.12, radius_y=1.24, radius_z=1.24),
            _ellipse_loop(0.42, radius_y=1.38, radius_z=1.38),
            _ellipse_loop(0.92, radius_y=1.56, radius_z=1.54),
            _ellipse_loop(1.38, radius_y=1.62, radius_z=1.60),
            _ellipse_loop(1.84, radius_y=1.28, radius_z=1.26),
            _ellipse_loop(2.26, radius_y=0.88, radius_z=0.86),
            _ellipse_loop(2.62, radius_y=0.42, radius_z=0.42),
            _ellipse_loop(2.82, radius_y=0.16, radius_z=0.16),
            _ellipse_loop(2.94, radius_y=0.06, radius_z=0.06),
        ]
        return repair_loft(section_loft(sections))

    def _build_blade_shell_mesh() -> MeshGeometry:
        sections = [
            _blade_loop(1.50, chord=3.80, thickness=0.84, twist_deg=14.0, x_offset=0.22),
            _blade_loop(6.00, chord=4.15, thickness=0.78, twist_deg=10.5, x_offset=0.14),
            _blade_loop(18.00, chord=3.55, thickness=0.58, twist_deg=6.5, x_offset=-0.12),
            _blade_loop(32.00, chord=2.55, thickness=0.34, twist_deg=3.5, x_offset=-0.62),
            _blade_loop(46.00, chord=1.42, thickness=0.18, twist_deg=1.2, x_offset=-1.02),
            _blade_loop(60.50, chord=0.52, thickness=0.06, twist_deg=0.0, x_offset=-1.34),
            _blade_loop(61.80, chord=0.10, thickness=0.012, twist_deg=0.0, x_offset=-1.42),
        ]
        return repair_loft(section_loft(sections))

    model = ArticulatedObject(name="premium_wind_turbine", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.72, 0.72, 0.70, 1.0))
    tower_matte = model.material("tower_matte", rgba=(0.84, 0.86, 0.88, 1.0))
    nacelle_satin = model.material("nacelle_satin", rgba=(0.88, 0.90, 0.92, 1.0))
    blade_composite = model.material("blade_composite", rgba=(0.93, 0.94, 0.95, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    bearing_dark = model.material("bearing_dark", rgba=(0.29, 0.31, 0.34, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.53, 0.56, 0.60, 1.0))

    tower_shell_mesh = _save_mesh("tower_shell.obj", _build_tower_shell_mesh())
    nacelle_shell_mesh = _save_mesh("nacelle_shell.obj", _build_nacelle_shell_mesh())
    hub_shell_mesh = _save_mesh("hub_shell.obj", _build_hub_shell_mesh())
    blade_shell_mesh = _save_mesh("blade_shell.obj", _build_blade_shell_mesh())

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=7.5, length=1.2),
        origin=Origin(xyz=(0.0, 0.0, 0.6)),
        material=concrete,
        name="foundation_plinth",
    )
    tower.visual(
        tower_shell_mesh,
        material=tower_matte,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.62, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 1.425)),
        material=seam_gray,
        name="tower_base_flange",
    )
    tower.visual(
        Cylinder(radius=1.70, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 87.45)),
        material=seam_gray,
        name="tower_head_flange",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.6, length=87.0),
        mass=220000.0,
        origin=Origin(xyz=(0.0, 0.0, 43.5)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=1.62, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=bearing_dark,
        name="yaw_bearing_shell",
    )
    nacelle.visual(
        Box((9.0, 4.2, 0.55)),
        origin=Origin(xyz=(0.1, 0.0, 0.825)),
        material=bearing_dark,
        name="bedplate",
    )
    nacelle.visual(
        Box((2.3, 2.8, 0.18)),
        origin=Origin(xyz=(-3.35, 0.0, 1.16)),
        material=seam_gray,
        name="rear_service_deck",
    )
    nacelle.visual(
        nacelle_shell_mesh,
        material=nacelle_satin,
        name="nacelle_shell",
    )
    nacelle.visual(
        Box((2.8, 2.5, 2.6)),
        origin=Origin(xyz=(2.95, 0.0, 2.30)),
        material=bearing_dark,
        name="front_bearing_block",
    )
    nacelle.visual(
        Cylinder(radius=0.98, length=1.40),
        origin=Origin(xyz=(4.25, 0.0, 2.8), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_dark,
        name="main_bearing_collar",
    )
    nacelle.visual(
        Cylinder(radius=0.58, length=1.70),
        origin=Origin(xyz=(5.75, 0.0, 2.8), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="main_shaft_spindle",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((10.6, 4.8, 4.8)),
        mass=240000.0,
        origin=Origin(xyz=(-0.2, 0.0, 2.4)),
    )

    rotor_hub = model.part("rotor_hub")
    rotor_hub.visual(
        Cylinder(radius=1.28, length=0.18),
        origin=Origin(xyz=(0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_dark,
        name="rear_hub_flange",
    )
    rotor_hub.visual(
        Cylinder(radius=1.16, length=0.42),
        origin=Origin(xyz=(0.30, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_dark,
        name="hub_core",
    )
    rotor_hub.visual(
        hub_shell_mesh,
        material=nacelle_satin,
        name="hub_shell",
    )
    rotor_hub.visual(
        Cylinder(radius=0.60, length=0.12),
        origin=Origin(xyz=(2.82, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seam_gray,
        name="spinner_seam_ring",
    )
    blade_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    blade_specs: list[tuple[str, float, tuple[float, float, float]]] = []
    for blade_index, angle in enumerate(blade_angles):
        radial_vec = (0.0, -math.sin(angle), math.cos(angle))
        blade_specs.append((f"blade_{blade_index}", angle, radial_vec))
        rotor_hub.visual(
            Cylinder(radius=0.52, length=1.24),
            origin=Origin(
                xyz=(1.08, 0.80 * radial_vec[1], 0.80 * radial_vec[2]),
                rpy=(angle, 0.0, 0.0),
            ),
            material=bearing_dark,
            name=f"pitch_connector_{blade_index}",
        )
        rotor_hub.visual(
            Cylinder(radius=0.82, length=0.72),
            origin=Origin(
                xyz=(1.18, 1.60 * radial_vec[1], 1.60 * radial_vec[2]),
                rpy=(angle, 0.0, 0.0),
            ),
            material=bearing_dark,
            name=f"pitch_bearing_{blade_index}",
        )
        rotor_hub.visual(
            Cylinder(radius=0.70, length=0.12),
            origin=Origin(
                xyz=(1.18, 1.90 * radial_vec[1], 1.90 * radial_vec[2]),
                rpy=(angle, 0.0, 0.0),
            ),
            material=seam_gray,
            name=f"pitch_face_{blade_index}",
        )
    rotor_hub.inertial = Inertial.from_geometry(
        Box((4.0, 5.0, 5.0)),
        mass=56000.0,
        origin=Origin(xyz=(1.8, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 87.65)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800000.0, velocity=0.05),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor_hub,
        origin=Origin(xyz=(6.6, 0.0, 2.8)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600000.0, velocity=0.25),
    )

    for blade_name, angle, radial_vec in blade_specs:
        blade = model.part(blade_name)
        blade.visual(
            Cylinder(radius=0.34, length=0.08),
            origin=Origin(
                xyz=(0.0, 0.0, 0.14),
            ),
            material=bearing_dark,
            name="pitch_cap",
        )
        blade.visual(
            Cylinder(radius=0.64, length=0.10),
            origin=Origin(
                xyz=(0.0, 0.0, 0.05),
            ),
            material=seam_gray,
            name="root_flange",
        )
        blade.visual(
            Cylinder(radius=0.56, length=2.00),
            origin=Origin(
                xyz=(0.0, 0.0, 1.18),
            ),
            material=blade_composite,
            name="root_sleeve",
        )
        blade.visual(
            blade_shell_mesh,
            material=blade_composite,
            name="blade_shell",
        )
        blade.inertial = Inertial.from_geometry(
            Box((4.4, 1.4, 61.8)),
            mass=18500.0,
            origin=Origin(xyz=(-0.7, 0.0, 30.9)),
        )
        model.articulation(
            f"rotor_to_{blade_name}_pitch",
            ArticulationType.REVOLUTE,
            parent=rotor_hub,
            child=blade,
            origin=Origin(
                xyz=(
                    1.18,
                    1.96 * radial_vec[1],
                    1.96 * radial_vec[2],
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=180000.0,
                velocity=0.18,
                lower=-0.25,
                upper=1.45,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor_hub = object_model.get_part("rotor_hub")
    blade_0 = object_model.get_part("blade_0")
    blade_1 = object_model.get_part("blade_1")
    blade_2 = object_model.get_part("blade_2")

    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    rotor_spin = object_model.get_articulation("nacelle_to_rotor_spin")
    blade_0_pitch = object_model.get_articulation("rotor_to_blade_0_pitch")
    blade_1_pitch = object_model.get_articulation("rotor_to_blade_1_pitch")
    blade_2_pitch = object_model.get_articulation("rotor_to_blade_2_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=8)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        nacelle,
        rotor_hub,
        elem_a="main_shaft_spindle",
        elem_b="rear_hub_flange",
        reason="The rotor hub mounts over a hollow main-shaft interface; solid proxy cylinders stand in for a spindle inside a bored hub collar.",
    )
    for blade_index, blade in enumerate((blade_0, blade_1, blade_2)):
        ctx.allow_overlap(
            blade,
            rotor_hub,
            elem_a="root_flange",
            elem_b=f"pitch_bearing_{blade_index}",
            reason="Each blade root sits inside a hollow pitch-bearing collar; the solid collar proxy intentionally occupies the receiver envelope.",
        )
        ctx.allow_overlap(
            blade,
            rotor_hub,
            elem_a="root_flange",
            elem_b=f"pitch_face_{blade_index}",
            reason="The pitch-face disc represents a clamped flange interface with a modeled solid face instead of a bored annulus.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=12,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(
        nacelle,
        tower,
        elem_a="yaw_bearing_shell",
        elem_b="tower_head_flange",
        contact_tol=1e-4,
        name="yaw_bearing_supported",
    )
    ctx.expect_overlap(
        nacelle,
        tower,
        axes="xy",
        elem_a="yaw_bearing_shell",
        elem_b="tower_head_flange",
        min_overlap=3.0,
        name="yaw_bearing_aligned",
    )
    ctx.expect_contact(
        rotor_hub,
        nacelle,
        elem_a="rear_hub_flange",
        elem_b="main_shaft_spindle",
        contact_tol=1e-4,
        name="hub_supported_on_main_shaft",
    )
    ctx.expect_overlap(
        rotor_hub,
        nacelle,
        axes="yz",
        elem_a="rear_hub_flange",
        elem_b="main_shaft_spindle",
        min_overlap=1.0,
        name="hub_spindle_overlap_in_cross_section",
    )
    for blade_index, blade in enumerate((blade_0, blade_1, blade_2)):
        ctx.expect_contact(
            blade,
            rotor_hub,
            elem_a="root_flange",
            elem_b=f"pitch_face_{blade_index}",
            contact_tol=1e-4,
            name=f"blade_{blade_index}_pitch_interface_supported",
        )

    ctx.expect_origin_gap(
        nacelle,
        tower,
        axis="z",
        min_gap=87.55,
        max_gap=87.75,
        name="hub_height_on_tower",
    )
    ctx.expect_origin_gap(
        rotor_hub,
        nacelle,
        axis="x",
        min_gap=6.5,
        max_gap=6.7,
        name="rotor_overhang_realistic",
    )
    ctx.expect_origin_gap(
        rotor_hub,
        nacelle,
        axis="z",
        min_gap=2.75,
        max_gap=2.85,
        name="shaft_centerline_above_bedplate",
    )

    ctx.check(
        "yaw_axis_vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        f"Unexpected yaw axis: {yaw.axis}",
    )
    ctx.check(
        "rotor_axis_horizontal",
        tuple(rotor_spin.axis) == (1.0, 0.0, 0.0),
        f"Unexpected rotor spin axis: {rotor_spin.axis}",
    )

    tower_aabb = ctx.part_world_aabb(tower)
    nacelle_aabb = ctx.part_world_aabb(nacelle)
    blade_aabb = ctx.part_world_aabb(blade_0)
    if tower_aabb is None:
        ctx.fail("tower_aabb_available", "Tower AABB was not available.")
    else:
        tower_height = tower_aabb[1][2] - tower_aabb[0][2]
        ctx.check(
            "tower_height_realistic",
            tower_height > 86.0,
            f"Tower height too small: {tower_height:.3f} m",
        )
    if nacelle_aabb is None:
        ctx.fail("nacelle_aabb_available", "Nacelle AABB was not available.")
    else:
        nacelle_length = nacelle_aabb[1][0] - nacelle_aabb[0][0]
        nacelle_width = nacelle_aabb[1][1] - nacelle_aabb[0][1]
        ctx.check(
            "nacelle_proportions_realistic",
            9.0 < nacelle_length <= 11.7 and 3.5 < nacelle_width < 5.2,
            f"Unexpected nacelle extents: length={nacelle_length:.3f}, width={nacelle_width:.3f}",
        )
    if blade_aabb is None:
        ctx.fail("blade_aabb_available", "Blade AABB was not available.")
    else:
        blade_span = blade_aabb[1][2] - blade_aabb[0][2]
        ctx.check(
            "blade_span_realistic",
            blade_span > 60.0,
            f"Blade span too short: {blade_span:.3f} m",
        )

    blade_0_rest_position = ctx.part_world_position(blade_0)
    with ctx.pose({rotor_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_spin_clearance")
        blade_0_quarter_turn = ctx.part_world_position(blade_0)
        if blade_0_rest_position is None or blade_0_quarter_turn is None:
            ctx.fail("blade_0_spin_motion_observable", "Blade world positions were unavailable.")
        else:
            ctx.check(
                "blade_0_rotor_spin_moves_root_around_hub",
                abs(blade_0_quarter_turn[1]) > 1.5 and blade_0_quarter_turn[2] < blade_0_rest_position[2] - 1.0,
                f"Unexpected blade root motion: rest={blade_0_rest_position}, turned={blade_0_quarter_turn}",
            )
        ctx.expect_contact(
            rotor_hub,
            nacelle,
            elem_a="rear_hub_flange",
            elem_b="main_shaft_spindle",
            contact_tol=1e-4,
            name="rotor_spin_keeps_hub_supported",
        )

    with ctx.pose({yaw: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="ninety_degree_yaw_clearance")
        ctx.fail_if_isolated_parts(name="ninety_degree_yaw_support")
        ctx.expect_origin_gap(
            rotor_hub,
            tower,
            axis="y",
            min_gap=6.5,
            max_gap=6.7,
            name="yaw_rotation_swings_rotor_sideways",
        )

    blade_0_rest_aabb = ctx.part_world_aabb(blade_0)
    with ctx.pose({blade_0_pitch: 1.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="blade_0_pitch_upper_clearance")
        ctx.expect_contact(
            blade_0,
            rotor_hub,
            elem_a="root_flange",
            elem_b="pitch_face_0",
            contact_tol=1e-4,
            name="blade_0_pitch_stays_supported",
        )
        blade_0_pitched_aabb = ctx.part_world_aabb(blade_0)
        if blade_0_rest_aabb is None or blade_0_pitched_aabb is None:
            ctx.fail("blade_0_pitch_changes_planform", "Blade AABB was unavailable.")
        else:
            rest_width_y = blade_0_rest_aabb[1][1] - blade_0_rest_aabb[0][1]
            pitched_width_y = blade_0_pitched_aabb[1][1] - blade_0_pitched_aabb[0][1]
            ctx.check(
                "blade_0_pitch_changes_planform",
                pitched_width_y > rest_width_y + 1.0,
                f"Pitch change too small: rest_y={rest_width_y:.3f}, pitched_y={pitched_width_y:.3f}",
            )

    pose_map = {
        yaw: 0.85,
        rotor_spin: 1.35,
        blade_0_pitch: 0.95,
        blade_1_pitch: 0.40,
        blade_2_pitch: 1.15,
    }
    with ctx.pose(pose_map):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_clearance")
        ctx.fail_if_isolated_parts(name="combined_pose_support")
        for blade_index, blade in enumerate((blade_0, blade_1, blade_2)):
            ctx.expect_contact(
                blade,
                rotor_hub,
                elem_a="root_flange",
                elem_b=f"pitch_face_{blade_index}",
                contact_tol=1e-4,
                name=f"combined_pose_blade_{blade_index}_support",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
