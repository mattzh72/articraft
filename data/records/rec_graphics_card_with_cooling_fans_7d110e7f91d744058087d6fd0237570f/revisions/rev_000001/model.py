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
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    def save_mesh(name: str, geometry: MeshGeometry):
        return mesh_from_geometry(geometry, name)

    def circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
        return superellipse_profile(radius * 2.0, radius * 2.0, exponent=2.0, segments=segments)

    def shifted_profile(
        profile: list[tuple[float, float]],
        dx: float,
        dy: float,
    ) -> list[tuple[float, float]]:
        return [(x + dx, y + dy) for x, y in profile]

    def annulus_geometry(
        *,
        outer_radius: float,
        inner_radius: float,
        thickness: float,
        segments: int = 40,
    ) -> MeshGeometry:
        return ExtrudeWithHolesGeometry(
            circle_profile(outer_radius, segments=segments),
            [circle_profile(inner_radius, segments=segments)],
            height=thickness,
            center=True,
        )

    def fan_rotor_geometry(
        *,
        outer_radius: float,
        hub_radius: float,
        ring_width: float,
        blade_count: int,
        blade_thickness: float,
        sweep_sign: float = 1.0,
    ) -> MeshGeometry:
        rotor = MeshGeometry()
        rotor.merge(
            annulus_geometry(
                outer_radius=outer_radius,
                inner_radius=outer_radius - ring_width,
                thickness=blade_thickness + 0.0004,
                segments=56,
            )
        )
        rotor.merge(CylinderGeometry(radius=hub_radius, height=0.006, radial_segments=40))
        rotor.merge(
            CylinderGeometry(radius=hub_radius * 0.62, height=0.007, radial_segments=32).translate(
                0.0, 0.0, 0.0005
            )
        )

        root_radius = hub_radius * 1.03
        tip_radius = outer_radius - ring_width * 1.15
        span = tip_radius - root_radius
        root_width = outer_radius * 0.22
        tip_width = outer_radius * 0.12
        sweep = outer_radius * 0.12 * sweep_sign
        blade_profile = [
            (root_radius, -root_width * 0.42),
            (root_radius + span * 0.30, -root_width * 0.54),
            (tip_radius - outer_radius * 0.05, -tip_width * 0.38),
            (tip_radius, 0.0),
            (tip_radius - outer_radius * 0.02, tip_width * 0.34),
            (root_radius + span * 0.18 + sweep, root_width * 0.44),
            (root_radius + outer_radius * 0.02, root_width * 0.10),
        ]
        blade_base = ExtrudeGeometry(blade_profile, blade_thickness, center=True)

        for blade_index in range(blade_count):
            blade_angle = (math.tau * blade_index) / blade_count
            rotor.merge(blade_base.copy().rotate_z(blade_angle))

        return rotor

    def shroud_shell_geometry() -> MeshGeometry:
        outer_profile = sample_catmull_rom_spline_2d(
            [
                (-0.148, -0.050),
                (-0.149, 0.048),
                (-0.129, 0.057),
                (-0.062, 0.060),
                (0.020, 0.058),
                (0.099, 0.060),
                (0.142, 0.055),
                (0.157, 0.035),
                (0.159, -0.008),
                (0.152, -0.040),
                (0.132, -0.054),
                (0.071, -0.058),
                (-0.008, -0.056),
                (-0.086, -0.053),
            ],
            samples_per_segment=6,
            closed=True,
        )
        large_hole = shifted_profile(circle_profile(0.050, segments=52), 0.018, -0.002)
        small_hole = shifted_profile(circle_profile(0.039, segments=44), 0.116, 0.006)
        return ExtrudeWithHolesGeometry(
            outer_profile,
            [large_hole, small_hole],
            height=0.010,
            center=True,
        )

    model = ArticulatedObject(name="dual_slot_graphics_card")

    shroud_black = model.material("shroud_black", rgba=(0.14, 0.15, 0.16, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    pcb_black = model.material("pcb_black", rgba=(0.07, 0.09, 0.08, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.68, 0.70, 0.72, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.10, 0.11, 0.12, 1.0))
    heatsink_metal = model.material("heatsink_metal", rgba=(0.55, 0.58, 0.61, 1.0))
    gold = model.material("gold", rgba=(0.79, 0.64, 0.20, 1.0))

    body = model.part("card_body")
    body.visual(
        Box((0.284, 0.110, 0.0015)),
        origin=Origin(xyz=(0.004, 0.0, -0.00075)),
        material=satin_graphite,
        name="backplate",
    )
    body.visual(
        Box((0.280, 0.110, 0.002)),
        origin=Origin(xyz=(0.004, 0.0, 0.001)),
        material=pcb_black,
        name="pcb",
    )
    body.visual(
        Box((0.270, 0.106, 0.022)),
        origin=Origin(xyz=(0.015, 0.0, 0.013)),
        material=heatsink_metal,
        name="heatsink_core",
    )
    body.visual(
        Box((0.086, 0.010, 0.0015)),
        origin=Origin(xyz=(-0.078, -0.050, -0.00075)),
        material=gold,
        name="pcie_edge_connector",
    )
    body.visual(
        save_mesh("gpu_shroud_shell", shroud_shell_geometry()),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=shroud_black,
        name="shroud_shell",
    )
    body.visual(
        save_mesh(
            "gpu_large_fan_lip",
            annulus_geometry(
                outer_radius=0.053,
                inner_radius=0.0465,
                thickness=0.003,
                segments=56,
            ),
        ),
        origin=Origin(xyz=(0.018, -0.002, 0.0325)),
        material=satin_graphite,
        name="large_fan_lip",
    )
    body.visual(
        save_mesh(
            "gpu_small_fan_lip",
            annulus_geometry(
                outer_radius=0.042,
                inner_radius=0.0355,
                thickness=0.003,
                segments=48,
            ),
        ),
        origin=Origin(xyz=(0.116, 0.006, 0.0325)),
        material=satin_graphite,
        name="small_fan_lip",
    )
    body.visual(
        Cylinder(radius=0.0145, length=0.020),
        origin=Origin(xyz=(0.018, -0.002, 0.014)),
        material=heatsink_metal,
        name="large_motor_mount",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.0025),
        origin=Origin(xyz=(0.018, -0.002, 0.02525)),
        material=satin_graphite,
        name="large_spindle_cap",
    )
    body.visual(
        Cylinder(radius=0.0115, length=0.020),
        origin=Origin(xyz=(0.116, 0.006, 0.014)),
        material=heatsink_metal,
        name="small_motor_mount",
    )
    body.visual(
        Cylinder(radius=0.0080, length=0.0025),
        origin=Origin(xyz=(0.116, 0.006, 0.02525)),
        material=satin_graphite,
        name="small_spindle_cap",
    )
    body.visual(
        Box((0.016, 0.118, 0.038)),
        origin=Origin(xyz=(-0.144, 0.0, 0.019)),
        material=bracket_metal,
        name="io_bracket",
    )
    body.visual(
        Box((0.005, 0.040, 0.020)),
        origin=Origin(xyz=(-0.147, -0.012, 0.019)),
        material=pcb_black,
        name="bracket_vents",
    )
    body.visual(
        Box((0.006, 0.016, 0.012)),
        origin=Origin(xyz=(-0.147, 0.021, 0.025)),
        material=pcb_black,
        name="displayport_upper",
    )
    body.visual(
        Box((0.006, 0.016, 0.012)),
        origin=Origin(xyz=(-0.147, 0.021, 0.009)),
        material=pcb_black,
        name="displayport_lower",
    )
    body.visual(
        Box((0.034, 0.010, 0.012)),
        origin=Origin(xyz=(0.106, 0.059, 0.028)),
        material=pcb_black,
        name="power_socket",
    )
    body.visual(
        Box((0.008, 0.090, 0.034)),
        origin=Origin(xyz=(0.154, 0.0, 0.017)),
        material=satin_graphite,
        name="end_cap",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.314, 0.118, 0.040)),
        mass=1.45,
        origin=Origin(xyz=(0.002, 0.0, 0.020)),
    )

    large_fan = model.part("large_fan_rotor")
    large_fan.visual(
        save_mesh(
            "gpu_large_fan_rotor",
            fan_rotor_geometry(
                outer_radius=0.044,
                hub_radius=0.017,
                ring_width=0.005,
                blade_count=9,
                blade_thickness=0.0016,
                sweep_sign=1.0,
            ),
        ),
        material=rotor_black,
        name="large_rotor",
    )
    large_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.044, length=0.006),
        mass=0.045,
    )

    small_fan = model.part("small_fan_rotor")
    small_fan.visual(
        save_mesh(
            "gpu_small_fan_rotor",
            fan_rotor_geometry(
                outer_radius=0.033,
                hub_radius=0.013,
                ring_width=0.0045,
                blade_count=7,
                blade_thickness=0.0014,
                sweep_sign=-1.0,
            ),
        ),
        material=rotor_black,
        name="small_rotor",
    )
    small_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.006),
        mass=0.028,
    )

    model.articulation(
        "large_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=large_fan,
        origin=Origin(xyz=(0.018, -0.002, 0.0295)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=40.0),
    )
    model.articulation(
        "small_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=small_fan,
        origin=Origin(xyz=(0.116, 0.006, 0.0295)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.9, velocity=48.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("card_body")
    large_fan = object_model.get_part("large_fan_rotor")
    small_fan = object_model.get_part("small_fan_rotor")
    large_spin = object_model.get_articulation("large_fan_spin")
    small_spin = object_model.get_articulation("small_fan_spin")

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
        "large_fan_axis_is_z",
        tuple(large_spin.axis) == (0.0, 0.0, 1.0),
        details=f"large_fan_spin axis was {large_spin.axis!r}",
    )
    ctx.check(
        "small_fan_axis_is_z",
        tuple(small_spin.axis) == (0.0, 0.0, 1.0),
        details=f"small_fan_spin axis was {small_spin.axis!r}",
    )
    ctx.check(
        "continuous_rotor_types",
        large_spin.joint_type == ArticulationType.CONTINUOUS
        and small_spin.joint_type == ArticulationType.CONTINUOUS,
        details="Both fans should spin continuously.",
    )

    ctx.expect_gap(
        large_fan,
        body,
        axis="z",
        min_gap=0.002,
        max_gap=0.004,
        negative_elem="heatsink_core",
        name="large_rotor_clears_heatsink",
    )
    ctx.expect_gap(
        small_fan,
        body,
        axis="z",
        min_gap=0.002,
        max_gap=0.004,
        negative_elem="heatsink_core",
        name="small_rotor_clears_heatsink",
    )
    ctx.expect_contact(
        large_fan,
        body,
        elem_a="large_rotor",
        elem_b="large_spindle_cap",
        name="large_rotor_supported_by_spindle",
    )
    ctx.expect_contact(
        small_fan,
        body,
        elem_a="small_rotor",
        elem_b="small_spindle_cap",
        name="small_rotor_supported_by_spindle",
    )
    ctx.expect_within(
        large_fan,
        body,
        axes="xy",
        inner_elem="large_rotor",
        outer_elem="large_fan_lip",
        name="large_rotor_captured_in_shroud",
    )
    ctx.expect_within(
        small_fan,
        body,
        axes="xy",
        inner_elem="small_rotor",
        outer_elem="small_fan_lip",
        name="small_rotor_captured_in_shroud",
    )
    ctx.expect_overlap(
        large_fan,
        body,
        axes="xy",
        min_overlap=0.080,
        elem_a="large_rotor",
        elem_b="large_fan_lip",
        name="large_fan_centered_in_opening",
    )
    ctx.expect_overlap(
        small_fan,
        body,
        axes="xy",
        min_overlap=0.058,
        elem_a="small_rotor",
        elem_b="small_fan_lip",
        name="small_fan_centered_in_opening",
    )

    large_rest = ctx.part_world_position(large_fan)
    small_rest = ctx.part_world_position(small_fan)
    if large_rest is None or small_rest is None:
        ctx.fail("fan_world_positions_available", "Expected both fan rotors to resolve in world space.")
    else:
        with ctx.pose({large_spin: math.pi / 5.0, small_spin: -math.pi / 7.0}):
            large_pose = ctx.part_world_position(large_fan)
            small_pose = ctx.part_world_position(small_fan)
            if large_pose is None or small_pose is None:
                ctx.fail(
                    "fan_world_positions_in_pose_available",
                    "Expected both fan rotors to resolve in posed world space.",
                )
            else:
                ctx.check(
                    "large_rotor_center_stable_under_spin",
                    max(abs(a - b) for a, b in zip(large_rest, large_pose)) < 1e-9,
                    details=f"Large fan moved from {large_rest!r} to {large_pose!r}",
                )
                ctx.check(
                    "small_rotor_center_stable_under_spin",
                    max(abs(a - b) for a, b in zip(small_rest, small_pose)) < 1e-9,
                    details=f"Small fan moved from {small_rest!r} to {small_pose!r}",
                )

            ctx.expect_gap(
                large_fan,
                body,
                axis="z",
                min_gap=0.002,
                max_gap=0.004,
                negative_elem="heatsink_core",
                name="large_rotor_clears_heatsink_spun",
            )
            ctx.expect_gap(
                small_fan,
                body,
                axis="z",
                min_gap=0.002,
                max_gap=0.004,
                negative_elem="heatsink_core",
                name="small_rotor_clears_heatsink_spun",
            )
            ctx.expect_contact(
                large_fan,
                body,
                elem_a="large_rotor",
                elem_b="large_spindle_cap",
                name="large_rotor_supported_by_spindle_spun",
            )
            ctx.expect_contact(
                small_fan,
                body,
                elem_a="small_rotor",
                elem_b="small_spindle_cap",
                name="small_rotor_supported_by_spindle_spun",
            )
            ctx.expect_within(
                large_fan,
                body,
                axes="xy",
                inner_elem="large_rotor",
                outer_elem="large_fan_lip",
                name="large_rotor_stays_captured_spun",
            )
            ctx.expect_within(
                small_fan,
                body,
                axes="xy",
                inner_elem="small_rotor",
                outer_elem="small_fan_lip",
                name="small_rotor_stays_captured_spun",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
