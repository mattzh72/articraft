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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_screwcap_bottle")

    enamel_olive = model.material("enamel_olive", rgba=(0.29, 0.35, 0.26, 1.0))
    aged_black = model.material("aged_black", rgba=(0.13, 0.13, 0.12, 1.0))
    cap_black = model.material("cap_black", rgba=(0.16, 0.15, 0.14, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.30, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    gasket = model.material("gasket", rgba=(0.11, 0.12, 0.11, 1.0))

    def helix_points(
        *,
        radius: float,
        start_z: float,
        pitch: float,
        turns: float,
        phase: float = 0.0,
        samples_per_turn: int = 24,
    ) -> list[tuple[float, float, float]]:
        sample_count = max(12, int(math.ceil(turns * samples_per_turn)))
        points: list[tuple[float, float, float]] = []
        for index in range(sample_count + 1):
            t = turns * index / sample_count
            angle = phase + math.tau * t
            z = start_z + pitch * t
            points.append((radius * math.cos(angle), radius * math.sin(angle), z))
        return points

    def helical_thread_mesh(
        name: str,
        *,
        radius: float,
        start_z: float,
        pitch: float,
        turns: float,
        thread_radius: float,
        phase: float = 0.0,
        radial_segments: int = 14,
    ):
        return mesh_from_geometry(
            tube_from_spline_points(
                helix_points(
                    radius=radius,
                    start_z=start_z,
                    pitch=pitch,
                    turns=turns,
                    phase=phase,
                ),
                radius=thread_radius,
                samples_per_segment=2,
                radial_segments=radial_segments,
                cap_ends=True,
                up_hint=(0.0, 0.0, 1.0),
            ),
            name,
        )

    bottle_body = model.part("bottle_body")

    body_outer = [
        (0.030, 0.000),
        (0.040, 0.010),
        (0.046, 0.022),
        (0.047, 0.130),
        (0.045, 0.176),
        (0.040, 0.207),
        (0.030, 0.226),
        (0.0220, 0.239),
        (0.0212, 0.267),
        (0.0225, 0.270),
    ]
    body_inner = [
        (0.000, 0.012),
        (0.028, 0.018),
        (0.039, 0.030),
        (0.041, 0.130),
        (0.039, 0.176),
        (0.034, 0.207),
        (0.024, 0.226),
        (0.0185, 0.239),
        (0.0185, 0.264),
    ]
    body_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            body_outer,
            body_inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "retro_bottle_body_shell",
    )
    bottle_body.visual(body_shell, material=enamel_olive, name="body_shell")
    bottle_body.visual(
        Cylinder(radius=0.0485, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=aged_black,
        name="base_rolling_band",
    )
    bottle_body.visual(
        Cylinder(radius=0.0475, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        material=aged_black,
        name="shoulder_rolling_band",
    )
    bottle_body.visual(
        Cylinder(radius=0.0295, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_steel,
        name="neck_reinforcement_collar",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        bottle_body.visual(
            Cylinder(radius=0.0045, length=0.032),
            origin=Origin(
                xyz=(0.017 * math.cos(angle), 0.017 * math.sin(angle), 0.220),
            ),
            material=dark_steel,
            name=f"neck_gusset_{index}",
        )
    bottle_body.visual(
        Box((0.046, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, 0.044, 0.118)),
        material=dark_steel,
        name="front_mount_pad",
    )
    bottle_body.visual(
        Box((0.040, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.044, 0.162)),
        material=dark_steel,
        name="rear_mount_pad",
    )
    bottle_body.visual(
        Box((0.008, 0.034, 0.030)),
        origin=Origin(xyz=(0.044, 0.0, 0.145)),
        material=dark_steel,
        name="side_mount_pad",
    )
    bottle_body.visual(
        helical_thread_mesh(
            "retro_bottle_neck_thread",
            radius=0.0220,
            start_z=0.245,
            pitch=0.006,
            turns=2.5,
            thread_radius=0.0014,
            phase=0.0,
        ),
        material=dark_steel,
        name="neck_thread",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.270)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    cap = model.part("cap")
    cap_outer = [
        (0.0315, 0.000),
        (0.0315, 0.030),
        (0.0295, 0.036),
        (0.0200, 0.040),
    ]
    cap_inner = [
        (0.0266, 0.001),
        (0.0266, 0.026),
        (0.0235, 0.033),
        (0.0160, 0.0365),
    ]
    cap.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                cap_outer,
                cap_inner,
                segments=72,
                start_cap="flat",
                end_cap="round",
                lip_samples=8,
            ),
            "retro_bottle_cap_shell",
        ),
        material=cap_black,
        name="cap_shell",
    )
    cap.visual(
        helical_thread_mesh(
            "retro_bottle_cap_thread",
            radius=0.0254,
            start_z=0.0045,
            pitch=0.006,
            turns=2.5,
            thread_radius=0.0015,
            phase=0.0,
        ),
        material=aged_black,
        name="cap_thread",
    )
    cap.visual(
        Cylinder(radius=0.0202, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=gasket,
        name="seal_liner",
    )
    cap.visual(
        Cylinder(radius=0.0038, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=aged_black,
        name="liner_retainer_post",
    )
    cap.visual(
        Box((0.050, 0.0035, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="cap_top_web_x",
    )
    cap.visual(
        Box((0.0035, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="cap_top_web_y",
    )
    cap.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=dark_steel,
        name="cap_service_boss",
    )
    for rib_index in range(12):
        angle = math.tau * rib_index / 12.0
        cap.visual(
            Box((0.0055, 0.0045, 0.021)),
            origin=Origin(
                xyz=(0.0293 * math.cos(angle), 0.0293 * math.sin(angle), 0.013),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"grip_rib_{rib_index}",
        )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.041),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
    )

    def add_service_hatch(part_name: str):
        hatch = model.part(part_name)
        hatch.visual(
            Box((0.042, 0.004, 0.028)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=dark_steel,
            name="cover_plate",
        )
        hatch.visual(
            Box((0.026, 0.0025, 0.014)),
            origin=Origin(xyz=(0.0, 0.00425, 0.0)),
            material=aged_black,
            name="inspection_step",
        )
        for bolt_index, (bx, bz) in enumerate(
            ((-0.015, -0.009), (0.015, -0.009), (-0.015, 0.009), (0.015, 0.009))
        ):
            hatch.visual(
                Cylinder(radius=0.0026, length=0.005),
                origin=Origin(
                    xyz=(bx, 0.0025, bz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=fastener_steel,
                name=f"bolt_{bolt_index}",
            )
        hatch.inertial = Inertial.from_geometry(
            Box((0.042, 0.006, 0.028)),
            mass=0.045,
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
        )
        return hatch

    front_service_hatch = add_service_hatch("front_service_hatch")
    rear_service_hatch = add_service_hatch("rear_service_hatch")

    side_adapter = model.part("side_adapter")
    side_adapter.visual(
        Box((0.004, 0.030, 0.024)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=dark_steel,
        name="adapter_flange",
    )
    side_adapter.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_steel,
        name="adapter_retainer_ring",
    )
    side_adapter.visual(
        Cylinder(radius=0.0095, length=0.020),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_black,
        name="adapter_spigot",
    )
    for bolt_index, (by, bz) in enumerate(
        ((-0.010, -0.007), (0.010, -0.007), (-0.010, 0.007), (0.010, 0.007))
    ):
        side_adapter.visual(
            Cylinder(radius=0.0024, length=0.005),
            origin=Origin(
                xyz=(0.0025, by, bz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=fastener_steel,
            name=f"adapter_bolt_{bolt_index}",
        )
    side_adapter.inertial = Inertial.from_geometry(
        Box((0.024, 0.030, 0.024)),
        mass=0.055,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_cap",
        ArticulationType.REVOLUTE,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.241)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=2.0 * math.tau,
        ),
    )
    model.articulation(
        "body_to_front_hatch",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=front_service_hatch,
        origin=Origin(xyz=(0.0, 0.048, 0.118)),
    )
    model.articulation(
        "body_to_rear_hatch",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=rear_service_hatch,
        origin=Origin(xyz=(0.0, -0.048, 0.162), rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "body_to_side_adapter",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=side_adapter,
        origin=Origin(xyz=(0.048, 0.0, 0.145)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    front_service_hatch = object_model.get_part("front_service_hatch")
    rear_service_hatch = object_model.get_part("rear_service_hatch")
    side_adapter = object_model.get_part("side_adapter")
    cap_joint = object_model.get_articulation("body_to_cap")

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

    ctx.expect_contact(
        front_service_hatch,
        bottle_body,
        name="front_service_hatch_is_supported",
    )
    ctx.expect_contact(
        rear_service_hatch,
        bottle_body,
        name="rear_service_hatch_is_supported",
    )
    ctx.expect_contact(
        side_adapter,
        bottle_body,
        name="side_adapter_is_supported",
    )
    ctx.expect_contact(
        cap,
        bottle_body,
        elem_a="seal_liner",
        name="cap_liner_contacts_bottle_lip",
    )
    ctx.expect_contact(
        cap,
        cap,
        elem_a="seal_liner",
        elem_b="liner_retainer_post",
        name="cap_liner_is_structurally_retained",
    )
    ctx.expect_contact(
        cap,
        cap,
        elem_a="cap_service_boss",
        elem_b="cap_top_web_x",
        name="cap_service_boss_is_supported",
    )
    ctx.expect_contact(
        cap,
        cap,
        elem_a="cap_shell",
        elem_b="cap_top_web_y",
        name="cap_top_web_is_tied_into_shell",
    )
    ctx.expect_origin_distance(
        cap,
        bottle_body,
        axes="xy",
        max_dist=0.0005,
        name="cap_axis_is_coaxial_with_bottle_axis",
    )
    ctx.expect_overlap(
        cap,
        bottle_body,
        axes="xy",
        min_overlap=0.040,
        name="cap_and_neck_share_thread_axis_footprint",
    )

    with ctx.pose({cap_joint: math.pi}):
        ctx.fail_if_parts_overlap_in_current_pose(name="cap_rotation_pose_has_no_interpenetration")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
