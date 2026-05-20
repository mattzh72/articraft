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
    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def helix_points(
        *,
        center_radius: float,
        start_z: float,
        pitch: float,
        turns: float,
        samples_per_turn: int = 36,
        phase: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        sample_count = max(16, int(math.ceil(turns * samples_per_turn)))
        points: list[tuple[float, float, float]] = []
        for index in range(sample_count + 1):
            t = turns * index / sample_count
            angle = phase + (math.tau * t)
            points.append(
                (
                    center_radius * math.cos(angle),
                    center_radius * math.sin(angle),
                    start_z + (pitch * t),
                )
            )
        return points

    def thread_mesh(
        *,
        name: str,
        center_radius: float,
        start_z: float,
        pitch: float,
        turns: float,
        profile_radius: float,
        phase: float = 0.0,
    ):
        return save_mesh(
            name,
            tube_from_spline_points(
                helix_points(
                    center_radius=center_radius,
                    start_z=start_z,
                    pitch=pitch,
                    turns=turns,
                    phase=phase,
                ),
                radius=profile_radius,
                samples_per_segment=4,
                radial_segments=18,
                cap_ends=True,
                up_hint=(0.0, 0.0, 1.0),
            ),
        )

    model = ArticulatedObject(name="field_service_screwcap_bottle")

    body_poly = model.material("body_poly", rgba=(0.40, 0.46, 0.36, 1.0))
    cap_poly = model.material("cap_poly", rgba=(0.15, 0.16, 0.18, 1.0))
    boot_poly = model.material("boot_poly", rgba=(0.84, 0.48, 0.16, 1.0))
    liner_poly = model.material("liner_poly", rgba=(0.85, 0.84, 0.78, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        save_mesh(
            "bottle_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.048, 0.000),
                    (0.051, 0.006),
                    (0.053, 0.018),
                    (0.053, 0.188),
                    (0.050, 0.221),
                    (0.043, 0.248),
                    (0.032, 0.268),
                    (0.026, 0.286),
                    (0.026, 0.298),
                ],
                [
                    (0.000, 0.010),
                    (0.0465, 0.014),
                    (0.0485, 0.020),
                    (0.0485, 0.186),
                    (0.0455, 0.219),
                    (0.0385, 0.245),
                    (0.0240, 0.267),
                    (0.0200, 0.286),
                    (0.0200, 0.298),
                ],
                segments=80,
                start_cap="flat",
                end_cap="flat",
                lip_samples=8,
            ),
        ),
        material=body_poly,
        name="bottle_shell",
    )
    bottle_body.visual(
        thread_mesh(
            name="neck_thread",
            center_radius=0.0272,
            start_z=0.280,
            pitch=0.0075,
            turns=1.75,
            profile_radius=0.0014,
            phase=0.15,
        ),
        material=body_poly,
        name="neck_thread",
    )
    bottle_body.visual(
        Box((0.018, 0.030, 0.160)),
        origin=Origin(xyz=(0.000, 0.041, 0.110)),
        material=body_poly,
        name="front_guard_rail",
    )
    bottle_body.visual(
        Box((0.018, 0.030, 0.160)),
        origin=Origin(xyz=(0.000, -0.041, 0.110)),
        material=body_poly,
        name="rear_guard_rail",
    )
    bottle_body.visual(
        Box((0.030, 0.018, 0.160)),
        origin=Origin(xyz=(0.041, 0.000, 0.110)),
        material=body_poly,
        name="right_guard_rail",
    )
    bottle_body.visual(
        Box((0.030, 0.018, 0.160)),
        origin=Origin(xyz=(-0.041, 0.000, 0.110)),
        material=body_poly,
        name="left_guard_rail",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.304),
        mass=0.90,
        origin=Origin(xyz=(0.0, 0.0, 0.152)),
    )

    base_boot = model.part("base_boot")
    base_boot.visual(
        Box((0.094, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.047, 0.005)),
        material=boot_poly,
        name="front_frame_rail",
    )
    base_boot.visual(
        Box((0.094, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.047, 0.005)),
        material=boot_poly,
        name="rear_frame_rail",
    )
    base_boot.visual(
        Box((0.018, 0.094, 0.010)),
        origin=Origin(xyz=(0.047, 0.0, 0.005)),
        material=boot_poly,
        name="right_frame_rail",
    )
    base_boot.visual(
        Box((0.018, 0.094, 0.010)),
        origin=Origin(xyz=(-0.047, 0.0, 0.005)),
        material=boot_poly,
        name="left_frame_rail",
    )
    base_boot.visual(
        Box((0.022, 0.018, 0.160)),
        origin=Origin(xyz=(0.0, 0.065, 0.080)),
        material=boot_poly,
        name="front_saddle",
    )
    base_boot.visual(
        Box((0.022, 0.018, 0.160)),
        origin=Origin(xyz=(0.0, -0.065, 0.080)),
        material=boot_poly,
        name="rear_saddle",
    )
    base_boot.visual(
        Box((0.018, 0.022, 0.160)),
        origin=Origin(xyz=(0.065, 0.0, 0.080)),
        material=boot_poly,
        name="right_saddle",
    )
    base_boot.visual(
        Box((0.018, 0.022, 0.160)),
        origin=Origin(xyz=(-0.065, 0.0, 0.080)),
        material=boot_poly,
        name="left_saddle",
    )
    base_boot.inertial = Inertial.from_geometry(
        Box((0.130, 0.130, 0.160)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    screw_cap = model.part("screw_cap")
    screw_cap.visual(
        save_mesh(
            "cap_skirt_mesh",
            LatheGeometry.from_shell_profiles(
                [
                    (0.0460, 0.000),
                    (0.0460, 0.030),
                    (0.0445, 0.036),
                ],
                [
                    (0.0410, 0.000),
                    (0.0410, 0.012),
                    (0.0345, 0.020),
                    (0.0315, 0.027),
                    (0.0315, 0.032),
                    (0.0270, 0.036),
                ],
                segments=80,
                start_cap="flat",
                end_cap="flat",
                lip_samples=6,
            ),
        ),
        material=cap_poly,
        name="cap_skirt",
    )
    screw_cap.visual(
        Cylinder(radius=0.046, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=cap_poly,
        name="cap_top",
    )
    screw_cap.visual(
        thread_mesh(
            name="cap_inner_thread_mesh",
            center_radius=0.0303,
            start_z=0.014,
            pitch=0.0075,
            turns=1.60,
            profile_radius=0.0012,
            phase=0.15,
        ),
        material=cap_poly,
        name="cap_inner_thread",
    )
    screw_cap.visual(
        Box((0.012, 0.003, 0.004)),
        origin=Origin(
            xyz=(0.0356 * math.cos(0.15), 0.0356 * math.sin(0.15), 0.014),
            rpy=(0.0, 0.0, 0.15),
        ),
        material=cap_poly,
        name="thread_bridge_start",
    )
    thread_end_angle = 0.15 + (math.tau * 1.60)
    screw_cap.visual(
        Box((0.006, 0.003, 0.004)),
        origin=Origin(
            xyz=(0.0326 * math.cos(thread_end_angle), 0.0326 * math.sin(thread_end_angle), 0.026),
            rpy=(0.0, 0.0, thread_end_angle),
        ),
        material=cap_poly,
        name="thread_bridge_end",
    )
    for index in range(6):
        angle = (math.tau * index) / 6.0
        screw_cap.visual(
            Box((0.010, 0.018, 0.022)),
            origin=Origin(
                xyz=(0.0420 * math.cos(angle), 0.0420 * math.sin(angle), 0.016),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_poly,
            name=f"grip_rib_{index:02d}",
        )
    screw_cap.visual(
        Box((0.016, 0.028, 0.020)),
        origin=Origin(xyz=(0.050, 0.0, 0.016)),
        material=cap_poly,
        name="service_lug",
    )
    screw_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.051),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
    )

    seal_liner = model.part("seal_liner")
    seal_liner.visual(
        Cylinder(radius=0.022, length=0.004),
        material=liner_poly,
        name="seal_disc",
    )
    seal_liner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.004),
        mass=0.03,
    )

    model.articulation(
        "body_to_boot",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=base_boot,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )
    model.articulation(
        "body_to_cap",
        ArticulationType.REVOLUTE,
        parent=bottle_body,
        child=screw_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.266)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=12.0,
            lower=0.0,
            upper=math.tau * 1.75,
        ),
    )
    model.articulation(
        "cap_to_liner",
        ArticulationType.FIXED,
        parent=screw_cap,
        child=seal_liner,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    base_boot = object_model.get_part("base_boot")
    screw_cap = object_model.get_part("screw_cap")
    seal_liner = object_model.get_part("seal_liner")
    body_to_cap = object_model.get_articulation("body_to_cap")

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
        base_boot,
        bottle_body,
        elem_a="front_saddle",
        elem_b="front_guard_rail",
        contact_tol=0.0005,
        name="base_boot_is_clamped_to_body",
    )
    ctx.expect_origin_distance(
        screw_cap,
        bottle_body,
        axes="xy",
        max_dist=0.0005,
        name="cap_axis_is_coaxial_with_bottle_axis",
    )
    ctx.expect_within(
        bottle_body,
        screw_cap,
        axes="xy",
        inner_elem="neck_thread",
        outer_elem="cap_skirt",
        margin=0.0030,
        name="threaded_neck_stays_inside_cap_skirt",
    )
    ctx.expect_contact(
        seal_liner,
        screw_cap,
        elem_a="seal_disc",
        elem_b="cap_top",
        contact_tol=0.0005,
        name="seal_liner_is_retained_by_cap_top",
    )
    ctx.expect_contact(
        seal_liner,
        bottle_body,
        elem_a="seal_disc",
        elem_b="bottle_shell",
        contact_tol=0.0015,
        name="seal_liner_reaches_bottle_lip",
    )
    ctx.check(
        "cap_joint_rotates_about_vertical_axis",
        tuple(round(value, 6) for value in body_to_cap.axis) == (0.0, 0.0, 1.0),
        details=f"axis={body_to_cap.axis}",
    )
    limits = body_to_cap.motion_limits
    ctx.check(
        "cap_joint_has_multi_turn_service_range",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= math.tau,
        details=f"limits={limits}",
    )

    with ctx.pose({body_to_cap: 0.8}):
        ctx.expect_origin_distance(
            screw_cap,
            bottle_body,
            axes="xy",
            max_dist=0.0005,
            name="cap_stays_coaxial_while_being_turned",
        )
        ctx.expect_within(
            bottle_body,
            screw_cap,
            axes="xy",
            inner_elem="neck_thread",
            outer_elem="cap_skirt",
            margin=0.0030,
            name="neck_remains_nested_during_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
