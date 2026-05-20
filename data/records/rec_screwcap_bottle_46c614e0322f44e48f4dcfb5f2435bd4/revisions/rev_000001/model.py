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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _helix_points(
    *,
    radius: float,
    z_start: float,
    pitch: float,
    turns: float,
    start_angle: float = 0.0,
    control_points_per_turn: int = 8,
) -> list[tuple[float, float, float]]:
    total_steps = max(8, int(math.ceil(turns * control_points_per_turn)))
    points: list[tuple[float, float, float]] = []
    for step in range(total_steps + 1):
        t = step / total_steps
        angle = start_angle + (math.tau * turns * t)
        z = z_start + (pitch * turns * t)
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _thread_mesh(
    *,
    radius: float,
    z_start: float,
    pitch: float,
    turns: float,
    start_angle: float,
    thread_radius: float,
):
    return tube_from_spline_points(
        _helix_points(
            radius=radius,
            z_start=z_start,
            pitch=pitch,
            turns=turns,
            start_angle=start_angle,
            control_points_per_turn=7,
        ),
        radius=thread_radius,
        samples_per_segment=6,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_screwcap_bottle")

    body_steel = model.material("body_steel", rgba=(0.59, 0.61, 0.63, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.84, 0.71, 0.12, 1.0))
    cap_graphite = model.material("cap_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.86, 0.34, 0.10, 1.0))
    thread_metal = model.material("thread_metal", rgba=(0.50, 0.52, 0.54, 1.0))

    thread_pitch = 0.014
    thread_turns = 1.65
    external_thread_z = 0.294
    cap_origin_z = 0.289
    internal_thread_local_z = 0.008
    external_thread_angle = 0.35
    internal_thread_angle = external_thread_angle + (
        math.tau * ((cap_origin_z + internal_thread_local_z) - external_thread_z) / thread_pitch
    )

    body = model.part("bottle_body")
    body_shell = _save_mesh(
        "bottle_body_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.061, 0.000),
                (0.063, 0.006),
                (0.062, 0.016),
                (0.060, 0.035),
                (0.059, 0.180),
                (0.056, 0.228),
                (0.050, 0.252),
                (0.041, 0.272),
                (0.032, 0.288),
                (0.029, 0.304),
                (0.029, 0.324),
            ],
            [
                (0.050, 0.008),
                (0.052, 0.018),
                (0.051, 0.180),
                (0.048, 0.228),
                (0.042, 0.252),
                (0.034, 0.272),
                (0.024, 0.288),
                (0.021, 0.304),
                (0.021, 0.324),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    body.visual(body_shell, material=body_steel, name="body_shell")
    body.visual(
        Cylinder(radius=0.050, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=body_steel,
        name="base_floor",
    )
    body.visual(
        _save_mesh(
            "bottle_base_bumper",
            TorusGeometry(
                radius=0.063,
                tube=0.004,
                radial_segments=14,
                tubular_segments=56,
            ).translate(0.0, 0.0, 0.016),
        ),
        material=guard_yellow,
        name="base_bumper",
    )
    body.visual(
        _save_mesh(
            "bottle_shoulder_reinforcement",
            TorusGeometry(
                radius=0.056,
                tube=0.004,
                radial_segments=14,
                tubular_segments=56,
            ).translate(0.0, 0.0, 0.220),
        ),
        material=guard_yellow,
        name="shoulder_reinforcement",
    )
    body.visual(
        _save_mesh(
            "bottle_neck_collar",
            TorusGeometry(
                radius=0.044,
                tube=0.004,
                radial_segments=14,
                tubular_segments=56,
            ).translate(0.0, 0.0, 0.268),
        ),
        material=guard_yellow,
        name="neck_collar",
    )
    body.visual(
        _save_mesh(
            "bottle_external_thread",
            _thread_mesh(
                radius=0.0295,
                z_start=external_thread_z,
                pitch=thread_pitch,
                turns=thread_turns,
                start_angle=external_thread_angle,
                thread_radius=0.0010,
            ),
        ),
        material=thread_metal,
        name="neck_thread",
    )

    cardinal_angles = (0.0, math.pi * 0.5, math.pi, math.pi * 1.5)
    for index, angle in enumerate(cardinal_angles):
        ux = math.cos(angle)
        uy = math.sin(angle)
        body.visual(
            Cylinder(radius=0.0045, length=0.110),
            origin=Origin(xyz=(0.068 * ux, 0.068 * uy, 0.270)),
            material=guard_yellow,
            name=f"guard_post_{index}",
        )
        body.visual(
            Box((0.040, 0.016, 0.010)),
            origin=Origin(xyz=(0.049 * ux, 0.049 * uy, 0.238), rpy=(0.0, 0.0, angle)),
            material=guard_yellow,
            name=f"shoulder_plate_{index}",
        )
        body.visual(
            Box((0.050, 0.016, 0.008)),
            origin=Origin(xyz=(0.049 * ux, 0.049 * uy, 0.285), rpy=(0.0, 0.0, angle)),
            material=guard_yellow,
            name=f"body_stop_{index}",
        )
        body.visual(
            Box((0.018, 0.014, 0.042)),
            origin=Origin(xyz=(0.058 * ux, 0.058 * uy, 0.262), rpy=(0.0, 0.0, angle)),
            material=guard_yellow,
            name=f"upright_web_{index}",
        )
        for bolt_level, bolt_z in enumerate((0.238, 0.285)):
            body.visual(
                Cylinder(radius=0.0022, length=0.018),
                origin=Origin(
                    xyz=(0.061 * ux, 0.061 * uy, bolt_z),
                    rpy=(0.0, math.pi * 0.5, angle),
                ),
                material=fastener_steel,
                name=f"radial_bolt_shaft_{index}_{bolt_level}",
            )
            body.visual(
                Cylinder(radius=0.0038, length=0.004),
                origin=Origin(
                    xyz=(0.0705 * ux, 0.0705 * uy, bolt_z),
                    rpy=(0.0, math.pi * 0.5, angle),
                ),
                material=fastener_steel,
                name=f"radial_bolt_head_{index}_{bolt_level}",
            )

    for index, angle in enumerate((math.pi * 0.5, math.pi * 1.5)):
        ux = math.cos(angle)
        uy = math.sin(angle)
        body.visual(
            Box((0.014, 0.016, 0.034)),
            origin=Origin(xyz=(0.068 * ux, 0.068 * uy, 0.299), rpy=(0.0, 0.0, angle)),
            material=safety_orange,
            name=f"lockout_tower_{index}",
        )
        body.visual(
            Box((0.014, 0.012, 0.014)),
            origin=Origin(xyz=(0.060 * ux, 0.060 * uy, 0.291), rpy=(0.0, 0.0, angle)),
            material=safety_orange,
            name=f"lockout_bridge_{index}",
        )
        body.visual(
            Box((0.014, 0.016, 0.008)),
            origin=Origin(xyz=(0.054 * ux, 0.054 * uy, 0.302), rpy=(0.0, 0.0, angle)),
            material=safety_orange,
            name=f"lockout_pawl_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.150, 0.150, 0.340)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    cap = model.part("safety_cap")
    cap_shell = _save_mesh(
        "safety_cap_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.043, 0.000),
                (0.043, 0.036),
                (0.041, 0.044),
            ],
            [
                (0.0364, 0.000),
                (0.0364, 0.034),
                (0.0310, 0.044),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    cap.visual(cap_shell, material=cap_graphite, name="cap_shell")
    cap.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=cap_graphite,
        name="cap_top",
    )
    cap.visual(
        _save_mesh(
            "safety_cap_internal_thread",
            _thread_mesh(
                radius=0.0358,
                z_start=internal_thread_local_z,
                pitch=thread_pitch,
                turns=thread_turns,
                start_angle=internal_thread_angle,
                thread_radius=0.0010,
            ),
        ),
        material=thread_metal,
        name="cap_thread",
    )

    for index, angle in enumerate(cardinal_angles):
        ux = math.cos(angle)
        uy = math.sin(angle)
        rib_angle = angle + (math.pi * 0.25)
        rib_ux = math.cos(rib_angle)
        rib_uy = math.sin(rib_angle)
        cap.visual(
            Box((0.014, 0.018, 0.030)),
            origin=Origin(xyz=(0.047 * rib_ux, 0.047 * rib_uy, 0.020), rpy=(0.0, 0.0, rib_angle)),
            material=cap_graphite,
            name=f"cap_rib_{index}",
        )
        cap.visual(
            Box((0.018, 0.016, 0.008)),
            origin=Origin(xyz=(0.047 * ux, 0.047 * uy, 0.004), rpy=(0.0, 0.0, angle)),
            material=safety_orange,
            name=f"cap_lug_{index}",
        )

    for index, angle in enumerate((math.pi * 0.5, math.pi * 1.5)):
        ux = math.cos(angle)
        uy = math.sin(angle)
        cap.visual(
            Box((0.012, 0.012, 0.026)),
            origin=Origin(xyz=(0.044 * ux, 0.044 * uy, 0.022), rpy=(0.0, 0.0, angle)),
            material=safety_orange,
            name=f"squeeze_spine_{index}",
        )
        cap.visual(
            Box((0.016, 0.024, 0.012)),
            origin=Origin(xyz=(0.053 * ux, 0.053 * uy, 0.027), rpy=(0.0, 0.0, angle)),
            material=safety_orange,
            name=f"squeeze_tab_{index}",
        )

    cap.visual(
        Box((0.014, 0.014, 0.018)),
        origin=Origin(xyz=(-0.020, 0.0, 0.050)),
        material=guard_yellow,
        name="handle_pedestal_left",
    )
    cap.visual(
        Box((0.014, 0.014, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.050)),
        material=guard_yellow,
        name="handle_pedestal_right",
    )
    cap.visual(
        Box((0.066, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=guard_yellow,
        name="handle_bridge",
    )
    for pedestal_x in (-0.020, 0.020):
        for bolt_index, bolt_y in enumerate((-0.004, 0.004)):
            cap.visual(
                Cylinder(radius=0.0020, length=0.015),
                origin=Origin(xyz=(pedestal_x, bolt_y, 0.046)),
                material=fastener_steel,
                name=f"cap_bolt_shaft_{'l' if pedestal_x < 0.0 else 'r'}_{bolt_index}",
            )
            cap.visual(
                Cylinder(radius=0.0035, length=0.003),
                origin=Origin(xyz=(pedestal_x, bolt_y, 0.053)),
                material=fastener_steel,
                name=f"cap_bolt_head_{'l' if pedestal_x < 0.0 else 'r'}_{bolt_index}",
            )

    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.070),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "body_to_cap",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, cap_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.10,
            lower=0.0,
            upper=0.030,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("safety_cap")
    cap_lift = object_model.get_articulation("body_to_cap")
    neck_thread = body.get_visual("neck_thread")
    cap_thread = cap.get_visual("cap_thread")
    body_stop = body.get_visual("body_stop_0")
    cap_lug = cap.get_visual("cap_lug_0")
    pawl = body.get_visual("lockout_pawl_0")
    squeeze_tab = cap.get_visual("squeeze_tab_0")

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

    limits = cap_lift.motion_limits
    ctx.check(
        "cap_joint_axis_is_vertical",
        tuple(round(value, 6) for value in cap_lift.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical prismatic lift, got axis={cap_lift.axis}",
    )
    ctx.check(
        "cap_joint_limits_match_service_lift",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.020 <= limits.upper <= 0.040,
        details=f"expected a realistic service-lift range, got limits={limits}",
    )

    with ctx.pose({cap_lift: 0.0}):
        ctx.expect_contact(
            cap,
            body,
            elem_a=cap_lug,
            elem_b=body_stop,
            name="closed_cap_stop_seats_on_body_stop",
        )
        ctx.expect_overlap(
            cap,
            body,
            axes="xy",
            elem_a=cap_thread,
            elem_b=neck_thread,
            min_overlap=0.060,
            name="thread_engagement_stays_coaxial",
        )
        ctx.expect_overlap(
            cap,
            body,
            axes="xy",
            elem_a=squeeze_tab,
            elem_b=pawl,
            min_overlap=0.012,
            name="lockout_tab_aligns_with_pawl",
        )

    with ctx.pose({cap_lift: limits.upper if limits and limits.upper is not None else 0.030}):
        ctx.expect_gap(
            cap,
            body,
            axis="z",
            positive_elem=cap_lug,
            negative_elem=body_stop,
            min_gap=0.020,
            name="cap_lifts_clear_of_overtravel_stop",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
