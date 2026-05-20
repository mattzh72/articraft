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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _helix_points(
    *,
    radius: float,
    z_start: float,
    pitch: float,
    turns: float,
    phase: float = 0.0,
    samples_per_turn: int = 40,
) -> list[tuple[float, float, float]]:
    total_steps = max(8, int(math.ceil(turns * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for step in range(total_steps + 1):
        t = turns * step / total_steps
        angle = phase + (math.tau * t)
        points.append((radius * math.cos(angle), radius * math.sin(angle), z_start + pitch * t))
    return points


def _build_body_shell_mesh():
    outer_profile = [
        (0.010, 0.000),
        (0.034, 0.002),
        (0.0415, 0.010),
        (0.0430, 0.155),
        (0.0385, 0.185),
        (0.0330, 0.198),
        (0.0308, 0.206),
        (0.0268, 0.221),
        (0.0255, 0.232),
        (0.0255, 0.242),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.030, 0.010),
        (0.0385, 0.016),
        (0.0390, 0.155),
        (0.0348, 0.183),
        (0.0298, 0.198),
        (0.0279, 0.206),
        (0.0242, 0.219),
        (0.0228, 0.230),
        (0.0228, 0.239),
    ]
    return LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=72)


def _build_cap_shell_mesh():
    outer_profile = [
        (0.0375, 0.000),
        (0.0375, 0.038),
        (0.0348, 0.046),
        (0.0220, 0.053),
        (0.0000, 0.057),
    ]
    inner_profile = [
        (0.0340, 0.0012),
        (0.0340, 0.034),
        (0.0308, 0.043),
        (0.0182, 0.049),
        (0.0000, 0.053),
    ]
    return LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=72)


def _build_cap_thread_carrier_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0340, 0.0190),
            (0.0340, 0.0365),
        ],
        [
            (0.0300, 0.0190),
            (0.0300, 0.0365),
        ],
        segments=60,
    )


def _build_thread_mesh(
    *,
    radius: float,
    z_start: float,
    pitch: float,
    turns: float,
    thread_radius: float,
    phase: float,
):
    return tube_from_spline_points(
        _helix_points(
            radius=radius,
            z_start=z_start,
            pitch=pitch,
            turns=turns,
            phase=phase,
            samples_per_turn=42,
        ),
        radius=thread_radius,
        samples_per_segment=2,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _build_neck_lip_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0262, 0.2385),
            (0.0262, 0.2420),
        ],
        [
            (0.0228, 0.2385),
            (0.0228, 0.2420),
        ],
        segments=56,
    )


def _build_drain_collar_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0320, 0.2030),
            (0.0320, 0.2070),
        ],
        [
            (0.0290, 0.2030),
            (0.0290, 0.2070),
        ],
        segments=56,
    )


def _build_carry_loop_mesh():
    return tube_from_spline_points(
        [
            (-0.016, 0.000, 0.055),
            (-0.019, 0.000, 0.067),
            (-0.012, 0.000, 0.081),
            (0.000, 0.000, 0.086),
            (0.012, 0.000, 0.081),
            (0.019, 0.000, 0.067),
            (0.016, 0.000, 0.055),
        ],
        radius=0.0026,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_screwcap_bottle")

    body_paint = model.material("body_paint", rgba=(0.29, 0.38, 0.28, 1.0))
    bumper_black = model.material("bumper_black", rgba=(0.11, 0.11, 0.12, 1.0))
    cap_polymer = model.material("cap_polymer", rgba=(0.14, 0.15, 0.16, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.76, 0.79, 1.0))
    seal_elastomer = model.material("seal_elastomer", rgba=(0.72, 0.36, 0.18, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        _mesh("body_shell", _build_body_shell_mesh()),
        material=body_paint,
        name="body_shell",
    )
    bottle_body.visual(
        _mesh(
            "neck_thread",
            _build_thread_mesh(
                radius=0.0269,
                z_start=0.212,
                pitch=0.0075,
                turns=2.7,
                thread_radius=0.0011,
                phase=0.38,
            ),
        ),
        material=body_paint,
        name="neck_thread",
    )
    bottle_body.visual(
        _mesh("neck_lip", _build_neck_lip_mesh()),
        material=stainless,
        name="neck_lip",
    )
    bottle_body.visual(
        _mesh("drain_collar", _build_drain_collar_mesh()),
        material=body_paint,
        name="drain_collar",
    )
    bottle_body.visual(
        _mesh(
            "base_bumper",
            TorusGeometry(
                radius=0.0395,
                tube=0.0038,
                radial_segments=16,
                tubular_segments=56,
            ).translate(0.0, 0.0, 0.0105),
        ),
        material=bumper_black,
        name="base_bumper",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.245)),
        mass=0.68,
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
    )

    cap = model.part("cap")
    cap.visual(
        _mesh("cap_shell", _build_cap_shell_mesh()),
        material=cap_polymer,
        name="cap_shell",
    )
    cap.visual(
        _mesh("cap_thread_carrier", _build_cap_thread_carrier_mesh()),
        material=cap_polymer,
        name="cap_thread_carrier",
    )
    cap.visual(
        _mesh(
            "cap_thread",
            _build_thread_mesh(
                radius=0.0291,
                z_start=0.020,
                pitch=0.0075,
                turns=2.7,
                thread_radius=0.0010,
                phase=0.38,
            ),
        ),
        material=cap_polymer,
        name="cap_thread",
    )
    cap.visual(
        Cylinder(radius=0.0235, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0451)),
        material=seal_elastomer,
        name="seal_liner",
    )
    cap.visual(
        Cylinder(radius=0.0050, length=0.010),
        origin=Origin(xyz=(-0.016, 0.0, 0.052)),
        material=cap_polymer,
        name="loop_lug_left",
    )
    cap.visual(
        Cylinder(radius=0.0050, length=0.010),
        origin=Origin(xyz=(0.016, 0.0, 0.052)),
        material=cap_polymer,
        name="loop_lug_right",
    )
    cap.visual(
        _mesh("carry_loop", _build_carry_loop_mesh()),
        material=stainless,
        name="carry_loop",
    )
    for rib_index in range(14):
        angle = rib_index * math.tau / 14.0
        cap.visual(
            Box((0.0046, 0.0072, 0.028)),
            origin=Origin(
                xyz=(0.0352 * math.cos(angle), 0.0352 * math.sin(angle), 0.018),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_polymer,
            name=f"grip_rib_{rib_index:02d}",
        )
    cap.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.090)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
    )

    # The real mechanism is a helical screw closure. The SDK does not expose a
    # screw joint, so the articulation captures the net axial travel while the
    # visible internal/external helices carry the thread engagement.
    model.articulation(
        "body_to_cap",
        ArticulationType.PRISMATIC,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.065,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    cap_travel = object_model.get_articulation("body_to_cap")

    neck_thread = bottle_body.get_visual("neck_thread")
    neck_lip = bottle_body.get_visual("neck_lip")
    cap_shell = cap.get_visual("cap_shell")
    cap_thread = cap.get_visual("cap_thread")
    seal_liner = cap.get_visual("seal_liner")

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
        "cap_travel_axis_is_vertical",
        tuple(cap_travel.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical cap travel, got axis={cap_travel.axis}",
    )

    with ctx.pose({cap_travel: 0.0}):
        ctx.expect_contact(
            cap,
            bottle_body,
            elem_a=seal_liner,
            elem_b=neck_lip,
            contact_tol=0.0008,
            name="seal_liner_contacts_neck_lip_when_closed",
        )
        ctx.expect_overlap(
            cap,
            bottle_body,
            elem_a=cap_thread,
            elem_b=neck_thread,
            axes="xy",
            min_overlap=0.050,
            name="thread_elements_are_coaxially_engaged_in_plan",
        )
        ctx.expect_within(
            bottle_body,
            cap,
            inner_elem=neck_thread,
            outer_elem=cap_shell,
            axes="xy",
            margin=0.0015,
            name="neck_thread_stays_within_cap_shroud",
        )

    with ctx.pose({cap_travel: 0.065}):
        ctx.expect_gap(
            cap,
            bottle_body,
            positive_elem=cap_thread,
            negative_elem=neck_thread,
            axis="z",
            min_gap=0.020,
            name="opened_cap_thread_clears_neck_thread_axially",
        )
        ctx.expect_gap(
            cap,
            bottle_body,
            positive_elem=cap_shell,
            negative_elem=neck_lip,
            axis="z",
            min_gap=0.015,
            name="opened_cap_lifts_clear_of_mouth",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
