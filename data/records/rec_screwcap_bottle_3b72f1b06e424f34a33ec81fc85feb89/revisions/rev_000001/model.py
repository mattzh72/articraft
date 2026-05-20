from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    phase: float = 0.0,
    samples_per_turn: int = 30,
) -> list[tuple[float, float, float]]:
    sample_count = max(2, int(math.ceil(turns * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for index in range(sample_count + 1):
        t = index / sample_count
        angle = phase + (math.tau * turns * t)
        points.append(
            (
                radius * math.cos(angle),
                radius * math.sin(angle),
                z_start + (pitch * turns * t),
            )
        )
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_screwcap_bottle")

    smoky_body = model.material("smoky_body", rgba=(0.62, 0.68, 0.74, 0.58))
    cap_charcoal = model.material("cap_charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    insert_gray = model.material("insert_gray", rgba=(0.76, 0.77, 0.79, 1.0))

    body = model.part("bottle_body")
    bottle_outer_profile = [
        (0.010, 0.000),
        (0.036, 0.003),
        (0.038, 0.010),
        (0.038, 0.094),
        (0.034, 0.108),
        (0.026, 0.118),
        (0.0205, 0.124),
        (0.0190, 0.132),
        (0.0190, 0.151),
    ]
    bottle_inner_profile = [
        (0.000, 0.004),
        (0.0325, 0.010),
        (0.0325, 0.093),
        (0.0285, 0.107),
        (0.0210, 0.117),
        (0.0166, 0.123),
        (0.0152, 0.132),
        (0.0152, 0.150),
    ]
    body.visual(
        _save_mesh(
            "bottle_shell",
            LatheGeometry.from_shell_profiles(
                bottle_outer_profile,
                bottle_inner_profile,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=smoky_body,
        name="body_shell",
    )
    body.visual(
        _save_mesh(
            "bottle_base_ring",
            TorusGeometry(
                radius=0.033,
                tube=0.0022,
                radial_segments=14,
                tubular_segments=60,
            ).translate(0.0, 0.0, 0.010),
        ),
        material=smoky_body,
        name="base_ring",
    )
    body.visual(
        _save_mesh(
            "neck_thread",
            tube_from_spline_points(
                _helix_points(
                    radius=0.0188,
                    z_start=0.132,
                    pitch=0.0064,
                    turns=1.6,
                    phase=math.pi / 5.0,
                    samples_per_turn=34,
                ),
                radius=0.00085,
                samples_per_segment=4,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=smoky_body,
        name="neck_thread",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.039, length=0.151),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.0755)),
    )

    cap_insert = model.part("cap_insert")
    cap_insert.visual(
        Cylinder(radius=0.0040, length=0.0240),
        origin=Origin(xyz=(0.0, 0.0, 0.0132)),
        material=insert_gray,
        name="guide_post",
    )
    cap_insert.visual(
        Cylinder(radius=0.0170, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
        material=insert_gray,
        name="seal_disc",
    )
    cap_insert.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0170, length=0.0252),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.0126)),
    )

    cap_shell = model.part("cap_shell")
    cap_outer_profile = [
        (0.0245, 0.000),
        (0.0245, 0.0205),
        (0.0225, 0.0250),
        (0.0140, 0.0283),
        (0.0000, 0.0292),
    ]
    cap_inner_profile = [
        (0.0230, 0.000),
        (0.0230, 0.0228),
        (0.0184, 0.0260),
        (0.0045, 0.0270),
    ]
    cap_shell.visual(
        _save_mesh(
            "cap_outer_shell",
            LatheGeometry.from_shell_profiles(
                cap_outer_profile,
                cap_inner_profile,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=cap_charcoal,
        name="cap_outer_shell",
    )
    for index, z_pos in enumerate((0.006, 0.012, 0.018)):
        cap_shell.visual(
            _save_mesh(
                f"cap_grip_ring_{index}",
                TorusGeometry(
                    radius=0.0248,
                    tube=0.0009,
                    radial_segments=12,
                    tubular_segments=48,
                ).translate(0.0, 0.0, z_pos),
            ),
            material=cap_charcoal,
            name=f"grip_ring_{index}",
        )
    cap_shell.visual(
        _save_mesh(
            "cap_thread",
            tube_from_spline_points(
                _helix_points(
                    radius=0.0208,
                    z_start=0.0062,
                    pitch=0.0064,
                    turns=1.6,
                    phase=math.pi / 5.0,
                    samples_per_turn=34,
                ),
                radius=0.0009,
                samples_per_segment=4,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=cap_charcoal,
        name="cap_thread",
    )
    cap_shell.visual(
        Cylinder(radius=0.0042, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0267)),
        material=cap_charcoal,
        name="inner_hub",
    )
    cap_shell.visual(
        Cylinder(radius=0.00075, length=0.0040),
        origin=Origin(
            xyz=(0.0219, 0.0, 0.0120),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cap_charcoal,
        name="thread_bridge",
    )
    cap_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0245, length=0.0292),
        mass=0.042,
        origin=Origin(xyz=(0.0, 0.0, 0.0146)),
    )

    model.articulation(
        "body_to_cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap_insert,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=0.016,
        ),
    )
    model.articulation(
        "cap_insert_to_shell_spin",
        ArticulationType.CONTINUOUS,
        parent=cap_insert,
        child=cap_shell,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bottle_body")
    cap_insert = object_model.get_part("cap_insert")
    cap_shell = object_model.get_part("cap_shell")
    cap_lift = object_model.get_articulation("body_to_cap_lift")
    cap_spin = object_model.get_articulation("cap_insert_to_shell_spin")

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
        "all_prompt_parts_present",
        all(part is not None for part in (body, cap_insert, cap_shell)),
        "Bottle body, cap insert, and cap shell must all exist.",
    )
    ctx.check(
        "thread_axes_are_coaxial_z",
        tuple(cap_lift.axis) == (0.0, 0.0, 1.0) and tuple(cap_spin.axis) == (0.0, 0.0, 1.0),
        "Lift and spin articulations must share the bottle neck axis.",
    )
    ctx.check(
        "lift_limits_allow_unscrewing_without_overtravel",
        cap_lift.motion_limits is not None
        and cap_lift.motion_limits.lower == 0.0
        and cap_lift.motion_limits.upper is not None
        and 0.010 <= cap_lift.motion_limits.upper <= 0.020,
        "Cap lift should allow a short unscrewing travel, not a detached floating cap.",
    )

    with ctx.pose({cap_lift: 0.0, cap_spin: 0.0}):
        ctx.expect_within(
            cap_insert,
            cap_shell,
            axes="xy",
            inner_elem="guide_post",
            outer_elem="cap_outer_shell",
            margin=0.0,
            name="liner_nested_inside_shell_closed",
        )
        ctx.expect_overlap(
            cap_shell,
            body,
            axes="xy",
            elem_a="cap_thread",
            elem_b="neck_thread",
            min_overlap=0.030,
            name="threads_share_coaxial_footprint_closed",
        )
        ctx.expect_within(
            body,
            cap_shell,
            axes="xy",
            inner_elem="neck_thread",
            outer_elem="cap_outer_shell",
            margin=0.0,
            name="neck_thread_fits_inside_cap_envelope_closed",
        )

    with ctx.pose({cap_lift: 0.0145, cap_spin: 1.5 * math.tau}):
        ctx.expect_gap(
            cap_shell,
            body,
            axis="z",
            positive_elem="cap_thread",
            negative_elem="neck_thread",
            min_gap=0.0012,
            name="threads_clear_when_unscrewed",
        )
        ctx.expect_origin_distance(
            cap_insert,
            cap_shell,
            axes="xy",
            max_dist=0.0005,
            name="cap_stays_coaxial_while_unscrewed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
