from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _helical_thread_mesh(
    *,
    center_radius: float,
    start_z: float,
    pitch: float,
    turns: float,
    thread_radius: float,
    start_angle: float = 0.0,
    samples_per_turn: int = 56,
) -> MeshGeometry:
    point_count = max(12, int(math.ceil(turns * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for index in range(point_count + 1):
        fraction = index / point_count
        angle = start_angle + fraction * turns * math.tau
        points.append(
            (
                center_radius * math.cos(angle),
                center_radius * math.sin(angle),
                start_z + fraction * turns * pitch,
            )
        )
    return tube_from_spline_points(
        points,
        radius=thread_radius,
        samples_per_segment=3,
        radial_segments=18,
        cap_ends=True,
    )


def _annular_ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    start_z: float,
    end_z: float,
    segments: int = 56,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, start_z), (outer_radius, end_z)],
        [(inner_radius, start_z), (inner_radius, end_z)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _build_bottle_body_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0335, 0.000),
            (0.0360, 0.004),
            (0.0365, 0.040),
            (0.0365, 0.152),
            (0.0360, 0.176),
            (0.0330, 0.190),
            (0.0260, 0.198),
            (0.0210, 0.202),
            (0.0187, 0.206),
            (0.0187, 0.236),
        ],
        [
            (0.0000, 0.005),
            (0.0295, 0.011),
            (0.0325, 0.040),
            (0.0325, 0.151),
            (0.0305, 0.175),
            (0.0280, 0.188),
            (0.0215, 0.198),
            (0.0172, 0.202),
            (0.0148, 0.206),
            (0.0148, 0.236),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_bottle_thread_mesh() -> MeshGeometry:
    return _helical_thread_mesh(
        center_radius=0.01965,
        start_z=0.210,
        pitch=0.006,
        turns=2.3,
        thread_radius=0.00095,
        samples_per_turn=60,
    )


def _build_cap_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0248, 0.0000),
            (0.0253, 0.0050),
            (0.0256, 0.0180),
            (0.0252, 0.0280),
            (0.0238, 0.0320),
        ],
        [
            (0.0216, 0.0000),
            (0.0212, 0.0200),
            (0.0210, 0.0295),
            (0.0180, 0.0310),
            (0.0000, 0.0320),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_cap_thread_mesh() -> MeshGeometry:
    return _helical_thread_mesh(
        center_radius=0.02175,
        start_z=0.0070,
        pitch=0.006,
        turns=2.3,
        thread_radius=0.00055,
        start_angle=math.pi,
        samples_per_turn=60,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screwcap_bottle")

    body_paint = model.material("body_paint", rgba=(0.62, 0.73, 0.84, 1.0))
    polymer_charcoal = model.material("polymer_charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    elastomer_black = model.material("elastomer_black", rgba=(0.07, 0.07, 0.08, 1.0))
    cap_badge_paint = model.material("cap_badge_paint", rgba=(0.70, 0.72, 0.76, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        _save_mesh("bottle_body_shell", _build_bottle_body_mesh()),
        material=body_paint,
        name="body_shell",
    )
    bottle_body.visual(
        _save_mesh("bottle_body_thread", _build_bottle_thread_mesh()),
        material=body_paint,
        name="neck_thread",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.236),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )

    cap = model.part("cap")
    cap.visual(
        _save_mesh("cap_shell", _build_cap_shell_mesh()),
        material=polymer_charcoal,
        name="cap_shell",
    )
    cap.visual(
        _save_mesh("cap_thread", _build_cap_thread_mesh()),
        material=polymer_charcoal,
        name="cap_thread",
    )
    cap.visual(
        _save_mesh(
            "cap_grip_band",
            _annular_ring_mesh(
                outer_radius=0.0262,
                inner_radius=0.0245,
                start_z=0.0020,
                end_z=0.0150,
            ),
        ),
        material=elastomer_black,
        name="grip_band",
    )
    cap.visual(
        _save_mesh(
            "cap_seal",
            _annular_ring_mesh(
                outer_radius=0.0178,
                inner_radius=0.0120,
                start_z=0.0298,
                end_z=0.0314,
                segments=48,
            ),
        ),
        material=elastomer_black,
        name="seal",
    )
    cap.visual(
        Cylinder(radius=0.0175, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0326)),
        material=cap_badge_paint,
        name="cap_badge",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0265, length=0.034),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    model.articulation(
        "bottle_to_cap",
        ArticulationType.PRISMATIC,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.2062005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.028,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    cap_slide = object_model.get_articulation("bottle_to_cap")
    limits = cap_slide.motion_limits

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
        "cap_joint_axis_is_vertical",
        tuple(cap_slide.axis) == (0.0, 0.0, 1.0),
        f"expected +Z prismatic axis, got {cap_slide.axis}",
    )
    ctx.check(
        "cap_joint_has_useful_travel",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper >= 0.025,
        f"unexpected motion limits: {limits}",
    )

    with ctx.pose({cap_slide: 0.0}):
        ctx.expect_contact(
            cap,
            bottle_body,
            elem_a="seal",
            contact_tol=0.001,
            name="seal_contacts_bottle_lip_closed",
        )
        ctx.expect_overlap(
            cap,
            bottle_body,
            axes="xy",
            min_overlap=0.030,
            name="cap_overlaps_bottle_footprint_closed",
        )
        ctx.expect_origin_distance(
            cap,
            bottle_body,
            axes="xy",
            max_dist=0.0005,
            name="cap_is_coaxial_closed",
        )

    with ctx.pose({cap_slide: 0.028}):
        ctx.expect_gap(
            cap,
            bottle_body,
            axis="z",
            min_gap=0.020,
            positive_elem="seal",
            name="opened_cap_lifts_clear_of_lip",
        )
        ctx.expect_origin_distance(
            cap,
            bottle_body,
            axes="xy",
            max_dist=0.0005,
            name="cap_is_coaxial_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
