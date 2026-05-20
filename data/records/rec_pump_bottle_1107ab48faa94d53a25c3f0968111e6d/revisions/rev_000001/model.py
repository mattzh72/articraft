from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_HEIGHT = 0.196
BODY_RADIUS = 0.0245
NECK_RADIUS = 0.0186
ACTUATOR_STROKE = 0.0045
RING_HEIGHT = 0.010
RING_CENTER_Z = 0.181


def _c_ring_profile(
    *,
    inner_radius: float,
    outer_radius: float,
    gap_angle: float,
    arc_segments: int = 48,
) -> list[tuple[float, float]]:
    start = gap_angle * 0.5
    end = (2.0 * math.pi) - start

    profile: list[tuple[float, float]] = []
    for index in range(arc_segments + 1):
        t = index / arc_segments
        angle = start + (end - start) * t
        profile.append((outer_radius * math.cos(angle), outer_radius * math.sin(angle)))
    for index in range(arc_segments, -1, -1):
        t = index / arc_segments
        angle = start + (end - start) * t
        profile.append((inner_radius * math.cos(angle), inner_radius * math.sin(angle)))
    return profile


def _build_actuator_cap_mesh():
    cap_profile = rounded_rect_profile(0.034, 0.028, 0.008, corner_segments=8)
    return ExtrudeGeometry.centered(cap_profile, 0.008).translate(0.0, 0.0, 0.0085)


def _build_actuator_piston_mesh():
    guide = CylinderGeometry(radius=0.0200, height=0.001, radial_segments=64).translate(0.0, 0.0, 0.0005)
    connector = CylinderGeometry(radius=0.0065, height=0.0035, radial_segments=40).translate(
        0.0,
        0.0,
        0.00275,
    )
    pump_stem = CylinderGeometry(radius=0.0045, height=0.030, radial_segments=32).translate(0.0, 0.0, -0.015)
    guide.merge(connector)
    guide.merge(pump_stem)
    return guide


def _build_lock_ring_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0208, -0.0050),
            (0.0232, -0.0040),
            (0.0232, 0.0050),
            (0.0216, 0.0050),
        ],
        [
            (0.0191, -0.0050),
            (0.0191, 0.0050),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="airless_pump_bottle")

    body_white = model.material("body_white", rgba=(0.95, 0.96, 0.97, 1.0))
    actuator_white = model.material("actuator_white", rgba=(0.98, 0.98, 0.99, 1.0))
    lock_silver = model.material("lock_silver", rgba=(0.78, 0.80, 0.83, 1.0))

    outer_body = model.part("outer_body")
    outer_body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.154),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=body_white,
        name="body_shell",
    )
    outer_body.visual(
        Cylinder(radius=0.0236, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.1595)),
        material=body_white,
        name="shoulder_lower",
    )
    outer_body.visual(
        Cylinder(radius=0.0214, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=body_white,
        name="shoulder_upper",
    )
    outer_body.visual(
        Cylinder(radius=NECK_RADIUS, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=body_white,
        name="neck_shell",
    )
    outer_body.visual(
        Cylinder(radius=0.0220, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        material=body_white,
        name="collar_shell",
    )
    outer_body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=BODY_HEIGHT),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    actuator = model.part("actuator")
    actuator.visual(
        mesh_from_geometry(_build_actuator_cap_mesh(), "actuator_cap"),
        material=actuator_white,
        name="actuator_cap",
    )
    actuator.visual(
        mesh_from_geometry(_build_actuator_piston_mesh(), "actuator_piston"),
        material=actuator_white,
        name="actuator_piston",
    )
    actuator.inertial = Inertial.from_geometry(
        Box((0.034, 0.028, 0.052)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )

    lock_ring = model.part("lock_ring")
    lock_ring.visual(
        mesh_from_geometry(_build_lock_ring_mesh(), "twist_lock_ring"),
        material=lock_silver,
        name="lock_ring_shell",
    )
    lock_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0248, length=RING_HEIGHT),
        mass=0.014,
    )

    model.articulation(
        "body_to_actuator",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=actuator,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=ACTUATOR_STROKE,
        ),
    )

    model.articulation(
        "body_to_lock_ring",
        ArticulationType.REVOLUTE,
        parent=outer_body,
        child=lock_ring,
        origin=Origin(xyz=(0.0, 0.0, RING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-0.7,
            upper=0.7,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_body = object_model.get_part("outer_body")
    actuator = object_model.get_part("actuator")
    lock_ring = object_model.get_part("lock_ring")
    pump_slide = object_model.get_articulation("body_to_actuator")
    ring_twist = object_model.get_articulation("body_to_lock_ring")

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
    ctx.allow_overlap(
        actuator,
        outer_body,
        elem_a="actuator_piston",
        elem_b="neck_shell",
        reason=(
            "The internal pump stem telescopes inside the bottle neck; the neck is "
            "represented as a simple solid sleeve rather than a boolean-open shell."
        ),
    )
    ctx.allow_overlap(
        actuator,
        outer_body,
        elem_a="actuator_piston",
        elem_b="shoulder_upper",
        reason=(
            "The actuator's hidden guide skirt continues through the upper shoulder "
            "region, which is represented by solid envelope cylinders instead of an open shell."
        ),
    )
    ctx.allow_overlap(
        actuator,
        outer_body,
        elem_a="actuator_piston",
        elem_b="collar_shell",
        reason=(
            "The actuator guide is captured within the top collar; this envelope collar is "
            "modeled as a solid ring rather than a hollow boolean shell."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    pump_limits = pump_slide.motion_limits
    ctx.check(
        "actuator_prismatic_axis",
        pump_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(value, 6) for value in pump_slide.axis) == (0.0, 0.0, -1.0)
        and pump_limits is not None
        and math.isclose(pump_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and pump_limits.upper is not None
        and math.isclose(pump_limits.upper, ACTUATOR_STROKE, abs_tol=1e-9),
        "Actuator should ride on a short downward prismatic stroke into the collar.",
    )

    ring_limits = ring_twist.motion_limits
    ctx.check(
        "lock_ring_revolute_axis",
        ring_twist.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in ring_twist.axis) == (0.0, 0.0, 1.0)
        and ring_limits is not None
        and ring_limits.lower is not None
        and ring_limits.upper is not None
        and ring_limits.lower < 0.0 < ring_limits.upper,
        "Twist-lock ring should rotate around the neck on its own vertical axis.",
    )

    ctx.expect_contact(
        actuator,
        outer_body,
        elem_a="actuator_piston",
        elem_b="collar_shell",
        contact_tol=5e-4,
        name="actuator_skirt_guided_by_collar",
    )
    ctx.expect_origin_distance(
        lock_ring,
        outer_body,
        axes="xy",
        max_dist=1e-6,
        name="lock_ring_is_concentric_with_neck",
    )
    ctx.expect_overlap(
        lock_ring,
        outer_body,
        axes="xy",
        elem_a="lock_ring_shell",
        elem_b="collar_shell",
        min_overlap=0.040,
        name="lock_ring_wraps_around_neck_footprint",
    )
    ctx.expect_contact(
        lock_ring,
        outer_body,
        elem_a="lock_ring_shell",
        elem_b="collar_shell",
        contact_tol=1e-5,
        name="lock_ring_is_captured_under_collar",
    )
    ctx.expect_gap(
        actuator,
        outer_body,
        axis="z",
        positive_elem="actuator_cap",
        negative_elem="collar_shell",
        min_gap=0.004,
        max_gap=0.006,
        name="actuator_cap_sits_above_collar",
    )
    ctx.expect_gap(
        actuator,
        lock_ring,
        axis="z",
        positive_elem="actuator_cap",
        min_gap=0.010,
        max_gap=0.022,
        name="actuator_cap_starts_above_lock_ring",
    )

    actuator_rest = ctx.part_world_position(actuator)
    with ctx.pose({pump_slide: ACTUATOR_STROKE}):
        actuator_pressed = ctx.part_world_position(actuator)
        ctx.check(
            "actuator_stroke_distance",
            actuator_rest is not None
            and actuator_pressed is not None
            and math.isclose(
                actuator_rest[2] - actuator_pressed[2],
                ACTUATOR_STROKE,
                abs_tol=1e-5,
            ),
            "Actuator should translate downward by its full short stroke.",
        )
        ctx.expect_within(
            actuator,
            outer_body,
            axes="xy",
            inner_elem="actuator_piston",
            outer_elem="collar_shell",
            margin=0.003,
            name="actuator_piston_remains_centered_when_pressed",
        )
        ctx.expect_gap(
            actuator,
            outer_body,
            axis="z",
            positive_elem="actuator_cap",
            negative_elem="collar_shell",
            min_gap=-0.0002,
            max_gap=0.001,
            name="actuator_cap_descends_toward_collar_when_pressed",
        )
        ctx.expect_gap(
            actuator,
            lock_ring,
            axis="z",
            positive_elem="actuator_cap",
            min_gap=0.005,
            max_gap=0.016,
            name="actuator_cap_clears_lock_ring_when_pressed",
        )

    ring_rest = ctx.part_world_position(lock_ring)
    with ctx.pose({ring_twist: 0.55}):
        ring_rotated = ctx.part_world_position(lock_ring)
        ctx.check(
            "lock_ring_rotates_in_place",
            ring_rest is not None
            and ring_rotated is not None
            and math.isclose(ring_rest[0], ring_rotated[0], abs_tol=1e-6)
            and math.isclose(ring_rest[1], ring_rotated[1], abs_tol=1e-6)
            and math.isclose(ring_rest[2], ring_rotated[2], abs_tol=1e-6),
            "Twisting the lock ring should not shift it off the neck.",
        )
        ctx.expect_contact(
            lock_ring,
            outer_body,
            elem_a="lock_ring_shell",
            elem_b="collar_shell",
            contact_tol=1e-5,
            name="lock_ring_stays_captured_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
