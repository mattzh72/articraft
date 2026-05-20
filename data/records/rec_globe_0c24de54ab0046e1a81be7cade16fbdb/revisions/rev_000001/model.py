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
    Sphere,
    TestContext,
    TestReport,
)


MERIDIAN_RADIUS = 0.150
GLOBE_RADIUS = 0.098
GLOBE_CENTER_Z = 0.036
POLAR_TILT = math.radians(23.5)
POLAR_AXIS = (math.sin(POLAR_TILT), 0.0, math.cos(POLAR_TILT))
BEARING_RADIUS = 0.007
PIVOT_PIN_RADIUS = 0.0026
PIVOT_PIN_LENGTH = 0.014


def _aabb_center(aabb):
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def _meridian_arch_points(radius: float, *, segments: int = 18) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(segments + 1):
        angle = math.pi - (math.pi * index / segments)
        points.append((radius * math.cos(angle), 0.0, radius * math.sin(angle)))
    return points


def _add_segment_visual(part, name, start, end, *, radius, material) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    midpoint = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    yaw = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length + 0.0012),
        origin=Origin(xyz=midpoint, rpy=(0.0, yaw, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="illuminated_classroom_globe")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_charcoal = model.material("satin_charcoal", rgba=(0.23, 0.24, 0.26, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.71, 0.60, 0.35, 1.0))
    warm_ivory = model.material("warm_ivory", rgba=(0.90, 0.91, 0.78, 0.94))
    land_green = model.material("land_green", rgba=(0.43, 0.53, 0.29, 1.0))
    deep_ocean = model.material("deep_ocean", rgba=(0.27, 0.43, 0.59, 1.0))

    stand = model.part("pedestal_stand")
    stand.visual(
        Cylinder(radius=0.118, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=matte_black,
        name="base_disk",
    )
    stand.visual(
        Cylinder(radius=0.043, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=satin_charcoal,
        name="base_neck",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=satin_charcoal,
        name="pedestal_stem",
    )
    stand.visual(
        Box((0.302, 0.036, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=satin_charcoal,
        name="yoke_bridge",
    )
    stand.visual(
        Box((0.050, 0.036, 0.110)),
        origin=Origin(xyz=(-0.175, 0.0, 0.166)),
        material=satin_charcoal,
        name="left_arm",
    )
    stand.visual(
        Box((0.050, 0.036, 0.110)),
        origin=Origin(xyz=(0.175, 0.0, 0.166)),
        material=satin_charcoal,
        name="right_arm",
    )
    stand.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(-0.162, 0.0, 0.195), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="left_trunnion_bushing",
    )
    stand.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.162, 0.0, 0.195), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="right_trunnion_bushing",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.424, 0.236, 0.220)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    meridian = model.part("meridian_frame")
    top_pin_center = (
        POLAR_AXIS[0] * (GLOBE_RADIUS + PIVOT_PIN_LENGTH * 0.5),
        0.0,
        GLOBE_CENTER_Z + POLAR_AXIS[2] * (GLOBE_RADIUS + PIVOT_PIN_LENGTH * 0.5),
    )
    bottom_pin_center = (
        -POLAR_AXIS[0] * (GLOBE_RADIUS + PIVOT_PIN_LENGTH * 0.5),
        0.0,
        GLOBE_CENTER_Z - POLAR_AXIS[2] * (GLOBE_RADIUS + PIVOT_PIN_LENGTH * 0.5),
    )
    top_pin_outer = (
        POLAR_AXIS[0] * (GLOBE_RADIUS + PIVOT_PIN_LENGTH),
        0.0,
        GLOBE_CENTER_Z + POLAR_AXIS[2] * (GLOBE_RADIUS + PIVOT_PIN_LENGTH),
    )
    bottom_pin_outer = (
        -POLAR_AXIS[0] * (GLOBE_RADIUS + PIVOT_PIN_LENGTH),
        0.0,
        GLOBE_CENTER_Z - POLAR_AXIS[2] * (GLOBE_RADIUS + PIVOT_PIN_LENGTH),
    )
    arch_points = _meridian_arch_points(MERIDIAN_RADIUS, segments=14)
    for index, (start, end) in enumerate(zip(arch_points[:-1], arch_points[1:])):
        _add_segment_visual(
            meridian,
            f"meridian_arch_{index:02d}",
            start,
            end,
            radius=0.0045,
            material=aged_brass,
        )
    top_bracket_points = [
        (0.0, 0.0, MERIDIAN_RADIUS),
        (0.030, 0.0, 0.146),
        top_pin_outer,
    ]
    for index, (start, end) in enumerate(zip(top_bracket_points[:-1], top_bracket_points[1:])):
        _add_segment_visual(
            meridian,
            f"top_pivot_bracket_{index}",
            start,
            end,
            radius=0.0032,
            material=aged_brass,
        )
    bottom_bracket_points = [
        (-0.108, 0.0, math.sqrt(MERIDIAN_RADIUS**2 - 0.108**2)),
        (-0.128, 0.0, 0.034),
        (-0.100, 0.0, -0.022),
        (-0.078, 0.0, -0.060),
        bottom_pin_outer,
    ]
    for index, (start, end) in enumerate(zip(bottom_bracket_points[:-1], bottom_bracket_points[1:])):
        _add_segment_visual(
            meridian,
            f"bottom_pivot_bracket_{index}",
            start,
            end,
            radius=0.0032,
            material=aged_brass,
        )
    meridian.visual(
        Cylinder(radius=PIVOT_PIN_RADIUS, length=PIVOT_PIN_LENGTH),
        origin=Origin(
            xyz=top_pin_center,
            rpy=(0.0, POLAR_TILT, 0.0),
        ),
        material=aged_brass,
        name="top_pivot_pin",
    )
    meridian.visual(
        Cylinder(radius=PIVOT_PIN_RADIUS, length=PIVOT_PIN_LENGTH),
        origin=Origin(
            xyz=bottom_pin_center,
            rpy=(0.0, POLAR_TILT, 0.0),
        ),
        material=aged_brass,
        name="bottom_pivot_pin",
    )
    meridian.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(-0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="left_trunnion_collar",
    )
    meridian.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="right_trunnion_collar",
    )
    meridian.inertial = Inertial.from_geometry(
        Box((0.340, 0.040, 0.224)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=warm_ivory,
        name="globe_shell",
    )
    globe.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(0.062, 0.030, 0.022)),
        material=land_green,
        name="continent_cluster",
    )
    globe.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(-0.018, -0.061, 0.014)),
        material=land_green,
        name="landmass_rear",
    )
    globe.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.022, -0.010, -0.071)),
        material=deep_ocean,
        name="southern_ocean_relief",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=GLOBE_RADIUS),
        mass=0.52,
    )

    left_knob = model.part("left_tilt_knob")
    left_knob.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="knob_shaft",
    )
    left_knob.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(-0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="knob_body",
    )
    left_knob.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=Origin(xyz=(-0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="knob_cap",
    )
    left_knob.visual(
        Box((0.010, 0.005, 0.010)),
        origin=Origin(xyz=(-0.028, 0.0, 0.019)),
        material=aged_brass,
        name="indicator_rib",
    )
    left_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.027, length=0.040),
        mass=0.08,
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_knob = model.part("right_tilt_knob")
    right_knob.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="knob_shaft",
    )
    right_knob.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="knob_body",
    )
    right_knob.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=Origin(xyz=(0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="knob_cap",
    )
    right_knob.visual(
        Box((0.010, 0.005, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, 0.019)),
        material=aged_brass,
        name="indicator_rib",
    )
    right_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.027, length=0.040),
        mass=0.08,
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stand_to_meridian_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.65,
        ),
    )
    model.articulation(
        "meridian_to_globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER_Z)),
        axis=POLAR_AXIS,
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=5.0,
        ),
    )
    model.articulation(
        "stand_to_left_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=left_knob,
        origin=Origin(xyz=(-0.200, 0.0, 0.195)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=8.0,
        ),
    )
    model.articulation(
        "stand_to_right_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=right_knob,
        origin=Origin(xyz=(0.200, 0.0, 0.195)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("pedestal_stand")
    meridian = object_model.get_part("meridian_frame")
    globe = object_model.get_part("globe")
    left_knob = object_model.get_part("left_tilt_knob")
    right_knob = object_model.get_part("right_tilt_knob")

    meridian_tilt = object_model.get_articulation("stand_to_meridian_tilt")
    globe_spin = object_model.get_articulation("meridian_to_globe_spin")
    left_knob_spin = object_model.get_articulation("stand_to_left_knob_spin")
    right_knob_spin = object_model.get_articulation("stand_to_right_knob_spin")

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
        globe,
        meridian,
        elem_a="globe_shell",
        elem_b="top_pivot_pin",
        reason="The globe is held on a small polar pivot pin that seats into a shallow polar socket.",
    )
    ctx.allow_overlap(
        globe,
        meridian,
        elem_a="globe_shell",
        elem_b="bottom_pivot_pin",
        reason="The lower polar pivot pin seats into the globe's lower socket as on a classroom globe axle.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        meridian,
        stand,
        elem_a="left_trunnion_collar",
        elem_b="left_trunnion_bushing",
        name="left_trunnion_contact",
    )
    ctx.expect_contact(
        meridian,
        stand,
        elem_a="right_trunnion_collar",
        elem_b="right_trunnion_bushing",
        name="right_trunnion_contact",
    )
    ctx.expect_overlap(
        meridian,
        stand,
        elem_a="left_trunnion_collar",
        elem_b="left_trunnion_bushing",
        axes="yz",
        min_overlap=0.020,
        name="left_trunnion_alignment",
    )
    ctx.expect_overlap(
        meridian,
        stand,
        elem_a="right_trunnion_collar",
        elem_b="right_trunnion_bushing",
        axes="yz",
        min_overlap=0.020,
        name="right_trunnion_alignment",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="globe_shell",
        elem_b="top_pivot_pin",
        name="top_polar_support_contact",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="globe_shell",
        elem_b="bottom_pivot_pin",
        name="bottom_polar_support_contact",
    )
    ctx.expect_contact(
        left_knob,
        stand,
        elem_a="knob_shaft",
        elem_b="left_arm",
        name="left_knob_seated",
    )
    ctx.expect_contact(
        right_knob,
        stand,
        elem_a="knob_shaft",
        elem_b="right_arm",
        name="right_knob_seated",
    )

    ctx.check(
        "meridian_tilt_axis_is_horizontal",
        all(
            math.isclose(value, target, abs_tol=1e-9)
            for value, target in zip(meridian_tilt.axis, (1.0, 0.0, 0.0))
        ),
        f"Expected horizontal x-axis tilt, got {meridian_tilt.axis!r}.",
    )
    ctx.check(
        "globe_spin_axis_matches_polar_axis",
        all(
            math.isclose(value, target, abs_tol=1e-6)
            for value, target in zip(globe_spin.axis, POLAR_AXIS)
        ),
        f"Expected polar axis {POLAR_AXIS!r}, got {globe_spin.axis!r}.",
    )
    ctx.check(
        "knob_axes_match_trunnion_axis",
        all(
            math.isclose(value, target, abs_tol=1e-9)
            for value, target in zip(left_knob_spin.axis, (1.0, 0.0, 0.0))
        )
        and all(
            math.isclose(value, target, abs_tol=1e-9)
            for value, target in zip(right_knob_spin.axis, (1.0, 0.0, 0.0))
        ),
        (
            "Both side knobs should spin about the trunnion axis; "
            f"got left={left_knob_spin.axis!r}, right={right_knob_spin.axis!r}."
        ),
    )

    globe_rest = ctx.part_world_position(globe)
    continent_rest_aabb = ctx.part_element_world_aabb(globe, elem="continent_cluster")
    left_indicator_rest_aabb = ctx.part_element_world_aabb(left_knob, elem="indicator_rib")

    with ctx.pose({meridian_tilt: 0.42}):
        globe_tilted = ctx.part_world_position(globe)
        meridian_moved = (
            globe_rest is not None
            and globe_tilted is not None
            and abs(globe_tilted[1] - globe_rest[1]) > 0.010
        )
        ctx.check(
            "meridian_tilt_swings_globe",
            meridian_moved,
            f"Expected globe center to swing forward/back under tilt; rest={globe_rest}, tilted={globe_tilted}.",
        )
        ctx.expect_contact(
            meridian,
            stand,
            elem_a="left_trunnion_collar",
            elem_b="left_trunnion_bushing",
            name="left_trunnion_contact_tilted",
        )
        ctx.expect_contact(
            meridian,
            stand,
            elem_a="right_trunnion_collar",
            elem_b="right_trunnion_bushing",
            name="right_trunnion_contact_tilted",
        )

    with ctx.pose({globe_spin: 1.1}):
        continent_spin_aabb = ctx.part_element_world_aabb(globe, elem="continent_cluster")
        moved = (
            continent_rest_aabb is not None
            and continent_spin_aabb is not None
            and max(
                abs(a - b)
                for a, b in zip(_aabb_center(continent_rest_aabb), _aabb_center(continent_spin_aabb))
            )
            > 0.010
        )
        ctx.check(
            "globe_spin_moves_surface_detail",
            moved,
            (
                "Expected off-axis globe surface detail to move when the sphere spins; "
                f"rest={continent_rest_aabb}, spun={continent_spin_aabb}."
            ),
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a="globe_shell",
            elem_b="top_pivot_pin",
            name="top_polar_support_contact_spun",
        )

    with ctx.pose({left_knob_spin: 1.0, right_knob_spin: -0.8}):
        left_indicator_spin_aabb = ctx.part_element_world_aabb(left_knob, elem="indicator_rib")
        right_indicator_rest_aabb = ctx.part_element_world_aabb(right_knob, elem="indicator_rib")
        left_moved = (
            left_indicator_rest_aabb is not None
            and left_indicator_spin_aabb is not None
            and max(
                abs(a - b)
                for a, b in zip(_aabb_center(left_indicator_rest_aabb), _aabb_center(left_indicator_spin_aabb))
            )
            > 0.008
        )
        ctx.check(
            "left_knob_rotation_moves_indicator",
            left_moved,
            (
                "Expected left knob indicator rib to move under rotation; "
                f"rest={left_indicator_rest_aabb}, spun={left_indicator_spin_aabb}."
            ),
        )
        ctx.check(
            "right_knob_indicator_present",
            right_indicator_rest_aabb is not None,
            "Right knob indicator rib AABB was unavailable.",
        )
        ctx.expect_contact(
            left_knob,
            stand,
            elem_a="knob_shaft",
            elem_b="left_arm",
            name="left_knob_stays_seated_when_spun",
        )
        ctx.expect_contact(
            right_knob,
            stand,
            elem_a="knob_shaft",
            elem_b="right_arm",
            name="right_knob_stays_seated_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
