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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


GLOBE_RADIUS = 0.10
GLOBE_TILT = math.radians(23.5)
BASE_OUTER_RADIUS = 0.155
BASE_INNER_RADIUS = 0.097
BASE_HEIGHT = 0.014
HUB_RADIUS = 0.052
PLATTER_RADIUS = 0.043
PLATTER_HEIGHT = 0.010
GLOBE_CENTER = (0.0, 0.0, 0.205)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _axis_vector() -> tuple[float, float, float]:
    return (math.sin(GLOBE_TILT), 0.0, math.cos(GLOBE_TILT))


def _offset_point(point: tuple[float, float, float], distance: float) -> tuple[float, float, float]:
    axis_x, axis_y, axis_z = _axis_vector()
    return (
        point[0] + axis_x * distance,
        point[1] + axis_y * distance,
        point[2] + axis_z * distance,
    )


def _north_pole_point() -> tuple[float, float, float]:
    return _offset_point(GLOBE_CENTER, GLOBE_RADIUS)


def _origin_for_segment_xz(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.sqrt((dx * dx) + (dz * dz))
    pitch = math.atan2(dx, dz)
    return (
        Origin(
            xyz=(
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]),
            ),
            rpy=(0.0, pitch, 0.0),
        ),
        length,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_canted_globe")

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.60, 0.32, 1.0))
    globe_blue = model.material("globe_blue", rgba=(0.18, 0.39, 0.63, 1.0))

    north_pole = _north_pole_point()
    outer_tip = _offset_point(north_pole, 0.030)
    pin_center = _offset_point(north_pole, 0.0245)
    disc_center = _offset_point(north_pole, 0.0175)
    mast_top = (0.136, 0.0, 0.244)
    arm_elbow = (0.094, 0.0, 0.286)
    upper_arm_origin, upper_arm_length = _origin_for_segment_xz(mast_top, arm_elbow)
    head_arm_origin, head_arm_length = _origin_for_segment_xz(arm_elbow, outer_tip)

    base_ring = model.part("base_ring")
    base_ring.visual(
        _mesh(
            "base_ring_plate",
            ExtrudeWithHolesGeometry(
                _circle_profile(BASE_OUTER_RADIUS),
                [_circle_profile(BASE_INNER_RADIUS)],
                height=BASE_HEIGHT,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
        material=graphite,
        name="outer_ring",
    )
    base_ring.visual(
        Cylinder(radius=HUB_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
        material=charcoal,
        name="center_hub",
    )
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        base_ring.visual(
            Box((0.110, 0.014, BASE_HEIGHT)),
            origin=Origin(
                xyz=(0.055 * math.cos(angle), 0.055 * math.sin(angle), BASE_HEIGHT * 0.5),
                rpy=(0.0, 0.0, angle),
            ),
            material=charcoal,
            name=f"bridge_{index}",
        )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_OUTER_RADIUS, length=BASE_HEIGHT),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
    )

    support_pedestal = model.part("support_pedestal")
    support_pedestal.visual(
        Cylinder(radius=PLATTER_RADIUS, length=PLATTER_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + (PLATTER_HEIGHT * 0.5))),
        material=charcoal,
        name="turntable_platter",
    )
    support_pedestal.visual(
        Box((0.136, 0.016, 0.016)),
        origin=Origin(xyz=(0.068, 0.0, BASE_HEIGHT + 0.018)),
        material=satin_metal,
        name="pedestal_bridge",
    )
    support_pedestal.visual(
        Cylinder(radius=0.009, length=0.220),
        origin=Origin(xyz=(0.136, 0.0, 0.134)),
        material=satin_metal,
        name="vertical_mast",
    )
    support_pedestal.visual(
        Cylinder(radius=0.005, length=upper_arm_length),
        origin=upper_arm_origin,
        material=satin_metal,
        name="upper_arm",
    )
    support_pedestal.visual(
        Cylinder(radius=0.005, length=head_arm_length),
        origin=head_arm_origin,
        material=satin_metal,
        name="head_arm",
    )
    support_pedestal.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=outer_tip),
        material=satin_metal,
        name="spindle_knuckle",
    )
    support_pedestal.visual(
        Cylinder(radius=0.006, length=0.011),
        origin=Origin(xyz=pin_center, rpy=(0.0, GLOBE_TILT, 0.0)),
        material=satin_metal,
        name="spindle_pin",
    )
    support_pedestal.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=disc_center, rpy=(0.0, GLOBE_TILT, 0.0)),
        material=brass,
        name="retainer_disc",
    )
    support_pedestal.inertial = Inertial.from_geometry(
        Box((0.19, 0.07, 0.33)),
        mass=1.9,
        origin=Origin(xyz=(0.07, 0.0, 0.165)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS)),
        material=globe_blue,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brass,
        name="polar_hub",
    )
    globe.visual(
        _mesh(
            "globe_equator_band",
            TorusGeometry(radius=0.099, tube=0.0018, radial_segments=14, tubular_segments=96),
        ),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS)),
        material=brass,
        name="equator_band",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=GLOBE_RADIUS),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS)),
    )

    model.articulation(
        "pedestal_turn",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=support_pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=support_pedestal,
        child=globe,
        origin=Origin(xyz=north_pole, rpy=(0.0, GLOBE_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_ring = object_model.get_part("base_ring")
    support_pedestal = object_model.get_part("support_pedestal")
    globe = object_model.get_part("globe")
    pedestal_turn = object_model.get_articulation("pedestal_turn")
    globe_spin = object_model.get_articulation("globe_spin")

    def _center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

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

    ctx.expect_gap(
        support_pedestal,
        base_ring,
        axis="z",
        positive_elem="turntable_platter",
        negative_elem="center_hub",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="turntable_seats_on_center_hub",
    )
    ctx.expect_within(
        support_pedestal,
        base_ring,
        axes="xy",
        inner_elem="turntable_platter",
        outer_elem="center_hub",
        margin=0.0005,
        name="turntable_stays_centered_on_hub",
    )
    ctx.expect_contact(
        globe,
        support_pedestal,
        elem_a="polar_hub",
        elem_b="retainer_disc",
        name="globe_captured_at_spindle_clip",
    )
    ctx.expect_gap(
        globe,
        base_ring,
        axis="z",
        positive_elem="globe_shell",
        negative_elem="outer_ring",
        min_gap=0.085,
        name="globe_clears_ring_base",
    )

    ctx.check(
        "pedestal_turn_is_vertical_continuous",
        pedestal_turn.articulation_type == ArticulationType.CONTINUOUS
        and pedestal_turn.axis == (0.0, 0.0, 1.0)
        and pedestal_turn.motion_limits is not None
        and pedestal_turn.motion_limits.lower is None
        and pedestal_turn.motion_limits.upper is None,
        details="Pedestal should rotate freely about the vertical base axis.",
    )
    ctx.check(
        "globe_spin_uses_tilted_polar_axis",
        globe_spin.articulation_type == ArticulationType.CONTINUOUS
        and globe_spin.axis == (0.0, 0.0, 1.0)
        and globe_spin.motion_limits is not None
        and globe_spin.motion_limits.lower is None
        and globe_spin.motion_limits.upper is None
        and abs(globe_spin.origin.rpy[1] - GLOBE_TILT) < 1e-6,
        details="Globe should spin continuously around its canted polar axis.",
    )

    shell_aabb = ctx.part_element_world_aabb(globe, elem="globe_shell")
    hub_aabb = ctx.part_element_world_aabb(globe, elem="polar_hub")
    if shell_aabb is not None and hub_aabb is not None:
        shell_center = _center(shell_aabb)
        hub_center = _center(hub_aabb)
        ctx.check(
            "polar_hub_sits_above_and_to_support_side",
            hub_center[0] > shell_center[0] + 0.025 and hub_center[2] > shell_center[2] + 0.09,
            details="The spindle hub should sit on an elevated, support-side tilted pole.",
        )

    with ctx.pose({pedestal_turn: 1.1, globe_spin: 2.0}):
        ctx.expect_contact(
            globe,
            support_pedestal,
            elem_a="polar_hub",
            elem_b="retainer_disc",
            name="globe_remains_captured_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
