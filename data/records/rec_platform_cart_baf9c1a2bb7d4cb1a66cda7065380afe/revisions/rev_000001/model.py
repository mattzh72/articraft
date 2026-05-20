from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _section_from_profile(
    profile: list[tuple[float, float]],
    *,
    y: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z_center + z_local) for x, z_local in profile]


def _wheel_tire_mesh(mesh_name: str, *, tire_radius: float, tire_width: float):
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.34, -half_width * 0.95),
        (tire_radius * 0.74, -half_width * 0.98),
        (tire_radius * 0.93, -half_width * 0.72),
        (tire_radius, -half_width * 0.22),
        (tire_radius, half_width * 0.22),
        (tire_radius * 0.93, half_width * 0.72),
        (tire_radius * 0.74, half_width * 0.98),
        (tire_radius * 0.34, half_width * 0.95),
        (tire_radius * 0.26, half_width * 0.44),
        (tire_radius * 0.23, 0.0),
        (tire_radius * 0.26, -half_width * 0.44),
        (tire_radius * 0.34, -half_width * 0.95),
    ]
    return _save_mesh(mesh_name, LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0))


def _add_rear_wheel_visuals(
    part,
    mesh_name: str,
    *,
    tire_radius: float,
    tire_width: float,
    sleeve_radius: float,
    sleeve_length: float,
    disc_radius: float,
    disc_thickness: float,
    side_sign: float,
    rubber,
    hub_metal,
) -> None:
    part.visual(
        _wheel_tire_mesh(mesh_name, tire_radius=tire_radius, tire_width=tire_width),
        material=rubber,
        name="tire",
    )
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=sleeve_radius, length=sleeve_length),
        origin=spin_origin,
        material=hub_metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=disc_radius, length=disc_thickness),
        origin=Origin(
            xyz=(side_sign * (tire_width * 0.5 - disc_thickness * 0.5), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hub_metal,
        name="outer_cap",
    )
    part.visual(
        Cylinder(radius=disc_radius * 0.58, length=disc_thickness),
        origin=Origin(
            xyz=(-side_sign * (sleeve_length * 0.5 - disc_thickness * 0.5), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hub_metal,
        name="inner_cap",
    )


def _add_caster_wheel_visuals(
    part,
    mesh_name: str,
    *,
    tire_radius: float,
    tire_width: float,
    hub_radius: float,
    hub_length: float,
    rubber,
    hub_metal,
) -> None:
    part.visual(
        _wheel_tire_mesh(mesh_name, tire_radius=tire_radius, tire_width=tire_width),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_metal,
        name="hub",
    )


def _add_caster_fork_visuals(part, *, metal) -> None:
    part.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=metal,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=metal,
        name="swivel_race",
    )
    part.visual(
        Box((0.040, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=metal,
        name="fork_crown",
    )
    part.visual(
        Box((0.008, 0.016, 0.072)),
        origin=Origin(xyz=(-0.016, 0.0, -0.044)),
        material=metal,
        name="left_leg",
    )
    part.visual(
        Box((0.008, 0.016, 0.072)),
        origin=Origin(xyz=(0.016, 0.0, -0.044)),
        material=metal,
        name="right_leg",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stockroom_platform_cart")

    steel = model.material("steel", rgba=(0.24, 0.28, 0.33, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.15, 0.17, 0.20, 1.0))
    wood = model.material("deck_wood", rgba=(0.56, 0.42, 0.26, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.68, 0.70, 0.73, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.92, 0.58, 0.82)),
        mass=21.0,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )

    deck_profile = rounded_rect_profile(0.58, 0.036, 0.016, corner_segments=8)
    deck_mesh = section_loft(
        [
            _section_from_profile(deck_profile, y=-0.43, z_center=0.132),
            _section_from_profile(deck_profile, y=0.00, z_center=0.162),
            _section_from_profile(deck_profile, y=0.43, z_center=0.192),
        ]
    )
    chassis.visual(
        _save_mesh("cart_deck", deck_mesh),
        material=wood,
        name="deck_shell",
    )

    chassis.visual(
        _save_mesh(
            "cart_left_frame_rail",
            tube_from_spline_points(
                [
                    (-0.252, -0.40, 0.150),
                    (-0.254, -0.12, 0.166),
                    (-0.257, 0.16, 0.184),
                    (-0.258, 0.40, 0.210),
                ],
                radius=0.013,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=steel,
        name="left_frame_rail",
    )
    chassis.visual(
        _save_mesh(
            "cart_right_frame_rail",
            tube_from_spline_points(
                [
                    (0.252, -0.40, 0.150),
                    (0.254, -0.12, 0.166),
                    (0.257, 0.16, 0.184),
                    (0.258, 0.40, 0.210),
                ],
                radius=0.013,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=steel,
        name="right_frame_rail",
    )
    chassis.visual(
        Cylinder(radius=0.013, length=0.50),
        origin=Origin(xyz=(0.0, -0.40, 0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="front_cross_rail",
    )
    chassis.visual(
        Cylinder(radius=0.013, length=0.44),
        origin=Origin(xyz=(0.0, 0.40, 0.210), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rear_cross_rail",
    )
    chassis.visual(
        Box((0.48, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.40, 0.151)),
        material=steel_dark,
        name="front_deck_strip",
    )
    chassis.visual(
        Box((0.48, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.26, 0.203)),
        material=steel_dark,
        name="rear_deck_strip",
    )

    chassis.visual(
        Cylinder(radius=0.013, length=0.52),
        origin=Origin(xyz=(0.0, 0.31, 0.205), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="rear_axle_beam",
    )
    chassis.visual(
        Box((0.028, 0.082, 0.090)),
        origin=Origin(xyz=(-0.185, 0.31, 0.232)),
        material=steel,
        name="left_axle_hanger",
    )
    chassis.visual(
        Box((0.028, 0.082, 0.090)),
        origin=Origin(xyz=(0.185, 0.31, 0.232)),
        material=steel,
        name="right_axle_hanger",
    )
    chassis.visual(
        Cylinder(radius=0.016, length=0.068),
        origin=Origin(xyz=(-0.294, 0.31, 0.205), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="left_axle_stub",
    )
    chassis.visual(
        Cylinder(radius=0.016, length=0.068),
        origin=Origin(xyz=(0.294, 0.31, 0.205), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="right_axle_stub",
    )

    chassis.visual(
        Box((0.076, 0.060, 0.010)),
        origin=Origin(xyz=(-0.325, -0.31, 0.147)),
        material=steel,
        name="left_caster_plate",
    )
    chassis.visual(
        Box((0.076, 0.060, 0.010)),
        origin=Origin(xyz=(0.325, -0.31, 0.147)),
        material=steel,
        name="right_caster_plate",
    )
    chassis.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(-0.325, -0.31, 0.142)),
        material=steel_dark,
        name="left_caster_boss",
    )
    chassis.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.325, -0.31, 0.142)),
        material=steel_dark,
        name="right_caster_boss",
    )

    chassis.visual(
        _save_mesh(
            "cart_left_push_upright",
            tube_from_spline_points(
                [
                    (-0.176, 0.372, 0.196),
                    (-0.188, 0.402, 0.470),
                    (-0.194, 0.430, 0.742),
                ],
                radius=0.017,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=steel,
        name="left_push_upright",
    )
    chassis.visual(
        _save_mesh(
            "cart_right_push_upright",
            tube_from_spline_points(
                [
                    (0.176, 0.372, 0.196),
                    (0.188, 0.402, 0.470),
                    (0.194, 0.430, 0.742),
                ],
                radius=0.017,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=steel,
        name="right_push_upright",
    )
    chassis.visual(
        Cylinder(radius=0.017, length=0.388),
        origin=Origin(xyz=(0.0, 0.430, 0.742), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="push_handle",
    )
    chassis.visual(
        _save_mesh(
            "cart_left_handle_brace",
            tube_from_spline_points(
                [
                    (-0.190, 0.315, 0.214),
                    (-0.204, 0.375, 0.295),
                    (-0.214, 0.402, 0.400),
                ],
                radius=0.010,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=steel,
        name="left_handle_brace",
    )
    chassis.visual(
        _save_mesh(
            "cart_right_handle_brace",
            tube_from_spline_points(
                [
                    (0.190, 0.315, 0.214),
                    (0.204, 0.375, 0.295),
                    (0.214, 0.402, 0.400),
                ],
                radius=0.010,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=steel,
        name="right_handle_brace",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.195, length=0.082),
        mass=3.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        rear_left_wheel,
        "rear_left_wheel_tire",
        tire_radius=0.195,
        tire_width=0.082,
        sleeve_radius=0.030,
        sleeve_length=0.064,
        disc_radius=0.074,
        disc_thickness=0.010,
        side_sign=-1.0,
        rubber=rubber,
        hub_metal=hub_metal,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.195, length=0.082),
        mass=3.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        rear_right_wheel,
        "rear_right_wheel_tire",
        tire_radius=0.195,
        tire_width=0.082,
        sleeve_radius=0.030,
        sleeve_length=0.064,
        disc_radius=0.074,
        disc_thickness=0.010,
        side_sign=1.0,
        rubber=rubber,
        hub_metal=hub_metal,
    )

    front_left_caster_fork = model.part("front_left_caster_fork")
    front_left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.098)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
    )
    _add_caster_fork_visuals(front_left_caster_fork, metal=steel)

    front_right_caster_fork = model.part("front_right_caster_fork")
    front_right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.098)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
    )
    _add_caster_fork_visuals(front_right_caster_fork, metal=steel)

    front_left_caster_wheel = model.part("front_left_caster_wheel")
    front_left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.022),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        front_left_caster_wheel,
        "front_left_caster_tire_narrow",
        tire_radius=0.055,
        tire_width=0.022,
        hub_radius=0.014,
        hub_length=0.028,
        rubber=rubber,
        hub_metal=hub_metal,
    )

    front_right_caster_wheel = model.part("front_right_caster_wheel")
    front_right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.022),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        front_right_caster_wheel,
        "front_right_caster_tire_narrow",
        tire_radius=0.055,
        tire_width=0.022,
        hub_radius=0.014,
        hub_length=0.028,
        rubber=rubber,
        hub_metal=hub_metal,
    )

    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.343, 0.31, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_right_wheel,
        origin=Origin(xyz=(0.343, 0.31, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "front_left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_left_caster_fork,
        origin=Origin(xyz=(-0.325, -0.31, 0.128)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "front_right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_right_caster_fork,
        origin=Origin(xyz=(0.325, -0.31, 0.128)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "front_left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_left_caster_fork,
        child=front_left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "front_right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_right_caster_fork,
        child=front_right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_caster_fork = object_model.get_part("front_left_caster_fork")
    front_right_caster_fork = object_model.get_part("front_right_caster_fork")
    front_left_caster_wheel = object_model.get_part("front_left_caster_wheel")
    front_right_caster_wheel = object_model.get_part("front_right_caster_wheel")

    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")
    front_left_caster_swivel = object_model.get_articulation("front_left_caster_swivel")
    front_right_caster_swivel = object_model.get_articulation("front_right_caster_swivel")
    front_left_caster_wheel_spin = object_model.get_articulation("front_left_caster_wheel_spin")
    front_right_caster_wheel_spin = object_model.get_articulation("front_right_caster_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        chassis,
        rear_left_wheel,
        elem_a="left_axle_stub",
        elem_b="hub",
        reason="rear left hub is visually simplified as a sleeve around the fixed axle stub",
    )
    ctx.allow_overlap(
        chassis,
        rear_left_wheel,
        elem_a="left_axle_stub",
        elem_b="inner_cap",
        reason="rear left inner bearing cap nests over the fixed axle stub",
    )
    ctx.allow_overlap(
        chassis,
        rear_right_wheel,
        elem_a="right_axle_stub",
        elem_b="hub",
        reason="rear right hub is visually simplified as a sleeve around the fixed axle stub",
    )
    ctx.allow_overlap(
        chassis,
        rear_right_wheel,
        elem_a="right_axle_stub",
        elem_b="inner_cap",
        reason="rear right inner bearing cap nests over the fixed axle stub",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(rear_left_wheel, chassis, name="rear left wheel mounted on fixed axle")
    ctx.expect_contact(rear_right_wheel, chassis, name="rear right wheel mounted on fixed axle")
    ctx.expect_contact(front_left_caster_fork, chassis, name="front left caster fork mounted to chassis")
    ctx.expect_contact(front_right_caster_fork, chassis, name="front right caster fork mounted to chassis")
    ctx.expect_contact(front_left_caster_wheel, front_left_caster_fork, name="front left caster wheel mounted in fork")
    ctx.expect_contact(front_right_caster_wheel, front_right_caster_fork, name="front right caster wheel mounted in fork")

    ctx.check(
        "rear wheels spin on transverse axle",
        rear_left_wheel_spin.axis == (1.0, 0.0, 0.0) and rear_right_wheel_spin.axis == (1.0, 0.0, 0.0),
        details=f"rear axes were {rear_left_wheel_spin.axis} and {rear_right_wheel_spin.axis}",
    )
    ctx.check(
        "caster forks swivel on vertical pivots",
        front_left_caster_swivel.axis == (0.0, 0.0, 1.0) and front_right_caster_swivel.axis == (0.0, 0.0, 1.0),
        details=f"caster swivel axes were {front_left_caster_swivel.axis} and {front_right_caster_swivel.axis}",
    )
    ctx.check(
        "caster wheels spin on fork axles",
        front_left_caster_wheel_spin.axis == (1.0, 0.0, 0.0)
        and front_right_caster_wheel_spin.axis == (1.0, 0.0, 0.0),
        details=(
            f"caster wheel axes were {front_left_caster_wheel_spin.axis} "
            f"and {front_right_caster_wheel_spin.axis}"
        ),
    )
    ctx.check(
        "wheel and caster joints are continuous",
        rear_left_wheel_spin.motion_limits.lower is None
        and rear_left_wheel_spin.motion_limits.upper is None
        and rear_right_wheel_spin.motion_limits.lower is None
        and rear_right_wheel_spin.motion_limits.upper is None
        and front_left_caster_swivel.motion_limits.lower is None
        and front_left_caster_swivel.motion_limits.upper is None
        and front_right_caster_swivel.motion_limits.lower is None
        and front_right_caster_swivel.motion_limits.upper is None,
        details="expected continuous wheel and caster swivel joints without lower/upper bounds",
    )

    front_strip_aabb = ctx.part_element_world_aabb(chassis, elem="front_deck_strip")
    rear_strip_aabb = ctx.part_element_world_aabb(chassis, elem="rear_deck_strip")
    handle_aabb = ctx.part_element_world_aabb(chassis, elem="push_handle")
    if front_strip_aabb is None or rear_strip_aabb is None or handle_aabb is None:
        ctx.fail("reference visuals measurable", "front/rear deck strips or push handle AABB was unavailable")
    else:
        front_top = front_strip_aabb[1][2]
        rear_top = rear_strip_aabb[1][2]
        handle_top = handle_aabb[1][2]
        ctx.check(
            "deck rises toward rear axle",
            rear_top > front_top + 0.045,
            details=f"front top={front_top:.4f} rear top={rear_top:.4f}",
        )
        ctx.check(
            "push handle rises well above deck",
            handle_top > rear_top + 0.52,
            details=f"rear deck top={rear_top:.4f} handle top={handle_top:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
