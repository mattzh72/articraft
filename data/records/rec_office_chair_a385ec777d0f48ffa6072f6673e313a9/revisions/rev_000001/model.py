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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _loop_xy(
    width: float,
    depth: float,
    radius: float,
    *,
    z: float,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + dx, y + dy, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _loop_xz(
    width: float,
    height: float,
    radius: float,
    *,
    y: float,
    dx: float = 0.0,
    dz: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + dx, y, z + dz) for x, z in rounded_rect_profile(width, height, radius)]


def _chair_seat_mesh():
    return LoftGeometry(
        [
            _loop_xy(0.500, 0.450, 0.060, z=0.020, dy=0.006),
            _loop_xy(0.490, 0.430, 0.066, z=0.048, dy=-0.004),
            _loop_xy(0.474, 0.410, 0.070, z=0.070, dy=-0.014),
        ],
        cap=True,
        closed=True,
    )


def _backrest_mesh():
    return (
        LoftGeometry(
            [
                _loop_xy(0.395, 0.220, 0.044, z=0.030, dy=0.235),
                _loop_xy(0.410, 0.235, 0.048, z=0.006, dy=0.246),
                _loop_xy(0.425, 0.248, 0.052, z=-0.028, dy=0.258),
            ],
            cap=True,
            closed=True,
        )
        .rotate_x(math.pi / 2.0)
    )


def _underseat_pan_mesh():
    return LoftGeometry(
        [
            _loop_xy(0.310, 0.240, 0.030, z=-0.040, dy=-0.010),
            _loop_xy(0.340, 0.250, 0.032, z=-0.012, dy=-0.006),
            _loop_xy(0.335, 0.235, 0.030, z=0.010, dy=-0.002),
        ],
        cap=True,
        closed=True,
    )


def _column_shell_mesh(
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _base_leg_mesh():
    profile = rounded_rect_profile(0.052, 0.024, 0.009)
    return sweep_profile_along_spline(
        [
            (0.018, 0.000, 0.034),
            (0.110, 0.000, 0.044),
            (0.206, 0.000, 0.048),
            (0.306, 0.000, 0.046),
        ],
        profile=profile,
        samples_per_segment=10,
        cap_profile=True,
    )


def _arm_support_mesh(side: float):
    profile = rounded_rect_profile(0.028, 0.018, 0.006)
    return sweep_profile_along_spline(
        [
            (0.185 * side, -0.105, 0.006),
            (0.218 * side, -0.080, 0.084),
            (0.224 * side, 0.012, 0.186),
            (0.185 * side, 0.152, 0.224),
        ],
        profile=profile,
        samples_per_segment=12,
        cap_profile=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conference_office_chair")

    upholstery = model.material("upholstery_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    back_shell = model.material("back_shell", rgba=(0.15, 0.16, 0.18, 1.0))
    nylon = model.material("nylon_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    seat_mesh = _save_mesh("chair_seat_cushion", _chair_seat_mesh())
    back_mesh = _save_mesh("chair_backrest_pad", _backrest_mesh())
    pan_mesh = _save_mesh("chair_underseat_pan", _underseat_pan_mesh())
    base_leg_mesh = _save_mesh("chair_base_leg", _base_leg_mesh())
    left_arm_support_mesh = _save_mesh("chair_left_arm_support", _arm_support_mesh(-1.0))
    right_arm_support_mesh = _save_mesh("chair_right_arm_support", _arm_support_mesh(1.0))
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.055, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=dark_metal,
        name="center_hub",
    )
    for guide_index in range(3):
        guide_angle = (2.0 * math.pi * guide_index) / 3.0 + (math.pi / 6.0)
        base.visual(
            Cylinder(radius=0.011, length=0.300),
            origin=Origin(
                xyz=(
                    math.cos(guide_angle) * 0.028,
                    math.sin(guide_angle) * 0.028,
                    0.156,
                )
            ),
            material=dark_metal,
            name=f"guide_post_{guide_index}",
        )
    for leg_index in range(5):
        angle = (2.0 * math.pi * leg_index) / 5.0
        tip_x = math.cos(angle) * 0.300
        tip_y = math.sin(angle) * 0.300
        base.visual(
            Box((0.310, 0.046, 0.020)),
            origin=Origin(
                xyz=(math.cos(angle) * 0.155, math.sin(angle) * 0.155, 0.024),
                rpy=(0.0, 0.0, angle),
            ),
            material=satin_metal,
            name=f"leg_{leg_index}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(tip_x, tip_y, 0.036)),
            material=dark_metal,
            name=f"caster_boss_{leg_index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.720, 0.720, 0.200)),
        mass=8.0,
        origin=Origin(xyz=(0.000, 0.000, 0.100)),
    )

    gas_column = model.part("gas_column")
    gas_column.visual(
        Cylinder(radius=0.017, length=0.150),
        origin=Origin(xyz=(0.000, 0.000, 0.085)),
        material=dark_metal,
        name="lower_guide",
    )
    gas_column.visual(
        Cylinder(radius=0.015, length=0.200),
        origin=Origin(xyz=(0.000, 0.000, 0.185)),
        material=satin_metal,
        name="column_shaft",
    )
    gas_column.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, 0.285)),
        material=satin_metal,
        name="upper_guide",
    )
    gas_column.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.300)),
        material=dark_metal,
        name="top_bearing",
    )
    gas_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.310),
        mass=3.5,
        origin=Origin(xyz=(0.000, 0.000, 0.155)),
    )

    seat_support = model.part("seat_support")
    seat_support.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=dark_metal,
        name="swivel_plate",
    )
    seat_support.visual(
        Box((0.016, 0.090, 0.044)),
        origin=Origin(xyz=(-0.040, 0.000, 0.022)),
        material=dark_metal,
        name="left_receiver_rail",
    )
    seat_support.visual(
        Box((0.016, 0.090, 0.044)),
        origin=Origin(xyz=(0.040, 0.000, 0.022)),
        material=dark_metal,
        name="right_receiver_rail",
    )
    seat_support.visual(
        Box((0.084, 0.018, 0.020)),
        origin=Origin(xyz=(0.000, 0.038, 0.010)),
        material=dark_metal,
        name="front_receiver_bridge",
    )
    seat_support.visual(
        Box((0.084, 0.018, 0.020)),
        origin=Origin(xyz=(0.000, -0.038, 0.010)),
        material=dark_metal,
        name="rear_receiver_bridge",
    )
    seat_support.visual(
        pan_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
        material=dark_metal,
        name="underseat_pan",
    )
    seat_support.visual(
        Box((0.440, 0.300, 0.018)),
        origin=Origin(xyz=(0.000, 0.010, 0.014)),
        material=nylon,
        name="seat_pan_plate",
    )
    seat_support.visual(
        seat_mesh,
        material=upholstery,
        name="seat_cushion",
    )
    seat_support.visual(
        left_arm_support_mesh,
        material=satin_metal,
        name="left_arm_support",
    )
    seat_support.visual(
        right_arm_support_mesh,
        material=satin_metal,
        name="right_arm_support",
    )
    seat_support.visual(
        Box((0.110, 0.255, 0.020)),
        origin=Origin(xyz=(-0.182, 0.120, 0.232)),
        material=nylon,
        name="left_arm_pad",
    )
    seat_support.visual(
        Box((0.110, 0.255, 0.020)),
        origin=Origin(xyz=(0.182, 0.120, 0.232)),
        material=nylon,
        name="right_arm_pad",
    )
    seat_support.visual(
        Box((0.080, 0.045, 0.130)),
        origin=Origin(xyz=(0.000, -0.214, 0.065)),
        material=dark_metal,
        name="rear_tilt_stem",
    )
    seat_support.visual(
        Box((0.150, 0.030, 0.034)),
        origin=Origin(xyz=(0.000, -0.240, 0.020)),
        material=dark_metal,
        name="hinge_crossbar",
    )
    seat_support.visual(
        Box((0.014, 0.030, 0.052)),
        origin=Origin(xyz=(-0.072, -0.240, 0.026)),
        material=dark_metal,
        name="left_hinge_ear",
    )
    seat_support.visual(
        Box((0.014, 0.030, 0.052)),
        origin=Origin(xyz=(0.072, -0.240, 0.026)),
        material=dark_metal,
        name="right_hinge_ear",
    )
    seat_support.visual(
        Box((0.060, 0.008, 0.008)),
        origin=Origin(xyz=(0.250, 0.125, 0.010)),
        material=satin_metal,
        name="height_lever",
    )
    seat_support.inertial = Inertial.from_geometry(
        Box((0.620, 0.560, 0.320)),
        mass=11.0,
        origin=Origin(xyz=(0.000, 0.020, 0.110)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.020, 0.028, 0.060)),
        origin=Origin(xyz=(-0.089, 0.000, 0.030)),
        material=dark_metal,
        name="left_mount_lug",
    )
    backrest.visual(
        Box((0.020, 0.028, 0.060)),
        origin=Origin(xyz=(0.089, 0.000, 0.030)),
        material=dark_metal,
        name="right_mount_lug",
    )
    backrest.visual(
        Box((0.180, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, -0.012, 0.058)),
        material=dark_metal,
        name="lug_bridge",
    )
    backrest.visual(
        Box((0.072, 0.030, 0.190)),
        origin=Origin(xyz=(0.000, -0.024, 0.140)),
        material=back_shell,
        name="back_spine",
    )
    backrest.visual(
        back_mesh,
        origin=Origin(xyz=(0.000, -0.032, 0.042)),
        material=upholstery,
        name="back_pad",
    )
    backrest.visual(
        Box((0.220, 0.016, 0.150)),
        origin=Origin(xyz=(0.000, -0.040, 0.210)),
        material=back_shell,
        name="rear_shell_strap",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.430, 0.070, 0.390)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.205)),
    )

    for caster_index in range(5):
        angle = (2.0 * math.pi * caster_index) / 5.0
        tip_x = math.cos(angle) * 0.321
        tip_y = math.sin(angle) * 0.321

        fork = model.part(f"caster_fork_{caster_index}")
        fork.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.000, 0.000, -0.004)),
            material=dark_metal,
            name="swivel_cap",
        )
        fork.visual(
            Cylinder(radius=0.007, length=0.020),
            origin=Origin(xyz=(0.000, 0.000, -0.014)),
            material=dark_metal,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.018, 0.040, 0.008)),
            origin=Origin(xyz=(0.014, 0.000, -0.012)),
            material=nylon,
            name="fork_bridge",
        )
        fork.visual(
            Box((0.006, 0.004, 0.024)),
            origin=Origin(xyz=(0.018, -0.018, -0.028)),
            material=nylon,
            name="left_jaw",
        )
        fork.visual(
            Box((0.006, 0.004, 0.024)),
            origin=Origin(xyz=(0.018, 0.018, -0.028)),
            material=nylon,
            name="right_jaw",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.050, 0.050, 0.050)),
            mass=0.20,
            origin=Origin(xyz=(0.012, 0.000, -0.020)),
        )

        wheel = model.part(f"caster_wheel_{caster_index}")
        wheel.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hub_pin",
        )
        wheel.visual(
            Cylinder(radius=0.008, length=0.002),
            origin=Origin(xyz=(0.000, -0.015, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="left_washer",
        )
        wheel.visual(
            Cylinder(radius=0.008, length=0.002),
            origin=Origin(xyz=(0.000, 0.015, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="right_washer",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.018, length=0.030),
            mass=0.25,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"base_to_caster_fork_{caster_index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(xyz=(tip_x, tip_y, 0.048), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )
        model.articulation(
            f"caster_fork_to_wheel_{caster_index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.024, 0.000, -0.036)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=35.0),
        )

    model.articulation(
        "base_to_gas_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gas_column,
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.20, lower=0.0, upper=0.10),
    )
    model.articulation(
        "gas_column_to_seat_support",
        ArticulationType.CONTINUOUS,
        parent=gas_column,
        child=seat_support,
        origin=Origin(xyz=(0.000, 0.000, 0.305)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0),
    )
    model.articulation(
        "seat_support_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_support,
        child=backrest,
        origin=Origin(xyz=(0.000, -0.240, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.10, upper=0.42),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    gas_column = object_model.get_part("gas_column")
    seat_support = object_model.get_part("seat_support")
    backrest = object_model.get_part("backrest")
    fork_0 = object_model.get_part("caster_fork_0")
    wheel_0 = object_model.get_part("caster_wheel_0")

    gas_lift = object_model.get_articulation("base_to_gas_column")
    seat_swivel = object_model.get_articulation("gas_column_to_seat_support")
    back_tilt = object_model.get_articulation("seat_support_to_backrest")
    caster_swivel = object_model.get_articulation("base_to_caster_fork_0")
    caster_wheel = object_model.get_articulation("caster_fork_to_wheel_0")

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
        "chair_parts_present",
        all(part is not None for part in (base, gas_column, seat_support, backrest, fork_0, wheel_0)),
        "Expected base, gas column, seat support, backrest, and one caster assembly.",
    )
    ctx.check(
        "gas_lift_axis_is_vertical",
        tuple(gas_lift.axis) == (0.0, 0.0, 1.0),
        f"Expected gas lift axis (0, 0, 1), got {gas_lift.axis}.",
    )
    ctx.check(
        "seat_swivel_axis_is_vertical",
        tuple(seat_swivel.axis) == (0.0, 0.0, 1.0),
        f"Expected seat swivel axis (0, 0, 1), got {seat_swivel.axis}.",
    )
    ctx.check(
        "back_tilt_axis_is_horizontal",
        tuple(back_tilt.axis) == (1.0, 0.0, 0.0),
        f"Expected back tilt axis (1, 0, 0), got {back_tilt.axis}.",
    )
    ctx.check(
        "caster_swivel_axis_is_vertical",
        tuple(caster_swivel.axis) == (0.0, 0.0, 1.0),
        f"Expected caster swivel axis (0, 0, 1), got {caster_swivel.axis}.",
    )
    ctx.check(
        "caster_wheel_axis_is_horizontal",
        tuple(caster_wheel.axis) == (0.0, 1.0, 0.0),
        f"Expected caster wheel axis (0, 1, 0), got {caster_wheel.axis}.",
    )

    ctx.expect_contact(backrest, seat_support, contact_tol=0.0008, name="backrest_contacts_tilt_bracket")
    ctx.expect_contact(fork_0, base, contact_tol=0.0008, name="caster_fork_contacts_base")
    ctx.expect_contact(wheel_0, fork_0, contact_tol=0.0008, name="caster_wheel_contacts_fork")

    with ctx.pose({gas_lift: 0.0}):
        ctx.expect_origin_distance(
            gas_column,
            base,
            axes="xy",
            max_dist=0.001,
            name="column_centered_over_base_at_low_height",
        )
        ctx.expect_contact(
            gas_column,
            base,
            elem_a="lower_guide",
            elem_b="guide_post_0",
            contact_tol=0.0008,
            name="column_contacts_base_guide_at_low_height",
        )
        ctx.expect_origin_distance(
            gas_column,
            seat_support,
            axes="xy",
            max_dist=0.001,
            name="column_centered_under_seat_support_at_low_height",
        )
        ctx.expect_contact(
            gas_column,
            seat_support,
            elem_a="top_bearing",
            elem_b="swivel_plate",
            contact_tol=0.0008,
            name="column_contacts_seat_receiver_at_low_height",
        )

    with ctx.pose({gas_lift: 0.10}):
        ctx.expect_origin_distance(
            gas_column,
            base,
            axes="xy",
            max_dist=0.001,
            name="column_centered_over_base_at_high_height",
        )
        ctx.expect_contact(
            gas_column,
            base,
            elem_a="lower_guide",
            elem_b="guide_post_0",
            contact_tol=0.0008,
            name="column_remains_clipped_to_base_at_high_height",
        )
        ctx.expect_origin_distance(
            gas_column,
            seat_support,
            axes="xy",
            max_dist=0.001,
            name="column_centered_under_seat_support_at_high_height",
        )
        ctx.expect_contact(
            gas_column,
            seat_support,
            elem_a="top_bearing",
            elem_b="swivel_plate",
            contact_tol=0.0008,
            name="column_remains_clipped_to_seat_receiver_at_high_height",
        )

    with ctx.pose({back_tilt: 0.35}):
        ctx.expect_overlap(
            backrest,
            seat_support,
            axes="x",
            min_overlap=0.180,
            name="backrest_remains_laterally_aligned_when_tilted",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
