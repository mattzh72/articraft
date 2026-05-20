from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    section_loft,
)


DECK_LENGTH = 0.81
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.011
DECK_UNDERSIDE_Z = -DECK_THICKNESS * 0.5
TRUCK_SPACING_X = 0.185
WHEEL_RADIUS = 0.027
WHEEL_WIDTH = 0.032
WHEEL_CENTER_Y = 0.076
AXLE_CENTER_Z = -0.028
TRUCK_PIVOT_Z = -0.018
KINGPIN_TILT = 0.58
MOUNT_OFFSETS = ((-0.0205, -0.0175), (-0.0205, 0.0175), (0.0205, -0.0175), (0.0205, 0.0175))


def _kingpin_axis(fore_aft_sign: float) -> tuple[float, float, float]:
    return (sin(KINGPIN_TILT) * fore_aft_sign, 0.0, cos(KINGPIN_TILT))


def _along_kingpin(
    base_xyz: tuple[float, float, float],
    offset: float,
    fore_aft_sign: float,
) -> tuple[float, float, float]:
    axis = _kingpin_axis(fore_aft_sign)
    return (
        base_xyz[0] + axis[0] * offset,
        base_xyz[1],
        base_xyz[2] + axis[2] * offset,
    )


def _deck_section(
    x_pos: float,
    width: float,
    *,
    z_shift: float,
    edge_lift: float,
    center_concave: float,
) -> tuple[tuple[float, float, float], ...]:
    half = width * 0.5
    t = DECK_THICKNESS
    loop_2d = (
        (-half, 0.50 * t + edge_lift),
        (-0.76 * half, 0.58 * t + edge_lift),
        (-0.34 * half, 0.42 * t + edge_lift * 0.40),
        (0.0, 0.28 * t - center_concave),
        (0.34 * half, 0.42 * t + edge_lift * 0.40),
        (0.76 * half, 0.58 * t + edge_lift),
        (half, 0.50 * t + edge_lift),
        (0.88 * half, -0.18 * t),
        (0.34 * half, -0.42 * t),
        (0.0, -0.50 * t),
        (-0.34 * half, -0.42 * t),
        (-0.88 * half, -0.18 * t),
    )
    return tuple((x_pos, y_pos, z_pos + z_shift) for y_pos, z_pos in loop_2d)


def _build_deck_mesh():
    section_specs = (
        (-0.405, 0.090, 0.070, 0.0008, 0.0000),
        (-0.350, 0.145, 0.040, 0.0018, 0.0003),
        (-0.285, 0.182, 0.014, 0.0030, 0.0010),
        (-TRUCK_SPACING_X, 0.202, 0.002, 0.0045, 0.0018),
        (0.0, DECK_WIDTH, 0.000, 0.0055, 0.0025),
        (TRUCK_SPACING_X, 0.202, 0.002, 0.0045, 0.0018),
        (0.285, 0.182, 0.014, 0.0030, 0.0010),
        (0.350, 0.145, 0.040, 0.0018, 0.0003),
        (0.405, 0.090, 0.070, 0.0008, 0.0000),
    )
    sections = [
        _deck_section(
            x_pos,
            width,
            z_shift=z_shift,
            edge_lift=edge_lift,
            center_concave=center_concave,
        )
        for x_pos, width, z_shift, edge_lift, center_concave in section_specs
    ]
    return section_loft(sections)


def _build_wheel_mesh(name: str):
    half = WHEEL_WIDTH * 0.5
    outer_profile = [
        (WHEEL_RADIUS * 0.90, -half),
        (WHEEL_RADIUS * 0.97, -half * 0.82),
        (WHEEL_RADIUS, -half * 0.30),
        (WHEEL_RADIUS, half * 0.30),
        (WHEEL_RADIUS * 0.97, half * 0.82),
        (WHEEL_RADIUS * 0.90, half),
    ]
    inner_profile = [
        (0.0068, -half),
        (0.0098, -half * 0.72),
        (0.0115, -half * 0.22),
        (0.0115, half * 0.22),
        (0.0098, half * 0.72),
        (0.0068, half),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(pi / 2.0),
        name,
    )


def _build_truck_base(
    model: ArticulatedObject,
    name: str,
    *,
    fore_aft_sign: float,
    cast_aluminum,
    bushing_rubber,
    steel_dark,
) -> None:
    base = model.part(name)
    base.visual(
        Box((0.110, 0.058, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=cast_aluminum,
        name="baseplate",
    )
    base.visual(
        Box((0.064, 0.042, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, -0.0115)),
        material=cast_aluminum,
        name="saddle",
    )
    base.visual(
        Box((0.016, 0.008, 0.014)),
        origin=Origin(xyz=(0.036 * fore_aft_sign, -0.010, -0.024)),
        material=cast_aluminum,
        name="pivot_cup_left",
    )
    base.visual(
        Box((0.016, 0.008, 0.014)),
        origin=Origin(xyz=(0.036 * fore_aft_sign, 0.010, -0.024)),
        material=cast_aluminum,
        name="pivot_cup_right",
    )
    for x_off, y_off in MOUNT_OFFSETS:
        base.visual(
            Cylinder(radius=0.0042, length=0.006),
            origin=Origin(xyz=(x_off, y_off, -0.003)),
            material=steel_dark,
        )

    joint_origin = (0.0, 0.0, TRUCK_PIVOT_Z)
    kingpin_pitch = KINGPIN_TILT * fore_aft_sign
    base.visual(
        Cylinder(radius=0.0050, length=0.050),
        origin=Origin(
            xyz=_along_kingpin(joint_origin, -0.006, fore_aft_sign),
            rpy=(0.0, kingpin_pitch, 0.0),
        ),
        material=steel_dark,
        name="kingpin",
    )
    for offset, radius, length, material, visual_name in (
        (-0.015, 0.0125, 0.003, steel_dark, "lower_washer"),
        (-0.010, 0.0115, 0.008, bushing_rubber, "lower_bushing"),
        (0.002, 0.0115, 0.008, bushing_rubber, "upper_bushing"),
        (0.008, 0.0125, 0.003, steel_dark, "upper_washer"),
        (0.012, 0.0080, 0.004, steel_dark, "kingpin_nut"),
    ):
        base.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=_along_kingpin(joint_origin, offset, fore_aft_sign),
                rpy=(0.0, kingpin_pitch, 0.0),
            ),
            material=material,
            name=visual_name,
        )

    base.inertial = Inertial.from_geometry(
        Box((0.110, 0.058, 0.036)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )


def _build_truck_hanger(
    model: ArticulatedObject,
    name: str,
    *,
    fore_aft_sign: float,
    cast_aluminum,
    steel_dark,
) -> None:
    hanger = model.part(name)
    neck_x = 0.015 * fore_aft_sign
    core_x = 0.022 * fore_aft_sign
    shoulder_x = 0.010 * fore_aft_sign
    arm_x = 0.004 * fore_aft_sign
    hanger.visual(
        Box((0.018, 0.028, 0.010)),
        origin=Origin(xyz=(0.010 * fore_aft_sign, 0.0, -0.021)),
        material=cast_aluminum,
        name="yoke_cap",
    )
    hanger.visual(
        Box((0.014, 0.008, 0.018)),
        origin=Origin(xyz=(neck_x, -0.009, -0.006)),
        material=cast_aluminum,
        name="pivot_cheek_left",
    )
    hanger.visual(
        Box((0.014, 0.008, 0.018)),
        origin=Origin(xyz=(neck_x, 0.009, -0.006)),
        material=cast_aluminum,
        name="pivot_cheek_right",
    )
    hanger.visual(
        Box((0.014, 0.010, 0.018)),
        origin=Origin(xyz=(core_x, 0.0, -0.006)),
        material=cast_aluminum,
        name="pivot_core",
    )
    hanger.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(
            xyz=(0.031 * fore_aft_sign, 0.0, -0.006),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel_dark,
        name="pivot_nose",
    )
    hanger.visual(
        Box((0.016, 0.014, 0.012)),
        origin=Origin(xyz=(shoulder_x, -0.020, -0.010)),
        material=cast_aluminum,
        name="shoulder_left",
    )
    hanger.visual(
        Box((0.016, 0.014, 0.012)),
        origin=Origin(xyz=(shoulder_x, 0.020, -0.010)),
        material=cast_aluminum,
        name="shoulder_right",
    )
    hanger.visual(
        Box((0.016, 0.022, 0.010)),
        origin=Origin(xyz=(arm_x, -0.036, -0.016)),
        material=cast_aluminum,
        name="mid_arm_left",
    )
    hanger.visual(
        Box((0.016, 0.022, 0.010)),
        origin=Origin(xyz=(arm_x, 0.036, -0.016)),
        material=cast_aluminum,
        name="mid_arm_right",
    )
    hanger.visual(
        Box((0.018, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, -0.050, -0.021)),
        material=cast_aluminum,
        name="outer_arm_left",
    )
    hanger.visual(
        Box((0.018, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.050, -0.021)),
        material=cast_aluminum,
        name="outer_arm_right",
    )
    hanger.visual(
        Cylinder(radius=0.0046, length=0.176),
        origin=Origin(xyz=(0.0, 0.0, AXLE_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="axle_shaft",
    )
    for y_off in (-0.058, 0.058):
        hanger.visual(
            Cylinder(radius=0.0075, length=0.004),
            origin=Origin(xyz=(0.0, y_off, AXLE_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
        )
    for y_off in (-0.091, 0.091):
        hanger.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.0, y_off, AXLE_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
        )

    hanger.inertial = Inertial.from_geometry(
        Box((0.050, 0.182, 0.038)),
        mass=0.44,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )


def _build_wheel(
    model: ArticulatedObject,
    name: str,
    *,
    wheel_mesh,
    wheel_urethane,
) -> None:
    wheel = model.part(name)
    wheel.visual(wheel_mesh, material=wheel_urethane, name="wheel_shell")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.18,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_skateboard")

    maple = model.material("maple", rgba=(0.71, 0.54, 0.32, 1.0))
    cast_aluminum = model.material("cast_aluminum", rgba=(0.69, 0.70, 0.72, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.25, 0.28, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.91, 0.89, 0.82, 1.0))
    bushing_rubber = model.material("bushing_rubber", rgba=(0.84, 0.83, 0.72, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_build_deck_mesh(), "deck_shell"),
        material=maple,
        name="deck_shell",
    )
    for truck_x in (-TRUCK_SPACING_X, TRUCK_SPACING_X):
        for x_off, y_off in MOUNT_OFFSETS:
            deck.visual(
                Cylinder(radius=0.0036, length=0.004),
                origin=Origin(xyz=(truck_x + x_off, y_off, 0.0048)),
                material=grip_black,
            )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.080)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    _build_truck_base(
        model,
        "front_truck_base",
        fore_aft_sign=-1.0,
        cast_aluminum=cast_aluminum,
        bushing_rubber=bushing_rubber,
        steel_dark=steel_dark,
    )
    _build_truck_base(
        model,
        "rear_truck_base",
        fore_aft_sign=1.0,
        cast_aluminum=cast_aluminum,
        bushing_rubber=bushing_rubber,
        steel_dark=steel_dark,
    )
    _build_truck_hanger(
        model,
        "front_hanger",
        fore_aft_sign=-1.0,
        cast_aluminum=cast_aluminum,
        steel_dark=steel_dark,
    )
    _build_truck_hanger(
        model,
        "rear_hanger",
        fore_aft_sign=1.0,
        cast_aluminum=cast_aluminum,
        steel_dark=steel_dark,
    )

    wheel_mesh = _build_wheel_mesh("skate_wheel")
    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        _build_wheel(model, wheel_name, wheel_mesh=wheel_mesh, wheel_urethane=wheel_urethane)

    model.articulation(
        "deck_to_front_base",
        ArticulationType.FIXED,
        parent=deck,
        child="front_truck_base",
        origin=Origin(xyz=(TRUCK_SPACING_X, 0.0, DECK_UNDERSIDE_Z + 0.0009)),
    )
    model.articulation(
        "deck_to_rear_base",
        ArticulationType.FIXED,
        parent=deck,
        child="rear_truck_base",
        origin=Origin(xyz=(-TRUCK_SPACING_X, 0.0, DECK_UNDERSIDE_Z + 0.0009)),
    )
    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent="front_truck_base",
        child="front_hanger",
        origin=Origin(xyz=(0.0, 0.0, TRUCK_PIVOT_Z)),
        axis=_kingpin_axis(-1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-0.32, upper=0.32),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent="rear_truck_base",
        child="rear_hanger",
        origin=Origin(xyz=(0.0, 0.0, TRUCK_PIVOT_Z)),
        axis=_kingpin_axis(1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-0.32, upper=0.32),
    )

    for prefix in ("front", "rear"):
        hanger_name = f"{prefix}_hanger"
        for side, y_off in (("left", -WHEEL_CENTER_Y), ("right", WHEEL_CENTER_Y)):
            model.articulation(
                f"{prefix}_{side}_wheel_spin",
                ArticulationType.CONTINUOUS,
                parent=hanger_name,
                child=f"{prefix}_{side}_wheel",
                origin=Origin(xyz=(0.0, y_off, AXLE_CENTER_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=3.0, velocity=35.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_base = object_model.get_part("front_truck_base")
    rear_base = object_model.get_part("rear_truck_base")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")
    wheel_spins = (
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    )

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

    ctx.expect_contact(front_base, deck, elem_a="baseplate", contact_tol=0.0015, name="front_base_clamped_to_deck")
    ctx.expect_contact(rear_base, deck, elem_a="baseplate", contact_tol=0.0015, name="rear_base_clamped_to_deck")
    ctx.expect_gap(deck, front_left_wheel, axis="z", min_gap=0.008, name="front_left_wheel_clears_deck_rest")
    ctx.expect_gap(deck, rear_right_wheel, axis="z", min_gap=0.008, name="rear_right_wheel_clears_deck_rest")

    front_axis_ok = abs(front_steer.axis[0]) > 0.45 and abs(front_steer.axis[2]) > 0.80 and abs(front_steer.axis[1]) < 1e-9
    rear_axis_ok = abs(rear_steer.axis[0]) > 0.45 and abs(rear_steer.axis[2]) > 0.80 and abs(rear_steer.axis[1]) < 1e-9
    ctx.check(
        "front_kingpin_axis_is_angled",
        front_axis_ok,
        details=f"expected a kingpin-like axis with fore-aft and vertical components, got {front_steer.axis}",
    )
    ctx.check(
        "rear_kingpin_axis_is_angled",
        rear_axis_ok,
        details=f"expected a kingpin-like axis with fore-aft and vertical components, got {rear_steer.axis}",
    )

    for spin in wheel_spins:
        axis_ok = all(abs(component - expected) < 1e-9 for component, expected in zip(spin.axis, (0.0, 1.0, 0.0)))
        ctx.check(
            f"{spin.name}_spins_on_axle_axis",
            axis_ok,
            details=f"expected wheel spin axis (0, 1, 0), got {spin.axis}",
        )

    with ctx.pose({front_steer: 0.24, rear_steer: -0.24}):
        ctx.expect_gap(deck, front_left_wheel, axis="z", min_gap=0.004, name="front_left_wheel_clears_deck_while_steering")
        ctx.expect_gap(deck, front_right_wheel, axis="z", min_gap=0.004, name="front_right_wheel_clears_deck_while_steering")
        ctx.expect_gap(deck, rear_left_wheel, axis="z", min_gap=0.004, name="rear_left_wheel_clears_deck_while_steering")
        ctx.expect_gap(deck, rear_right_wheel, axis="z", min_gap=0.004, name="rear_right_wheel_clears_deck_while_steering")

        fl_pos = ctx.part_world_position(front_left_wheel)
        fr_pos = ctx.part_world_position(front_right_wheel)
        rl_pos = ctx.part_world_position(rear_left_wheel)
        rr_pos = ctx.part_world_position(rear_right_wheel)
        front_roll_ok = fl_pos is not None and fr_pos is not None and abs(fl_pos[2] - fr_pos[2]) > 0.010
        rear_roll_ok = rl_pos is not None and rr_pos is not None and abs(rl_pos[2] - rr_pos[2]) > 0.010
        ctx.check(
            "front_truck_steer_rocks_axle_across_bushings",
            front_roll_ok,
            details=f"expected front wheel heights to diverge under steer, got left={fl_pos}, right={fr_pos}",
        )
        ctx.check(
            "rear_truck_steer_rocks_axle_across_bushings",
            rear_roll_ok,
            details=f"expected rear wheel heights to diverge under steer, got left={rl_pos}, right={rr_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
