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
    wire_from_points,
)


DECK_WIDTH = 0.56
DECK_LENGTH = 0.92
DECK_TOP = 0.18
DECK_THICKNESS = 0.03
DECK_CENTER_Z = DECK_TOP - DECK_THICKNESS * 0.5
DECK_UNDERSIDE = DECK_TOP - DECK_THICKNESS

CASTER_STEM_Z = 0.124
CASTER_X = 0.215
CASTER_Y = 0.378
WHEEL_RADIUS = 0.052
WHEEL_WIDTH = 0.036
WHEEL_CENTER_LOCAL = (0.0, -0.034, -0.074)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_tread_mesh():
    half_width = WHEEL_WIDTH * 0.5
    profile = [
        (WHEEL_RADIUS * 0.58, -half_width * 0.94),
        (WHEEL_RADIUS * 0.82, -half_width),
        (WHEEL_RADIUS * 0.95, -half_width * 0.74),
        (WHEEL_RADIUS, -half_width * 0.22),
        (WHEEL_RADIUS, half_width * 0.22),
        (WHEEL_RADIUS * 0.95, half_width * 0.74),
        (WHEEL_RADIUS * 0.82, half_width),
        (WHEEL_RADIUS * 0.58, half_width * 0.94),
        (WHEEL_RADIUS * 0.44, half_width * 0.36),
        (WHEEL_RADIUS * 0.40, 0.0),
        (WHEEL_RADIUS * 0.44, -half_width * 0.36),
        (WHEEL_RADIUS * 0.58, -half_width * 0.94),
    ]
    return _save_mesh("caster_wheel_tread", LatheGeometry(profile, segments=56).rotate_y(pi / 2.0))


def _handle_mesh():
    return _save_mesh(
        "platform_cart_handle",
        wire_from_points(
            [
                (-0.18, 0.0, 0.0),
                (-0.18, 0.0, 0.68),
                (-0.18, 0.03, 0.78),
                (0.18, 0.03, 0.78),
                (0.18, 0.0, 0.68),
                (0.18, 0.0, 0.0),
            ],
            radius=0.016,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.055,
            corner_segments=10,
        ),
    )


def _add_caster(
    model: ArticulatedObject,
    deck,
    *,
    prefix: str,
    stem_x: float,
    stem_y: float,
    steel,
    dark_steel,
    rubber,
    tread_mesh,
) -> None:
    stem = model.part(prefix + "_stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.082, 0.060, 0.026)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )
    stem.visual(
        Box((0.082, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=steel,
        name="mount_plate",
    )
    stem.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="kingpin",
    )

    fork = model.part(prefix + "_fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.052, 0.060, 0.090)),
        mass=1.0,
        origin=Origin(xyz=(0.0, -0.028, -0.045)),
    )
    fork.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=dark_steel,
        name="crown",
    )
    fork.visual(
        Box((0.050, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.020, -0.006)),
        material=steel,
        name="yoke_body",
    )
    fork.visual(
        Box((0.006, 0.022, 0.064)),
        origin=Origin(xyz=(-0.022, -0.032, -0.046)),
        material=steel,
        name="left_leg",
    )
    fork.visual(
        Box((0.006, 0.022, 0.064)),
        origin=Origin(xyz=(0.022, -0.032, -0.046)),
        material=steel,
        name="right_leg",
    )
    fork.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(-0.017, -0.034, -0.074), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_axle_stub",
    )
    fork.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.017, -0.034, -0.074), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_axle_stub",
    )

    wheel = model.part(prefix + "_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    wheel.visual(tread_mesh, material=rubber, name="tread")
    wheel.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="bushing",
    )

    model.articulation(
        prefix + "_stem_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(stem_x, stem_y, CASTER_STEM_Z)),
    )
    model.articulation(
        prefix + "_swivel",
        ArticulationType.CONTINUOUS,
        parent=stem,
        child=fork,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        prefix + "_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=WHEEL_CENTER_LOCAL),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_platform_cart")

    deck_blue = model.material("deck_blue", rgba=(0.12, 0.34, 0.68, 1.0))
    handle_gray = model.material("handle_gray", rgba=(0.78, 0.79, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    deck_mat = model.material("deck_mat", rgba=(0.17, 0.18, 0.19, 1.0))
    pedal_red = model.material("pedal_red", rgba=(0.74, 0.10, 0.07, 1.0))

    tread_mesh = _wheel_tread_mesh()
    handle_mesh = _handle_mesh()

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((DECK_WIDTH, DECK_LENGTH, 0.05)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )
    deck.visual(
        Box((DECK_WIDTH, DECK_LENGTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=deck_blue,
        name="deck_plate",
    )
    deck.visual(
        Box((0.060, 0.760, 0.018)),
        origin=Origin(xyz=(-0.14, 0.0, 0.141)),
        material=dark_steel,
        name="left_rib",
    )
    deck.visual(
        Box((0.060, 0.760, 0.018)),
        origin=Origin(xyz=(0.14, 0.0, 0.141)),
        material=dark_steel,
        name="right_rib",
    )
    deck.visual(
        Box((0.340, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, -0.02, 0.141)),
        material=dark_steel,
        name="center_rib",
    )
    deck.visual(
        Box((0.490, 0.840, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP + 0.002)),
        material=deck_mat,
        name="deck_mat",
    )
    deck.visual(
        Box((0.012, 0.900, 0.018)),
        origin=Origin(xyz=(-0.274, 0.0, 0.171)),
        material=dark_steel,
        name="left_rail",
    )
    deck.visual(
        Box((0.012, 0.900, 0.018)),
        origin=Origin(xyz=(0.274, 0.0, 0.171)),
        material=dark_steel,
        name="right_rail",
    )
    deck.visual(
        Box((0.520, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.454, 0.171)),
        material=dark_steel,
        name="front_rail",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.420, 0.060, 0.800)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.015, 0.40)),
    )
    handle.visual(handle_mesh, material=handle_gray, name="u_bar")
    handle.visual(
        Box((0.050, 0.040, 0.008)),
        origin=Origin(xyz=(-0.18, 0.0, 0.004)),
        material=dark_steel,
        name="left_foot",
    )
    handle.visual(
        Box((0.050, 0.040, 0.008)),
        origin=Origin(xyz=(0.18, 0.0, 0.004)),
        material=dark_steel,
        name="right_foot",
    )
    model.articulation(
        "handle_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(0.0, 0.435, DECK_TOP)),
    )

    _add_caster(
        model,
        deck,
        prefix="front_left",
        stem_x=-CASTER_X,
        stem_y=-CASTER_Y,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
        tread_mesh=tread_mesh,
    )
    _add_caster(
        model,
        deck,
        prefix="front_right",
        stem_x=CASTER_X,
        stem_y=-CASTER_Y,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
        tread_mesh=tread_mesh,
    )
    _add_caster(
        model,
        deck,
        prefix="rear_left",
        stem_x=-CASTER_X,
        stem_y=CASTER_Y,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
        tread_mesh=tread_mesh,
    )
    _add_caster(
        model,
        deck,
        prefix="rear_right",
        stem_x=CASTER_X,
        stem_y=CASTER_Y,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
        tread_mesh=tread_mesh,
    )

    brake_bracket = model.part("brake_bracket")
    brake_bracket.inertial = Inertial.from_geometry(
        Box((0.044, 0.032, 0.044)),
        mass=0.6,
        origin=Origin(xyz=(0.0, -0.010, -0.018)),
    )
    brake_bracket.visual(
        Box((0.044, 0.028, 0.010)),
        material=steel,
        name="mount_plate",
    )
    brake_bracket.visual(
        Box((0.012, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, -0.010, -0.016)),
        material=dark_steel,
        name="hanger",
    )
    brake_bracket.visual(
        Box((0.024, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.018, -0.024)),
        material=dark_steel,
        name="ear_bridge",
    )
    brake_bracket.visual(
        Box((0.004, 0.010, 0.018)),
        origin=Origin(xyz=(-0.010, -0.018, -0.033)),
        material=steel,
        name="left_ear",
    )
    brake_bracket.visual(
        Box((0.004, 0.010, 0.018)),
        origin=Origin(xyz=(0.010, -0.018, -0.033)),
        material=steel,
        name="right_ear",
    )
    model.articulation(
        "brake_bracket_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=brake_bracket,
        origin=Origin(xyz=(0.249, 0.434, 0.145)),
    )

    brake_pedal = model.part("brake_pedal")
    brake_pedal.inertial = Inertial.from_geometry(
        Box((0.026, 0.056, 0.020)),
        mass=0.5,
        origin=Origin(xyz=(-0.014, -0.028, 0.004)),
    )
    brake_pedal.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    brake_pedal.visual(
        Box((0.010, 0.030, 0.008)),
        origin=Origin(xyz=(-0.010, -0.018, 0.004)),
        material=pedal_red,
        name="arm",
    )
    brake_pedal.visual(
        Box((0.016, 0.016, 0.008)),
        origin=Origin(xyz=(-0.018, -0.040, 0.010)),
        material=pedal_red,
        name="pedal_pad",
    )
    brake_pedal.visual(
        Box((0.020, 0.012, 0.008)),
        origin=Origin(xyz=(-0.018, -0.040, 0.006)),
        material=rubber,
        name="brake_shoe",
    )
    model.articulation(
        "rear_right_brake",
        ArticulationType.REVOLUTE,
        parent=brake_bracket,
        child=brake_pedal,
        origin=Origin(xyz=(0.0, -0.018, -0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    brake_bracket = object_model.get_part("brake_bracket")
    brake_pedal = object_model.get_part("brake_pedal")
    brake_joint = object_model.get_articulation("rear_right_brake")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    caster_prefixes = ("front_left", "front_right", "rear_left", "rear_right")

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
        "rear_right_brake_axis",
        tuple(brake_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected brake pedal hinge axis (1, 0, 0), got {brake_joint.axis}",
    )
    ctx.expect_contact(handle, deck, name="handle_is_mounted_to_deck")
    ctx.expect_contact(brake_bracket, deck, name="brake_bracket_is_mounted_to_deck")
    ctx.expect_overlap(
        brake_pedal,
        brake_bracket,
        axes="x",
        min_overlap=0.014,
        name="brake_pedal_is_clipped_to_bracket",
    )
    ctx.expect_origin_distance(
        brake_pedal,
        brake_bracket,
        axes="xy",
        min_dist=0.008,
        max_dist=0.035,
        name="brake_hinge_sits_at_rear_corner",
    )

    for prefix in caster_prefixes:
        stem = object_model.get_part(prefix + "_stem")
        fork = object_model.get_part(prefix + "_fork")
        wheel = object_model.get_part(prefix + "_wheel")
        swivel = object_model.get_articulation(prefix + "_swivel")
        spin = object_model.get_articulation(prefix + "_spin")

        ctx.check(
            prefix + "_swivel_axis",
            tuple(swivel.axis) == (0.0, 0.0, 1.0),
            details=f"expected vertical swivel axis for {prefix}, got {swivel.axis}",
        )
        ctx.check(
            prefix + "_spin_axis",
            tuple(spin.axis) == (1.0, 0.0, 0.0),
            details=f"expected wheel spin axis for {prefix}, got {spin.axis}",
        )
        ctx.expect_contact(stem, deck, name=prefix + "_stem_mount_contact")
        ctx.expect_contact(fork, stem, name=prefix + "_fork_contacts_stem")
        ctx.expect_contact(wheel, fork, name=prefix + "_wheel_on_axle")

    with ctx.pose({brake_joint: 0.0}):
        ctx.expect_gap(
            brake_pedal,
            rear_right_wheel,
            axis="z",
            positive_elem="brake_shoe",
            negative_elem="tread",
            min_gap=0.010,
            name="brake_shoe_clears_wheel_when_released",
        )

    with ctx.pose({brake_joint: 0.22}):
        ctx.expect_overlap(
            brake_pedal,
            rear_right_wheel,
            axes="xy",
            elem_a="brake_shoe",
            elem_b="tread",
            min_overlap=0.004,
            name="brake_shoe_tracks_wheel_face",
        )
        ctx.expect_gap(
            brake_pedal,
            rear_right_wheel,
            axis="z",
            positive_elem="brake_shoe",
            negative_elem="tread",
            max_gap=0.003,
            max_penetration=0.0,
            name="brake_shoe_reaches_wheel_when_pressed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
