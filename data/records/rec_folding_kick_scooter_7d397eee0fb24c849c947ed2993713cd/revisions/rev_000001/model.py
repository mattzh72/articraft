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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_tire_mesh(name: str, *, radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.56, -half_width * 0.98),
        (radius * 0.82, -half_width),
        (radius * 0.95, -half_width * 0.72),
        (radius, -half_width * 0.22),
        (radius, half_width * 0.22),
        (radius * 0.95, half_width * 0.72),
        (radius * 0.82, half_width),
        (radius * 0.56, half_width * 0.98),
        (radius * 0.47, half_width * 0.35),
        (radius * 0.44, 0.0),
        (radius * 0.47, -half_width * 0.35),
        (radius * 0.56, -half_width * 0.98),
    ]
    return _mesh(name, LatheGeometry(profile, segments=56).rotate_x(pi / 2.0))


def _add_wheel_visuals(
    part,
    *,
    mesh_name: str,
    tire_radius: float,
    tire_width: float,
    rim_radius: float,
    rim_width: float,
    hub_radius: float,
    hub_width: float,
    axle_radius: float,
    axle_width: float,
    rubber,
    metal,
    dark_metal,
) -> None:
    y_axis = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    tire_mesh = _wheel_tire_mesh(mesh_name, radius=tire_radius, width=tire_width)
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=rim_radius, length=rim_width),
        origin=y_axis,
        material=metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=y_axis,
        material=dark_metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=axle_radius, length=axle_width),
        origin=y_axis,
        material=metal,
        name="axle",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.55, length=axle_width * 0.96),
        origin=y_axis,
        material=metal,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="seated_ride_on_scooter")

    deck_blue = model.material("deck_blue", rgba=(0.18, 0.24, 0.33, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    silver = model.material("silver", rgba=(0.68, 0.70, 0.73, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.16, 0.16, 0.17, 1.0))
    stem_grey = model.material("stem_grey", rgba=(0.33, 0.35, 0.38, 1.0))

    deck_length = 0.70
    deck_width = 0.18
    deck_thickness = 0.040
    wheel_radius = 0.105
    wheel_tire_width = 0.045
    wheel_rim_width = 0.036
    wheel_hub_width = 0.028
    wheel_axle_width = 0.056
    hinge_origin_z = 0.034

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.72, 0.20, 0.10)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    deck.visual(
        _mesh(
            "scooter_deck_shell",
            ExtrudeGeometry(
                rounded_rect_profile(deck_length, deck_width, 0.030, corner_segments=10),
                deck_thickness,
                center=True,
            ),
        ),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.50, 0.12, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, deck_thickness * 0.5 + 0.002)),
        material=matte_black,
        name="deck_pad",
    )
    deck.visual(
        Box((0.46, 0.11, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=matte_black,
        name="battery_box",
    )
    deck.visual(
        Box((0.030, 0.018, 0.028)),
        origin=Origin(xyz=(0.325, -0.029, hinge_origin_z)),
        material=dark_metal,
        name="front_hinge_left",
    )
    deck.visual(
        Box((0.030, 0.018, 0.028)),
        origin=Origin(xyz=(0.325, 0.029, hinge_origin_z)),
        material=dark_metal,
        name="front_hinge_right",
    )
    deck.visual(
        Box((0.028, 0.018, 0.028)),
        origin=Origin(xyz=(-0.305, -0.029, hinge_origin_z)),
        material=dark_metal,
        name="rear_hinge_left",
    )
    deck.visual(
        Box((0.028, 0.018, 0.028)),
        origin=Origin(xyz=(-0.305, 0.029, hinge_origin_z)),
        material=dark_metal,
        name="rear_hinge_right",
    )
    deck.visual(
        Box((0.130, 0.018, 0.100)),
        origin=Origin(xyz=(-0.370, -0.034, -0.067)),
        material=dark_metal,
        name="rear_stay_left",
    )
    deck.visual(
        Box((0.130, 0.018, 0.100)),
        origin=Origin(xyz=(-0.370, 0.034, -0.067)),
        material=dark_metal,
        name="rear_stay_right",
    )
    deck.visual(
        Box((0.040, 0.012, 0.036)),
        origin=Origin(xyz=(-0.445, -0.034, -0.135)),
        material=dark_metal,
        name="rear_axle_left",
    )
    deck.visual(
        Box((0.040, 0.012, 0.036)),
        origin=Origin(xyz=(-0.445, 0.034, -0.135)),
        material=dark_metal,
        name="rear_axle_right",
    )

    stem = model.part("stem_assembly")
    stem.inertial = Inertial.from_geometry(
        Box((0.68, 0.68, 0.95)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )
    stem.visual(
        Box((0.030, 0.040, 0.016)),
        material=dark_metal,
        name="hinge_tongue",
    )
    stem.visual(
        Cylinder(radius=0.024, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=dark_metal,
        name="stem_base_collar",
    )
    stem.visual(
        Cylinder(radius=0.020, length=0.694),
        origin=Origin(xyz=(-0.026, 0.0, 0.409), rpy=(0.0, -0.074, 0.0)),
        material=stem_grey,
        name="stem_tube",
    )
    stem.visual(
        Box((0.160, 0.085, 0.034)),
        origin=Origin(xyz=(0.057, 0.0, 0.055)),
        material=dark_metal,
        name="fork_crown",
    )
    stem.visual(
        Box((0.022, 0.012, 0.206)),
        origin=Origin(xyz=(0.135, -0.034, -0.028)),
        material=dark_metal,
        name="fork_left",
    )
    stem.visual(
        Box((0.022, 0.012, 0.206)),
        origin=Origin(xyz=(0.135, 0.034, -0.028)),
        material=dark_metal,
        name="fork_right",
    )
    stem.visual(
        Box((0.060, 0.085, 0.018)),
        origin=Origin(xyz=(0.135, 0.0, 0.018)),
        material=dark_metal,
        name="fork_bridge",
    )
    stem.visual(
        Box((0.060, 0.090, 0.062)),
        origin=Origin(xyz=(-0.050, 0.0, 0.735)),
        material=dark_metal,
        name="handlebar_clamp",
    )
    stem.visual(
        _mesh(
            "scooter_handlebar_bar",
            tube_from_spline_points(
                [
                    (-0.050, -0.315, 0.735),
                    (-0.050, -0.180, 0.750),
                    (-0.050, -0.060, 0.760),
                    (-0.050, 0.060, 0.760),
                    (-0.050, 0.180, 0.750),
                    (-0.050, 0.315, 0.735),
                ],
                radius=0.014,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=stem_grey,
        name="handlebar_bar",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(-0.050, -0.335, 0.735), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_left",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(-0.050, 0.335, 0.735), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_right",
    )

    seat_arm = model.part("seat_support_arm")
    seat_arm.inertial = Inertial.from_geometry(
        Box((0.42, 0.22, 0.72)),
        mass=4.5,
        origin=Origin(xyz=(0.12, 0.0, 0.30)),
    )
    seat_arm.visual(
        Box((0.028, 0.040, 0.016)),
        material=dark_metal,
        name="hinge_tongue",
    )
    seat_arm.visual(
        Cylinder(radius=0.022, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0365)),
        material=dark_metal,
        name="seat_base_collar",
    )
    seat_arm.visual(
        Cylinder(radius=0.018, length=0.576),
        origin=Origin(xyz=(0.091, 0.0, 0.300), rpy=(0.0, 0.322, 0.0)),
        material=stem_grey,
        name="support_tube",
    )
    seat_arm.visual(
        Cylinder(radius=0.022, length=0.090),
        origin=Origin(xyz=(0.182, 0.0, 0.570)),
        material=dark_metal,
        name="seat_post",
    )
    seat_arm.visual(
        _mesh(
            "scooter_seat_pad",
            ExtrudeGeometry(
                rounded_rect_profile(0.220, 0.150, 0.032, corner_segments=10),
                0.038,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.190, 0.0, 0.625)),
        material=seat_vinyl,
        name="seat_pad",
    )
    seat_arm.visual(
        Box((0.180, 0.120, 0.008)),
        origin=Origin(xyz=(0.190, 0.0, 0.602)),
        material=dark_metal,
        name="seat_base",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_tire_width),
        mass=2.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_wheel,
        mesh_name="scooter_front_tire",
        tire_radius=wheel_radius,
        tire_width=wheel_tire_width,
        rim_radius=0.074,
        rim_width=wheel_rim_width,
        hub_radius=0.022,
        hub_width=wheel_hub_width,
        axle_radius=0.010,
        axle_width=wheel_axle_width,
        rubber=rubber,
        metal=silver,
        dark_metal=dark_metal,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_tire_width),
        mass=2.4,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_wheel,
        mesh_name="scooter_rear_tire",
        tire_radius=wheel_radius,
        tire_width=wheel_tire_width,
        rim_radius=0.074,
        rim_width=wheel_rim_width,
        hub_radius=0.022,
        hub_width=wheel_hub_width,
        axle_radius=0.010,
        axle_width=wheel_axle_width,
        rubber=rubber,
        metal=silver,
        dark_metal=dark_metal,
    )

    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(0.325, 0.0, hinge_origin_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=-1.55, upper=0.08),
    )
    model.articulation(
        "deck_to_seat_arm",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=seat_arm,
        origin=Origin(xyz=(-0.305, 0.0, hinge_origin_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.10),
    )
    model.articulation(
        "stem_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=stem,
        child=front_wheel,
        origin=Origin(xyz=(0.135, 0.0, -0.135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.463, 0.0, -0.135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem_assembly")
    seat_arm = object_model.get_part("seat_support_arm")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    stem_hinge = object_model.get_articulation("deck_to_stem")
    seat_hinge = object_model.get_articulation("deck_to_seat_arm")
    front_axle = object_model.get_articulation("stem_to_front_wheel")
    rear_axle = object_model.get_articulation("deck_to_rear_wheel")

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
        "articulation_axes_match_scooter_mechanics",
        stem_hinge.axis == (0.0, 1.0, 0.0)
        and seat_hinge.axis == (0.0, 1.0, 0.0)
        and front_axle.axis == (0.0, 1.0, 0.0)
        and rear_axle.axis == (0.0, 1.0, 0.0),
        details="Stem fold, seat fold, and both wheel axles should rotate around the scooter width axis.",
    )
    ctx.check(
        "joint_types_match_intent",
        stem_hinge.joint_type == ArticulationType.REVOLUTE
        and seat_hinge.joint_type == ArticulationType.REVOLUTE
        and front_axle.joint_type == ArticulationType.CONTINUOUS
        and rear_axle.joint_type == ArticulationType.CONTINUOUS,
        details="The two folding mechanisms should be revolute and the wheel axles should spin continuously.",
    )

    ctx.expect_contact(stem, deck, name="stem_hinge_is_physically_connected")
    ctx.expect_contact(seat_arm, deck, name="seat_hinge_is_physically_connected")
    ctx.expect_contact(front_wheel, stem, name="front_wheel_contacts_fork")
    ctx.expect_contact(rear_wheel, deck, name="rear_wheel_contacts_rear_mount")

    ctx.expect_gap(
        stem,
        deck,
        axis="z",
        positive_elem="handlebar_bar",
        negative_elem="deck_shell",
        min_gap=0.68,
        max_gap=0.82,
        name="upright_handlebar_height",
    )
    ctx.expect_gap(
        seat_arm,
        deck,
        axis="z",
        positive_elem="seat_pad",
        negative_elem="deck_shell",
        min_gap=0.55,
        max_gap=0.68,
        name="deployed_seat_height",
    )
    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.88,
        max_gap=0.96,
        name="wheelbase_is_scooter_like",
    )

    with ctx.pose({stem_hinge: -1.45}):
        ctx.expect_gap(
            stem,
            deck,
            axis="z",
            positive_elem="handlebar_bar",
            negative_elem="deck_shell",
            min_gap=0.01,
            max_gap=0.12,
            name="folded_stem_lays_near_deck",
        )

    with ctx.pose({seat_hinge: 1.10}):
        ctx.expect_gap(
            seat_arm,
            deck,
            axis="z",
            positive_elem="seat_pad",
            negative_elem="deck_shell",
            min_gap=0.01,
            max_gap=0.12,
            name="folded_seat_rest_above_deck",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
