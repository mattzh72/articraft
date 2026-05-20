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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    z_center: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    tire_radius: float,
    tire_width: float,
    rim_radius: float,
    hub_radius: float,
    hub_width: float,
    rim_material,
    hub_material,
    rubber_material,
    motorized: bool,
) -> None:
    wheel_rpy = (math.pi / 2.0, 0.0, 0.0)
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.72, -half_width * 1.00),
        (tire_radius * 0.86, -half_width * 0.96),
        (tire_radius * 0.96, -half_width * 0.52),
        (tire_radius, 0.0),
        (tire_radius * 0.96, half_width * 0.52),
        (tire_radius * 0.86, half_width * 0.96),
        (tire_radius * 0.72, half_width * 1.00),
        (tire_radius * 0.63, half_width * 0.36),
        (tire_radius * 0.60, 0.0),
        (tire_radius * 0.63, -half_width * 0.36),
        (tire_radius * 0.72, -half_width * 1.00),
    ]
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(tire_profile, segments=56).rotate_x(math.pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber_material, name="tire")

    rim_outer = superellipse_profile(rim_radius * 2.0, rim_radius * 2.0, exponent=2.0, segments=42)
    spoke_window = rounded_rect_profile(
        rim_radius * 0.28,
        rim_radius * 0.14,
        rim_radius * 0.035,
        corner_segments=6,
    )
    hole_profiles = [superellipse_profile(hub_radius * 1.15, hub_radius * 1.15, exponent=2.0, segments=24)]
    window_radius = rim_radius * 0.48
    for index in range(6):
        angle = index * math.tau / 6.0
        hole_profiles.append(
            _transform_profile(
                spoke_window,
                dx=math.cos(angle) * window_radius,
                dy=math.sin(angle) * window_radius,
                angle=angle,
            )
        )
    rim_face_mesh = _save_mesh(
        f"{mesh_prefix}_rim_face",
        ExtrudeWithHolesGeometry(
            rim_outer,
            hole_profiles,
            height=tire_width * 0.10,
            center=True,
        ).rotate_x(math.pi / 2.0),
    )
    part.visual(
        rim_face_mesh,
        origin=Origin(xyz=(0.0, -tire_width * 0.16, 0.0)),
        material=rim_material,
        name="rim_face_left",
    )
    part.visual(
        rim_face_mesh,
        origin=Origin(xyz=(0.0, tire_width * 0.16, 0.0)),
        material=rim_material,
        name="rim_face_right",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.97, length=tire_width * 0.46),
        origin=Origin(rpy=wheel_rpy),
        material=rim_material,
        name="rim_barrel",
    )

    hub_name = "motor_hub" if motorized else "rear_hub"
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=Origin(rpy=wheel_rpy),
        material=hub_material,
        name=hub_name,
    )
    cover_radius = hub_radius * (1.10 if motorized else 0.92)
    cover_offset = hub_width * 0.32
    part.visual(
        Cylinder(radius=cover_radius, length=0.006),
        origin=Origin(xyz=(0.0, -cover_offset, 0.0), rpy=wheel_rpy),
        material=hub_material,
        name=f"{hub_name}_cover_left",
    )
    part.visual(
        Cylinder(radius=cover_radius, length=0.006),
        origin=Origin(xyz=(0.0, cover_offset, 0.0), rpy=wheel_rpy),
        material=hub_material,
        name=f"{hub_name}_cover_right",
    )

    if motorized:
        part.visual(
            Box((0.024, 0.010, 0.030)),
            origin=Origin(xyz=(0.054, 0.0, 0.0)),
            material=hub_material,
            name="motor_cable_gland",
        )


def _stem_axis_offset(length: float, stem_pitch: float) -> tuple[float, float, float]:
    return (
        math.sin(stem_pitch) * length * 0.5,
        0.0,
        math.cos(stem_pitch) * length * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_commuter_scooter")

    matte_black = model.material("matte_black", rgba=(0.14, 0.15, 0.16, 1.0))
    charcoal = model.material("charcoal", rgba=(0.21, 0.22, 0.24, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.33, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    deck_grip = model.material("deck_grip", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_red = model.material("accent_red", rgba=(0.70, 0.15, 0.12, 1.0))

    wheel_radius = 0.108
    wheel_width = 0.048
    deck_top_z = 0.103
    hinge_x = 0.305
    hinge_z = 0.125
    stem_pitch = -0.28
    lower_stem_length = 0.62
    lower_top = (
        math.sin(stem_pitch) * lower_stem_length,
        0.0,
        math.cos(stem_pitch) * lower_stem_length,
    )

    deck = model.part("deck")
    deck_shell = section_loft(
        [
            _yz_section(-0.24, width=0.134, height=0.032, z_center=0.074, radius=0.014),
            _yz_section(-0.12, width=0.162, height=0.044, z_center=0.080, radius=0.016),
            _yz_section(0.16, width=0.162, height=0.044, z_center=0.080, radius=0.016),
            _yz_section(0.33, width=0.096, height=0.036, z_center=0.087, radius=0.014),
        ]
    )
    deck.visual(
        _save_mesh("scooter_deck_shell", deck_shell),
        material=matte_black,
        name="deck_shell",
    )
    deck.visual(
        Box((0.470, 0.118, 0.004)),
        origin=Origin(xyz=(-0.015, 0.0, deck_top_z + 0.002)),
        material=deck_grip,
        name="grip_tape",
    )
    deck.visual(
        Box((0.100, 0.120, 0.010)),
        origin=Origin(xyz=(-0.160, 0.0, deck_top_z + 0.005)),
        material=charcoal,
        name="rear_fender_mount",
    )
    deck.visual(
        Box((0.088, 0.116, 0.020)),
        origin=Origin(xyz=(0.242, 0.0, 0.096)),
        material=charcoal,
        name="neck_block",
    )
    deck.visual(
        Box((0.080, 0.090, 0.030)),
        origin=Origin(xyz=(0.220, 0.0, 0.093)),
        material=charcoal,
        name="neck_gusset",
    )
    deck.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(hinge_x, 0.045, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_lug_left",
    )
    deck.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(hinge_x, -0.045, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_lug_right",
    )
    deck.visual(
        Box((0.150, 0.014, 0.044)),
        origin=Origin(xyz=(-0.300, 0.041, 0.079)),
        material=charcoal,
        name="rear_stay_left",
    )
    deck.visual(
        Box((0.150, 0.014, 0.044)),
        origin=Origin(xyz=(-0.300, -0.041, 0.079)),
        material=charcoal,
        name="rear_stay_right",
    )
    deck.visual(
        Box((0.024, 0.014, 0.080)),
        origin=Origin(xyz=(-0.366, 0.041, 0.086)),
        material=dark_steel,
        name="rear_dropout_left",
    )
    deck.visual(
        Box((0.024, 0.014, 0.080)),
        origin=Origin(xyz=(-0.366, -0.041, 0.086)),
        material=dark_steel,
        name="rear_dropout_right",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.72, 0.20, 0.18)),
        mass=14.0,
        origin=Origin(xyz=(0.00, 0.0, 0.090)),
    )

    stem_lower = model.part("stem_lower")
    stem_lower.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    stem_lower.visual(
        Cylinder(radius=0.024, length=lower_stem_length),
        origin=Origin(xyz=_stem_axis_offset(lower_stem_length, stem_pitch), rpy=(0.0, stem_pitch, 0.0)),
        material=satin_silver,
        name="outer_sleeve",
    )
    stem_lower.visual(
        Box((0.076, 0.058, 0.048)),
        origin=Origin(xyz=(0.034, 0.0, 0.014)),
        material=charcoal,
        name="fork_crown",
    )
    stem_lower.visual(
        Box((0.180, 0.090, 0.042)),
        origin=Origin(xyz=(0.010, 0.0, 0.056), rpy=(0.0, stem_pitch, 0.0)),
        material=charcoal,
        name="neck_web",
    )
    fork_left = tube_from_spline_points(
        [(0.030, 0.034, 0.012), (0.095, 0.034, 0.004), (0.190, 0.034, -0.017)],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    fork_right = tube_from_spline_points(
        [(0.030, -0.034, 0.012), (0.095, -0.034, 0.004), (0.190, -0.034, -0.017)],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    stem_lower.visual(
        _save_mesh("scooter_fork_left", fork_left),
        material=charcoal,
        name="fork_left",
    )
    stem_lower.visual(
        _save_mesh("scooter_fork_right", fork_right),
        material=charcoal,
        name="fork_right",
    )
    stem_lower.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(
            xyz=(lower_top[0] * 0.97, 0.0, lower_top[2] * 0.97),
            rpy=(0.0, stem_pitch, 0.0),
        ),
        material=dark_steel,
        name="telescoping_collar",
    )
    stem_lower.inertial = Inertial.from_geometry(
        Box((0.48, 0.14, 0.78)),
        mass=4.2,
        origin=Origin(xyz=(0.02, 0.0, 0.34)),
    )

    stem_upper = model.part("stem_upper")
    stem_upper.visual(
        Cylinder(radius=0.018, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=satin_silver,
        name="inner_tube",
    )
    stem_upper.visual(
        Cylinder(radius=0.026, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        material=charcoal,
        name="handlebar_clamp",
    )
    handlebar_mesh = tube_from_spline_points(
        [(-0.020, -0.250, 0.485), (-0.010, -0.140, 0.500), (0.0, 0.0, 0.515), (-0.010, 0.140, 0.500), (-0.020, 0.250, 0.485)],
        radius=0.013,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    stem_upper.visual(
        _save_mesh("scooter_handlebar_bar", handlebar_mesh),
        material=charcoal,
        name="handlebar_bar",
    )
    stem_upper.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(-0.026, -0.260, 0.483), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    stem_upper.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(-0.026, 0.260, 0.483), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    stem_upper.visual(
        Box((0.078, 0.032, 0.040)),
        origin=Origin(xyz=(0.010, 0.0, 0.520)),
        material=accent_red,
        name="display_cluster",
    )
    stem_upper.inertial = Inertial.from_geometry(
        Box((0.16, 0.64, 0.86)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    front_wheel = model.part("front_wheel")
    _wheel_visuals(
        front_wheel,
        mesh_prefix="front_motor_wheel",
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        rim_radius=0.081,
        hub_radius=0.070,
        hub_width=0.044,
        rim_material=satin_silver,
        hub_material=dark_steel,
        rubber_material=rubber,
        motorized=True,
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=3.6,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    _wheel_visuals(
        rear_wheel,
        mesh_prefix="rear_wheel",
        tire_radius=wheel_radius,
        tire_width=wheel_width * 0.92,
        rim_radius=0.079,
        hub_radius=0.052,
        hub_width=0.036,
        rim_material=satin_silver,
        hub_material=charcoal,
        rubber_material=rubber,
        motorized=False,
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width * 0.92),
        mass=2.4,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    rear_wheel.visual(
        Cylinder(radius=0.007, length=0.068),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_shaft",
    )
    rear_wheel.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, -0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_nut_left",
    )
    rear_wheel.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_nut_right",
    )

    rear_fender = model.part("rear_fender")
    rear_fender.visual(
        Box((0.052, 0.110, 0.012)),
        origin=Origin(xyz=(0.028, 0.0, 0.006)),
        material=charcoal,
        name="mount_block",
    )
    rear_fender.visual(
        Box((0.040, 0.098, 0.024)),
        origin=Origin(xyz=(0.004, 0.0, 0.022)),
        material=charcoal,
        name="front_stiffener",
    )
    fender_shell = section_loft(
        [
            _yz_section(0.018, width=0.104, height=0.014, z_center=0.030, radius=0.005),
            _yz_section(-0.045, width=0.108, height=0.012, z_center=0.072, radius=0.005),
            _yz_section(-0.120, width=0.112, height=0.012, z_center=0.118, radius=0.005),
            _yz_section(-0.188, width=0.114, height=0.012, z_center=0.141, radius=0.005),
            _yz_section(-0.248, width=0.106, height=0.014, z_center=0.129, radius=0.005),
        ]
    )
    rear_fender.visual(
        _save_mesh("scooter_rear_fender_shell", fender_shell),
        material=matte_black,
        name="fender_shell",
    )
    rear_fender.visual(
        Box((0.030, 0.050, 0.046)),
        origin=Origin(xyz=(-0.212, 0.0, 0.130)),
        material=charcoal,
        name="brake_support",
    )
    rear_fender.visual(
        Box((0.040, 0.082, 0.010)),
        origin=Origin(xyz=(-0.245, 0.0, 0.124), rpy=(0.0, -0.22, 0.0)),
        material=matte_black,
        name="brake_pad",
    )
    rear_fender.inertial = Inertial.from_geometry(
        Box((0.34, 0.13, 0.16)),
        mass=0.9,
        origin=Origin(xyz=(-0.11, 0.0, 0.072)),
    )

    model.articulation(
        "deck_to_stem_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem_lower,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "stem_lower_to_stem_upper",
        ArticulationType.PRISMATIC,
        parent=stem_lower,
        child=stem_upper,
        origin=Origin(xyz=lower_top, rpy=(0.0, stem_pitch, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=0.180,
        ),
    )
    model.articulation(
        "stem_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=stem_lower,
        child=front_wheel,
        origin=Origin(xyz=(0.190, 0.0, -0.017)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.366, 0.0, wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )
    model.articulation(
        "deck_to_rear_fender",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_fender,
        origin=Origin(xyz=(-0.160, 0.0, deck_top_z + 0.010)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    deck = object_model.get_part("deck")
    stem_lower = object_model.get_part("stem_lower")
    stem_upper = object_model.get_part("stem_upper")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    rear_fender = object_model.get_part("rear_fender")

    fold_hinge = object_model.get_articulation("deck_to_stem_fold")
    telescope = object_model.get_articulation("stem_lower_to_stem_upper")
    front_spin = object_model.get_articulation("stem_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")

    fold_limits = fold_hinge.motion_limits
    telescope_limits = telescope.motion_limits

    ctx.check(
        "all primary scooter parts exist",
        all(part is not None for part in (deck, stem_lower, stem_upper, front_wheel, rear_wheel, rear_fender)),
        details="Missing one or more prompt-critical parts.",
    )
    ctx.check(
        "fold hinge is a deck-neck revolute joint",
        fold_hinge.articulation_type == ArticulationType.REVOLUTE
        and fold_hinge.axis == (0.0, 1.0, 0.0)
        and fold_limits is not None
        and fold_limits.lower == 0.0
        and fold_limits.upper is not None
        and fold_limits.upper >= 1.20,
        details=f"type={fold_hinge.articulation_type}, axis={fold_hinge.axis}, limits={fold_limits}",
    )
    ctx.check(
        "stem telescopes upward along its own axis",
        telescope.articulation_type == ArticulationType.PRISMATIC
        and telescope.axis == (0.0, 0.0, 1.0)
        and telescope_limits is not None
        and telescope_limits.upper is not None
        and telescope_limits.upper >= 0.16,
        details=f"type={telescope.articulation_type}, axis={telescope.axis}, limits={telescope_limits}",
    )
    ctx.check(
        "both wheels are continuous spin joints",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_spin.axis == (0.0, 1.0, 0.0)
        and rear_spin.axis == (0.0, 1.0, 0.0),
        details=f"front={front_spin.articulation_type, front_spin.axis}, rear={rear_spin.articulation_type, rear_spin.axis}",
    )

    ctx.allow_overlap(
        stem_lower,
        stem_upper,
        reason="The telescoping inner mast is intentionally represented as sliding inside the outer stem sleeve proxy.",
    )

    ctx.expect_contact(
        stem_lower,
        deck,
        elem_a="hinge_barrel",
        elem_b="hinge_lug_left",
        name="fold hinge barrel bears on the left deck lug",
    )
    ctx.expect_contact(
        stem_lower,
        deck,
        elem_a="hinge_barrel",
        elem_b="hinge_lug_right",
        name="fold hinge barrel bears on the right deck lug",
    )
    ctx.expect_contact(
        rear_fender,
        deck,
        elem_a="mount_block",
        elem_b="rear_fender_mount",
        name="rear fender mount sits on the deck pad",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="axle_shaft",
        elem_b="rear_dropout_left",
        name="rear axle shaft seats in the left dropout",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="axle_shaft",
        elem_b="rear_dropout_right",
        name="rear axle shaft seats in the right dropout",
    )
    ctx.expect_gap(
        rear_fender,
        rear_wheel,
        axis="z",
        positive_elem="brake_pad",
        negative_elem="tire",
        min_gap=0.002,
        max_gap=0.018,
        name="rear brake pad hovers just above the rear tire",
    )

    rest_upper_pos = ctx.part_world_position(stem_upper)
    with ctx.pose({telescope: telescope_limits.upper}):
        extended_upper_pos = ctx.part_world_position(stem_upper)
    ctx.check(
        "telescoping stem raises the handlebar assembly",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.12,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    with ctx.pose({fold_hinge: fold_limits.upper}):
        folded_upper_pos = ctx.part_world_position(stem_upper)
    ctx.check(
        "fold hinge lowers the stem toward the deck",
        rest_upper_pos is not None
        and folded_upper_pos is not None
        and folded_upper_pos[2] < rest_upper_pos[2] - 0.18
        and folded_upper_pos[0] > rest_upper_pos[0] + 0.18,
        details=f"rest={rest_upper_pos}, folded={folded_upper_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
