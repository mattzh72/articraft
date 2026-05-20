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
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xz_section(
    *,
    y: float,
    width: float,
    height: float,
    z_center: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for x, z in rounded_rect_profile(width, height, radius, corner_segments=6)]


def _wheel_profile(radius: float, half_width: float) -> list[tuple[float, float]]:
    return [
        (radius * 0.54, -half_width * 0.96),
        (radius * 0.78, -half_width),
        (radius * 0.92, -half_width * 0.78),
        (radius, -half_width * 0.24),
        (radius, half_width * 0.24),
        (radius * 0.92, half_width * 0.78),
        (radius * 0.78, half_width),
        (radius * 0.54, half_width * 0.96),
        (radius * 0.47, half_width * 0.26),
        (radius * 0.44, 0.0),
        (radius * 0.47, -half_width * 0.26),
        (radius * 0.54, -half_width * 0.96),
    ]


def _add_wheel_visuals(
    part,
    mesh_name: str,
    *,
    radius: float,
    tire_width: float,
    hub_width: float,
    rubber,
    rim,
    core,
) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    half_width = tire_width * 0.5
    tire_mesh = _mesh(
        f"{mesh_name}_tire",
        LatheGeometry(_wheel_profile(radius, half_width), segments=56).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=radius * 0.70, length=0.005),
        origin=Origin(xyz=(hub_width * 0.5 - 0.0025, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim,
        name="left_rim",
    )
    part.visual(
        Cylinder(radius=radius * 0.70, length=0.005),
        origin=Origin(xyz=(-hub_width * 0.5 + 0.0025, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim,
        name="right_rim",
    )
    part.visual(Cylinder(radius=radius * 0.56, length=hub_width), origin=spin_origin, material=core, name="hub")
    part.visual(
        Cylinder(radius=radius * 0.17, length=hub_width * 0.96),
        origin=spin_origin,
        material=rim,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="children_kick_scooter")

    body_teal = model.material("body_teal", rgba=(0.11, 0.67, 0.72, 1.0))
    body_teal_dark = model.material("body_teal_dark", rgba=(0.07, 0.48, 0.53, 1.0))
    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.54, 0.56, 0.60, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.58, 0.17, 0.09)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.03, 0.078)),
    )
    deck_shell = section_loft(
        [
            _xz_section(y=-0.295, width=0.100, height=0.030, z_center=0.074, radius=0.009),
            _xz_section(y=-0.210, width=0.126, height=0.035, z_center=0.076, radius=0.011),
            _xz_section(y=-0.060, width=0.142, height=0.036, z_center=0.077, radius=0.012),
            _xz_section(y=0.090, width=0.140, height=0.034, z_center=0.078, radius=0.012),
            _xz_section(y=0.165, width=0.112, height=0.032, z_center=0.079, radius=0.011),
            _xz_section(y=0.190, width=0.078, height=0.030, z_center=0.082, radius=0.009),
        ]
    )
    deck.visual(_mesh("deck_shell", deck_shell), material=body_teal, name="deck_shell")
    deck.visual(
        Box((0.114, 0.328, 0.003)),
        origin=Origin(xyz=(0.0, -0.048, 0.094)),
        material=charcoal,
        name="grip_tape",
    )
    deck.visual(
        Box((0.052, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, 0.183, 0.086)),
        material=body_teal_dark,
        name="nose_block",
    )
    deck.visual(
        Box((0.028, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, 0.217, 0.090)),
        material=body_teal_dark,
        name="pivot_neck",
    )
    deck.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.0, 0.256, 0.095), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lean_housing",
    )
    deck.visual(
        Box((0.012, 0.150, 0.048)),
        origin=Origin(xyz=(0.022, -0.236, 0.079)),
        material=body_teal_dark,
        name="left_rear_fork",
    )
    deck.visual(
        Box((0.012, 0.150, 0.048)),
        origin=Origin(xyz=(-0.022, -0.236, 0.079)),
        material=body_teal_dark,
        name="right_rear_fork",
    )
    deck.visual(
        Box((0.058, 0.082, 0.012)),
        origin=Origin(xyz=(0.0, -0.258, 0.105), rpy=(0.55, 0.0, 0.0)),
        material=body_teal_dark,
        name="rear_brake_fender",
    )

    front_assembly = model.part("front_assembly")
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.42, 0.22, 0.72)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )
    front_assembly.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_pivot_washer",
    )
    front_assembly.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_pivot_washer",
    )
    front_assembly.visual(
        Box((0.018, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, -0.037, 0.025)),
        material=steel,
        name="rear_pivot_upright",
    )
    front_assembly.visual(
        Box((0.018, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, 0.037, 0.025)),
        material=steel,
        name="front_pivot_upright",
    )
    front_assembly.visual(
        Box((0.042, 0.076, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=aluminum,
        name="steering_head_block",
    )
    front_assembly.visual(
        _mesh(
            "stem_tube",
            tube_from_spline_points(
                [
                    (0.0, -0.010, 0.061),
                    (0.0, -0.028, 0.255),
                    (0.0, -0.050, 0.520),
                    (0.0, -0.058, 0.612),
                ],
                radius=0.018,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=aluminum,
        name="stem_tube",
    )
    front_assembly.visual(
        _mesh(
            "lower_yoke",
            tube_from_spline_points(
                [
                    (0.0, 0.018, 0.048),
                    (0.0, 0.044, 0.035),
                    (0.0, 0.078, 0.014),
                ],
                radius=0.015,
                samples_per_segment=10,
                radial_segments=18,
            ),
        ),
        material=aluminum,
        name="lower_yoke",
    )
    front_assembly.visual(
        Box((0.170, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.050, 0.034)),
        material=body_teal_dark,
        name="front_bogie",
    )
    front_assembly.visual(
        Box((0.016, 0.054, 0.100)),
        origin=Origin(xyz=(0.087, 0.082, -0.008)),
        material=body_teal_dark,
        name="left_front_fork",
    )
    front_assembly.visual(
        Box((0.016, 0.054, 0.100)),
        origin=Origin(xyz=(-0.087, 0.082, -0.008)),
        material=body_teal_dark,
        name="right_front_fork",
    )
    front_assembly.visual(
        Box((0.052, 0.040, 0.038)),
        origin=Origin(xyz=(0.0, -0.056, 0.628)),
        material=body_teal_dark,
        name="handlebar_clamp",
    )
    front_assembly.visual(
        _mesh(
            "handlebar_tube",
            tube_from_spline_points(
                [
                    (-0.182, -0.056, 0.624),
                    (-0.092, -0.056, 0.640),
                    (0.0, -0.056, 0.646),
                    (0.092, -0.056, 0.640),
                    (0.182, -0.056, 0.624),
                ],
                radius=0.011,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=aluminum,
        name="handlebar_tube",
    )
    front_assembly.visual(
        Cylinder(radius=0.015, length=0.094),
        origin=Origin(xyz=(0.214, -0.056, 0.620), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    front_assembly.visual(
        Cylinder(radius=0.015, length=0.094),
        origin=Origin(xyz=(-0.214, -0.056, 0.620), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.036),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        front_left_wheel,
        "front_left_wheel",
        radius=0.058,
        tire_width=0.036,
        hub_width=0.040,
        rubber=rubber,
        rim=body_teal,
        core=charcoal,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.036),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        front_right_wheel,
        "front_right_wheel",
        radius=0.058,
        tire_width=0.036,
        hub_width=0.040,
        rubber=rubber,
        rim=body_teal,
        core=charcoal,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.032),
        mass=0.50,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_wheel,
        "rear_wheel",
        radius=0.055,
        tire_width=0.032,
        hub_width=0.040,
        rubber=rubber,
        rim=body_teal,
        core=charcoal,
    )

    lean_joint = model.articulation(
        "deck_to_front_assembly",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=(0.0, 0.256, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-0.45, upper=0.45),
    )
    front_left_spin = model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_left_wheel,
        origin=Origin(xyz=(0.115, 0.110, -0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=24.0),
    )
    front_right_spin = model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_right_wheel,
        origin=Origin(xyz=(-0.115, 0.110, -0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=24.0),
    )
    rear_spin = model.articulation(
        "rear_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(0.0, -0.350, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=24.0),
    )

    deck.meta["primary_lean_joint"] = lean_joint.name
    front_assembly.meta["front_track_m"] = 0.230
    rear_wheel.meta["mounted_at_tail"] = True

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_assembly = object_model.get_part("front_assembly")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    lean_joint = object_model.get_articulation("deck_to_front_assembly")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_spin = object_model.get_articulation("rear_spin")

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
        "lean_joint_axis_is_longitudinal",
        tuple(lean_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected deck_to_front_assembly axis (0, 1, 0), got {lean_joint.axis}",
    )
    ctx.check(
        "wheel_axes_are_axles",
        tuple(front_left_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(front_right_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(rear_spin.axis) == (1.0, 0.0, 0.0),
        details=(
            f"expected all wheel axes to run along +x, got "
            f"{front_left_spin.axis}, {front_right_spin.axis}, {rear_spin.axis}"
        ),
    )

    ctx.expect_contact(front_assembly, deck, contact_tol=0.0015, name="front_assembly_mounted_to_deck")
    ctx.expect_contact(front_left_wheel, front_assembly, contact_tol=0.0015, name="left_front_wheel_on_bogie")
    ctx.expect_contact(front_right_wheel, front_assembly, contact_tol=0.0015, name="right_front_wheel_on_bogie")
    ctx.expect_contact(rear_wheel, deck, contact_tol=0.0015, name="rear_wheel_on_tail_fork")

    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="x",
        min_dist=0.22,
        max_dist=0.24,
        name="front_track_width",
    )

    with ctx.pose({lean_joint: 0.0}):
        ctx.expect_gap(
            front_left_wheel,
            deck,
            axis="y",
            min_gap=0.015,
            name="left_front_wheel_ahead_of_deck",
        )
        ctx.expect_gap(
            front_right_wheel,
            deck,
            axis="y",
            min_gap=0.015,
            name="right_front_wheel_ahead_of_deck",
        )

    with ctx.pose({lean_joint: 0.0}):
        rest_left = ctx.part_world_position(front_left_wheel)
    with ctx.pose({lean_joint: 0.32}):
        leaned_left = ctx.part_world_position(front_left_wheel)
    front_bogie_leans = (
        rest_left is not None
        and leaned_left is not None
        and abs(leaned_left[0] - rest_left[0]) > 0.01
        and abs(leaned_left[2] - rest_left[2]) > 0.02
    )
    ctx.check(
        "front_bogie_leans_as_one_unit",
        front_bogie_leans,
        details=f"rest wheel center={rest_left}, leaned wheel center={leaned_left}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
