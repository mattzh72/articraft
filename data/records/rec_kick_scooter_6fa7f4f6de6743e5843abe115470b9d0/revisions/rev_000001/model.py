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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _add_wheel_visuals(
    part,
    *,
    tire_radius: float,
    tire_width: float,
    rim_radius: float,
    hub_radius: float,
    hub_width: float,
    rubber,
    alloy,
    dark_metal,
) -> None:
    axle_axis = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=axle_axis,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=tire_width * 0.72),
        origin=axle_axis,
        material=alloy,
        name="rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=axle_axis,
        material=dark_metal,
        name="hub",
    )
    axle_end_length = 0.022
    axle_end_center = hub_width * 0.5 + axle_end_length * 0.5
    part.visual(
        Cylinder(radius=hub_radius * 0.45, length=axle_end_length),
        origin=Origin(xyz=(0.0, axle_end_center, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_axle_end",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.45, length=axle_end_length),
        origin=Origin(xyz=(0.0, -axle_end_center, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_axle_end",
    )
    spoke_radius = rim_radius * 0.42
    for spoke_index in range(6):
        angle = spoke_index * pi / 3.0
        part.visual(
            Box((0.010, tire_width * 0.28, rim_radius * 0.72)),
            origin=Origin(
                xyz=(cos(angle) * spoke_radius, 0.0, sin(angle) * spoke_radius),
                rpy=(0.0, angle, 0.0),
            ),
            material=alloy,
            name=f"spoke_{spoke_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commuter_kick_scooter")

    frame_silver = model.material("frame_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    frame_dark = model.material("frame_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    alloy = model.material("alloy", rgba=(0.62, 0.65, 0.70, 1.0))

    deck = model.part("deck")
    deck.visual(
        _save_mesh(
            "deck_shell",
            section_loft(
                [
                    _yz_section(-0.28, width=0.136, height=0.024, radius=0.011, z_center=0.101),
                    _yz_section(-0.10, width=0.145, height=0.028, radius=0.012, z_center=0.102),
                    _yz_section(0.06, width=0.138, height=0.026, radius=0.011, z_center=0.101),
                    _yz_section(0.18, width=0.074, height=0.020, radius=0.008, z_center=0.099),
                ]
            ),
        ),
        material=frame_silver,
        name="deck_shell",
    )
    deck.visual(
        Box((0.40, 0.118, 0.004)),
        origin=Origin(xyz=(-0.045, 0.0, 0.117)),
        material=grip_black,
        name="grip_pad",
    )
    deck.visual(
        _save_mesh(
            "head_tube_shell",
            LatheGeometry.from_shell_profiles(
                [(0.034, -0.080), (0.034, 0.080)],
                [(0.028, -0.080), (0.028, 0.080)],
                segments=40,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.233, 0.0, 0.179), rpy=(0.0, -0.30, 0.0)),
        material=frame_silver,
        name="head_tube_shell",
    )
    deck.visual(
        Box((0.088, 0.022, 0.046)),
        origin=Origin(xyz=(0.214, 0.040, 0.122), rpy=(0.0, -0.30, 0.0)),
        material=frame_dark,
        name="nose_gusset_left",
    )
    deck.visual(
        Box((0.088, 0.022, 0.046)),
        origin=Origin(xyz=(0.214, -0.040, 0.122), rpy=(0.0, -0.30, 0.0)),
        material=frame_dark,
        name="nose_gusset_right",
    )
    deck.visual(
        Box((0.024, 0.016, 0.018)),
        origin=Origin(xyz=(-0.220, 0.043, 0.086)),
        material=frame_dark,
        name="pivot_support_left",
    )
    deck.visual(
        Box((0.024, 0.016, 0.018)),
        origin=Origin(xyz=(-0.220, -0.043, 0.086)),
        material=frame_dark,
        name="pivot_support_right",
    )
    deck.visual(
        Box((0.040, 0.098, 0.010)),
        origin=Origin(xyz=(-0.220, 0.0, 0.079)),
        material=frame_dark,
        name="pivot_bridge",
    )
    deck.visual(
        Box((0.048, 0.012, 0.016)),
        origin=Origin(xyz=(-0.050, 0.076, 0.086)),
        material=frame_dark,
        name="kickstand_bracket",
    )
    deck.visual(
        Box((0.014, 0.016, 0.004)),
        origin=Origin(xyz=(-0.061, 0.085, 0.084)),
        material=rubber,
        name="kickstand_stop",
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.860),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=frame_dark,
        name="steer_stem",
    )
    front_assembly.visual(
        Cylinder(radius=0.039, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=frame_dark,
        name="lower_headset",
    )
    front_assembly.visual(
        Cylinder(radius=0.039, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=frame_dark,
        name="upper_headset",
    )
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=frame_dark,
        name="fork_spine",
    )
    front_assembly.visual(
        Box((0.040, 0.110, 0.018)),
        origin=Origin(xyz=(0.022, 0.0, -0.028)),
        material=frame_dark,
        name="fork_bridge",
    )
    front_assembly.visual(
        _save_mesh(
            "left_fork_leg",
            tube_from_spline_points(
                [(0.018, 0.045, -0.028), (0.098, 0.049, -0.040), (0.182, 0.053, -0.054)],
                radius=0.0085,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_dark,
        name="left_fork_leg",
    )
    front_assembly.visual(
        _save_mesh(
            "right_fork_leg",
            tube_from_spline_points(
                [(0.018, -0.045, -0.028), (0.098, -0.049, -0.040), (0.182, -0.053, -0.054)],
                radius=0.0085,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_dark,
        name="right_fork_leg",
    )
    front_assembly.visual(
        Box((0.014, 0.014, 0.026)),
        origin=Origin(xyz=(0.182, 0.053, -0.054)),
        material=frame_dark,
        name="left_dropout",
    )
    front_assembly.visual(
        Box((0.014, 0.014, 0.026)),
        origin=Origin(xyz=(0.182, -0.053, -0.054)),
        material=frame_dark,
        name="right_dropout",
    )
    front_assembly.visual(
        Box((0.070, 0.080, 0.032)),
        origin=Origin(xyz=(-0.010, 0.0, 0.865)),
        material=frame_dark,
        name="bar_clamp",
    )
    front_assembly.visual(
        _save_mesh(
            "handlebar",
            tube_from_spline_points(
                [
                    (-0.040, -0.280, 0.860),
                    (-0.010, -0.120, 0.882),
                    (0.0, 0.0, 0.890),
                    (-0.010, 0.120, 0.882),
                    (-0.040, 0.280, 0.860),
                ],
                radius=0.011,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=frame_dark,
        name="handlebar",
    )
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(xyz=(-0.040, 0.295, 0.860), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(xyz=(-0.040, -0.295, 0.860), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )

    front_wheel = model.part("front_wheel")
    _add_wheel_visuals(
        front_wheel,
        tire_radius=0.100,
        tire_width=0.040,
        rim_radius=0.076,
        hub_radius=0.020,
        hub_width=0.050,
        rubber=rubber,
        alloy=alloy,
        dark_metal=frame_dark,
    )

    rear_swingarm = model.part("rear_swingarm")
    rear_swingarm.visual(
        Cylinder(radius=0.014, length=0.093),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_dark,
        name="pivot_sleeve",
    )
    rear_swingarm.visual(
        _save_mesh(
            "left_swingarm_rail",
            tube_from_spline_points(
                [(0.000, 0.055, 0.000), (-0.060, 0.055, 0.010), (-0.130, 0.056, 0.024), (-0.190, 0.055, 0.040)],
                radius=0.0085,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_dark,
        name="left_swingarm_rail",
    )
    rear_swingarm.visual(
        _save_mesh(
            "right_swingarm_rail",
            tube_from_spline_points(
                [(0.000, -0.055, 0.000), (-0.060, -0.055, 0.010), (-0.130, -0.056, 0.024), (-0.190, -0.055, 0.040)],
                radius=0.0085,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_dark,
        name="right_swingarm_rail",
    )
    rear_swingarm.visual(
        Box((0.016, 0.012, 0.028)),
        origin=Origin(xyz=(-0.190, 0.053, 0.040)),
        material=frame_dark,
        name="left_rear_dropout",
    )
    rear_swingarm.visual(
        Box((0.016, 0.012, 0.028)),
        origin=Origin(xyz=(-0.190, -0.053, 0.040)),
        material=frame_dark,
        name="right_rear_dropout",
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel_visuals(
        rear_wheel,
        tire_radius=0.100,
        tire_width=0.040,
        rim_radius=0.076,
        hub_radius=0.020,
        hub_width=0.050,
        rubber=rubber,
        alloy=alloy,
        dark_metal=frame_dark,
    )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.008, length=0.082),
        origin=Origin(xyz=(-0.041, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="stand_leg",
    )
    kickstand.visual(
        Box((0.024, 0.016, 0.006)),
        origin=Origin(xyz=(-0.094, 0.0, 0.0)),
        material=frame_dark,
        name="foot_pad",
    )
    kickstand.visual(
        Box((0.010, 0.016, 0.004)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
        material=rubber,
        name="stand_bumper",
    )

    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=(0.257, 0.0, 0.103), rpy=(0.0, -0.30, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.182, 0.0, -0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_suspension",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_swingarm,
        origin=Origin(xyz=(-0.220, 0.0, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-0.18, upper=0.28),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_swingarm,
        child=rear_wheel,
        origin=Origin(xyz=(-0.190, 0.0, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "kickstand_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=(-0.045, 0.085, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-1.10, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_assembly = object_model.get_part("front_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_swingarm = object_model.get_part("rear_swingarm")
    rear_wheel = object_model.get_part("rear_wheel")
    kickstand = object_model.get_part("kickstand")

    steering = object_model.get_articulation("steering")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_suspension = object_model.get_articulation("rear_suspension")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")
    kickstand_fold = object_model.get_articulation("kickstand_fold")

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
        deck,
        front_assembly,
        elem_a="head_tube_shell",
        elem_b="lower_headset",
        reason="Lower headset bearing sits nested inside the scooter head tube.",
    )
    ctx.allow_overlap(
        deck,
        front_assembly,
        elem_a="head_tube_shell",
        elem_b="upper_headset",
        reason="Upper headset bearing sits nested inside the scooter head tube.",
    )
    ctx.allow_overlap(
        front_assembly,
        front_wheel,
        elem_a="left_fork_leg",
        elem_b="left_axle_end",
        reason="The left axle stub nests into the simplified solid fork tip at the dropout.",
    )
    ctx.allow_overlap(
        front_assembly,
        front_wheel,
        elem_a="right_fork_leg",
        elem_b="right_axle_end",
        reason="The right axle stub nests into the simplified solid fork tip at the dropout.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected_parts_present",
        {part.name for part in object_model.parts}
        == {"deck", "front_assembly", "front_wheel", "rear_swingarm", "rear_wheel", "kickstand"},
        "Model should contain the deck, steering assembly, both wheels, rear swingarm, and kickstand.",
    )
    ctx.check(
        "steering_joint_is_inclined_revolute",
        steering.articulation_type == ArticulationType.REVOLUTE
        and steering.axis == (0.0, 0.0, 1.0)
        and abs(steering.origin.rpy[1]) > 0.20,
        "Front fork should yaw on an inclined steering axis.",
    )
    ctx.check(
        "wheel_spin_joints_are_axle_aligned",
        front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_wheel_spin.axis == (0.0, 1.0, 0.0)
        and rear_wheel_spin.axis == (0.0, 1.0, 0.0),
        "Both wheels should spin continuously around transverse axle axes.",
    )
    ctx.check(
        "rear_suspension_and_kickstand_are_hinged",
        rear_suspension.articulation_type == ArticulationType.REVOLUTE
        and rear_suspension.axis == (0.0, 1.0, 0.0)
        and kickstand_fold.articulation_type == ArticulationType.REVOLUTE
        and kickstand_fold.axis == (0.0, 1.0, 0.0)
        and rear_suspension.motion_limits is not None
        and kickstand_fold.motion_limits is not None
        and rear_suspension.motion_limits.upper is not None
        and kickstand_fold.motion_limits.lower is not None
        and rear_suspension.motion_limits.upper > 0.20
        and kickstand_fold.motion_limits.lower < -1.0,
        "Rear swingarm and kickstand should both have realistic folding travel.",
    )

    ctx.expect_contact(front_assembly, deck, contact_tol=0.001, name="front_assembly_contacts_head_tube")
    ctx.expect_contact(
        front_assembly,
        deck,
        elem_a="lower_headset",
        elem_b="head_tube_shell",
        contact_tol=0.001,
        name="lower_headset_seats_in_head_tube",
    )
    ctx.expect_contact(front_wheel, front_assembly, contact_tol=0.001, name="front_wheel_captured_in_fork")
    ctx.expect_contact(
        front_wheel,
        front_assembly,
        elem_a="left_axle_end",
        elem_b="left_dropout",
        contact_tol=0.001,
        name="left_axle_stub_meets_dropout",
    )
    ctx.expect_contact(
        front_wheel,
        front_assembly,
        elem_a="right_axle_end",
        elem_b="right_dropout",
        contact_tol=0.001,
        name="right_axle_stub_meets_dropout",
    )
    ctx.expect_contact(rear_swingarm, deck, contact_tol=0.001, name="rear_swingarm_captured_at_pivot")
    ctx.expect_contact(rear_wheel, rear_swingarm, contact_tol=0.001, name="rear_wheel_captured_in_swingarm")
    ctx.expect_origin_gap(front_wheel, rear_wheel, axis="x", min_gap=0.78, max_gap=0.86, name="wheelbase_in_commuter_range")
    ctx.expect_origin_gap(front_wheel, rear_wheel, axis="z", min_gap=-0.005, max_gap=0.005, name="axle_heights_match")

    with ctx.pose({kickstand_fold: 0.0}):
        ctx.expect_contact(
            kickstand,
            deck,
            elem_a="stand_bumper",
            elem_b="kickstand_stop",
            contact_tol=0.001,
            name="kickstand_stows_against_stop",
        )
        ctx.expect_gap(
            deck,
            kickstand,
            axis="z",
            positive_elem="deck_shell",
            negative_elem="foot_pad",
            min_gap=0.003,
            max_gap=0.015,
            name="stowed_kickstand_tucks_below_deck",
        )

    with ctx.pose({rear_suspension: 0.22}):
        ctx.expect_within(
            rear_swingarm,
            deck,
            axes="y",
            inner_elem="pivot_sleeve",
            outer_elem="pivot_bridge",
            margin=0.0,
            name="pivot_sleeve_stays_clipped_between_brackets",
        )
        ctx.expect_gap(
            deck,
            rear_swingarm,
            axis="z",
            positive_elem="pivot_bridge",
            negative_elem="pivot_sleeve",
            min_gap=0.0,
            max_gap=0.003,
            name="pivot_sleeve_remains_under_bridge_through_travel",
        )

    with ctx.pose({kickstand_fold: -1.05}):
        ctx.expect_gap(
            deck,
            kickstand,
            axis="z",
            positive_elem="deck_shell",
            negative_elem="foot_pad",
            min_gap=0.070,
            max_gap=0.090,
            name="deployed_kickstand_reaches_ground_plane_zone",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
