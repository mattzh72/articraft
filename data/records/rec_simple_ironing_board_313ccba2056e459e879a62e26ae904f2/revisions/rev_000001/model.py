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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _board_profile(scale_xy: float = 1.0) -> list[tuple[float, float]]:
    control = [
        (-0.73, 0.00),
        (-0.69, 0.18),
        (-0.47, 0.20),
        (-0.12, 0.19),
        (0.24, 0.17),
        (0.51, 0.12),
        (0.68, 0.06),
        (0.73, 0.00),
        (0.68, -0.06),
        (0.51, -0.12),
        (0.24, -0.17),
        (-0.12, -0.19),
        (-0.47, -0.20),
        (-0.69, -0.18),
    ]
    return [
        (x * scale_xy, y * scale_xy)
        for x, y in sample_catmull_rom_spline_2d(
            control,
            samples_per_segment=8,
            closed=True,
        )
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_ironing_board")

    deck_gray = model.material("deck_gray", rgba=(0.74, 0.76, 0.77, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.37, 0.39, 1.0))
    plated = model.material("plated", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    cover_fabric = model.material("cover_fabric", rgba=(0.82, 0.84, 0.86, 1.0))
    nylon = model.material("nylon", rgba=(0.86, 0.87, 0.82, 1.0))

    deck = model.part("board_deck")
    deck_profile = _board_profile()
    pad_profile = _board_profile(scale_xy=0.97)
    deck.visual(
        _mesh("ironing_board_deck_panel", ExtrudeGeometry(deck_profile, 0.024)),
        material=deck_gray,
        name="deck_panel",
    )
    deck.visual(
        Box((0.34, 0.050, 0.050)),
        origin=Origin(xyz=(0.00, 0.145, -0.037)),
        material=steel,
        name="left_channel",
    )
    deck.visual(
        Box((0.34, 0.050, 0.050)),
        origin=Origin(xyz=(0.00, -0.145, -0.037)),
        material=steel,
        name="right_channel",
    )
    deck.visual(
        Box((0.20, 0.24, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.0345)),
        material=steel,
        name="center_crossmember",
    )
    deck.visual(
        Box((0.060, 0.340, 0.038)),
        origin=Origin(xyz=(0.26, 0.0, -0.031)),
        material=plated,
        name="front_hinge_saddle",
    )
    deck.visual(
        Box((0.060, 0.220, 0.038)),
        origin=Origin(xyz=(-0.26, 0.0, -0.031)),
        material=plated,
        name="rear_hinge_saddle",
    )
    deck.visual(
        Box((0.20, 0.050, 0.018)),
        origin=Origin(xyz=(-0.18, 0.180, -0.024)),
        material=steel,
        name="lock_mount_pad",
    )
    deck.inertial = Inertial.from_geometry(
        Box((1.46, 0.40, 0.10)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    cover = model.part("cover_pad")
    cover.visual(
        _mesh("ironing_board_cover_pad", ExtrudeGeometry(pad_profile, 0.010)),
        material=cover_fabric,
        name="pad_surface",
    )
    cover.inertial = Inertial.from_geometry(
        Box((1.42, 0.38, 0.010)),
        mass=0.7,
    )

    front_leg = model.part("front_leg_frame")
    front_leg.visual(
        _mesh(
            "ironing_board_front_leg_loop",
            tube_from_spline_points(
                [
                    (0.0, 0.15, 0.0),
                    (-0.20, 0.15, -0.40),
                    (-0.44, 0.15, -0.88),
                    (-0.44, -0.15, -0.88),
                    (-0.20, -0.15, -0.40),
                    (0.0, -0.15, 0.0),
                ],
                radius=0.014,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=steel,
        name="main_loop",
    )
    front_leg.visual(
        Cylinder(radius=0.016, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plated,
        name="top_pivot_tube",
    )
    front_leg.visual(
        Box((0.06, 0.32, 0.032)),
        origin=Origin(xyz=(-0.01, 0.0, -0.028)),
        material=steel,
        name="upper_yoke",
    )
    front_leg.visual(
        Box((0.14, 0.34, 0.03)),
        origin=Origin(xyz=(-0.34, 0.0, -0.74)),
        material=steel,
        name="lower_service_rung",
    )
    front_leg.visual(
        Box((0.05, 0.30, 0.09)),
        origin=Origin(xyz=(-0.18, 0.0, -0.43)),
        material=plated,
        name="cross_bracket",
    )
    front_leg.visual(
        Box((0.05, 0.08, 0.07)),
        origin=Origin(xyz=(-0.14, 0.180, -0.395)),
        material=plated,
        name="brace_pivot_lug",
    )
    front_leg.visual(
        Box((0.055, 0.055, 0.032)),
        origin=Origin(xyz=(-0.44, 0.15, -0.888)),
        material=rubber,
        name="front_left_foot",
    )
    front_leg.visual(
        Box((0.055, 0.055, 0.032)),
        origin=Origin(xyz=(-0.44, -0.15, -0.888)),
        material=rubber,
        name="front_right_foot",
    )
    front_leg.inertial = Inertial.from_geometry(
        Box((0.56, 0.34, 0.94)),
        mass=4.4,
        origin=Origin(xyz=(-0.22, 0.0, -0.46)),
    )

    rear_leg = model.part("rear_leg_frame")
    rear_leg.visual(
        _mesh(
            "ironing_board_rear_leg_loop",
            tube_from_spline_points(
                [
                    (0.0, 0.10, 0.0),
                    (0.20, 0.10, -0.40),
                    (0.44, 0.10, -0.88),
                    (0.44, -0.10, -0.88),
                    (0.20, -0.10, -0.40),
                    (0.0, -0.10, 0.0),
                ],
                radius=0.014,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=steel,
        name="main_loop",
    )
    rear_leg.visual(
        Cylinder(radius=0.016, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plated,
        name="top_pivot_tube",
    )
    rear_leg.visual(
        Box((0.06, 0.20, 0.032)),
        origin=Origin(xyz=(0.01, 0.0, -0.028)),
        material=steel,
        name="upper_yoke",
    )
    rear_leg.visual(
        Box((0.14, 0.24, 0.03)),
        origin=Origin(xyz=(0.34, 0.0, -0.74)),
        material=steel,
        name="lower_spreader",
    )
    rear_leg.visual(
        Box((0.26, 0.20, 0.09)),
        origin=Origin(xyz=(0.34, 0.0, -0.43)),
        material=plated,
        name="cross_collar",
    )
    rear_leg.visual(
        Box((0.055, 0.055, 0.032)),
        origin=Origin(xyz=(0.44, 0.10, -0.888)),
        material=rubber,
        name="rear_left_foot",
    )
    rear_leg.visual(
        Box((0.055, 0.055, 0.032)),
        origin=Origin(xyz=(0.44, -0.10, -0.888)),
        material=rubber,
        name="rear_right_foot",
    )
    rear_leg.inertial = Inertial.from_geometry(
        Box((0.56, 0.24, 0.94)),
        mass=4.2,
        origin=Origin(xyz=(0.22, 0.0, -0.46)),
    )

    cross_shaft = model.part("cross_shaft")
    cross_shaft.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plated,
        name="cross_pin",
    )
    cross_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.10),
        mass=0.2,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    lock_rail = model.part("lock_rail")
    lock_rail.visual(
        Box((0.04, 0.016, 0.040)),
        origin=Origin(xyz=(-0.08, 0.0, -0.020)),
        material=plated,
        name="front_hanger",
    )
    lock_rail.visual(
        Box((0.04, 0.016, 0.040)),
        origin=Origin(xyz=(0.08, 0.0, -0.020)),
        material=plated,
        name="rear_hanger",
    )
    lock_rail.visual(
        Box((0.20, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=steel,
        name="rail_beam",
    )
    lock_rail.visual(
        Box((0.028, 0.016, 0.024)),
        origin=Origin(xyz=(-0.005, 0.0, -0.070)),
        material=steel,
        name="tooth_0",
    )
    lock_rail.visual(
        Box((0.020, 0.016, 0.024)),
        origin=Origin(xyz=(0.085, 0.0, -0.070)),
        material=steel,
        name="engaged_tooth",
    )
    lock_rail.inertial = Inertial.from_geometry(
        Box((0.22, 0.02, 0.10)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    lock_brace = model.part("lock_brace")
    brace_path = tube_from_spline_points(
        [
            (0.009, 0.0, 0.025),
            (-0.040, 0.0, 0.090),
            (-0.095, 0.0, 0.170),
            (-0.155, 0.0, 0.255),
            (-0.205, 0.0, 0.332),
        ],
        radius=0.008,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    lock_brace.visual(
        Box((0.018, 0.026, 0.050)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=plated,
        name="lower_block",
    )
    lock_brace.visual(
        _mesh("ironing_board_lock_brace_bar", brace_path),
        material=steel,
        name="brace_bar",
    )
    lock_brace.visual(
        Box((0.020, 0.016, 0.024)),
        origin=Origin(xyz=(-0.215, 0.0, 0.332)),
        material=nylon,
        name="brace_tip",
    )
    lock_brace.visual(
        Box((0.05, 0.018, 0.010)),
        origin=Origin(xyz=(-0.135, 0.0, 0.220)),
        material=plated,
        name="release_tab",
    )
    lock_brace.inertial = Inertial.from_geometry(
        Box((0.26, 0.03, 0.40)),
        mass=0.8,
        origin=Origin(xyz=(-0.11, 0.0, 0.18)),
    )

    model.articulation(
        "deck_to_cover",
        ArticulationType.FIXED,
        parent=deck,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )
    model.articulation(
        "deck_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_leg,
        origin=Origin(xyz=(0.26, 0.0, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "deck_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_leg,
        origin=Origin(xyz=(-0.26, 0.0, -0.064)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "front_leg_to_cross_shaft",
        ArticulationType.FIXED,
        parent=front_leg,
        child=cross_shaft,
        origin=Origin(xyz=(-0.18, 0.0, -0.43)),
    )
    model.articulation(
        "deck_to_lock_rail",
        ArticulationType.FIXED,
        parent=deck,
        child=lock_rail,
        origin=Origin(xyz=(-0.18, 0.180, -0.033)),
    )
    model.articulation(
        "front_leg_to_lock_brace",
        ArticulationType.REVOLUTE,
        parent=front_leg,
        child=lock_brace,
        origin=Origin(xyz=(-0.14, 0.180, -0.395)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.30, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("board_deck")
    cover = object_model.get_part("cover_pad")
    front_leg = object_model.get_part("front_leg_frame")
    rear_leg = object_model.get_part("rear_leg_frame")
    cross_shaft = object_model.get_part("cross_shaft")
    lock_rail = object_model.get_part("lock_rail")
    lock_brace = object_model.get_part("lock_brace")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        front_leg,
        rear_leg,
        elem_a="cross_bracket",
        elem_b="cross_collar",
        reason="The front fork bracket wraps around the rear collar at the scissor pivot; the solids proxy the slotted hardware as nested stock.",
    )
    ctx.allow_overlap(
        front_leg,
        cross_shaft,
        elem_a="cross_bracket",
        elem_b="cross_pin",
        reason="The serviceable cross pin runs through the front scissor bracket bore.",
    )
    ctx.allow_overlap(
        rear_leg,
        cross_shaft,
        elem_a="cross_collar",
        elem_b="cross_pin",
        reason="The same cross pin passes through the rear collar bore.",
    )
    ctx.allow_overlap(
        front_leg,
        lock_brace,
        elem_a="brace_pivot_lug",
        elem_b="lower_block",
        reason="Brace hinge block nests inside the front-leg clevis lug as a serviceable pivot.",
    )
    ctx.allow_overlap(
        front_leg,
        lock_brace,
        elem_a="brace_pivot_lug",
        elem_b="brace_bar",
        reason="The first segment of the brace bar sits inside the lug slot near the hinge pivot.",
    )
    ctx.allow_overlap(
        lock_rail,
        lock_brace,
        elem_a="engaged_tooth",
        elem_b="brace_bar",
        reason="The lock tooth nests into the brace hook profile to hold the board open.",
    )
    ctx.allow_overlap(
        lock_rail,
        lock_brace,
        elem_a="engaged_tooth",
        elem_b="brace_tip",
        reason="The plastic wear tip captures the engaged tooth in the open stance.",
    )

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
        cover,
        deck,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="pad_surface",
        negative_elem="deck_panel",
        name="cover_pad_seats_on_deck",
    )
    ctx.expect_overlap(
        cover,
        deck,
        axes="xy",
        min_overlap=0.25,
        elem_a="pad_surface",
        elem_b="deck_panel",
        name="cover_pad_spans_work_surface",
    )
    ctx.expect_contact(
        front_leg,
        deck,
        elem_a="top_pivot_tube",
        elem_b="front_hinge_saddle",
        contact_tol=0.002,
        name="front_leg_hinge_contacts_deck_saddle",
    )
    ctx.expect_contact(
        rear_leg,
        deck,
        elem_a="top_pivot_tube",
        elem_b="rear_hinge_saddle",
        contact_tol=0.002,
        name="rear_leg_hinge_contacts_deck_saddle",
    )
    ctx.expect_overlap(
        front_leg,
        rear_leg,
        axes="yz",
        min_overlap=0.04,
        elem_a="cross_bracket",
        elem_b="cross_collar",
        name="scissor_pivot_brackets_share_real_capture_area",
    )
    ctx.expect_overlap(
        cross_shaft,
        front_leg,
        axes="yz",
        min_overlap=0.02,
        elem_a="cross_pin",
        elem_b="cross_bracket",
        name="cross_pin_passes_through_front_bracket",
    )
    ctx.expect_overlap(
        cross_shaft,
        rear_leg,
        axes="yz",
        min_overlap=0.02,
        elem_a="cross_pin",
        elem_b="cross_collar",
        name="cross_pin_passes_through_rear_collar",
    )
    ctx.expect_overlap(
        lock_brace,
        front_leg,
        axes="yz",
        min_overlap=0.02,
        elem_a="lower_block",
        elem_b="brace_pivot_lug",
        name="lock_brace_is_captured_by_clevis_lug",
    )
    ctx.expect_contact(
        lock_brace,
        front_leg,
        elem_a="brace_bar",
        elem_b="brace_pivot_lug",
        contact_tol=0.001,
        name="brace_bar_starts_inside_lug_slot",
    )
    ctx.expect_contact(
        lock_rail,
        deck,
        elem_a="front_hanger",
        elem_b="lock_mount_pad",
        contact_tol=0.001,
        name="lock_rail_hangs_from_service_pad",
    )
    ctx.expect_overlap(
        lock_brace,
        lock_rail,
        axes="y",
        min_overlap=0.015,
        elem_a="brace_tip",
        elem_b="engaged_tooth",
        name="lock_tip_captures_engaged_tooth",
    )
    ctx.expect_overlap(
        lock_brace,
        lock_rail,
        axes="y",
        min_overlap=0.006,
        elem_a="brace_bar",
        elem_b="engaged_tooth",
        name="brace_bar_bears_on_lock_tooth",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
