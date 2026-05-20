from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
)


def _translated_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _scaled_profile(profile, sx: float = 1.0, sy: float = 1.0):
    return [(x * sx, y * sy) for x, y in profile]


def _deck_outline():
    half = [
        (-0.61, -0.18),
        (-0.30, -0.18),
        (0.00, -0.16),
        (0.30, -0.11),
        (0.50, -0.05),
        (0.61, 0.00),
    ]
    controls = half + [(x, -y) for x, y in reversed(half[:-1])]
    return sample_catmull_rom_spline_2d(
        controls,
        samples_per_segment=8,
        closed=True,
    )


def _bar_origin(start, end):
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    return Origin(
        xyz=(
            0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]),
            0.5 * (start[2] + end[2]),
        ),
        rpy=(0.0, atan2(-dz, dx), 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_ironing_board")

    steel = model.material("powder_coat_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    cover = model.material("cover_fabric", rgba=(0.73, 0.80, 0.88, 1.0))
    plastic = model.material("plastic_trim", rgba=(0.17, 0.17, 0.18, 1.0))

    deck_outline = _deck_outline()
    deck_pad_outline = _scaled_profile(deck_outline, sx=0.965, sy=0.94)

    vent_holes = [
        _translated_profile(rounded_rect_profile(0.110, 0.026, 0.008), dx=-0.28),
        _translated_profile(rounded_rect_profile(0.120, 0.028, 0.009), dx=-0.10),
        _translated_profile(rounded_rect_profile(0.112, 0.026, 0.008), dx=0.09),
        _translated_profile(rounded_rect_profile(0.090, 0.022, 0.007), dx=0.28),
        _translated_profile(rounded_rect_profile(0.060, 0.018, 0.006), dx=0.45),
    ]

    deck_shell = mesh_from_geometry(
        ExtrudeWithHolesGeometry(deck_outline, vent_holes, 0.0012, center=False),
        "deck_shell",
    )
    cover_pad = mesh_from_geometry(
        ExtrudeGeometry.from_z0(deck_pad_outline, 0.006),
        "cover_pad",
    )

    front_leg_length = sqrt(0.26 * 0.26 + 0.82 * 0.82)
    rear_leg_length = sqrt(0.64 * 0.64 + 0.834 * 0.834)
    lock_bar_length = sqrt(0.13 * 0.13 + 0.285 * 0.285)

    deck = model.part("deck")
    deck.visual(deck_shell, material=steel, name="top_shell")
    deck.visual(cover_pad, origin=Origin(xyz=(0.0, 0.0, 0.0012)), material=cover, name="cover")
    deck.visual(
        Box((0.62, 0.016, 0.016)),
        origin=Origin(xyz=(-0.08, -0.110, -0.008)),
        material=dark_steel,
        name="left_rail",
    )
    deck.visual(
        Box((0.62, 0.016, 0.016)),
        origin=Origin(xyz=(-0.08, 0.110, -0.008)),
        material=dark_steel,
        name="right_rail",
    )
    deck.visual(
        Box((0.040, 0.026, 0.015)),
        origin=Origin(xyz=(-0.22, -0.16, -0.0075)),
        material=dark_steel,
        name="left_pivot_bracket",
    )
    deck.visual(
        Box((0.040, 0.026, 0.015)),
        origin=Origin(xyz=(-0.22, 0.16, -0.0075)),
        material=dark_steel,
        name="right_pivot_bracket",
    )
    deck.visual(
        Box((0.038, 0.024, 0.015)),
        origin=Origin(xyz=(0.22, -0.115, -0.0075)),
        material=plastic,
        name="left_guide_bracket",
    )
    deck.visual(
        Box((0.038, 0.024, 0.015)),
        origin=Origin(xyz=(0.22, 0.115, -0.0075)),
        material=plastic,
        name="right_guide_bracket",
    )
    deck.visual(
        Box((0.18, 0.018, 0.010)),
        origin=Origin(xyz=(0.030, 0.145, -0.005)),
        material=dark_steel,
        name="lock_rack",
    )

    front_leg = model.part("front_leg_frame")
    front_leg.visual(
        Box((front_leg_length, 0.014, 0.020)),
        origin=_bar_origin((0.0, -0.160, 0.0), (0.26, -0.160, -0.82)),
        material=steel,
        name="left_leg_tube",
    )
    front_leg.visual(
        Box((front_leg_length, 0.014, 0.020)),
        origin=_bar_origin((0.0, 0.160, 0.0), (0.26, 0.160, -0.82)),
        material=steel,
        name="right_leg_tube",
    )
    front_leg.visual(
        Box((0.040, 0.350, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, -0.055)),
        material=dark_steel,
        name="top_tie",
    )
    front_leg.visual(
        Box((0.028, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, -0.160, 0.005)),
        material=dark_steel,
        name="left_pivot_cap",
    )
    front_leg.visual(
        Box((0.028, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.160, 0.005)),
        material=dark_steel,
        name="right_pivot_cap",
    )
    front_leg.visual(
        Box((0.030, 0.360, 0.020)),
        origin=Origin(xyz=(0.260, 0.0, -0.820)),
        material=plastic,
        name="foot_bar",
    )
    front_leg.visual(
        Box((0.024, 0.040, 0.028)),
        origin=Origin(xyz=(0.120, 0.135, -0.410)),
        material=dark_steel,
        name="hinge_plate",
    )
    front_leg.visual(
        Box((0.016, 0.020, 0.130)),
        origin=Origin(xyz=(0.120, 0.145, -0.340)),
        material=dark_steel,
        name="lock_support",
    )
    front_leg.visual(
        Box((lock_bar_length, 0.010, 0.014)),
        origin=_bar_origin((0.120, 0.145, -0.280), (0.250, 0.145, 0.005)),
        material=dark_steel,
        name="lock_bar",
    )
    front_leg.visual(
        Box((0.024, 0.014, 0.020)),
        origin=Origin(xyz=(0.250, 0.145, 0.005)),
        material=dark_steel,
        name="lock_hook",
    )

    rear_leg = model.part("rear_leg_frame")
    rear_leg.visual(
        Box((rear_leg_length, 0.014, 0.020)),
        origin=_bar_origin((0.32, -0.095, 0.414), (-0.32, -0.095, -0.420)),
        material=steel,
        name="left_leg_tube",
    )
    rear_leg.visual(
        Box((rear_leg_length, 0.014, 0.020)),
        origin=_bar_origin((0.32, 0.095, 0.414), (-0.32, 0.095, -0.420)),
        material=steel,
        name="right_leg_tube",
    )
    rear_leg.visual(
        Box((0.030, 0.230, 0.018)),
        origin=Origin(xyz=(0.320, 0.0, 0.414)),
        material=dark_steel,
        name="top_tie",
    )
    rear_leg.visual(
        Box((0.030, 0.230, 0.020)),
        origin=Origin(xyz=(-0.320, 0.0, -0.420)),
        material=plastic,
        name="foot_bar",
    )
    rear_leg.visual(
        Box((0.024, 0.080, 0.028)),
        origin=Origin(xyz=(0.0, 0.075, 0.0)),
        material=dark_steel,
        name="hinge_bridge",
    )
    rear_leg.visual(
        Box((0.034, 0.024, 0.012)),
        origin=Origin(xyz=(0.320, -0.115, 0.414)),
        material=plastic,
        name="left_slider_block",
    )
    rear_leg.visual(
        Box((0.034, 0.024, 0.012)),
        origin=Origin(xyz=(0.320, 0.115, 0.414)),
        material=plastic,
        name="right_slider_block",
    )

    model.articulation(
        "deck_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_leg,
        origin=Origin(xyz=(-0.22, 0.0, -0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-1.10,
            upper=0.08,
        ),
    )
    model.articulation(
        "front_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=front_leg,
        child=rear_leg,
        origin=Origin(xyz=(0.12, 0.0, -0.41)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_leg = object_model.get_part("front_leg_frame")
    rear_leg = object_model.get_part("rear_leg_frame")
    front_joint = object_model.get_articulation("deck_to_front_leg")
    rear_joint = object_model.get_articulation("front_to_rear_leg")

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
        "ironing_board_parts_present",
        all(part is not None for part in (deck, front_leg, rear_leg)),
        "Deck and both scissor leg frames must be authored as real parts.",
    )
    ctx.check(
        "front_leg_axis_is_pitch_axis",
        tuple(front_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected front hinge axis (0, 1, 0), got {front_joint.axis}.",
    )
    ctx.check(
        "front_leg_limits_allow_folding",
        (
            front_joint.motion_limits is not None
            and front_joint.motion_limits.lower is not None
            and front_joint.motion_limits.upper is not None
            and front_joint.motion_limits.lower < 0.0 <= front_joint.motion_limits.upper
        ),
        "Front leg hinge should open at rest and fold upward with negative travel.",
    )
    ctx.check(
        "rear_leg_limits_form_scissor",
        (
            rear_joint.motion_limits is not None
            and rear_joint.motion_limits.lower is not None
            and rear_joint.motion_limits.upper is not None
            and rear_joint.motion_limits.lower == 0.0
            and rear_joint.motion_limits.upper >= 1.0
        ),
        "Rear frame should close the scissor by rotating positively from the open stance.",
    )

    with ctx.pose({front_joint: 0.0, rear_joint: 0.0}):
        ctx.expect_contact(
            front_leg,
            deck,
            elem_a="left_pivot_cap",
            elem_b="left_pivot_bracket",
            name="left_pivot_cap_seated",
        )
        ctx.expect_contact(
            front_leg,
            deck,
            elem_a="right_pivot_cap",
            elem_b="right_pivot_bracket",
            name="right_pivot_cap_seated",
        )
        ctx.expect_contact(
            rear_leg,
            deck,
            elem_a="left_slider_block",
            elem_b="left_guide_bracket",
            name="left_slider_block_supported",
        )
        ctx.expect_contact(
            rear_leg,
            deck,
            elem_a="right_slider_block",
            elem_b="right_guide_bracket",
            name="right_slider_block_supported",
        )
        ctx.expect_contact(
            rear_leg,
            front_leg,
            elem_a="hinge_bridge",
            elem_b="hinge_plate",
            name="scissor_hinge_bridge_contact",
        )
        ctx.expect_contact(
            front_leg,
            deck,
            elem_a="lock_hook",
            elem_b="lock_rack",
            name="lock_hook_resting_on_rack",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
