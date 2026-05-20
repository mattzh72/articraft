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


SPIN_Y = Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _tube_shell(name: str, *, outer_radius: float, inner_radius: float, length: float):
    outer_profile = [
        (outer_radius, 0.0),
        (outer_radius, length),
    ]
    inner_profile = [
        (inner_radius, 0.004),
        (inner_radius, max(0.006, length - 0.004)),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=40,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _add_wheel(part, *, tire_radius: float, tire_width: float, hub_radius: float, hub_width: float, rubber, wheel_core):
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=SPIN_Y,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.72, length=tire_width * 0.74),
        origin=SPIN_Y,
        material=wheel_core,
        name="rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=SPIN_Y,
        material=wheel_core,
        name="hub",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.42, length=hub_width * 0.92),
        origin=SPIN_Y,
        material="axle_cap",
        name="axle_cap",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=tire_radius, length=tire_width),
        mass=0.42,
        origin=SPIN_Y,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="childrens_kick_scooter")

    deck_teal = model.material("deck_teal", rgba=(0.23, 0.69, 0.72, 1.0))
    grip_orange = model.material("grip_orange", rgba=(0.95, 0.49, 0.18, 1.0))
    stem_silver = model.material("stem_silver", rgba=(0.83, 0.85, 0.88, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    axle_cap = model.material("axle_cap", rgba=(0.55, 0.57, 0.61, 1.0))

    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.34, 0.11, 0.018, corner_segments=10), 0.016, cap=True, center=True),
        "scooter_deck_board",
    )
    outer_tube_mesh = _tube_shell(
        "front_outer_tube",
        outer_radius=0.019,
        inner_radius=0.0148,
        length=0.432,
    )
    deck = model.part("deck")
    deck.visual(
        deck_mesh,
        origin=Origin(xyz=(0.01, 0.0, 0.048)),
        material=deck_teal,
        name="deck_board",
    )
    deck.visual(
        Box((0.22, 0.084, 0.003)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0575)),
        material=dark_gray,
        name="grip_tape",
    )
    deck.visual(
        Box((0.045, 0.045, 0.026)),
        origin=Origin(xyz=(0.150, 0.0, 0.061)),
        material=dark_gray,
        name="head_block",
    )
    deck.visual(
        Box((0.018, 0.026, 0.050)),
        origin=Origin(xyz=(0.185, 0.0, 0.072)),
        material=dark_gray,
        name="hinge_lug",
    )
    deck.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(0.185, 0.0, 0.072), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_core,
        name="hinge_pin",
    )
    deck.visual(
        Box((0.100, 0.008, 0.070)),
        origin=Origin(xyz=(-0.205, 0.018, 0.035)),
        material=dark_gray,
        name="rear_left_fork",
    )
    deck.visual(
        Box((0.100, 0.008, 0.070)),
        origin=Origin(xyz=(-0.205, -0.018, 0.035)),
        material=dark_gray,
        name="rear_right_fork",
    )
    deck.visual(
        Box((0.018, 0.008, 0.020)),
        origin=Origin(xyz=(-0.240, 0.018, 0.056)),
        material=wheel_core,
        name="rear_left_dropout",
    )
    deck.visual(
        Box((0.018, 0.008, 0.020)),
        origin=Origin(xyz=(-0.240, -0.018, 0.056)),
        material=wheel_core,
        name="rear_right_dropout",
    )
    deck.visual(
        Box((0.032, 0.044, 0.020)),
        origin=Origin(xyz=(-0.160, 0.0, 0.050)),
        material=dark_gray,
        name="tail_bridge",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.50, 0.12, 0.15)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Box((0.024, 0.008, 0.050)),
        origin=Origin(xyz=(0.012, 0.018, 0.008)),
        material=dark_gray,
        name="left_hinge_cheek",
    )
    front_assembly.visual(
        Box((0.024, 0.008, 0.050)),
        origin=Origin(xyz=(0.012, -0.018, 0.008)),
        material=dark_gray,
        name="right_hinge_cheek",
    )
    front_assembly.visual(
        Box((0.016, 0.008, 0.034)),
        origin=Origin(xyz=(0.008, 0.020, 0.043)),
        material=dark_gray,
        name="left_hinge_gusset",
    )
    front_assembly.visual(
        Box((0.016, 0.008, 0.034)),
        origin=Origin(xyz=(0.008, -0.020, 0.043)),
        material=dark_gray,
        name="right_hinge_gusset",
    )
    front_assembly.visual(
        Box((0.020, 0.032, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, 0.048)),
        material=dark_gray,
        name="stem_socket_bridge",
    )
    front_assembly.visual(
        Cylinder(radius=0.009, length=0.090),
        origin=Origin(xyz=(0.020, 0.0, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_orange,
        name="t_latch_bar",
    )
    front_assembly.visual(
        outer_tube_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=stem_silver,
        name="outer_tube",
    )
    front_assembly.visual(
        Box((0.030, 0.044, 0.020)),
        origin=Origin(xyz=(0.003, 0.0, 0.032)),
        material=wheel_core,
        name="fork_crown",
    )
    front_assembly.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.000, 0.018, 0.024), (0.034, 0.018, 0.006), (0.077, 0.018, -0.016)],
                radius=0.006,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
            "front_left_leg",
        ),
        material=wheel_core,
        name="front_left_leg",
    )
    front_assembly.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.000, -0.018, 0.024), (0.034, -0.018, 0.006), (0.077, -0.018, -0.016)],
                radius=0.006,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
            "front_right_leg",
        ),
        material=wheel_core,
        name="front_right_leg",
    )
    front_assembly.visual(
        Box((0.018, 0.008, 0.020)),
        origin=Origin(xyz=(0.077, 0.018, -0.016)),
        material=wheel_core,
        name="front_left_dropout",
    )
    front_assembly.visual(
        Box((0.018, 0.008, 0.020)),
        origin=Origin(xyz=(0.077, -0.018, -0.016)),
        material=wheel_core,
        name="front_right_dropout",
    )
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.18, 0.08, 0.60)),
        mass=1.0,
        origin=Origin(xyz=(0.030, 0.0, 0.230)),
    )

    upper_stem = model.part("upper_stem")
    upper_stem.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=dark_gray,
        name="height_collar",
    )
    upper_stem.visual(
        Cylinder(radius=0.013, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=stem_silver,
        name="inner_post",
    )
    upper_stem.visual(
        Box((0.034, 0.032, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.143)),
        material=dark_gray,
        name="handlebar_clamp",
    )
    upper_stem.visual(
        Cylinder(radius=0.010, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.165), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stem_silver,
        name="handlebar_bar",
    )
    upper_stem.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.0, 0.142, 0.165), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_orange,
        name="left_grip",
    )
    upper_stem.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.0, -0.142, 0.165), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_orange,
        name="right_grip",
    )
    upper_stem.inertial = Inertial.from_geometry(
        Box((0.05, 0.34, 0.46)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    front_wheel = model.part("front_wheel")
    _add_wheel(
        front_wheel,
        tire_radius=0.056,
        tire_width=0.024,
        hub_radius=0.014,
        hub_width=0.028,
        rubber=rubber,
        wheel_core=wheel_core,
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel(
        rear_wheel,
        tire_radius=0.056,
        tire_width=0.024,
        hub_radius=0.014,
        hub_width=0.028,
        rubber=rubber,
        wheel_core=wheel_core,
    )

    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=(0.185, 0.0, 0.072)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "stem_height_adjust",
        ArticulationType.PRISMATIC,
        parent=front_assembly,
        child=upper_stem,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=0.160,
        ),
    )
    model.articulation(
        "front_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.077, 0.0, -0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "rear_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.240, 0.0, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_assembly = object_model.get_part("front_assembly")
    upper_stem = object_model.get_part("upper_stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    fold_hinge = object_model.get_articulation("fold_hinge")
    stem_height_adjust = object_model.get_articulation("stem_height_adjust")
    front_wheel_axle = object_model.get_articulation("front_wheel_axle")
    rear_wheel_axle = object_model.get_articulation("rear_wheel_axle")

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
        "fold_hinge_axis",
        tuple(fold_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"expected fold hinge axis (0,-1,0), got {fold_hinge.axis}",
    )
    ctx.check(
        "stem_height_axis",
        tuple(stem_height_adjust.axis) == (0.0, 0.0, 1.0),
        details=f"expected stem height axis (0,0,1), got {stem_height_adjust.axis}",
    )
    ctx.check(
        "wheel_axles_spin_about_y",
        tuple(front_wheel_axle.axis) == (0.0, 1.0, 0.0) and tuple(rear_wheel_axle.axis) == (0.0, 1.0, 0.0),
        details=(
            f"front axle axis={front_wheel_axle.axis}, "
            f"rear axle axis={rear_wheel_axle.axis}"
        ),
    )

    ctx.expect_contact(front_assembly, deck, name="fold_hinge_contacts_deck")
    ctx.expect_contact(front_wheel, front_assembly, name="front_wheel_captured_in_fork")
    ctx.expect_contact(rear_wheel, deck, name="rear_wheel_captured_in_dropouts")
    ctx.expect_contact(
        upper_stem,
        front_assembly,
        elem_a="height_collar",
        elem_b="outer_tube",
        name="upper_stem_seats_on_outer_tube",
    )
    ctx.expect_within(
        upper_stem,
        front_assembly,
        axes="xy",
        margin=0.006,
        inner_elem="inner_post",
        outer_elem="outer_tube",
        name="inner_post_stays_centered_in_outer_tube",
    )
    ctx.expect_gap(front_wheel, deck, axis="x", min_gap=0.008, name="front_wheel_ahead_of_deck")
    ctx.expect_origin_gap(deck, rear_wheel, axis="x", min_gap=0.18, name="rear_wheel_behind_deck")
    ctx.expect_origin_distance(
        front_wheel,
        rear_wheel,
        axes="x",
        min_dist=0.48,
        max_dist=0.53,
        name="child_scooter_wheelbase",
    )

    rest_height = ctx.part_world_position(upper_stem)
    assert rest_height is not None
    with ctx.pose({stem_height_adjust: 0.160}):
        raised_height = ctx.part_world_position(upper_stem)
        assert raised_height is not None
        ctx.check(
            "stem_telescopes_upward",
            raised_height[2] > rest_height[2] + 0.150,
            details=f"rest={rest_height}, raised={raised_height}",
        )
        ctx.expect_within(
            upper_stem,
            front_assembly,
            axes="xy",
            margin=0.006,
            inner_elem="inner_post",
            outer_elem="outer_tube",
            name="inner_post_remains_aligned_when_raised",
        )

    with ctx.pose({fold_hinge: 1.10}):
        folded_height = ctx.part_world_position(upper_stem)
        assert folded_height is not None
        ctx.check(
            "stem_folds_back_over_deck",
            folded_height[0] < rest_height[0] - 0.30 and folded_height[2] < rest_height[2] - 0.18,
            details=f"upright={rest_height}, folded={folded_height}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
