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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _extruded_mesh(name: str, profile, height: float):
    return mesh_from_geometry(
        ExtrudeGeometry(profile, height, center=True),
        name,
    )


def _extruded_hollow_mesh(name: str, outer_profile, hole_profiles, height: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kids_kick_scooter")

    deck_color = model.material("deck_color", rgba=(0.37, 0.83, 0.78, 1.0))
    deck_pad = model.material("deck_pad", rgba=(0.13, 0.16, 0.18, 1.0))
    frame_silver = model.material("frame_silver", rgba=(0.82, 0.84, 0.86, 1.0))
    hub_blue = model.material("hub_blue", rgba=(0.22, 0.52, 0.88, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.06, 0.06, 0.07, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.18, 0.20, 0.22, 1.0))

    deck_length = 0.48
    deck_width = 0.13
    deck_thickness = 0.022
    pad_length = 0.30
    pad_width = 0.082
    pad_thickness = 0.003

    wheel_radius = 0.06
    tire_width = 0.014
    hub_width = 0.018
    rear_hub_width = 0.024
    hub_radius = 0.028

    front_hinge_x = 0.245
    front_hinge_z = deck_thickness * 0.5 + 0.017
    rear_axle_x = -0.175
    axle_z = -deck_thickness * 0.5 - wheel_radius

    sleeve_center_x = 0.02
    sleeve_height = 0.31
    sleeve_base_z = 0.055
    inner_column_total = 0.32
    inner_column_insert = 0.18

    deck = model.part("deck")
    deck.visual(
        _extruded_mesh(
            "kids_scooter_deck_shell_v3",
            superellipse_profile(deck_length, deck_width, exponent=2.3, segments=64),
            deck_thickness,
        ),
        material=deck_color,
        name="deck_shell",
    )
    deck.visual(
        _extruded_mesh(
            "kids_scooter_deck_pad_v3",
            superellipse_profile(pad_length, pad_width, exponent=2.4, segments=48),
            pad_thickness,
        ),
        origin=Origin(xyz=(0.0, 0.0, deck_thickness * 0.5 + pad_thickness * 0.5)),
        material=deck_pad,
        name="deck_pad",
    )
    deck.visual(
        Box((0.05, 0.036, 0.032)),
        origin=Origin(xyz=(0.215, 0.0, 0.016)),
        material=frame_silver,
        name="deck_hinge_bridge",
    )
    deck.visual(
        Box((0.028, 0.006, 0.05)),
        origin=Origin(xyz=(front_hinge_x, 0.015, 0.036)),
        material=frame_silver,
        name="deck_clevis_left",
    )
    deck.visual(
        Box((0.028, 0.006, 0.05)),
        origin=Origin(xyz=(front_hinge_x, -0.015, 0.036)),
        material=frame_silver,
        name="deck_clevis_right",
    )
    deck.visual(
        Box((0.07, 0.006, 0.086)),
        origin=Origin(xyz=(rear_axle_x, 0.015, -0.04)),
        material=frame_silver,
        name="rear_drop_left",
    )
    deck.visual(
        Box((0.07, 0.006, 0.086)),
        origin=Origin(xyz=(rear_axle_x, -0.015, -0.04)),
        material=frame_silver,
        name="rear_drop_right",
    )
    deck.visual(
        Box((0.035, 0.036, 0.02)),
        origin=Origin(xyz=(rear_axle_x - 0.018, 0.0, -0.001)),
        material=frame_silver,
        name="rear_dropout_bridge",
    )

    outer_stem = model.part("outer_stem")
    outer_stem.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="stem_hinge_lug",
    )
    outer_stem.visual(
        Box((0.018, 0.016, 0.07)),
        origin=Origin(xyz=(0.009, 0.0, 0.035)),
        material=frame_silver,
        name="stem_neck",
    )
    outer_stem.visual(
        Box((0.026, 0.022, 0.024)),
        origin=Origin(xyz=(sleeve_center_x, 0.0, 0.067)),
        material=frame_silver,
        name="sleeve_lower_collar",
    )
    outer_stem.visual(
        Box((0.003, 0.022, 0.27)),
        origin=Origin(xyz=(0.0315, 0.0, 0.205)),
        material=frame_silver,
        name="sleeve_front_wall",
    )
    outer_stem.visual(
        Box((0.003, 0.022, 0.27)),
        origin=Origin(xyz=(0.0085, 0.0, 0.205)),
        material=frame_silver,
        name="sleeve_rear_wall",
    )
    outer_stem.visual(
        Box((0.020, 0.003, 0.27)),
        origin=Origin(xyz=(sleeve_center_x, 0.0095, 0.205)),
        material=frame_silver,
        name="sleeve_left_wall",
    )
    outer_stem.visual(
        Box((0.020, 0.003, 0.27)),
        origin=Origin(xyz=(sleeve_center_x, -0.0095, 0.205)),
        material=frame_silver,
        name="sleeve_right_wall",
    )
    outer_stem.visual(
        Box((0.03, 0.018, 0.02)),
        origin=Origin(xyz=(0.018, 0.0, -0.006)),
        material=frame_silver,
        name="fork_bridge",
    )
    outer_stem.visual(
        Box((0.05, 0.026, 0.018)),
        origin=Origin(xyz=(0.045, 0.0, -0.014)),
        material=frame_silver,
        name="fork_crown",
    )
    outer_stem.visual(
        Box((0.04, 0.004, 0.082)),
        origin=Origin(xyz=(0.06, 0.011, -0.051)),
        material=frame_silver,
        name="fork_left",
    )
    outer_stem.visual(
        Box((0.04, 0.004, 0.082)),
        origin=Origin(xyz=(0.06, -0.011, -0.051)),
        material=frame_silver,
        name="fork_right",
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.014, 0.012, inner_column_total)),
        origin=Origin(xyz=(sleeve_center_x, 0.0, inner_column_total * 0.5 - inner_column_insert)),
        material=frame_silver,
        name="inner_column_shaft",
    )
    inner_column.visual(
        Box((0.002, 0.002, 0.20)),
        origin=Origin(xyz=(sleeve_center_x, 0.007, -0.07)),
        material=frame_silver,
        name="left_runner",
    )
    inner_column.visual(
        Box((0.002, 0.002, 0.20)),
        origin=Origin(xyz=(sleeve_center_x, -0.007, -0.07)),
        material=frame_silver,
        name="right_runner",
    )
    inner_column.visual(
        Box((0.036, 0.02, 0.04)),
        origin=Origin(xyz=(sleeve_center_x, 0.0, 0.15)),
        material=frame_silver,
        name="handlebar_clamp",
    )
    inner_column.visual(
        Cylinder(radius=0.011, length=0.28),
        origin=Origin(xyz=(sleeve_center_x, 0.0, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="handlebar_bar",
    )
    inner_column.visual(
        Cylinder(radius=0.014, length=0.09),
        origin=Origin(xyz=(sleeve_center_x, 0.14, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="left_grip",
    )
    inner_column.visual(
        Cylinder(radius=0.014, length=0.09),
        origin=Origin(xyz=(sleeve_center_x, -0.14, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="right_grip",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=wheel_radius, length=tire_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="front_tire",
    )
    front_wheel.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_blue,
        name="front_hub",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=wheel_radius, length=tire_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="rear_tire",
    )
    rear_wheel.visual(
        Cylinder(radius=hub_radius, length=rear_hub_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_blue,
        name="rear_hub",
    )

    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=outer_stem,
        origin=Origin(xyz=(front_hinge_x, 0.0, front_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-1.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "stem_extension",
        ArticulationType.PRISMATIC,
        parent=outer_stem,
        child=inner_column,
        origin=Origin(xyz=(sleeve_center_x, 0.0, sleeve_base_z + sleeve_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.08,
        ),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=outer_stem,
        child=front_wheel,
        origin=Origin(xyz=(0.06, 0.0, axle_z - front_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=20.0,
        ),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(rear_axle_x, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=20.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    outer_stem = object_model.get_part("outer_stem")
    inner_column = object_model.get_part("inner_column")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    fold_hinge = object_model.get_articulation("fold_hinge")
    stem_extension = object_model.get_articulation("stem_extension")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")

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
        inner_column,
        outer_stem,
        elem_a="inner_column_shaft",
        elem_b="sleeve_front_wall",
        reason="The telescoping steering column nests inside the outer stem sleeve during fold motion.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "joint_types_match_scooter_mechanics",
        fold_hinge.articulation_type == ArticulationType.REVOLUTE
        and stem_extension.articulation_type == ArticulationType.PRISMATIC
        and front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected fold hinge, telescoping stem, and spinning wheels.",
    )
    ctx.check(
        "joint_axes_are_correct",
        fold_hinge.axis == (0.0, 1.0, 0.0)
        and stem_extension.axis == (0.0, 0.0, 1.0)
        and front_wheel_spin.axis == (0.0, 1.0, 0.0)
        and rear_wheel_spin.axis == (0.0, 1.0, 0.0),
        details="Scooter fold hinge should pitch around Y, stem should slide on Z, and wheels should spin on Y.",
    )

    ctx.expect_contact(
        outer_stem,
        deck,
        elem_a="stem_hinge_lug",
        elem_b="deck_clevis_left",
        name="fold_hinge_left_contact",
    )
    ctx.expect_contact(
        outer_stem,
        deck,
        elem_a="stem_hinge_lug",
        elem_b="deck_clevis_right",
        name="fold_hinge_right_contact",
    )
    ctx.expect_contact(
        inner_column,
        outer_stem,
        elem_a="inner_column_shaft",
        elem_b="sleeve_front_wall",
        name="telescoping_shaft_guided_in_sleeve",
    )
    ctx.expect_contact(
        front_wheel,
        outer_stem,
        elem_a="front_hub",
        elem_b="fork_left",
        name="front_hub_contacts_left_fork",
    )
    ctx.expect_contact(
        front_wheel,
        outer_stem,
        elem_a="front_hub",
        elem_b="fork_right",
        name="front_hub_contacts_right_fork",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="rear_hub",
        elem_b="rear_drop_left",
        name="rear_hub_contacts_left_drop",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="rear_hub",
        elem_b="rear_drop_right",
        name="rear_hub_contacts_right_drop",
    )
    ctx.expect_origin_distance(
        front_wheel,
        rear_wheel,
        axes="x",
        min_dist=0.45,
        max_dist=0.50,
        name="child_scooter_wheelbase",
    )
    ctx.expect_origin_gap(
        inner_column,
        deck,
        axis="z",
        min_gap=0.38,
        name="handlebar_height_above_deck",
    )

    fold_limits = fold_hinge.motion_limits
    if fold_limits is not None and fold_limits.lower is not None and fold_limits.upper is not None:
        with ctx.pose({fold_hinge: fold_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fold_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="fold_hinge_lower_no_floating")
            ctx.expect_contact(
                outer_stem,
                deck,
                elem_a="stem_hinge_lug",
                elem_b="deck_clevis_left",
                name="folded_hinge_left_contact",
            )
            ctx.expect_contact(
                outer_stem,
                deck,
                elem_a="stem_hinge_lug",
                elem_b="deck_clevis_right",
                name="folded_hinge_right_contact",
            )
        with ctx.pose({fold_hinge: fold_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fold_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="fold_hinge_upper_no_floating")

    stem_limits = stem_extension.motion_limits
    if stem_limits is not None and stem_limits.lower is not None and stem_limits.upper is not None:
        with ctx.pose({stem_extension: stem_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="stem_extension_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="stem_extension_lower_no_floating")
            ctx.expect_contact(
                inner_column,
                outer_stem,
                elem_a="inner_column_shaft",
                elem_b="sleeve_front_wall",
                name="stem_extension_lower_contact",
            )
        with ctx.pose({stem_extension: stem_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="stem_extension_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="stem_extension_upper_no_floating")
            ctx.expect_contact(
                inner_column,
                outer_stem,
                elem_a="inner_column_shaft",
                elem_b="sleeve_front_wall",
                name="stem_extension_upper_contact",
            )
            ctx.expect_origin_gap(
                inner_column,
                deck,
                axis="z",
                min_gap=0.46,
                name="extended_handlebar_height",
            )

    with ctx.pose({front_wheel_spin: math.pi / 2.0, rear_wheel_spin: math.pi / 3.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_spin_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
