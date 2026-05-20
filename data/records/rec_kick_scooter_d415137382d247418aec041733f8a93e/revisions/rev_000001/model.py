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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_commuter_kick_scooter")

    deck_aluminum = model.material("deck_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    stem_charcoal = model.material("stem_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.13, 0.13, 0.14, 1.0))
    deck_grip = model.material("deck_grip", rgba=(0.16, 0.16, 0.17, 1.0))

    wheel_radius = 0.09
    wheel_width = 0.03

    tire_profile = [
        (wheel_radius * 0.58, -wheel_width * 0.50),
        (wheel_radius * 0.78, -wheel_width * 0.50),
        (wheel_radius * 0.92, -wheel_width * 0.42),
        (wheel_radius * 0.98, -wheel_width * 0.22),
        (wheel_radius, 0.0),
        (wheel_radius * 0.98, wheel_width * 0.22),
        (wheel_radius * 0.92, wheel_width * 0.42),
        (wheel_radius * 0.78, wheel_width * 0.50),
        (wheel_radius * 0.58, wheel_width * 0.50),
        (wheel_radius * 0.46, wheel_width * 0.18),
        (wheel_radius * 0.44, 0.0),
        (wheel_radius * 0.46, -wheel_width * 0.18),
        (wheel_radius * 0.58, -wheel_width * 0.50),
    ]
    tire_mesh = mesh_from_geometry(
        LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0),
        "scooter_wheel_tire",
    )

    def add_wheel_visuals(part, *, prefix: str) -> None:
        part.visual(tire_mesh, material=wheel_rubber, name="tire")
        part.visual(
            Cylinder(radius=wheel_radius * 0.72, length=wheel_width * 0.78),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=deck_aluminum,
            name="rim",
        )
        part.visual(
            Cylinder(radius=wheel_radius * 0.24, length=wheel_width + 0.004),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="hub",
        )
        part.visual(
            Cylinder(radius=wheel_radius * 0.10, length=0.006),
            origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="axle_left_cap",
        )
        part.visual(
            Cylinder(radius=wheel_radius * 0.10, length=0.006),
            origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="axle_right_cap",
        )

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.14, 0.62, 0.10)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.04, 0.10)),
    )
    deck.visual(
        Box((0.118, 0.48, 0.004)),
        origin=Origin(xyz=(0.0, -0.04, 0.106)),
        material=deck_aluminum,
        name="deck_top",
    )
    deck.visual(
        Box((0.104, 0.46, 0.003)),
        origin=Origin(xyz=(0.0, -0.04, 0.0795)),
        material=deck_aluminum,
        name="deck_bottom",
    )
    deck.visual(
        Box((0.007, 0.46, 0.027)),
        origin=Origin(xyz=(0.0555, -0.04, 0.0925)),
        material=deck_aluminum,
        name="left_side_rail",
    )
    deck.visual(
        Box((0.007, 0.46, 0.027)),
        origin=Origin(xyz=(-0.0555, -0.04, 0.0925)),
        material=deck_aluminum,
        name="right_side_rail",
    )
    deck.visual(
        Box((0.094, 0.40, 0.002)),
        origin=Origin(xyz=(0.0, -0.04, 0.109)),
        material=deck_grip,
        name="grip_strip",
    )
    deck.visual(
        Box((0.084, 0.036, 0.028)),
        origin=Origin(xyz=(0.0, 0.170, 0.082)),
        material=deck_aluminum,
        name="nose_block",
    )
    deck.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.027, 0.205, 0.112), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_left_ear",
    )
    deck.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(-0.027, 0.205, 0.112), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_right_ear",
    )
    deck.visual(
        Box((0.012, 0.10, 0.024)),
        origin=Origin(xyz=(0.028, -0.315, 0.081)),
        material=steel,
        name="rear_left_tail_arm",
    )
    deck.visual(
        Box((0.012, 0.10, 0.024)),
        origin=Origin(xyz=(-0.028, -0.315, 0.081)),
        material=steel,
        name="rear_right_tail_arm",
    )
    deck.visual(
        Box((0.008, 0.10, 0.05)),
        origin=Origin(xyz=(0.027, -0.365, 0.075)),
        material=steel,
        name="rear_left_dropout",
    )
    deck.visual(
        Box((0.008, 0.10, 0.05)),
        origin=Origin(xyz=(-0.027, -0.365, 0.075)),
        material=steel,
        name="rear_right_dropout",
    )
    hinge_block = model.part("hinge_block")
    hinge_block.inertial = Inertial.from_geometry(
        Box((0.08, 0.08, 0.10)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )
    hinge_block.visual(
        Cylinder(radius=0.015, length=0.036),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    hinge_block.visual(
        Box((0.022, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=stem_charcoal,
        name="fold_yoke",
    )
    hinge_block.visual(
        Cylinder(radius=0.016, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=stem_charcoal,
        name="head_tube",
    )
    hinge_block.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=steel,
        name="headset_base",
    )

    front_fork_stem = model.part("front_fork_stem")
    front_fork_stem.inertial = Inertial.from_geometry(
        Box((0.46, 0.26, 0.92)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.04, 0.40)),
    )
    front_fork_stem.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="steering_bearing",
    )
    front_fork_stem.visual(
        Cylinder(radius=0.019, length=0.730),
        origin=Origin(xyz=(0.0, 0.0, 0.373)),
        material=stem_charcoal,
        name="stem_tube",
    )
    front_fork_stem.visual(
        Box((0.052, 0.034, 0.008)),
        origin=Origin(xyz=(0.0, 0.018, 0.008)),
        material=stem_charcoal,
        name="fork_bridge",
    )
    front_fork_stem.visual(
        Box((0.006, 0.012, 0.140)),
        origin=Origin(xyz=(0.026, 0.070, -0.050), rpy=(0.60, 0.0, 0.0)),
        material=stem_charcoal,
        name="left_fork_blade",
    )
    front_fork_stem.visual(
        Box((0.006, 0.012, 0.140)),
        origin=Origin(xyz=(-0.026, 0.070, -0.050), rpy=(0.60, 0.0, 0.0)),
        material=stem_charcoal,
        name="right_fork_blade",
    )
    front_fork_stem.visual(
        Cylinder(radius=0.013, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.744), rpy=(0.0, pi / 2.0, 0.0)),
        material=stem_charcoal,
        name="handlebar",
    )
    front_fork_stem.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(0.160, 0.0, 0.744), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    front_fork_stem.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(-0.160, 0.0, 0.744), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.4,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel_visuals(front_wheel, prefix="front")

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel_visuals(rear_wheel, prefix="rear")

    model.articulation(
        "deck_to_hinge_block",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=hinge_block,
        origin=Origin(xyz=(0.0, 0.205, 0.112)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.5, lower=-0.65, upper=0.0),
    )
    model.articulation(
        "hinge_block_to_front_fork_stem",
        ArticulationType.REVOLUTE,
        parent=hinge_block,
        child=front_fork_stem,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork_stem,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.118, -0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(0.0, -0.365, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    hinge_block = object_model.get_part("hinge_block")
    front_fork_stem = object_model.get_part("front_fork_stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    fold = object_model.get_articulation("deck_to_hinge_block")
    steer = object_model.get_articulation("hinge_block_to_front_fork_stem")
    front_spin = object_model.get_articulation("front_wheel_spin")
    rear_spin = object_model.get_articulation("rear_wheel_spin")

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

    ctx.expect_contact(
        hinge_block,
        deck,
        elem_a="hinge_barrel",
        elem_b="hinge_left_ear",
        name="hinge_left_bearing_contact",
    )
    ctx.expect_contact(
        hinge_block,
        deck,
        elem_a="hinge_barrel",
        elem_b="hinge_right_ear",
        name="hinge_right_bearing_contact",
    )
    ctx.expect_contact(
        front_fork_stem,
        hinge_block,
        elem_a="steering_bearing",
        elem_b="headset_base",
        name="steering_headset_contact",
    )
    ctx.expect_contact(
        front_wheel,
        front_fork_stem,
        elem_a="axle_left_cap",
        elem_b="left_fork_blade",
        name="front_left_axle_contact",
    )
    ctx.expect_contact(
        front_wheel,
        front_fork_stem,
        elem_a="axle_right_cap",
        elem_b="right_fork_blade",
        name="front_right_axle_contact",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="axle_left_cap",
        elem_b="rear_left_dropout",
        name="rear_left_axle_contact",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="axle_right_cap",
        elem_b="rear_right_dropout",
        name="rear_right_axle_contact",
    )
    ctx.expect_gap(
        front_wheel,
        deck,
        axis="y",
        min_gap=0.008,
        max_gap=0.03,
        name="front_wheel_clear_of_deck_nose",
    )
    ctx.expect_overlap(
        rear_wheel,
        deck,
        axes="x",
        min_overlap=0.04,
        name="rear_wheel_centered_under_tail",
    )

    ctx.check(
        "fold_joint_axis_is_transverse",
        tuple(round(value, 6) for value in fold.axis) == (1.0, 0.0, 0.0),
        f"expected transverse fold axis along +x, got {fold.axis}",
    )
    ctx.check(
        "steer_joint_axis_is_vertical",
        tuple(round(value, 6) for value in steer.axis) == (0.0, 0.0, 1.0),
        f"expected steering axis along +z, got {steer.axis}",
    )
    ctx.check(
        "front_wheel_spins_on_axle",
        tuple(round(value, 6) for value in front_spin.axis) == (1.0, 0.0, 0.0),
        f"expected front wheel axle along +x, got {front_spin.axis}",
    )
    ctx.check(
        "rear_wheel_spins_on_axle",
        tuple(round(value, 6) for value in rear_spin.axis) == (1.0, 0.0, 0.0),
        f"expected rear wheel axle along +x, got {rear_spin.axis}",
    )

    deck_aabb = ctx.part_world_aabb(deck)
    front_wheel_aabb = ctx.part_world_aabb(front_wheel)
    rear_wheel_aabb = ctx.part_world_aabb(rear_wheel)
    front_fork_stem_aabb = ctx.part_world_aabb(front_fork_stem)
    front_wheel_pos = ctx.part_world_position(front_wheel)
    rear_wheel_pos = ctx.part_world_position(rear_wheel)

    if deck_aabb is not None:
        deck_width = deck_aabb[1][0] - deck_aabb[0][0]
        deck_length = deck_aabb[1][1] - deck_aabb[0][1]
        ctx.check(
            "deck_has_narrow_commuter_proportions",
            0.11 <= deck_width <= 0.13 and 0.58 <= deck_length <= 0.64,
            f"deck width/length were {deck_width:.3f} m and {deck_length:.3f} m",
        )

    if front_wheel_aabb is not None:
        front_wheel_diameter = front_wheel_aabb[1][2] - front_wheel_aabb[0][2]
        ctx.check(
            "front_wheel_is_commuter_sized",
            0.17 <= front_wheel_diameter <= 0.19,
            f"front wheel diameter was {front_wheel_diameter:.3f} m",
        )

    if rear_wheel_aabb is not None:
        rear_wheel_diameter = rear_wheel_aabb[1][2] - rear_wheel_aabb[0][2]
        ctx.check(
            "rear_wheel_matches_front_size",
            0.17 <= rear_wheel_diameter <= 0.19,
            f"rear wheel diameter was {rear_wheel_diameter:.3f} m",
        )

    if front_fork_stem_aabb is not None:
        handlebar_height = front_fork_stem_aabb[1][2]
        handlebar_width = front_fork_stem_aabb[1][0] - front_fork_stem_aabb[0][0]
        ctx.check(
            "stem_reaches_realistic_height",
            0.93 <= handlebar_height <= 1.02,
            f"handlebar height was {handlebar_height:.3f} m",
        )
        ctx.check(
            "handlebar_has_commuter_width",
            0.40 <= handlebar_width <= 0.48,
            f"handlebar width was {handlebar_width:.3f} m",
        )

    if front_wheel_pos is not None and rear_wheel_pos is not None:
        wheelbase = front_wheel_pos[1] - rear_wheel_pos[1]
        ctx.check(
            "wheelbase_reads_as_scooter",
            0.68 <= wheelbase <= 0.75,
            f"wheelbase was {wheelbase:.3f} m",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_pose_no_floating")

    fold_limits = fold.motion_limits
    if fold_limits is not None and fold_limits.lower is not None and fold_limits.upper is not None:
        with ctx.pose({fold: fold_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fold_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="fold_lower_no_floating")
        with ctx.pose({fold: fold_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fold_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="fold_upper_no_floating")

    steer_limits = steer.motion_limits
    if steer_limits is not None and steer_limits.lower is not None and steer_limits.upper is not None:
        with ctx.pose({steer: steer_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steer_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="steer_lower_no_floating")
            ctx.expect_contact(
                front_wheel,
                front_fork_stem,
                elem_a="axle_left_cap",
                elem_b="left_fork_blade",
                name="steer_lower_left_axle_contact",
            )
        with ctx.pose({steer: steer_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steer_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="steer_upper_no_floating")
            ctx.expect_contact(
                front_wheel,
                front_fork_stem,
                elem_a="axle_right_cap",
                elem_b="right_fork_blade",
                name="steer_upper_right_axle_contact",
            )

    with ctx.pose({front_spin: 1.4, rear_spin: 2.2}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_spin_pose_no_floating")
        ctx.expect_contact(
            rear_wheel,
            deck,
            elem_a="axle_left_cap",
            elem_b="rear_left_dropout",
            name="wheel_spin_rear_axle_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
