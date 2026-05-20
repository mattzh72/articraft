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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _add_wheel_visuals(
    part,
    *,
    prefix: str,
    radius: float,
    tire_width: float,
    rim_radius: float,
    hub_radius: float,
    hub_width: float,
    axle_length: float,
    tire_material,
    rim_material,
    hub_material,
) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=tire_width),
        origin=spin_origin,
        material=tire_material,
        name=f"{prefix}_tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=tire_width + 0.004),
        origin=spin_origin,
        material=rim_material,
        name=f"{prefix}_rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=spin_origin,
        material=hub_material,
        name=f"{prefix}_hub",
    )
    part.visual(
        Cylinder(radius=0.004, length=axle_length),
        origin=spin_origin,
        material=hub_material,
        name=f"{prefix}_axle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kick_scooter")

    deck_gray = model.material("deck_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.07, 0.08, 1.0))
    tire_black = model.material("tire_black", rgba=(0.04, 0.04, 0.04, 1.0))
    brake_red = model.material("brake_red", rgba=(0.77, 0.16, 0.14, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.50, 0.13, 0.016)),
        origin=Origin(xyz=(-0.02, 0.0, 0.050)),
        material=deck_gray,
        name="deck_platform",
    )
    deck.visual(
        Box((0.44, 0.118, 0.003)),
        origin=Origin(xyz=(-0.03, 0.0, 0.0595)),
        material=grip_black,
        name="deck_grip_tape",
    )
    deck.visual(
        Box((0.030, 0.070, 0.068)),
        origin=Origin(xyz=(0.215, 0.0, 0.092)),
        material=steel_dark,
        name="front_head_pedestal",
    )
    deck.visual(
        Box((0.042, 0.060, 0.024)),
        origin=Origin(xyz=(0.190, 0.0, 0.070)),
        material=steel_dark,
        name="front_head_gusset",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel_dark,
        name="rear_mount_block",
    )
    rear_frame.visual(
        Box((0.050, 0.004, 0.076)),
        origin=Origin(xyz=(-0.075, 0.022, 0.032)),
        material=steel_dark,
        name="rear_left_support",
    )
    rear_frame.visual(
        Box((0.050, 0.004, 0.076)),
        origin=Origin(xyz=(-0.075, -0.022, 0.032)),
        material=steel_dark,
        name="rear_right_support",
    )
    rear_frame.visual(
        Box((0.014, 0.046, 0.010)),
        origin=Origin(xyz=(-0.105, 0.0, 0.070)),
        material=steel_dark,
        name="rear_hinge_crossbar",
    )
    rear_frame.visual(
        Box((0.010, 0.004, 0.052)),
        origin=Origin(xyz=(-0.025, 0.022, 0.032)),
        material=steel_dark,
        name="rear_left_post",
    )
    rear_frame.visual(
        Box((0.010, 0.004, 0.052)),
        origin=Origin(xyz=(-0.025, -0.022, 0.032)),
        material=steel_dark,
        name="rear_right_post",
    )
    rear_frame.visual(
        Box((0.050, 0.004, 0.010)),
        origin=Origin(xyz=(-0.055, 0.022, 0.060)),
        material=steel_dark,
        name="rear_left_link",
    )
    rear_frame.visual(
        Box((0.050, 0.004, 0.010)),
        origin=Origin(xyz=(-0.055, -0.022, 0.060)),
        material=steel_dark,
        name="rear_right_link",
    )

    steering_assembly = model.part("steering_assembly")
    steering_assembly.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel_dark,
        name="lower_bearing",
    )
    steering_assembly.visual(
        Cylinder(radius=0.021, length=0.674),
        origin=Origin(xyz=(0.0, 0.0, 0.363)),
        material=brushed_aluminum,
        name="steering_column",
    )
    steering_assembly.visual(
        Box((0.055, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.713)),
        material=steel_dark,
        name="handlebar_clamp",
    )
    steering_assembly.visual(
        Cylinder(radius=0.016, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.730), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="handlebar_crossbar",
    )
    steering_assembly.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.0, 0.182, 0.730), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    steering_assembly.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.0, -0.182, 0.730), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    steering_assembly.visual(
        Box((0.075, 0.048, 0.016)),
        origin=Origin(xyz=(0.085, 0.0, -0.004)),
        material=steel_dark,
        name="fork_crown",
    )
    steering_assembly.visual(
        Box((0.080, 0.024, 0.020)),
        origin=Origin(xyz=(0.040, 0.0, 0.006)),
        material=steel_dark,
        name="fork_bridge",
    )
    steering_assembly.visual(
        Box((0.010, 0.004, 0.060)),
        origin=Origin(xyz=(0.125, 0.022, -0.040)),
        material=steel_dark,
        name="left_fork_leg",
    )
    steering_assembly.visual(
        Box((0.010, 0.004, 0.060)),
        origin=Origin(xyz=(0.125, -0.022, -0.040)),
        material=steel_dark,
        name="right_fork_leg",
    )

    front_wheel = model.part("front_wheel")
    _add_wheel_visuals(
        front_wheel,
        prefix="front",
        radius=0.055,
        tire_width=0.020,
        rim_radius=0.038,
        hub_radius=0.018,
        hub_width=0.022,
        axle_length=0.040,
        tire_material=tire_black,
        rim_material=deck_gray,
        hub_material=steel_dark,
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel_visuals(
        rear_wheel,
        prefix="rear",
        radius=0.055,
        tire_width=0.020,
        rim_radius=0.038,
        hub_radius=0.018,
        hub_width=0.022,
        axle_length=0.040,
        tire_material=tire_black,
        rim_material=deck_gray,
        hub_material=steel_dark,
    )

    brake_leaf = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.012, 0.0, 0.012),
                (0.040, 0.0, 0.013),
                (0.082, 0.0, 0.009),
                (0.126, 0.0, 0.004),
                (0.162, 0.0, 0.001),
            ],
            profile=rounded_rect_profile(0.026, 0.0055, radius=0.0012, corner_segments=4),
            samples_per_segment=14,
            cap_profile=True,
        ),
        "kick_scooter_brake_leaf_v2",
    )
    brake_fender = model.part("brake_fender")
    brake_fender.visual(
        brake_leaf,
        material=brake_red,
        name="brake_leaf",
    )

    model.articulation(
        "rear_frame_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_frame,
        origin=Origin(xyz=(-0.280, 0.0, 0.058)),
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=steering_assembly,
        origin=Origin(xyz=(0.230, 0.0, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.8, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.125, 0.0, -0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_frame,
        child=rear_wheel,
        origin=Origin(xyz=(-0.090, 0.0, -0.003)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "rear_brake_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=brake_fender,
        origin=Origin(xyz=(-0.105, 0.0, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    rear_frame = object_model.get_part("rear_frame")
    steering_assembly = object_model.get_part("steering_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    brake_fender = object_model.get_part("brake_fender")

    rear_frame_mount = object_model.get_articulation("rear_frame_mount")
    steering_yaw = object_model.get_articulation("steering_yaw")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")
    rear_brake_hinge = object_model.get_articulation("rear_brake_hinge")

    front_head_pedestal = deck.get_visual("front_head_pedestal")
    deck_platform = deck.get_visual("deck_platform")
    rear_left_support = rear_frame.get_visual("rear_left_support")
    rear_right_support = rear_frame.get_visual("rear_right_support")

    lower_bearing = steering_assembly.get_visual("lower_bearing")
    left_fork_leg = steering_assembly.get_visual("left_fork_leg")
    right_fork_leg = steering_assembly.get_visual("right_fork_leg")
    handlebar_crossbar = steering_assembly.get_visual("handlebar_crossbar")

    front_axle = front_wheel.get_visual("front_axle")
    rear_axle = rear_wheel.get_visual("rear_axle")
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_isolated_part(
        brake_fender,
        reason="Rear fender is mechanically mounted by the revolute hinge axis; the simplified visual omits an interpenetrating hinge barrel to keep the articulated brake clear through motion.",
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

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (deck, rear_frame, steering_assembly, front_wheel, rear_wheel, brake_fender)),
        "Expected deck, rear frame, steering assembly, both wheels, and brake fender.",
    )
    ctx.check(
        "articulation_axes_match_scooter_mechanics",
        steering_yaw.axis == (0.0, 0.0, 1.0)
        and front_wheel_spin.axis == (0.0, 1.0, 0.0)
        and rear_wheel_spin.axis == (0.0, 1.0, 0.0)
        and rear_brake_hinge.axis == (0.0, 1.0, 0.0)
        and rear_frame_mount.articulation_type == ArticulationType.FIXED,
        "Steering must yaw about +Z while wheels and brake hinge rotate about the axle-width Y axis.",
    )

    ctx.expect_contact(
        steering_assembly,
        deck,
        elem_a=lower_bearing,
        elem_b=front_head_pedestal,
        name="steering_bearing_seats_on_front_pedestal",
    )
    ctx.expect_contact(
        front_wheel,
        steering_assembly,
        elem_a=front_axle,
        elem_b=left_fork_leg,
        name="front_axle_contacts_left_fork_leg",
    )
    ctx.expect_contact(
        front_wheel,
        steering_assembly,
        elem_a=front_axle,
        elem_b=right_fork_leg,
        name="front_axle_contacts_right_fork_leg",
    )
    ctx.expect_contact(
        rear_wheel,
        rear_frame,
        elem_a=rear_axle,
        elem_b=rear_left_support,
        name="rear_axle_contacts_left_support",
    )
    ctx.expect_contact(
        rear_wheel,
        rear_frame,
        elem_a=rear_axle,
        elem_b=rear_right_support,
        name="rear_axle_contacts_right_support",
    )
    ctx.check(
        "rear_brake_hinge_is_mounted",
        rear_brake_hinge.parent == rear_frame.name and rear_brake_hinge.child == brake_fender.name,
        "Rear brake fender must hinge from the rear frame.",
    )

    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.66,
        max_gap=0.74,
        name="wheelbase_is_scooter_length",
    )
    with ctx.pose({rear_brake_hinge: 0.0}):
        ctx.expect_gap(
            steering_assembly,
            deck,
            axis="z",
            positive_elem=handlebar_crossbar,
            negative_elem=deck_platform,
            min_gap=0.77,
            max_gap=0.83,
            name="handlebar_height_above_deck",
        )
        ctx.expect_overlap(
            brake_fender,
            rear_wheel,
            axes="xy",
            min_overlap=0.020,
            name="brake_fender_covers_rear_wheel",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulations_clear_through_motion")

    steering_limits = steering_yaw.motion_limits
    if steering_limits is not None and steering_limits.lower is not None and steering_limits.upper is not None:
        with ctx.pose({steering_yaw: steering_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_lower_no_floating")
        with ctx.pose({steering_yaw: steering_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_upper_no_floating")

    brake_limits = rear_brake_hinge.motion_limits
    if brake_limits is not None and brake_limits.lower is not None and brake_limits.upper is not None:
        with ctx.pose({rear_brake_hinge: brake_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="brake_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="brake_lower_no_floating")
            ctx.expect_overlap(
                brake_fender,
                rear_wheel,
                axes="xy",
                min_overlap=0.020,
                name="brake_lower_stays_over_wheel",
            )
        with ctx.pose({rear_brake_hinge: brake_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="brake_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="brake_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
