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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stunt_scooter")

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    raw_aluminum = model.material("raw_aluminum", rgba=(0.71, 0.73, 0.76, 1.0))
    fork_black = model.material("fork_black", rgba=(0.14, 0.15, 0.17, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.06, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.28, 0.29, 0.31, 1.0))

    fold_angle = 0.23

    deck = model.part("deck")
    deck.visual(
        Box((0.34, 0.125, 0.006)),
        origin=Origin(xyz=(0.03, 0.0, 0.049)),
        material=matte_black,
        name="main_deck_top",
    )
    deck.visual(
        Box((0.12, 0.030, 0.006)),
        origin=Origin(xyz=(-0.19, 0.047, 0.049)),
        material=matte_black,
        name="tail_strip_left",
    )
    deck.visual(
        Box((0.12, 0.030, 0.006)),
        origin=Origin(xyz=(-0.19, -0.047, 0.049)),
        material=matte_black,
        name="tail_strip_right",
    )
    deck.visual(
        Box((0.06, 0.095, 0.006)),
        origin=Origin(xyz=(0.155, 0.0, 0.049)),
        material=matte_black,
        name="nose_top",
    )
    deck.visual(
        Box((0.52, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.0565, 0.038)),
        material=matte_black,
        name="left_rail",
    )
    deck.visual(
        Box((0.52, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.0565, 0.038)),
        material=matte_black,
        name="right_rail",
    )
    deck.visual(
        Box((0.07, 0.030, 0.014)),
        origin=Origin(xyz=(-0.205, 0.043, 0.038)),
        material=matte_black,
        name="rear_arm_left",
    )
    deck.visual(
        Box((0.07, 0.030, 0.014)),
        origin=Origin(xyz=(-0.205, -0.043, 0.038)),
        material=matte_black,
        name="rear_arm_right",
    )
    deck.visual(
        Box((0.11, 0.085, 0.028)),
        origin=Origin(xyz=(0.135, 0.0, 0.042)),
        material=matte_black,
        name="neck_gusset",
    )
    deck.visual(
        Box((0.026, 0.016, 0.058)),
        origin=Origin(xyz=(0.184, 0.029, 0.083)),
        material=raw_aluminum,
        name="hinge_cheek_left",
    )
    deck.visual(
        Box((0.026, 0.016, 0.058)),
        origin=Origin(xyz=(0.184, -0.029, 0.083)),
        material=raw_aluminum,
        name="hinge_cheek_right",
    )
    deck.visual(
        Box((0.044, 0.012, 0.038)),
        origin=Origin(xyz=(-0.225, 0.028, 0.049)),
        material=fork_black,
        name="rear_dropout_left",
    )
    deck.visual(
        Box((0.044, 0.012, 0.038)),
        origin=Origin(xyz=(-0.225, -0.028, 0.049)),
        material=fork_black,
        name="rear_dropout_right",
    )
    deck.visual(
        Box((0.082, 0.055, 0.012)),
        origin=Origin(xyz=(-0.22, 0.0, 0.118)),
        material=matte_black,
        name="rear_brake_fender",
    )
    deck.visual(
        Box((0.014, 0.008, 0.070)),
        origin=Origin(xyz=(-0.224, 0.026, 0.083)),
        material=matte_black,
        name="brake_support_left",
    )
    deck.visual(
        Box((0.014, 0.008, 0.070)),
        origin=Origin(xyz=(-0.224, -0.026, 0.083)),
        material=matte_black,
        name="brake_support_right",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.54, 0.125, 0.10)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Cylinder(radius=0.0115, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=raw_aluminum,
        name="hinge_barrel",
    )
    front_assembly.visual(
        Box((0.026, 0.044, 0.040)),
        origin=Origin(xyz=(-0.008, 0.0, 0.020)),
        material=raw_aluminum,
        name="fold_yoke",
    )
    front_assembly.visual(
        Cylinder(radius=0.022, length=0.108),
        origin=Origin(xyz=(-0.014, 0.0, 0.082), rpy=(0.0, -fold_angle, 0.0)),
        material=raw_aluminum,
        name="headtube_shell",
    )
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.60),
        origin=Origin(xyz=(-0.086, 0.0, 0.348), rpy=(0.0, -fold_angle, 0.0)),
        material=fork_black,
        name="stem_tube",
    )
    front_assembly.visual(
        Cylinder(radius=0.030, length=0.055),
        origin=Origin(xyz=(-0.142, 0.0, 0.594), rpy=(0.0, -fold_angle, 0.0)),
        material=raw_aluminum,
        name="stem_clamp",
    )
    front_assembly.visual(
        Box((0.050, 0.046, 0.058)),
        origin=Origin(xyz=(-0.142, 0.0, 0.594), rpy=(0.0, -fold_angle, 0.0)),
        material=raw_aluminum,
        name="clamp_body",
    )
    front_assembly.visual(
        Box((0.140, 0.044, 0.024)),
        origin=Origin(xyz=(0.040, 0.0, 0.047)),
        material=fork_black,
        name="fork_crown",
    )
    front_assembly.visual(
        Box((0.016, 0.012, 0.082)),
        origin=Origin(xyz=(0.094, 0.028, -0.006)),
        material=fork_black,
        name="fork_leg_left",
    )
    front_assembly.visual(
        Box((0.016, 0.012, 0.082)),
        origin=Origin(xyz=(0.094, -0.028, -0.006)),
        material=fork_black,
        name="fork_leg_right",
    )
    front_assembly.visual(
        Box((0.110, 0.020, 0.020)),
        origin=Origin(xyz=(0.006, 0.0, 0.061)),
        material=fork_black,
        name="crown_bridge",
    )
    front_assembly.visual(
        Cylinder(radius=0.015, length=0.18),
        origin=Origin(xyz=(-0.158, 0.0, 0.660), rpy=(0.0, 0.0, 0.0)),
        material=fork_black,
        name="handlebar_riser",
    )
    front_assembly.visual(
        Cylinder(radius=0.014, length=0.36),
        origin=Origin(xyz=(-0.158, 0.0, 0.742), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fork_black,
        name="handlebar_crossbar",
    )
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.158, 0.23, 0.742), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="left_grip",
    )
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.158, -0.23, 0.742), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="right_grip",
    )
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.34, 0.56, 0.86)),
        mass=2.0,
        origin=Origin(xyz=(-0.04, 0.0, 0.39)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.041, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_gray,
        name="rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=raw_aluminum,
        name="hub",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.024),
        mass=0.35,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.041, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_gray,
        name="rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=raw_aluminum,
        name="hub",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.024),
        mass=0.35,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "neck_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=(0.184, 0.0, 0.083)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-1.08,
            upper=0.0,
        ),
    )
    model.articulation(
        "front_axle",
        ArticulationType.REVOLUTE,
        parent=front_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.084, 0.0, -0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=30.0,
            lower=-6.283,
            upper=6.283,
        ),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.225, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=30.0,
            lower=-6.283,
            upper=6.283,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_assembly = object_model.get_part("front_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    neck_fold = object_model.get_articulation("neck_fold")

    main_deck_top = deck.get_visual("main_deck_top")
    left_cheek = deck.get_visual("hinge_cheek_left")
    right_cheek = deck.get_visual("hinge_cheek_right")
    rear_dropout_left = deck.get_visual("rear_dropout_left")
    rear_dropout_right = deck.get_visual("rear_dropout_right")
    brake_fender = deck.get_visual("rear_brake_fender")

    hinge_barrel = front_assembly.get_visual("hinge_barrel")
    stem_tube = front_assembly.get_visual("stem_tube")
    handlebar_crossbar = front_assembly.get_visual("handlebar_crossbar")
    left_fork = front_assembly.get_visual("fork_leg_left")
    right_fork = front_assembly.get_visual("fork_leg_right")

    front_tire = front_wheel.get_visual("tire")
    front_hub = front_wheel.get_visual("hub")
    rear_tire = rear_wheel.get_visual("tire")
    rear_hub = rear_wheel.get_visual("hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = neck_fold.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({neck_fold: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="neck_fold_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="neck_fold_lower_no_floating")
        with ctx.pose({neck_fold: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="neck_fold_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="neck_fold_upper_no_floating")

    ctx.expect_origin_distance(front_wheel, deck, axes="y", max_dist=0.001)
    ctx.expect_origin_distance(rear_wheel, deck, axes="y", max_dist=0.001)
    ctx.expect_origin_gap(front_wheel, rear_wheel, axis="x", min_gap=0.30, name="wheelbase_is_long")

    ctx.expect_within(
        front_wheel,
        deck,
        axes="y",
        inner_elem=front_tire,
        outer_elem=main_deck_top,
        name="deck_is_wider_than_front_wheel",
    )
    ctx.expect_within(
        rear_wheel,
        deck,
        axes="y",
        inner_elem=rear_tire,
        outer_elem=main_deck_top,
        name="deck_is_wider_than_rear_wheel",
    )

    ctx.expect_gap(
        deck,
        front_assembly,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=left_cheek,
        negative_elem=hinge_barrel,
        name="hinge_barrel_seated_in_left_clamp_cheek",
    )
    ctx.expect_gap(
        front_assembly,
        deck,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=hinge_barrel,
        negative_elem=right_cheek,
        name="hinge_barrel_seated_in_right_clamp_cheek",
    )

    ctx.expect_gap(
        front_assembly,
        front_wheel,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=left_fork,
        negative_elem=front_hub,
        name="front_hub_seated_against_left_fork_leg",
    )
    ctx.expect_gap(
        front_wheel,
        front_assembly,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=front_hub,
        negative_elem=right_fork,
        name="front_hub_seated_against_right_fork_leg",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rear_dropout_left,
        negative_elem=rear_hub,
        name="rear_hub_seated_against_left_dropout",
    )
    ctx.expect_gap(
        rear_wheel,
        deck,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rear_hub,
        negative_elem=rear_dropout_right,
        name="rear_hub_seated_against_right_dropout",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="z",
        min_gap=0.001,
        max_gap=0.015,
        positive_elem=brake_fender,
        negative_elem=rear_tire,
        name="rear_brake_fender_clears_tire",
    )

    with ctx.pose({neck_fold: -1.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="folded_pose_no_floating")
        ctx.expect_gap(
            front_assembly,
            deck,
            axis="z",
            max_gap=0.30,
            min_gap=0.05,
            positive_elem=handlebar_crossbar,
            negative_elem=main_deck_top,
            name="folded_handlebar_drops_over_deck",
        )
        ctx.expect_overlap(
            front_assembly,
            deck,
            axes="xy",
            min_overlap=0.03,
            elem_a=stem_tube,
            elem_b=main_deck_top,
            name="folded_stem_tracks_over_deck_footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
