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


CABINET_LENGTH = 1.18
CABINET_WIDTH = 0.62
PLINTH_HEIGHT = 0.08
WALL_THICKNESS = 0.03
CABINET_WALL_HEIGHT = 0.78
ROOF_THICKNESS = 0.03
CABINET_TOP_Z = PLINTH_HEIGHT + CABINET_WALL_HEIGHT + ROOF_THICKNESS

HINGE_X = CABINET_LENGTH * 0.5 - 0.08
HINGE_Z = CABINET_TOP_Z + 0.14

MAIN_ARM_LENGTH = 3.35
TIP_ARM_LENGTH = 3.05
BOOM_WIDTH = 0.12
BOOM_HEIGHT = 0.10
BOOM_START_X = 0.065

MAIN_ARM_UPPER = math.radians(82.0)
FOLD_ARM_LOWER = math.radians(-95.0)


def _boom_beam_length(total_length: float, *, start_x: float = BOOM_START_X) -> float:
    return total_length - (start_x + 0.07)


def _add_striped_boom(
    part,
    *,
    prefix: str,
    total_length: float,
    stripe_count: int,
    base_material,
    stripe_material,
    start_x: float = BOOM_START_X,
) -> None:
    beam_length = _boom_beam_length(total_length, start_x=start_x)
    part.visual(
        Box((beam_length, BOOM_WIDTH, BOOM_HEIGHT)),
        origin=Origin(xyz=(start_x + beam_length * 0.5, 0.0, 0.0)),
        material=base_material,
        name=f"{prefix}_beam",
    )

    stripe_length = min(0.42, beam_length / max(stripe_count + 1, 2))
    for stripe_index in range(stripe_count):
        center_x = start_x + beam_length * (stripe_index + 1) / (stripe_count + 1)
        part.visual(
            Box((stripe_length, BOOM_WIDTH + 0.006, BOOM_HEIGHT + 0.006)),
            origin=Origin(xyz=(center_x, 0.0, 0.0)),
            material=stripe_material,
            name=f"{prefix}_stripe_{stripe_index + 1}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="railway_level_crossing_barrier")

    cabinet_grey = model.material("cabinet_grey", rgba=(0.53, 0.56, 0.59, 1.0))
    frame_grey = model.material("frame_grey", rgba=(0.34, 0.36, 0.38, 1.0))
    arm_white = model.material("arm_white", rgba=(0.94, 0.95, 0.96, 1.0))
    arm_red = model.material("arm_red", rgba=(0.78, 0.08, 0.10, 1.0))
    light_red = model.material("light_red", rgba=(0.86, 0.12, 0.14, 1.0))
    bumper_black = model.material("bumper_black", rgba=(0.10, 0.10, 0.10, 1.0))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((1.30, 0.74, 1.30)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )

    wall_center_z = PLINTH_HEIGHT + CABINET_WALL_HEIGHT * 0.5
    cabinet.visual(
        Box((CABINET_LENGTH + 0.06, CABINET_WIDTH + 0.06, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=frame_grey,
        name="foundation_plinth",
    )
    cabinet.visual(
        Box((CABINET_LENGTH, WALL_THICKNESS, CABINET_WALL_HEIGHT)),
        origin=Origin(xyz=(0.0, CABINET_WIDTH * 0.5 - WALL_THICKNESS * 0.5, wall_center_z)),
        material=cabinet_grey,
        name="front_wall",
    )
    cabinet.visual(
        Box((CABINET_LENGTH, WALL_THICKNESS, CABINET_WALL_HEIGHT)),
        origin=Origin(xyz=(0.0, -CABINET_WIDTH * 0.5 + WALL_THICKNESS * 0.5, wall_center_z)),
        material=cabinet_grey,
        name="rear_wall",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, CABINET_WIDTH + WALL_THICKNESS, CABINET_WALL_HEIGHT)),
        origin=Origin(xyz=(-CABINET_LENGTH * 0.5 + WALL_THICKNESS * 0.5, 0.0, wall_center_z)),
        material=cabinet_grey,
        name="left_wall",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, CABINET_WIDTH + WALL_THICKNESS, CABINET_WALL_HEIGHT)),
        origin=Origin(xyz=(CABINET_LENGTH * 0.5 - WALL_THICKNESS * 0.5, 0.0, wall_center_z)),
        material=cabinet_grey,
        name="right_wall",
    )
    cabinet.visual(
        Box((CABINET_LENGTH - 0.02, CABINET_WIDTH - 0.02, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + WALL_THICKNESS * 0.5)),
        material=frame_grey,
        name="floor_plate",
    )
    cabinet.visual(
        Box((CABINET_LENGTH + 0.02, CABINET_WIDTH + 0.02, ROOF_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_TOP_Z - ROOF_THICKNESS * 0.5)),
        material=cabinet_grey,
        name="roof_plate",
    )
    cabinet.visual(
        Box((0.52, 0.014, 0.60)),
        origin=Origin(xyz=(-0.06, CABINET_WIDTH * 0.5 - 0.004, 0.42)),
        material=frame_grey,
        name="service_door_panel",
    )
    cabinet.visual(
        Box((0.28, 0.26, 0.42)),
        origin=Origin(xyz=(0.19, 0.0, CABINET_TOP_Z + 0.12)),
        material=cabinet_grey,
        name="drive_housing",
    )
    cabinet.visual(
        Box((0.12, 0.22, 0.30)),
        origin=Origin(xyz=(HINGE_X - 0.01, 0.0, HINGE_Z - 0.23)),
        material=frame_grey,
        name="hinge_tower",
    )
    cabinet.visual(
        Box((0.14, 0.028, 0.24)),
        origin=Origin(xyz=(HINGE_X, 0.084, HINGE_Z)),
        material=frame_grey,
        name="upper_bracket_plate",
    )
    cabinet.visual(
        Box((0.14, 0.028, 0.24)),
        origin=Origin(xyz=(HINGE_X, -0.084, HINGE_Z)),
        material=frame_grey,
        name="lower_bracket_plate",
    )
    cabinet.visual(
        Cylinder(radius=0.052, length=0.032),
        origin=Origin(
            xyz=(0.22, CABINET_WIDTH * 0.5 + 0.001, 0.60),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=light_red,
        name="warning_light_front",
    )
    cabinet.visual(
        Cylinder(radius=0.052, length=0.032),
        origin=Origin(
            xyz=(0.22, -CABINET_WIDTH * 0.5 - 0.001, 0.60),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=light_red,
        name="warning_light_rear",
    )

    main_arm = model.part("main_arm")
    main_arm.inertial = Inertial.from_geometry(
        Box((MAIN_ARM_LENGTH, 0.18, 0.18)),
        mass=62.0,
        origin=Origin(xyz=(MAIN_ARM_LENGTH * 0.5, 0.0, 0.0)),
    )
    main_arm.visual(
        Cylinder(radius=0.072, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_grey,
        name="pivot_hub",
    )
    _add_striped_boom(
        main_arm,
        prefix="main",
        total_length=MAIN_ARM_LENGTH,
        stripe_count=4,
        base_material=arm_white,
        stripe_material=arm_red,
    )
    main_arm.visual(
        Box((0.10, 0.014, 0.16)),
        origin=Origin(xyz=(MAIN_ARM_LENGTH - 0.05, 0.065, 0.0)),
        material=frame_grey,
        name="fold_clevis_front",
    )
    main_arm.visual(
        Box((0.10, 0.014, 0.16)),
        origin=Origin(xyz=(MAIN_ARM_LENGTH - 0.05, -0.065, 0.0)),
        material=frame_grey,
        name="fold_clevis_rear",
    )
    main_arm.visual(
        Box((0.44, 0.09, 0.05)),
        origin=Origin(xyz=(0.38, 0.0, -0.055)),
        material=frame_grey,
        name="counterweight_bar",
    )

    tip_arm = model.part("tip_arm")
    tip_arm.inertial = Inertial.from_geometry(
        Box((TIP_ARM_LENGTH, 0.16, 0.16)),
        mass=34.0,
        origin=Origin(xyz=(TIP_ARM_LENGTH * 0.5, 0.0, 0.0)),
    )
    tip_arm.visual(
        Cylinder(radius=0.04, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_grey,
        name="fold_knuckle",
    )
    tip_arm.visual(
        Box((0.05, 0.08, 0.08)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=frame_grey,
        name="fold_knuckle_web",
    )
    _add_striped_boom(
        tip_arm,
        prefix="tip",
        total_length=TIP_ARM_LENGTH,
        stripe_count=3,
        base_material=arm_white,
        stripe_material=arm_red,
        start_x=0.045,
    )
    tip_arm.visual(
        Box((0.07, BOOM_WIDTH + 0.012, BOOM_HEIGHT + 0.012)),
        origin=Origin(xyz=(TIP_ARM_LENGTH - 0.035, 0.0, 0.0)),
        material=arm_red,
        name="end_cap",
    )
    tip_arm.visual(
        Box((0.12, 0.08, 0.05)),
        origin=Origin(xyz=(TIP_ARM_LENGTH - 0.02, 0.0, -0.055)),
        material=bumper_black,
        name="rubber_bumper",
    )

    fold_pin = model.part("fold_pin")
    fold_pin.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.116),
        mass=1.2,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    fold_pin.visual(
        Cylinder(radius=0.016, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_grey,
        name="fold_pin_body",
    )

    model.articulation(
        "cabinet_to_main_arm",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=main_arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.8,
            lower=0.0,
            upper=MAIN_ARM_UPPER,
        ),
    )
    model.articulation(
        "main_arm_to_tip_arm",
        ArticulationType.REVOLUTE,
        parent=main_arm,
        child=tip_arm,
        origin=Origin(xyz=(MAIN_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=FOLD_ARM_LOWER,
            upper=0.0,
        ),
    )
    model.articulation(
        "main_arm_to_fold_pin",
        ArticulationType.FIXED,
        parent=main_arm,
        child=fold_pin,
        origin=Origin(xyz=(MAIN_ARM_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    main_arm = object_model.get_part("main_arm")
    tip_arm = object_model.get_part("tip_arm")
    fold_pin = object_model.get_part("fold_pin")

    main_hinge = object_model.get_articulation("cabinet_to_main_arm")
    fold_hinge = object_model.get_articulation("main_arm_to_tip_arm")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        fold_pin,
        tip_arm,
        reason="The folding-section hinge pin passes through the tip knuckle.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(cabinet, main_arm, name="cabinet_contacts_main_arm_at_pivot")
    ctx.expect_contact(main_arm, fold_pin, name="main_arm_supports_fold_pin")
    ctx.expect_contact(fold_pin, tip_arm, name="tip_arm_is_captured_by_fold_pin")
    ctx.expect_origin_gap(
        tip_arm,
        main_arm,
        axis="x",
        min_gap=3.1,
        max_gap=3.6,
        name="fold_joint_is_near_boom_midspan",
    )

    ctx.check(
        "main_hinge_axis_is_lateral",
        abs(main_hinge.axis[1]) > 0.99 and abs(main_hinge.axis[0]) < 1e-6 and abs(main_hinge.axis[2]) < 1e-6,
        details=f"unexpected main hinge axis {main_hinge.axis}",
    )
    ctx.check(
        "fold_hinge_axis_matches_main_axis",
        abs(fold_hinge.axis[1]) > 0.99 and abs(fold_hinge.axis[0]) < 1e-6 and abs(fold_hinge.axis[2]) < 1e-6,
        details=f"unexpected fold hinge axis {fold_hinge.axis}",
    )

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    assert cabinet_aabb is not None
    cabinet_dx = cabinet_aabb[1][0] - cabinet_aabb[0][0]
    cabinet_dy = cabinet_aabb[1][1] - cabinet_aabb[0][1]
    cabinet_dz = cabinet_aabb[1][2] - cabinet_aabb[0][2]
    ctx.check(
        "cabinet_reads_as_wide_ground_cabinet",
        cabinet_dx > 1.1 and cabinet_dy > 0.6 and cabinet_dz > 1.0,
        details=f"cabinet spans {(cabinet_dx, cabinet_dy, cabinet_dz)}",
    )

    rest_main_aabb = ctx.part_world_aabb(main_arm)
    assert rest_main_aabb is not None
    rest_main_dx = rest_main_aabb[1][0] - rest_main_aabb[0][0]
    rest_main_dz = rest_main_aabb[1][2] - rest_main_aabb[0][2]
    ctx.check(
        "main_arm_rest_pose_is_horizontal",
        rest_main_dx > 3.0 and rest_main_dz < 0.35,
        details=f"rest boom spans dx={rest_main_dx:.3f}, dz={rest_main_dz:.3f}",
    )

    with ctx.pose({main_hinge: main_hinge.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="main_hinge_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="main_hinge_lower_no_floating")

    with ctx.pose({main_hinge: main_hinge.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="main_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="main_hinge_upper_no_floating")
        raised_main_aabb = ctx.part_world_aabb(main_arm)
        assert raised_main_aabb is not None
        raised_main_dx = raised_main_aabb[1][0] - raised_main_aabb[0][0]
        raised_main_dz = raised_main_aabb[1][2] - raised_main_aabb[0][2]
        ctx.check(
            "main_arm_raises_upward",
            raised_main_dz > 3.0 and raised_main_dx < 0.9,
            details=f"raised boom spans dx={raised_main_dx:.3f}, dz={raised_main_dz:.3f}",
        )

    with ctx.pose({fold_hinge: fold_hinge.motion_limits.lower}):
        pass

    with ctx.pose({fold_hinge: fold_hinge.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fold_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="fold_hinge_upper_no_floating")

    with ctx.pose({main_hinge: MAIN_ARM_UPPER, fold_hinge: 0.0}):
        raised_straight_tip_aabb = ctx.part_world_aabb(tip_arm)
        assert raised_straight_tip_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="raised_straight_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="raised_straight_pose_no_floating")

    with ctx.pose({main_hinge: MAIN_ARM_UPPER, fold_hinge: FOLD_ARM_LOWER}):
        folded_tip_aabb = ctx.part_world_aabb(tip_arm)
        raised_main_aabb = ctx.part_world_aabb(main_arm)
        assert folded_tip_aabb is not None
        assert raised_main_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="raised_folded_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="raised_folded_pose_no_floating")
        ctx.expect_contact(fold_pin, tip_arm, name="raised_folded_pose_pin_capture")
        ctx.expect_gap(
            tip_arm,
            cabinet,
            axis="z",
            min_gap=0.45,
            name="folded_tip_stays_clear_of_cabinet",
        )
        ctx.check(
            "folded_tip_drops_below_raised_joint_height",
            folded_tip_aabb[0][2] < raised_main_aabb[1][2] - 0.45,
            details=(
                f"folded tip min z {folded_tip_aabb[0][2]:.3f} "
                f"vs raised main max z {raised_main_aabb[1][2]:.3f}"
            ),
        )
        ctx.check(
            "folded_tip_reduces_overall_height_relative_to_straight_raised_pose",
            folded_tip_aabb[1][2] < raised_straight_tip_aabb[1][2] - 2.0,
            details=(
                f"folded tip max z {folded_tip_aabb[1][2]:.3f} "
                f"vs straight raised max z {raised_straight_tip_aabb[1][2]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
