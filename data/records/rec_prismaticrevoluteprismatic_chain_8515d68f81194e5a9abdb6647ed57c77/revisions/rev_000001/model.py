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
)


BASE_PLATE_L = 0.86
BASE_PLATE_W = 0.12
BASE_PLATE_H = 0.02

GUIDE_RAIL_L = 0.78
GUIDE_RAIL_W = 0.08
GUIDE_RAIL_H = 0.02
GUIDE_TOP_Z = BASE_PLATE_H + GUIDE_RAIL_H

CARRIAGE_L = 0.16
CARRIAGE_W = 0.12
CARRIAGE_H = 0.05

HINGE_CHEEK_L = 0.05
HINGE_CHEEK_T = 0.018
HINGE_CHEEK_H = 0.09
HINGE_CHEEK_CENTER_Y = (CARRIAGE_W * 0.5) + (HINGE_CHEEK_T * 0.5)
PIVOT_CENTER_Y = HINGE_CHEEK_CENTER_Y + 0.021
PIVOT_CENTER_Z = CARRIAGE_H + 0.045
PIVOT_HUB_R = 0.032
PIVOT_HUB_T = 0.024

ARM_BEAM_OFFSET_Y = 0.018
ARM_FLOOR_L = 0.36
ARM_FLOOR_W = 0.05
ARM_FLOOR_T = 0.01
ARM_FLOOR_CENTER_X = 0.22
ARM_FLOOR_CENTER_Z = -0.03
ARM_RAIL_L = 0.34
ARM_RAIL_T = 0.01
ARM_RAIL_H = 0.03
ARM_RAIL_CENTER_X = 0.22
ARM_RAIL_CENTER_Z = -0.01
ARM_RAIL_CENTER_Y = ARM_BEAM_OFFSET_Y + 0.02
ARM_TIP_HOUSING_L = 0.04
ARM_TIP_HOUSING_W = 0.042
ARM_TIP_HOUSING_H = 0.026
ARM_TIP_HOUSING_CENTER_X = 0.38
ARM_TIP_HOUSING_CENTER_Y = ARM_BEAM_OFFSET_Y
ARM_TIP_HOUSING_CENTER_Z = -0.015
ARM_ROOT_WEB_L = 0.06
ARM_ROOT_WEB_W = 0.036
ARM_ROOT_WEB_H = 0.04
ARM_ROOT_WEB_CENTER_X = 0.03
ARM_ROOT_WEB_CENTER_Y = ARM_BEAM_OFFSET_Y
ARM_ROOT_WEB_CENTER_Z = -0.01

END_SLIDE_L = 0.18
END_SLIDE_W = 0.03
END_SLIDE_H = 0.02
END_SLIDE_JOINT_X = 0.40
END_SLIDE_BODY_OFFSET_X = END_SLIDE_L * 0.5
END_SLIDE_CENTER_Y = ARM_BEAM_OFFSET_Y
END_SLIDE_CENTER_Z = -0.015
END_PAD_L = 0.025
END_PAD_W = 0.05
END_PAD_H = 0.028
END_PAD_CENTER_X = END_SLIDE_L + (END_PAD_L * 0.5)
END_PAD_CENTER_Z = 0.004

BASE_TRAVEL = 0.48
ARM_SWING_MIN = 0.0
ARM_SWING_MAX = 1.2
END_TRAVEL = 0.12


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-9) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_rotary_transfer_chain")

    steel_dark = model.material("steel_dark", color=(0.22, 0.24, 0.27, 1.0))
    steel_mid = model.material("steel_mid", color=(0.56, 0.60, 0.64, 1.0))
    steel_light = model.material("steel_light", color=(0.76, 0.79, 0.82, 1.0))
    blue_paint = model.material("blue_paint", color=(0.19, 0.36, 0.68, 1.0))
    amber = model.material("amber", color=(0.91, 0.62, 0.18, 1.0))

    base = model.part("base_guide")
    base.visual(
        Box((BASE_PLATE_L, BASE_PLATE_W, BASE_PLATE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_H * 0.5)),
        material=steel_dark,
        name="base_plate",
    )
    base.visual(
        Box((GUIDE_RAIL_L, GUIDE_RAIL_W, GUIDE_RAIL_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_H + (GUIDE_RAIL_H * 0.5))),
        material=steel_mid,
        name="guide_rail",
    )

    carriage = model.part("sliding_carriage")
    carriage.visual(
        Box((CARRIAGE_L, CARRIAGE_W, CARRIAGE_H)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_H * 0.5)),
        material=steel_light,
        name="carriage_body",
    )
    carriage.visual(
        Box((HINGE_CHEEK_L, HINGE_CHEEK_T, HINGE_CHEEK_H)),
        origin=Origin(
            xyz=(
                0.0,
                HINGE_CHEEK_CENTER_Y,
                CARRIAGE_H + (HINGE_CHEEK_H * 0.5),
            )
        ),
        material=steel_mid,
        name="hinge_cheek",
    )

    arm = model.part("middle_arm")
    arm.visual(
        Cylinder(radius=PIVOT_HUB_R, length=PIVOT_HUB_T),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=blue_paint,
        name="pivot_hub",
    )
    arm.visual(
        Box((ARM_ROOT_WEB_L, ARM_ROOT_WEB_W, ARM_ROOT_WEB_H)),
        origin=Origin(
            xyz=(
                ARM_ROOT_WEB_CENTER_X,
                ARM_ROOT_WEB_CENTER_Y,
                ARM_ROOT_WEB_CENTER_Z,
            )
        ),
        material=blue_paint,
        name="arm_root_web",
    )
    arm.visual(
        Box((ARM_FLOOR_L, ARM_FLOOR_W, ARM_FLOOR_T)),
        origin=Origin(
            xyz=(
                ARM_FLOOR_CENTER_X,
                ARM_BEAM_OFFSET_Y,
                ARM_FLOOR_CENTER_Z,
            )
        ),
        material=blue_paint,
        name="arm_floor",
    )
    arm.visual(
        Box((ARM_RAIL_L, ARM_RAIL_T, ARM_RAIL_H)),
        origin=Origin(
            xyz=(
                ARM_RAIL_CENTER_X,
                ARM_RAIL_CENTER_Y,
                ARM_RAIL_CENTER_Z,
            )
        ),
        material=blue_paint,
        name="arm_outer_rail",
    )
    arm.visual(
        Box((ARM_RAIL_L, ARM_RAIL_T, ARM_RAIL_H)),
        origin=Origin(
            xyz=(
                ARM_RAIL_CENTER_X,
                ARM_BEAM_OFFSET_Y - 0.02,
                ARM_RAIL_CENTER_Z,
            )
        ),
        material=blue_paint,
        name="arm_inner_rail",
    )
    arm.visual(
        Box((ARM_TIP_HOUSING_L, ARM_TIP_HOUSING_W, ARM_TIP_HOUSING_H)),
        origin=Origin(
            xyz=(
                ARM_TIP_HOUSING_CENTER_X,
                ARM_TIP_HOUSING_CENTER_Y,
                ARM_TIP_HOUSING_CENTER_Z,
            )
        ),
        material=steel_mid,
        name="arm_tip_housing",
    )

    end_slide = model.part("end_slide")
    end_slide.visual(
        Box((END_SLIDE_L, END_SLIDE_W, END_SLIDE_H)),
        origin=Origin(xyz=(END_SLIDE_BODY_OFFSET_X, 0.0, 0.0)),
        material=steel_light,
        name="slide_body",
    )
    end_slide.visual(
        Box((END_PAD_L, END_PAD_W, END_PAD_H)),
        origin=Origin(xyz=(END_PAD_CENTER_X, 0.0, END_PAD_CENTER_Z)),
        material=amber,
        name="end_pad",
    )

    model.articulation(
        "base_to_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.24, 0.0, GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.8,
            lower=0.0,
            upper=BASE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, PIVOT_CENTER_Y, PIVOT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.4,
            lower=ARM_SWING_MIN,
            upper=ARM_SWING_MAX,
        ),
    )
    model.articulation(
        "arm_to_end_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=end_slide,
        origin=Origin(
            xyz=(
                END_SLIDE_JOINT_X,
                END_SLIDE_CENTER_Y,
                END_SLIDE_CENTER_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.5,
            lower=0.0,
            upper=END_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)

    base = object_model.get_part("base_guide")
    carriage = object_model.get_part("sliding_carriage")
    arm = object_model.get_part("middle_arm")
    end_slide = object_model.get_part("end_slide")

    base_slide = object_model.get_articulation("base_to_carriage_slide")
    arm_hinge = object_model.get_articulation("carriage_to_arm_hinge")
    end_slide_joint = object_model.get_articulation("arm_to_end_slide")

    base.get_visual("guide_rail")
    carriage.get_visual("carriage_body")
    carriage.get_visual("hinge_cheek")
    arm.get_visual("pivot_hub")
    arm.get_visual("arm_floor")
    arm.get_visual("arm_tip_housing")
    end_slide.get_visual("slide_body")

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
        "part_count",
        len(object_model.parts) == 4,
        f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "articulation_count",
        len(object_model.articulations) == 3,
        f"expected 3 articulations, found {len(object_model.articulations)}",
    )
    ctx.check(
        "base_slide_axis",
        _axis_matches(base_slide.axis, (1.0, 0.0, 0.0)),
        f"axis={base_slide.axis}",
    )
    ctx.check(
        "arm_hinge_axis",
        _axis_matches(arm_hinge.axis, (0.0, 1.0, 0.0)),
        f"axis={arm_hinge.axis}",
    )
    ctx.check(
        "end_slide_axis",
        _axis_matches(end_slide_joint.axis, (1.0, 0.0, 0.0)),
        f"axis={end_slide_joint.axis}",
    )

    ctx.fail_if_articulation_overlaps(max_pose_samples=48)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    for pose_name, slide_q in (
        ("lower", 0.0),
        ("upper", BASE_TRAVEL),
    ):
        with ctx.pose({base_slide: slide_q}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"base_slide_{pose_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"base_slide_{pose_name}_no_floating")
            ctx.expect_gap(
                carriage,
                base,
                axis="z",
                max_gap=1e-6,
                max_penetration=0.0,
                positive_elem="carriage_body",
                negative_elem="guide_rail",
                name=f"carriage_seated_on_guide_{pose_name}",
            )
            ctx.expect_overlap(
                carriage,
                base,
                axes="xy",
                min_overlap=0.08,
                elem_a="carriage_body",
                elem_b="guide_rail",
                name=f"carriage_tracks_guide_{pose_name}",
            )

    for pose_name, angle in (
        ("lower", ARM_SWING_MIN),
        ("upper", ARM_SWING_MAX),
    ):
        with ctx.pose({base_slide: BASE_TRAVEL * 0.5, arm_hinge: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"arm_hinge_{pose_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"arm_hinge_{pose_name}_no_floating")
            ctx.expect_contact(
                arm,
                carriage,
                elem_a="pivot_hub",
                elem_b="hinge_cheek",
                name=f"arm_hub_contacts_cheek_{pose_name}",
            )
            ctx.expect_overlap(
                arm,
                carriage,
                axes="xz",
                min_overlap=0.03,
                elem_a="pivot_hub",
                elem_b="hinge_cheek",
                name=f"arm_hub_aligned_with_cheek_{pose_name}",
            )

    for pose_name, extension in (
        ("lower", 0.0),
        ("upper", END_TRAVEL),
    ):
        with ctx.pose(
            {
                base_slide: BASE_TRAVEL * 0.5,
                arm_hinge: 0.0,
                end_slide_joint: extension,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"end_slide_{pose_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"end_slide_{pose_name}_no_floating")
            ctx.expect_overlap(
                end_slide,
                arm,
                axes="yz",
                min_overlap=0.018,
                elem_a="slide_body",
                elem_b="arm_tip_housing",
                name=f"end_slide_aligned_with_housing_{pose_name}",
            )
            ctx.expect_gap(
                end_slide,
                arm,
                axis="x",
                min_gap=extension,
                max_gap=extension + 1e-6,
                positive_elem="slide_body",
                negative_elem="arm_tip_housing",
                name=f"end_slide_extension_gap_{pose_name}",
            )
            if extension == 0.0:
                ctx.expect_contact(
                    end_slide,
                    arm,
                    elem_a="slide_body",
                    elem_b="arm_tip_housing",
                    name="end_slide_retracted_contacts_housing",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
