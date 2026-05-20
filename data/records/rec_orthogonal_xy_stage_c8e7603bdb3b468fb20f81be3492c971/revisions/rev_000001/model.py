from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_X = 0.42
BASE_Y = 0.26
BASE_H = 0.045

BASE_RAIL_LEN = 0.30
BASE_RAIL_W = 0.030
BASE_RAIL_H = 0.012
GUIDE_Y = 0.072

BRIDGE_UPRIGHT_W = 0.028
BRIDGE_UPRIGHT_D = 0.028
BRIDGE_UPRIGHT_H = 0.150
BRIDGE_UPRIGHT_X = 0.182
BRIDGE_Y = 0.112
BRIDGE_BEAM_X = 0.340
BRIDGE_BEAM_D = 0.030
BRIDGE_BEAM_H = 0.028

LOWER_SLIDE_TRAVEL = 0.050
LOWER_SHOE_X = 0.200
LOWER_SHOE_W = 0.034
LOWER_SHOE_H = 0.014
LOWER_TOP_X = 0.220
LOWER_TOP_Y = 0.190
LOWER_TOP_H = 0.018
LOWER_POCKET_X = 0.120
LOWER_POCKET_Y = 0.090
LOWER_POCKET_D = 0.010
LOWER_POCKET_FLOOR = 0.004
LOWER_PEDESTAL_X = 0.150
LOWER_PEDESTAL_Y = 0.130
LOWER_PEDESTAL_H = 0.012
UPPER_RAIL_X = 0.036
UPPER_RAIL_W = 0.024
UPPER_RAIL_LEN = 0.220
UPPER_RAIL_H = 0.008

UPPER_SLIDE_TRAVEL = 0.040
UPPER_RUNNER_X = 0.032
UPPER_RUNNER_LEN = 0.180
UPPER_RUNNER_H = 0.010
UPPER_BRIDGE_X = 0.100
UPPER_BRIDGE_Y = 0.180
UPPER_BRIDGE_H = 0.012
UPPER_TOP_X = 0.120
UPPER_TOP_Y = 0.220
UPPER_TOP_H = 0.018
UPPER_TOP_POCKET_X = 0.056
UPPER_TOP_POCKET_Y = 0.118
UPPER_TOP_POCKET_D = 0.010
UPPER_TOP_POCKET_FLOOR = 0.004
UPPER_PAD_X = 0.050
UPPER_PAD_Y = 0.070
UPPER_PAD_H = 0.006
PART_FUSE = 0.001


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_xy_translation_table")

    base_color = model.material("painted_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    lower_color = model.material("anodized_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    upper_color = model.material("machined_aluminum", rgba=(0.82, 0.83, 0.85, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_X, BASE_Y, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=base_color,
        name="base_plate",
    )
    base.visual(
        Box((BASE_RAIL_LEN, BASE_RAIL_W, BASE_RAIL_H)),
        origin=Origin(xyz=(0.0, GUIDE_Y, BASE_H + BASE_RAIL_H / 2.0)),
        material=base_color,
        name="base_rail_left",
    )
    base.visual(
        Box((BASE_RAIL_LEN, BASE_RAIL_W, BASE_RAIL_H)),
        origin=Origin(xyz=(0.0, -GUIDE_Y, BASE_H + BASE_RAIL_H / 2.0)),
        material=base_color,
        name="base_rail_right",
    )
    base.visual(
        Box((BRIDGE_UPRIGHT_W, BRIDGE_UPRIGHT_D, BRIDGE_UPRIGHT_H)),
        origin=Origin(
            xyz=(-BRIDGE_UPRIGHT_X, BRIDGE_Y, BASE_H + BRIDGE_UPRIGHT_H / 2.0)
        ),
        material=base_color,
        name="bridge_upright_left",
    )
    base.visual(
        Box((BRIDGE_UPRIGHT_W, BRIDGE_UPRIGHT_D, BRIDGE_UPRIGHT_H)),
        origin=Origin(
            xyz=(BRIDGE_UPRIGHT_X, BRIDGE_Y, BASE_H + BRIDGE_UPRIGHT_H / 2.0)
        ),
        material=base_color,
        name="bridge_upright_right",
    )
    base.visual(
        Box((BRIDGE_BEAM_X, BRIDGE_BEAM_D, BRIDGE_BEAM_H)),
        origin=Origin(
            xyz=(
                0.0,
                BRIDGE_Y,
                BASE_H + BRIDGE_UPRIGHT_H - BRIDGE_BEAM_H / 2.0,
            )
        ),
        material=base_color,
        name="bridge_beam",
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        Box((LOWER_SHOE_X, LOWER_SHOE_W, LOWER_SHOE_H)),
        origin=Origin(xyz=(0.0, GUIDE_Y, LOWER_SHOE_H / 2.0)),
        material=lower_color,
        name="lower_shoe_left",
    )
    lower_carriage.visual(
        Box((LOWER_SHOE_X, LOWER_SHOE_W, LOWER_SHOE_H)),
        origin=Origin(xyz=(0.0, -GUIDE_Y, LOWER_SHOE_H / 2.0)),
        material=lower_color,
        name="lower_shoe_right",
    )
    lower_carriage.visual(
        Box((LOWER_TOP_X, LOWER_TOP_Y, LOWER_TOP_H)),
        origin=Origin(
            xyz=(0.0, 0.0, LOWER_SHOE_H - PART_FUSE + LOWER_TOP_H / 2.0)
        ),
        material=lower_color,
        name="lower_table",
    )
    lower_carriage.visual(
        Box((LOWER_PEDESTAL_X, LOWER_PEDESTAL_Y, LOWER_PEDESTAL_H)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                LOWER_SHOE_H
                + LOWER_TOP_H
                - 2.0 * PART_FUSE
                + LOWER_PEDESTAL_H / 2.0,
            )
        ),
        material=lower_color,
        name="lower_pedestal",
    )
    lower_carriage.visual(
        Box((UPPER_RAIL_W, UPPER_RAIL_LEN, UPPER_RAIL_H)),
        origin=Origin(
            xyz=(
                -UPPER_RAIL_X,
                0.0,
                LOWER_SHOE_H
                + LOWER_TOP_H
                + LOWER_PEDESTAL_H
                - 3.0 * PART_FUSE
                + UPPER_RAIL_H / 2.0,
            )
        ),
        material=lower_color,
        name="upper_stage_rail_left",
    )
    lower_carriage.visual(
        Box((UPPER_RAIL_W, UPPER_RAIL_LEN, UPPER_RAIL_H)),
        origin=Origin(
            xyz=(
                UPPER_RAIL_X,
                0.0,
                LOWER_SHOE_H
                + LOWER_TOP_H
                + LOWER_PEDESTAL_H
                - 3.0 * PART_FUSE
                + UPPER_RAIL_H / 2.0,
            )
        ),
        material=lower_color,
        name="upper_stage_rail_right",
    )

    upper_saddle = model.part("upper_saddle")
    upper_saddle.visual(
        Box((UPPER_RUNNER_X, UPPER_RUNNER_LEN, UPPER_RUNNER_H)),
        origin=Origin(xyz=(-UPPER_RAIL_X, 0.0, UPPER_RUNNER_H / 2.0)),
        material=upper_color,
        name="upper_runner_left",
    )
    upper_saddle.visual(
        Box((UPPER_RUNNER_X, UPPER_RUNNER_LEN, UPPER_RUNNER_H)),
        origin=Origin(xyz=(UPPER_RAIL_X, 0.0, UPPER_RUNNER_H / 2.0)),
        material=upper_color,
        name="upper_runner_right",
    )
    upper_saddle.visual(
        Box((UPPER_BRIDGE_X, UPPER_BRIDGE_Y, UPPER_BRIDGE_H)),
        origin=Origin(
            xyz=(0.0, 0.0, UPPER_RUNNER_H - PART_FUSE + UPPER_BRIDGE_H / 2.0)
        ),
        material=upper_color,
        name="upper_bridge",
    )
    upper_saddle.visual(
        Box((UPPER_TOP_X, UPPER_TOP_Y, UPPER_TOP_H)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                UPPER_RUNNER_H
                + UPPER_BRIDGE_H
                - 2.0 * PART_FUSE
                + UPPER_TOP_H / 2.0,
            )
        ),
        material=upper_color,
        name="upper_top",
    )
    upper_saddle.visual(
        Box((UPPER_PAD_X, UPPER_PAD_Y, UPPER_PAD_H)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                UPPER_RUNNER_H
                + UPPER_BRIDGE_H
                + UPPER_TOP_H
                - 3.0 * PART_FUSE
                + UPPER_PAD_H / 2.0,
            )
        ),
        material=upper_color,
        name="upper_pad",
    )

    model.articulation(
        "base_to_lower_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + BASE_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.15,
            lower=-LOWER_SLIDE_TRAVEL,
            upper=LOWER_SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "lower_carriage_to_upper_saddle",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_saddle,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                LOWER_SHOE_H
                + LOWER_TOP_H
                + LOWER_PEDESTAL_H
                + UPPER_RAIL_H
                - 3.0 * PART_FUSE
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.12,
            lower=-UPPER_SLIDE_TRAVEL,
            upper=UPPER_SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_carriage = object_model.get_part("lower_carriage")
    upper_saddle = object_model.get_part("upper_saddle")
    base_plate = base.get_visual("base_plate")
    bridge_beam = base.get_visual("bridge_beam")
    lower_slide = object_model.get_articulation("base_to_lower_carriage")
    upper_slide = object_model.get_articulation("lower_carriage_to_upper_saddle")

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
        "both stages use prismatic joints",
        lower_slide.joint_type == ArticulationType.PRISMATIC
        and upper_slide.joint_type == ArticulationType.PRISMATIC,
        details=(
            f"lower={lower_slide.joint_type!r}, upper={upper_slide.joint_type!r}"
        ),
    )
    ctx.check(
        "slide axes are horizontal and orthogonal",
        tuple(lower_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(upper_slide.axis) == (0.0, 1.0, 0.0),
        details=f"lower axis={lower_slide.axis}, upper axis={upper_slide.axis}",
    )

    with ctx.pose({lower_slide: 0.0, upper_slide: 0.0}):
        ctx.expect_contact(
            lower_carriage,
            base,
            name="lower carriage is supported by base rails",
        )
        ctx.expect_contact(
            upper_saddle,
            lower_carriage,
            name="upper saddle is supported by lower carriage rails",
        )
        ctx.expect_gap(
            upper_saddle,
            base,
            axis="z",
            min_gap=0.050,
            negative_elem=base_plate,
            name="upper saddle stays above the base plate",
        )
        ctx.expect_gap(
            base,
            upper_saddle,
            axis="z",
            min_gap=0.012,
            positive_elem=bridge_beam,
            name="bridge beam clears the upper saddle",
        )

    rest_lower = ctx.part_world_position(lower_carriage)
    rest_upper = ctx.part_world_position(upper_saddle)

    with ctx.pose({lower_slide: 0.030, upper_slide: 0.0}):
        moved_lower = ctx.part_world_position(lower_carriage)
        moved_upper = ctx.part_world_position(upper_saddle)
        lower_ok = (
            rest_lower is not None
            and moved_lower is not None
            and abs((moved_lower[0] - rest_lower[0]) - 0.030) <= 1e-6
            and abs(moved_lower[1] - rest_lower[1]) <= 1e-6
            and abs(moved_lower[2] - rest_lower[2]) <= 1e-6
        )
        upper_follow_ok = (
            rest_upper is not None
            and moved_upper is not None
            and abs((moved_upper[0] - rest_upper[0]) - 0.030) <= 1e-6
            and abs(moved_upper[1] - rest_upper[1]) <= 1e-6
            and abs(moved_upper[2] - rest_upper[2]) <= 1e-6
        )
        ctx.check(
            "lower carriage translates only along x",
            lower_ok,
            details=f"rest={rest_lower}, moved={moved_lower}",
        )
        ctx.check(
            "upper assembly follows lower x translation",
            upper_follow_ok,
            details=f"rest={rest_upper}, moved={moved_upper}",
        )

    with ctx.pose({lower_slide: 0.0, upper_slide: 0.025}):
        moved_lower = ctx.part_world_position(lower_carriage)
        moved_upper = ctx.part_world_position(upper_saddle)
        upper_ok = (
            rest_lower is not None
            and moved_lower is not None
            and rest_upper is not None
            and moved_upper is not None
            and abs(moved_lower[0] - rest_lower[0]) <= 1e-6
            and abs(moved_lower[1] - rest_lower[1]) <= 1e-6
            and abs((moved_upper[1] - rest_upper[1]) - 0.025) <= 1e-6
            and abs(moved_upper[0] - rest_upper[0]) <= 1e-6
            and abs(moved_upper[2] - rest_upper[2]) <= 1e-6
        )
        ctx.check(
            "upper saddle translates only along y",
            upper_ok,
            details=f"lower rest={rest_lower}, lower moved={moved_lower}, upper rest={rest_upper}, upper moved={moved_upper}",
        )

    with ctx.pose({lower_slide: LOWER_SLIDE_TRAVEL, upper_slide: UPPER_SLIDE_TRAVEL}):
        ctx.expect_contact(
            lower_carriage,
            base,
            name="lower carriage remains supported at positive travel",
        )
        ctx.expect_contact(
            upper_saddle,
            lower_carriage,
            name="upper saddle remains supported at positive travel",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no interpenetration at positive travel")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
