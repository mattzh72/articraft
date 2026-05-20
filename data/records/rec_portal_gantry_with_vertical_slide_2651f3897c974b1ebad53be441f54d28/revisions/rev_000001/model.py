from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


RAIL_CENTER_X = 0.55
RAIL_BODY_WIDTH = 0.12
RAIL_LENGTH = 1.60
RAIL_BODY_HEIGHT = 0.08
RAIL_CAP_WIDTH = 0.05
RAIL_CAP_HEIGHT = 0.015
RAIL_TOP_Z = RAIL_BODY_HEIGHT + RAIL_CAP_HEIGHT

GANTRY_TRAVEL = 0.48
CARRIAGE_TRAVEL = 0.34
TOOL_TRAVEL = 0.28

GANTRY_SIDE_DEPTH = 0.18
GANTRY_SIDE_WIDTH = 0.12
GANTRY_SIDE_HEIGHT = 1.02
GANTRY_FOOT_WIDTH = 0.18
GANTRY_FOOT_DEPTH = 0.20
GANTRY_FOOT_HEIGHT = 0.10
BEAM_LENGTH = 1.04
BEAM_DEPTH = 0.12
BEAM_HEIGHT = 0.18
BEAM_CENTER_Z = 0.93
BEAM_FRONT_Y = 0.09
BEAM_GUIDE_DEPTH = 0.03
BEAM_GUIDE_CENTER_Y = BEAM_FRONT_Y - BEAM_GUIDE_DEPTH / 2.0

CARRIAGE_WIDTH = 0.24
CARRIAGE_BODY_DEPTH = 0.06
CARRIAGE_BODY_CENTER_Y = 0.13
CARRIAGE_HEIGHT = 0.26
CARRIAGE_GUIDE_WIDTH = 0.05
CARRIAGE_GUIDE_DEPTH = 0.03
CARRIAGE_GUIDE_HEIGHT = 0.40
CARRIAGE_MOUNT_DEPTH = 0.08
CARRIAGE_MOUNT_CENTER_Y = 0.17
CARRIAGE_TOOL_ORIGIN_Y = 0.18

SLIDE_WIDTH = 0.18
SLIDE_PLATE_DEPTH = 0.04
SLIDE_PLATE_HEIGHT = 0.42
SLIDE_PLATE_CENTER_Z = -0.09
SLIDE_RUNNER_WIDTH = 0.035
SLIDE_RUNNER_DEPTH = 0.02
SLIDE_RUNNER_HEIGHT = 0.40
SLIDE_RUNNER_CENTER_Z = 0.0
SPINDLE_BODY_WIDTH = 0.10
SPINDLE_BODY_DEPTH = 0.10
SPINDLE_BODY_HEIGHT = 0.18
SPINDLE_BODY_CENTER_Z = -0.24
SPINDLE_CENTER_Y = 0.09
COLLET_RADIUS = 0.02
COLLET_LENGTH = 0.08
COLLET_CENTER_Z = -0.37


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) <= 1e-9 for a, b in zip(actual, expected))


def _check_prismatic_joint(
    ctx: TestContext,
    joint,
    *,
    name: str,
    axis: tuple[float, float, float],
    lower: float,
    upper: float,
) -> None:
    limits = joint.motion_limits
    ok = (
        joint.articulation_type == ArticulationType.PRISMATIC
        and _axis_matches(joint.axis, axis)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - lower) <= 1e-9
        and abs(limits.upper - upper) <= 1e-9
    )
    ctx.check(
        name,
        ok,
        details=(
            f"type={joint.articulation_type}, axis={joint.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_module")

    machine_gray = model.material("machine_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.11, 0.12, 0.13, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.92, 0.43, 0.12, 1.0))
    spindle_black = model.material("spindle_black", rgba=(0.16, 0.16, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((RAIL_BODY_WIDTH, RAIL_LENGTH, RAIL_BODY_HEIGHT)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, RAIL_BODY_HEIGHT / 2.0)),
        material=steel_dark,
        name="left_rail",
    )
    base.visual(
        Box((RAIL_BODY_WIDTH, RAIL_LENGTH, RAIL_BODY_HEIGHT)),
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, RAIL_BODY_HEIGHT / 2.0)),
        material=steel_dark,
        name="right_rail",
    )
    base.visual(
        Box((2.0 * RAIL_CENTER_X + RAIL_BODY_WIDTH, 0.16, 0.06)),
        origin=Origin(xyz=(0.0, -0.66, 0.03)),
        material=steel_dark,
        name="front_crossmember",
    )
    base.visual(
        Box((2.0 * RAIL_CENTER_X + RAIL_BODY_WIDTH, 0.18, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=steel_dark,
        name="middle_crossmember",
    )
    base.visual(
        Box((2.0 * RAIL_CENTER_X + RAIL_BODY_WIDTH, 0.16, 0.06)),
        origin=Origin(xyz=(0.0, 0.66, 0.03)),
        material=steel_dark,
        name="rear_crossmember",
    )
    base.visual(
        Box((RAIL_CAP_WIDTH, 1.50, RAIL_CAP_HEIGHT)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, RAIL_BODY_HEIGHT + RAIL_CAP_HEIGHT / 2.0)),
        material=machine_gray,
        name="left_rail_cap",
    )
    base.visual(
        Box((RAIL_CAP_WIDTH, 1.50, RAIL_CAP_HEIGHT)),
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, RAIL_BODY_HEIGHT + RAIL_CAP_HEIGHT / 2.0)),
        material=machine_gray,
        name="right_rail_cap",
    )

    gantry = model.part("gantry")
    gantry.visual(
        Box((GANTRY_FOOT_WIDTH, GANTRY_FOOT_DEPTH, GANTRY_FOOT_HEIGHT)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, GANTRY_FOOT_HEIGHT / 2.0)),
        material=machine_gray,
        name="left_foot",
    )
    gantry.visual(
        Box((GANTRY_FOOT_WIDTH, GANTRY_FOOT_DEPTH, GANTRY_FOOT_HEIGHT)),
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, GANTRY_FOOT_HEIGHT / 2.0)),
        material=machine_gray,
        name="right_foot",
    )
    gantry.visual(
        Box((GANTRY_SIDE_WIDTH, GANTRY_SIDE_DEPTH, GANTRY_SIDE_HEIGHT)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, GANTRY_SIDE_HEIGHT / 2.0)),
        material=machine_gray,
        name="left_upright",
    )
    gantry.visual(
        Box((GANTRY_SIDE_WIDTH, GANTRY_SIDE_DEPTH, GANTRY_SIDE_HEIGHT)),
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, GANTRY_SIDE_HEIGHT / 2.0)),
        material=machine_gray,
        name="right_upright",
    )
    gantry.visual(
        Box((BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BEAM_CENTER_Z)),
        material=machine_gray,
        name="bridge_beam",
    )
    gantry.visual(
        Box((BEAM_LENGTH - 0.12, BEAM_GUIDE_DEPTH, 0.08)),
        origin=Origin(xyz=(0.0, BEAM_GUIDE_CENTER_Y, BEAM_CENTER_Z)),
        material=steel_dark,
        name="beam_guide",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_GUIDE_WIDTH, CARRIAGE_GUIDE_DEPTH, CARRIAGE_GUIDE_HEIGHT)),
        origin=Origin(xyz=(-0.075, CARRIAGE_GUIDE_DEPTH / 2.0, 0.0)),
        material=safety_orange,
        name="left_runner",
    )
    carriage.visual(
        Box((CARRIAGE_GUIDE_WIDTH, CARRIAGE_GUIDE_DEPTH, CARRIAGE_GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.075, CARRIAGE_GUIDE_DEPTH / 2.0, 0.0)),
        material=safety_orange,
        name="right_runner",
    )
    carriage.visual(
        Box((CARRIAGE_WIDTH, 0.08, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.07, 0.0)),
        material=safety_orange,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.18, CARRIAGE_MOUNT_DEPTH, 0.34)),
        origin=Origin(xyz=(0.0, CARRIAGE_TOOL_ORIGIN_Y - CARRIAGE_MOUNT_DEPTH / 2.0, -0.01)),
        material=anodized_black,
        name="tool_mount_plate",
    )

    tool_slide = model.part("tool_slide")
    tool_slide.visual(
        Box((SLIDE_WIDTH, SLIDE_PLATE_DEPTH, SLIDE_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, SLIDE_PLATE_DEPTH / 2.0, SLIDE_PLATE_CENTER_Z)),
        material=anodized_black,
        name="slide_plate",
    )
    tool_slide.visual(
        Box((SLIDE_RUNNER_WIDTH, SLIDE_RUNNER_DEPTH, SLIDE_RUNNER_HEIGHT)),
        origin=Origin(xyz=(-0.055, 0.05, SLIDE_RUNNER_CENTER_Z)),
        material=machine_gray,
        name="left_slide_runner",
    )
    tool_slide.visual(
        Box((SLIDE_RUNNER_WIDTH, SLIDE_RUNNER_DEPTH, SLIDE_RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.055, 0.05, SLIDE_RUNNER_CENTER_Z)),
        material=machine_gray,
        name="right_slide_runner",
    )
    tool_slide.visual(
        Box((SPINDLE_BODY_WIDTH, SPINDLE_BODY_DEPTH, SPINDLE_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, SPINDLE_CENTER_Y, SPINDLE_BODY_CENTER_Z)),
        material=spindle_black,
        name="spindle_body",
    )
    tool_slide.visual(
        Cylinder(radius=COLLET_RADIUS, length=COLLET_LENGTH),
        origin=Origin(xyz=(0.0, SPINDLE_CENTER_Y, COLLET_CENTER_Z)),
        material=spindle_black,
        name="cutter_collet",
    )

    model.articulation(
        "base_to_gantry",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gantry,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.60,
            lower=-GANTRY_TRAVEL,
            upper=GANTRY_TRAVEL,
        ),
    )

    model.articulation(
        "gantry_to_carriage",
        ArticulationType.PRISMATIC,
        parent=gantry,
        child=carriage,
        origin=Origin(xyz=(0.0, BEAM_FRONT_Y, BEAM_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=650.0,
            velocity=0.45,
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_tool_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=tool_slide,
        origin=Origin(xyz=(0.0, CARRIAGE_TOOL_ORIGIN_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1100.0,
            velocity=0.25,
            lower=-TOOL_TRAVEL,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    gantry = object_model.get_part("gantry")
    carriage = object_model.get_part("carriage")
    tool_slide = object_model.get_part("tool_slide")

    gantry_slide = object_model.get_articulation("base_to_gantry")
    carriage_slide = object_model.get_articulation("gantry_to_carriage")
    tool_axis = object_model.get_articulation("carriage_to_tool_slide")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=36)

    _check_prismatic_joint(
        ctx,
        gantry_slide,
        name="gantry_slide_axis_and_limits",
        axis=(0.0, 1.0, 0.0),
        lower=-GANTRY_TRAVEL,
        upper=GANTRY_TRAVEL,
    )
    _check_prismatic_joint(
        ctx,
        carriage_slide,
        name="carriage_slide_axis_and_limits",
        axis=(1.0, 0.0, 0.0),
        lower=-CARRIAGE_TRAVEL,
        upper=CARRIAGE_TRAVEL,
    )
    _check_prismatic_joint(
        ctx,
        tool_axis,
        name="tool_slide_axis_and_limits",
        axis=(0.0, 0.0, 1.0),
        lower=-TOOL_TRAVEL,
        upper=0.0,
    )

    ctx.expect_contact(
        gantry,
        base,
        elem_a="left_foot",
        elem_b="left_rail_cap",
        name="left_foot_contacts_left_rail",
    )
    ctx.expect_contact(
        gantry,
        base,
        elem_a="right_foot",
        elem_b="right_rail_cap",
        name="right_foot_contacts_right_rail",
    )
    ctx.expect_gap(
        gantry,
        base,
        axis="z",
        positive_elem="left_foot",
        negative_elem="left_rail_cap",
        max_gap=0.001,
        max_penetration=1e-5,
        name="left_foot_seated_on_left_rail",
    )
    ctx.expect_gap(
        carriage,
        gantry,
        axis="y",
        positive_elem="left_runner",
        negative_elem="beam_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="left_runner_runs_on_beam_guide",
    )
    ctx.expect_gap(
        carriage,
        gantry,
        axis="y",
        positive_elem="right_runner",
        negative_elem="beam_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="right_runner_runs_on_beam_guide",
    )
    ctx.expect_contact(
        carriage,
        gantry,
        elem_a="left_runner",
        elem_b="beam_guide",
        name="left_runner_contacts_beam_guide",
    )
    ctx.expect_contact(
        carriage,
        gantry,
        elem_a="right_runner",
        elem_b="beam_guide",
        name="right_runner_contacts_beam_guide",
    )
    ctx.expect_contact(
        carriage,
        tool_slide,
        elem_a="tool_mount_plate",
        elem_b="slide_plate",
        name="tool_slide_contacts_carriage",
    )
    ctx.expect_gap(
        tool_slide,
        carriage,
        axis="y",
        positive_elem="slide_plate",
        negative_elem="tool_mount_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="tool_slide_runs_on_carriage_face",
    )
    ctx.expect_origin_gap(
        carriage,
        base,
        axis="z",
        min_gap=0.85,
        name="carriage_is_mounted_high_above_base",
    )
    ctx.expect_origin_gap(
        tool_slide,
        base,
        axis="z",
        min_gap=0.75,
        name="tool_axis_origin_above_base",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    for joint, prefix, parent_part, child_part, axis, child_elem, parent_elem, max_penetration in (
        (gantry_slide, "gantry_slide", base, gantry, "z", "left_foot", "left_rail_cap", 1e-5),
        (carriage_slide, "carriage_slide", gantry, carriage, "y", "left_runner", "beam_guide", 0.0),
        (tool_axis, "tool_slide", carriage, tool_slide, "y", "slide_plate", "tool_mount_plate", 0.0),
    ):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{prefix}_lower_no_floating")
            ctx.expect_contact(
                parent_part,
                child_part,
                elem_a=parent_elem,
                elem_b=child_elem,
                name=f"{prefix}_lower_contact",
            )
            ctx.expect_gap(
                child_part,
                parent_part,
                axis=axis,
                positive_elem=child_elem,
                negative_elem=parent_elem,
                max_gap=0.001,
                max_penetration=max_penetration,
                name=f"{prefix}_lower_guided_gap",
            )
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{prefix}_upper_no_floating")
            ctx.expect_contact(
                parent_part,
                child_part,
                elem_a=parent_elem,
                elem_b=child_elem,
                name=f"{prefix}_upper_contact",
            )
            ctx.expect_gap(
                child_part,
                parent_part,
                axis=axis,
                positive_elem=child_elem,
                negative_elem=parent_elem,
                max_gap=0.001,
                max_penetration=max_penetration,
                name=f"{prefix}_upper_guided_gap",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
