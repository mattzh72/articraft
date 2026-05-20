from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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
    model = ArticulatedObject(name="hinged_transfer_arm")

    steel_dark = model.material("steel_dark", color=(0.23, 0.25, 0.28))
    steel_mid = model.material("steel_mid", color=(0.58, 0.60, 0.62))
    safety_orange = model.material("safety_orange", color=(0.88, 0.50, 0.16))
    polymer_black = model.material("polymer_black", color=(0.12, 0.13, 0.15))

    base = model.part(
        "base",
        inertial=Inertial.from_geometry(
            Box((0.44, 0.44, 0.42)),
            mass=28.0,
            origin=Origin(xyz=(0.0, 0.0, 0.21)),
        ),
    )
    base.visual(
        Cylinder(radius=0.22, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel_dark,
        name="foot",
    )
    base.visual(
        Box((0.18, 0.14, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=steel_mid,
        name="column",
    )
    base.visual(
        Cylinder(radius=0.10, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=steel_dark,
        name="shoulder",
    )

    arm = model.part(
        "arm",
        inertial=Inertial.from_geometry(
            Box((0.72, 0.16, 0.14)),
            mass=12.0,
            origin=Origin(xyz=(0.36, 0.0, 0.06)),
        ),
    )
    arm.visual(
        Cylinder(radius=0.085, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=steel_dark,
        name="pivot_disc",
    )
    arm.visual(
        Box((0.08, 0.14, 0.04)),
        origin=Origin(xyz=(0.04, 0.0, 0.02)),
        material=steel_mid,
        name="rear_block",
    )
    arm.visual(
        Box((0.68, 0.10, 0.012)),
        origin=Origin(xyz=(0.37, 0.0, 0.006)),
        material=steel_mid,
        name="track_floor",
    )
    arm.visual(
        Box((0.68, 0.02, 0.11)),
        origin=Origin(xyz=(0.37, 0.06, 0.055)),
        material=safety_orange,
        name="side_left",
    )
    arm.visual(
        Box((0.68, 0.02, 0.11)),
        origin=Origin(xyz=(0.37, -0.06, 0.055)),
        material=safety_orange,
        name="side_right",
    )
    arm.visual(
        Box((0.02, 0.14, 0.07)),
        origin=Origin(xyz=(0.71, 0.0, 0.035)),
        material=steel_mid,
        name="front_bulkhead",
    )

    carriage = model.part(
        "carriage",
        inertial=Inertial.from_geometry(
            Box((0.16, 0.12, 0.16)),
            mass=5.0,
            origin=Origin(xyz=(0.03, 0.0, 0.08)),
        ),
    )
    carriage.visual(
        Box((0.10, 0.034, 0.01)),
        origin=Origin(xyz=(0.0, 0.025, 0.005)),
        material=polymer_black,
        name="runner_left",
    )
    carriage.visual(
        Box((0.10, 0.034, 0.01)),
        origin=Origin(xyz=(0.0, -0.025, 0.005)),
        material=polymer_black,
        name="runner_right",
    )
    carriage.visual(
        Box((0.11, 0.092, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=steel_dark,
        name="body",
    )
    carriage.visual(
        Box((0.04, 0.04, 0.066)),
        origin=Origin(xyz=(0.02, 0.0, 0.077)),
        material=steel_mid,
        name="post",
    )
    carriage.visual(
        Box((0.16, 0.12, 0.015)),
        origin=Origin(xyz=(0.03, 0.0, 0.1175)),
        material=steel_mid,
        name="saddle",
    )
    carriage.visual(
        Box((0.02, 0.01, 0.026)),
        origin=Origin(xyz=(0.12, 0.013, 0.138)),
        material=steel_dark,
        name="ear_left",
    )
    carriage.visual(
        Box((0.02, 0.01, 0.026)),
        origin=Origin(xyz=(0.12, -0.013, 0.138)),
        material=steel_dark,
        name="ear_right",
    )

    wrist_plate = model.part(
        "wrist_plate",
        inertial=Inertial.from_geometry(
            Box((0.03, 0.07, 0.10)),
            mass=1.0,
            origin=Origin(xyz=(0.015, 0.0, -0.05)),
        ),
    )
    wrist_plate.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="barrel",
    )
    wrist_plate.visual(
        Box((0.018, 0.014, 0.05)),
        origin=Origin(xyz=(0.009, 0.0, -0.025)),
        material=steel_mid,
        name="neck",
    )
    wrist_plate.visual(
        Box((0.008, 0.07, 0.08)),
        origin=Origin(xyz=(0.022, 0.0, -0.06)),
        material=steel_mid,
        name="panel",
    )

    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.18, 0.0, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.35,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_plate,
        origin=Origin(xyz=(0.12, 0.0, 0.138)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-1.1,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    wrist_plate = object_model.get_part("wrist_plate")

    base_pivot = object_model.get_articulation("base_pivot")
    carriage_slide = object_model.get_articulation("carriage_slide")
    wrist_hinge = object_model.get_articulation("wrist_hinge")

    shoulder = base.get_visual("shoulder")
    pivot_disc = arm.get_visual("pivot_disc")
    track_floor = arm.get_visual("track_floor")
    runner_left = carriage.get_visual("runner_left")
    runner_right = carriage.get_visual("runner_right")
    barrel = wrist_plate.get_visual("barrel")
    ear_left = carriage.get_visual("ear_left")
    ear_right = carriage.get_visual("ear_right")

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
    ctx.fail_if_isolated_parts(max_pose_samples=24, name="sampled_pose_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=40)

    ctx.check(
        "base_pivot_axis_is_vertical",
        tuple(base_pivot.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {base_pivot.axis}",
    )
    ctx.check(
        "carriage_slide_axis_is_along_arm",
        tuple(carriage_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {carriage_slide.axis}",
    )
    ctx.check(
        "wrist_hinge_axis_is_lateral",
        tuple(wrist_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {wrist_hinge.axis}",
    )
    ctx.check(
        "carriage_slide_range_realistic",
        carriage_slide.motion_limits is not None
        and isclose(carriage_slide.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and isclose(carriage_slide.motion_limits.upper or 0.0, 0.30, abs_tol=1e-9),
        details=f"unexpected slide limits: {carriage_slide.motion_limits}",
    )
    ctx.check(
        "wrist_hinge_range_realistic",
        wrist_hinge.motion_limits is not None
        and isclose(wrist_hinge.motion_limits.lower or 0.0, -1.1, abs_tol=1e-9)
        and isclose(wrist_hinge.motion_limits.upper or 0.0, 0.0, abs_tol=1e-9),
        details=f"unexpected wrist limits: {wrist_hinge.motion_limits}",
    )

    with ctx.pose({base_pivot: 0.0, carriage_slide: 0.0, wrist_hinge: 0.0}):
        ctx.expect_contact(
            arm,
            base,
            elem_a=pivot_disc,
            elem_b=shoulder,
            name="base_pivot_seats_on_shoulder",
        )
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=runner_left,
            elem_b=track_floor,
            name="left_runner_contacts_track_at_rest",
        )
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=runner_right,
            elem_b=track_floor,
            name="right_runner_contacts_track_at_rest",
        )
        ctx.expect_contact(
            wrist_plate,
            carriage,
            elem_a=barrel,
            elem_b=ear_left,
            name="wrist_barrel_contacts_left_ear_at_rest",
        )
        ctx.expect_contact(
            wrist_plate,
            carriage,
            elem_a=barrel,
            elem_b=ear_right,
            name="wrist_barrel_contacts_right_ear_at_rest",
        )
        ctx.expect_origin_gap(
            arm,
            base,
            axis="z",
            min_gap=0.439,
            max_gap=0.441,
            name="arm_origin_above_base_origin",
        )
        ctx.expect_origin_gap(
            carriage,
            arm,
            axis="x",
            min_gap=0.179,
            max_gap=0.181,
            name="carriage_retracted_offset_along_arm",
        )
        ctx.expect_origin_gap(
            carriage,
            arm,
            axis="z",
            min_gap=0.011,
            max_gap=0.013,
            name="carriage_runs_on_arm_track_height",
        )
        ctx.expect_origin_gap(
            wrist_plate,
            carriage,
            axis="x",
            min_gap=0.119,
            max_gap=0.121,
            name="wrist_hinge_forward_of_carriage_center",
        )
        ctx.expect_origin_gap(
            wrist_plate,
            carriage,
            axis="z",
            min_gap=0.137,
            max_gap=0.139,
            name="wrist_hinge_elevated_above_carriage_track",
        )

    with ctx.pose({carriage_slide: 0.30}):
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=runner_left,
            elem_b=track_floor,
            name="left_runner_contacts_track_fully_extended",
        )
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=runner_right,
            elem_b=track_floor,
            name="right_runner_contacts_track_fully_extended",
        )
        ctx.expect_origin_gap(
            carriage,
            arm,
            axis="x",
            min_gap=0.479,
            max_gap=0.481,
            name="carriage_extension_matches_prismatic_travel",
        )

    with ctx.pose({wrist_hinge: -1.1}):
        ctx.expect_contact(
            wrist_plate,
            carriage,
            elem_a=barrel,
            elem_b=ear_left,
            name="wrist_barrel_contacts_left_ear_folded",
        )
        ctx.expect_contact(
            wrist_plate,
            carriage,
            elem_a=barrel,
            elem_b=ear_right,
            name="wrist_barrel_contacts_right_ear_folded",
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
    for articulation in (base_pivot, carriage_slide, wrist_hinge):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{articulation.name}_lower_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{articulation.name}_upper_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
