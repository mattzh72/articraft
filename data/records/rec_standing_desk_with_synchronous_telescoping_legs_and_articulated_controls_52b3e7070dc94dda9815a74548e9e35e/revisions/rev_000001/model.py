from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq
# from sdk import mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_standing_desk")

    model.material("powder_coat", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("steel_dark", rgba=(0.27, 0.28, 0.30, 1.0))
    model.material("top_laminate", rgba=(0.70, 0.63, 0.52, 1.0))
    model.material("plastic_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("button_light", rgba=(0.82, 0.83, 0.85, 1.0))
    model.material("power_blue", rgba=(0.30, 0.52, 0.92, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.08, 0.486, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material="powder_coat",
        name="stretcher",
    )
    frame.visual(
        Box((0.12, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material="powder_coat",
        name="hub",
    )

    left_outer = model.part("left_outer")
    left_outer.visual(
        Box((0.56, 0.09, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="powder_coat",
        name="foot",
    )
    left_outer.visual(
        Box((0.10, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="powder_coat",
        name="base_block",
    )
    left_outer.visual(
        Box((0.006, 0.06, 0.60)),
        origin=Origin(xyz=(0.037, 0.0, 0.38)),
        material="steel_dark",
        name="front_wall",
    )
    left_outer.visual(
        Box((0.006, 0.06, 0.60)),
        origin=Origin(xyz=(-0.037, 0.0, 0.38)),
        material="steel_dark",
        name="rear_wall",
    )
    left_outer.visual(
        Box((0.068, 0.006, 0.60)),
        origin=Origin(xyz=(0.0, 0.027, 0.38)),
        material="steel_dark",
        name="inboard_wall",
    )
    left_outer.visual(
        Box((0.068, 0.006, 0.60)),
        origin=Origin(xyz=(0.0, -0.027, 0.38)),
        material="steel_dark",
        name="outboard_wall",
    )

    right_outer = model.part("right_outer")
    right_outer.visual(
        Box((0.56, 0.09, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="powder_coat",
        name="foot",
    )
    right_outer.visual(
        Box((0.10, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="powder_coat",
        name="base_block",
    )
    right_outer.visual(
        Box((0.006, 0.06, 0.60)),
        origin=Origin(xyz=(0.037, 0.0, 0.38)),
        material="steel_dark",
        name="front_wall",
    )
    right_outer.visual(
        Box((0.006, 0.06, 0.60)),
        origin=Origin(xyz=(-0.037, 0.0, 0.38)),
        material="steel_dark",
        name="rear_wall",
    )
    right_outer.visual(
        Box((0.068, 0.006, 0.60)),
        origin=Origin(xyz=(0.0, 0.027, 0.38)),
        material="steel_dark",
        name="outboard_wall",
    )
    right_outer.visual(
        Box((0.068, 0.006, 0.60)),
        origin=Origin(xyz=(0.0, -0.027, 0.38)),
        material="steel_dark",
        name="inboard_wall",
    )

    left_stage = model.part("left_stage")
    left_stage.visual(
        Box((0.068, 0.048, 0.64)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="steel_dark",
        name="stage",
    )
    right_stage = model.part("right_stage")
    right_stage.visual(
        Box((0.068, 0.048, 0.64)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="steel_dark",
        name="stage",
    )

    lift_beam = model.part("lift_beam")
    lift_beam.visual(
        Box((0.10, 0.62, 0.04)),
        origin=Origin(xyz=(0.0, 0.27, 0.02)),
        material="powder_coat",
        name="beam",
    )
    lift_beam.visual(
        Box((0.03, 0.05, 0.14)),
        origin=Origin(xyz=(-0.03, 0.00, 0.09)),
        material="powder_coat",
        name="left_riser",
    )
    lift_beam.visual(
        Box((0.03, 0.05, 0.14)),
        origin=Origin(xyz=(-0.03, 0.54, 0.09)),
        material="powder_coat",
        name="right_riser",
    )
    lift_beam.visual(
        Cylinder(radius=0.018, length=0.62),
        origin=Origin(xyz=(-0.05, 0.27, 0.16), rpy=(1.57079632679, 0.0, 0.0)),
        material="steel_dark",
        name="hinge_beam",
    )

    work_surface = model.part("work_surface")
    work_surface.visual(
        Box((0.72, 1.12, 0.028)),
        origin=Origin(xyz=(0.39, 0.0, 0.014)),
        material="top_laminate",
        name="top",
    )
    work_surface.visual(
        Box((0.03, 1.06, 0.055)),
        origin=Origin(xyz=(0.735, 0.0, -0.0135)),
        material="powder_coat",
        name="front_lip",
    )
    work_surface.visual(
        Box((0.10, 0.86, 0.024)),
        origin=Origin(xyz=(0.14, 0.0, -0.002)),
        material="powder_coat",
        name="rear_stiffener",
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((0.05, 0.30, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material="plastic_dark",
        name="strip",
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.055, 0.115, 0.036)),
        origin=Origin(xyz=(0.0275, 0.0, -0.018)),
        material="plastic_dark",
        name="pod_body",
    )
    control_pod.visual(
        Box((0.025, 0.115, 0.010)),
        origin=Origin(xyz=(0.0125, 0.0, -0.005)),
        material="plastic_dark",
        name="pod_mount",
    )

    preset_0 = model.part("preset_0")
    preset_0.visual(
        Box((0.008, 0.019, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material="button_light",
        name="cap",
    )

    preset_1 = model.part("preset_1")
    preset_1.visual(
        Box((0.008, 0.019, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material="button_light",
        name="cap",
    )

    preset_2 = model.part("preset_2")
    preset_2.visual(
        Box((0.008, 0.019, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material="button_light",
        name="cap",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material="power_blue",
        name="cap",
    )

    model.articulation(
        "frame_to_left_outer",
        ArticulationType.FIXED,
        parent=frame,
        child=left_outer,
        origin=Origin(xyz=(0.0, -0.27, 0.0)),
    )
    model.articulation(
        "frame_to_right_outer",
        ArticulationType.FIXED,
        parent=frame,
        child=right_outer,
        origin=Origin(xyz=(0.0, 0.27, 0.0)),
    )
    model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=left_outer,
        child=left_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=240.0, velocity=0.10, lower=0.0, upper=0.14),
    )
    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=right_outer,
        child=right_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=240.0, velocity=0.10, lower=0.0, upper=0.14),
        mimic=Mimic("left_lift"),
    )
    model.articulation(
        "stage_to_beam",
        ArticulationType.FIXED,
        parent=left_stage,
        child=lift_beam,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )
    model.articulation(
        "surface_tilt",
        ArticulationType.REVOLUTE,
        parent=lift_beam,
        child=work_surface,
        origin=Origin(xyz=(-0.05, 0.27, 0.16)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.60, lower=0.0, upper=0.95),
    )
    model.articulation(
        "surface_to_strip",
        ArticulationType.FIXED,
        parent=work_surface,
        child=control_strip,
        origin=Origin(xyz=(0.59, 0.0, 0.0)),
    )
    model.articulation(
        "strip_to_pod",
        ArticulationType.FIXED,
        parent=control_strip,
        child=control_pod,
        origin=Origin(xyz=(0.025, 0.10, -0.018)),
    )
    model.articulation(
        "preset_0_press",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=preset_0,
        origin=Origin(xyz=(0.055, -0.030, -0.011)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.004),
    )
    model.articulation(
        "preset_1_press",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=preset_1,
        origin=Origin(xyz=(0.055, -0.005, -0.011)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.004),
    )
    model.articulation(
        "preset_2_press",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=preset_2,
        origin=Origin(xyz=(0.055, 0.020, -0.011)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.004),
    )
    model.articulation(
        "power_press",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=power_button,
        origin=Origin(xyz=(0.055, 0.048, -0.024)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.003),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_outer = object_model.get_part("left_outer")
    right_outer = object_model.get_part("right_outer")
    left_stage = object_model.get_part("left_stage")
    right_stage = object_model.get_part("right_stage")
    lift_beam = object_model.get_part("lift_beam")
    work_surface = object_model.get_part("work_surface")
    control_strip = object_model.get_part("control_strip")
    control_pod = object_model.get_part("control_pod")
    preset_0 = object_model.get_part("preset_0")
    preset_1 = object_model.get_part("preset_1")
    preset_2 = object_model.get_part("preset_2")
    power_button = object_model.get_part("power_button")

    left_lift = object_model.get_articulation("left_lift")
    surface_tilt = object_model.get_articulation("surface_tilt")
    preset_0_press = object_model.get_articulation("preset_0_press")
    preset_1_press = object_model.get_articulation("preset_1_press")
    preset_2_press = object_model.get_articulation("preset_2_press")
    power_press = object_model.get_articulation("power_press")

    ctx.expect_origin_distance(
        left_stage,
        left_outer,
        axes="xy",
        max_dist=0.001,
        name="left stage stays centered in the outer sleeve",
    )
    ctx.expect_origin_distance(
        right_stage,
        right_outer,
        axes="xy",
        max_dist=0.001,
        name="right stage stays centered in the outer sleeve",
    )
    ctx.expect_overlap(
        left_stage,
        left_outer,
        axes="z",
        min_overlap=0.20,
        name="left stage remains inserted at rest",
    )
    ctx.expect_overlap(
        right_stage,
        right_outer,
        axes="z",
        min_overlap=0.20,
        name="right stage remains inserted at rest",
    )
    ctx.expect_gap(
        work_surface,
        lift_beam,
        axis="z",
        min_gap=0.02,
        positive_elem="top",
        negative_elem="beam",
        name="work surface sits above the lift beam",
    )
    ctx.expect_contact(
        control_strip,
        work_surface,
        name="control strip mounts to the underside of the work surface",
    )
    ctx.expect_contact(
        control_pod,
        control_strip,
        name="control pod mounts to the slim control strip",
    )

    left_limits = left_lift.motion_limits
    tilt_limits = surface_tilt.motion_limits
    rest_stage = ctx.part_world_position(left_stage)
    rest_top = ctx.part_world_aabb(work_surface)
    if left_limits is not None and left_limits.upper is not None:
        with ctx.pose({left_lift: left_limits.upper}):
            ctx.expect_overlap(
                left_stage,
                left_outer,
                axes="z",
                min_overlap=0.08,
                name="left stage retains insertion when raised",
            )
            ctx.expect_overlap(
                right_stage,
                right_outer,
                axes="z",
                min_overlap=0.08,
                name="right stage retains insertion when raised",
            )
            raised_stage = ctx.part_world_position(left_stage)
        ctx.check(
            "lift motion raises the beam assembly",
            rest_stage is not None
            and raised_stage is not None
            and raised_stage[2] > rest_stage[2] + 0.10,
            details=f"rest={rest_stage}, raised={raised_stage}",
        )

    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({surface_tilt: tilt_limits.upper}):
            tilted_top = ctx.part_world_aabb(work_surface)
        ctx.check(
            "tilt motion lifts the front edge",
            rest_top is not None
            and tilted_top is not None
            and tilted_top[1][2] > rest_top[1][2] + 0.16,
            details=f"rest={rest_top}, tilted={tilted_top}",
        )

    button_pairs = (
        ("preset 1 button depresses independently", preset_0, preset_0_press, preset_1),
        ("preset 2 button depresses independently", preset_1, preset_1_press, preset_0),
        ("preset 3 button depresses independently", preset_2, preset_2_press, power_button),
        ("power button depresses independently", power_button, power_press, preset_2),
    )
    for check_name, moving_button, joint, stationary_button in button_pairs:
        limits = joint.motion_limits
        rest_button = ctx.part_world_position(moving_button)
        rest_stationary = ctx.part_world_position(stationary_button)
        if limits is None or limits.upper is None:
            ctx.fail(check_name, "motion limits were missing")
            continue
        with ctx.pose({joint: limits.upper}):
            pressed_button = ctx.part_world_position(moving_button)
            pressed_stationary = ctx.part_world_position(stationary_button)
        moved_inward = (
            rest_button is not None
            and pressed_button is not None
            and pressed_button[0] < rest_button[0] - 0.0015
        )
        stationary_held = (
            rest_stationary is not None
            and pressed_stationary is not None
            and abs(pressed_stationary[0] - rest_stationary[0]) < 1e-5
            and abs(pressed_stationary[1] - rest_stationary[1]) < 1e-5
            and abs(pressed_stationary[2] - rest_stationary[2]) < 1e-5
        )
        ctx.check(
            check_name,
            moved_inward and stationary_held,
            details=(
                f"moving_rest={rest_button}, moving_pressed={pressed_button}, "
                f"stationary_rest={rest_stationary}, stationary_pressed={pressed_stationary}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
