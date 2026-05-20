from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A centered box with softened vertical corners for manufactured panels."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_drafting_standing_desk")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.018, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    birch_top = model.material("pale_birch_laminate", rgba=(0.78, 0.67, 0.49, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    button_gray = model.material("button_gray", rgba=(0.20, 0.21, 0.22, 1.0))
    white_mark = model.material("white_marking", rgba=(0.92, 0.92, 0.88, 1.0))
    power_green = model.material("power_green", rgba=(0.05, 0.75, 0.26, 1.0))

    base = model.part("base_frame")

    column_front_names = ("column_0_front_wall", "column_1_front_wall")
    column_rear_names = ("column_0_rear_wall", "column_1_rear_wall")
    column_side_0_names = ("column_0_side_wall_0", "column_1_side_wall_0")
    column_side_1_names = ("column_0_side_wall_1", "column_1_side_wall_1")

    for idx, x in enumerate((-0.45, 0.45)):
        base.visual(
            Box((0.14, 0.86, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0275)),
            material=dark_metal,
            name=f"foot_{idx}",
        )
        base.visual(
            Box((0.11, 0.012, 0.73)),
            origin=Origin(xyz=(x, -0.015, 0.42)),
            material=matte_black,
            name=column_front_names[idx],
        )
        base.visual(
            Box((0.11, 0.012, 0.73)),
            origin=Origin(xyz=(x, 0.075, 0.42)),
            material=matte_black,
            name=column_rear_names[idx],
        )
        base.visual(
            Box((0.012, 0.09, 0.73)),
            origin=Origin(xyz=(x - 0.049, 0.03, 0.42)),
            material=matte_black,
            name=column_side_0_names[idx],
        )
        base.visual(
            Box((0.012, 0.09, 0.73)),
            origin=Origin(xyz=(x + 0.049, 0.03, 0.42)),
            material=matte_black,
            name=column_side_1_names[idx],
        )
        base.visual(
            Box((0.14, 0.012, 0.030)),
            origin=Origin(xyz=(x, -0.015, 0.795)),
            material=satin_metal,
            name=f"column_{idx}_front_collar",
        )
        base.visual(
            Box((0.14, 0.012, 0.030)),
            origin=Origin(xyz=(x, 0.075, 0.795)),
            material=satin_metal,
            name=f"column_{idx}_rear_collar",
        )
        base.visual(
            Box((0.012, 0.09, 0.030)),
            origin=Origin(xyz=(x - 0.049, 0.03, 0.795)),
            material=satin_metal,
            name=f"column_{idx}_side_collar_0",
        )
        base.visual(
            Box((0.012, 0.09, 0.030)),
            origin=Origin(xyz=(x + 0.049, 0.03, 0.795)),
            material=satin_metal,
            name=f"column_{idx}_side_collar_1",
        )
        base.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(x, -0.34, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"front_glide_{idx}",
        )
        base.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(x, 0.36, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"rear_glide_{idx}",
        )

    base.visual(
        Box((0.90, 0.070, 0.045)),
        origin=Origin(xyz=(0.0, -0.32, 0.075)),
        material=dark_metal,
        name="front_floor_spreader",
    )
    base.visual(
        Box((0.90, 0.070, 0.045)),
        origin=Origin(xyz=(0.0, 0.30, 0.075)),
        material=dark_metal,
        name="rear_floor_spreader",
    )
    base.visual(
        Box((0.98, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, 0.095, 0.55)),
        material=dark_metal,
        name="rear_mid_spreader",
    )

    leg_stage_0 = model.part("leg_stage_0")
    leg_stage_0.visual(
        Box((0.086, 0.046, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=satin_metal,
        name="inner_tube",
    )

    leg_stage_1 = model.part("leg_stage_1")
    leg_stage_1.visual(
        Box((0.086, 0.046, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=satin_metal,
        name="inner_tube",
    )

    lift = model.part("lift_frame")
    lift.visual(
        Box((1.05, 0.080, 0.080)),
        origin=Origin(xyz=(0.45, 0.03, 0.370)),
        material=dark_metal,
        name="upper_crossbar",
    )
    for idx, x in enumerate((0.17, 0.73)):
        lift.visual(
            Box((0.075, 0.31, 0.035)),
            origin=Origin(xyz=(x, 0.165, 0.420)),
            material=dark_metal,
            name=f"rear_arm_{idx}",
        )
        lift.visual(
            Box((0.095, 0.070, 0.075)),
            origin=Origin(xyz=(x, 0.285, 0.455)),
            material=dark_metal,
            name=f"hinge_saddle_{idx}",
        )
    lift.visual(
        Cylinder(radius=0.026, length=1.12),
        origin=Origin(xyz=(0.45, 0.30, 0.470), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_beam",
    )

    top = model.part("work_surface")
    desktop_mesh = mesh_from_cadquery(
        _rounded_box((1.20, 0.82, 0.040), 0.030),
        "rounded_desktop_panel",
        tolerance=0.0015,
        angular_tolerance=0.15,
    )
    top.visual(
        desktop_mesh,
        origin=Origin(xyz=(0.0, -0.435, 0.054)),
        material=birch_top,
        name="desktop_panel",
    )
    top.visual(
        Box((1.12, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -0.833, 0.0915)),
        material=birch_top,
        name="front_pencil_lip",
    )
    top.visual(
        Box((1.16, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, 0.040)),
        material=dark_metal,
        name="rear_edge_band",
    )
    for idx, x in enumerate((-0.34, 0.0, 0.34)):
        top.visual(
            Box((0.16, 0.11, 0.008)),
            origin=Origin(xyz=(x, -0.045, 0.030)),
            material=dark_metal,
            name=f"hinge_leaf_{idx}",
        )

    pod = model.part("control_pod")
    pod.visual(
        Box((0.95, 0.028, 0.036)),
        origin=Origin(xyz=(0.0, 0.014, -0.018)),
        material=graphite,
        name="slim_control_strip",
    )
    pod.visual(
        Box((0.34, 0.075, 0.055)),
        origin=Origin(xyz=(0.28, 0.0375, -0.032)),
        material=graphite,
        name="memory_bank_body",
    )
    pod.visual(
        Box((0.31, 0.004, 0.035)),
        origin=Origin(xyz=(0.28, -0.002, -0.032)),
        material=matte_black,
        name="recessed_front_face",
    )

    preset_xs = (0.20, 0.28, 0.36)
    preset_buttons = []
    for idx, x in enumerate(preset_xs):
        button = model.part(f"preset_button_{idx}")
        button.visual(
            Box((0.052, 0.008, 0.022)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.020, 0.0012, 0.0028)),
            origin=Origin(xyz=(0.0, -0.0086, 0.0045)),
            material=white_mark,
            name="preset_mark",
        )
        preset_buttons.append(button)

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_gray,
        name="button_cap",
    )
    power_button.visual(
        Cylinder(radius=0.006, length=0.0014),
        origin=Origin(xyz=(0.0, -0.0087, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=power_green,
        name="power_mark",
    )

    lift_limits = MotionLimits(effort=250.0, velocity=0.12, lower=0.0, upper=0.30)
    leg_drive = model.articulation(
        "base_to_leg_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=leg_stage_0,
        origin=Origin(xyz=(-0.45, 0.03, 0.82)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "base_to_leg_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=leg_stage_1,
        origin=Origin(xyz=(0.45, 0.03, 0.82)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="base_to_leg_stage_0"),
    )
    model.articulation(
        "leg_stage_0_to_lift_frame",
        ArticulationType.FIXED,
        parent=leg_stage_0,
        child=lift,
        origin=Origin(),
    )
    model.articulation(
        "lift_frame_to_work_surface",
        ArticulationType.REVOLUTE,
        parent=lift,
        child=top,
        origin=Origin(xyz=(0.45, 0.30, 0.47)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=0.0, upper=0.65),
    )
    model.articulation(
        "work_surface_to_control_pod",
        ArticulationType.FIXED,
        parent=top,
        child=pod,
        origin=Origin(xyz=(0.0, -0.845, 0.034)),
    )

    button_limits = MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.006)
    for idx, (button, x) in enumerate(zip(preset_buttons, preset_xs)):
        model.articulation(
            f"control_pod_to_preset_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=pod,
            child=button,
            origin=Origin(xyz=(x, 0.0, -0.032)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=button_limits,
        )
    model.articulation(
        "control_pod_to_power_button",
        ArticulationType.PRISMATIC,
        parent=pod,
        child=power_button,
        origin=Origin(xyz=(0.125, 0.0, -0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=button_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    leg0 = object_model.get_part("leg_stage_0")
    leg1 = object_model.get_part("leg_stage_1")
    lift = object_model.get_part("lift_frame")
    top = object_model.get_part("work_surface")
    pod = object_model.get_part("control_pod")

    leg_drive = object_model.get_articulation("base_to_leg_stage_0")
    leg_follower = object_model.get_articulation("base_to_leg_stage_1")
    tilt = object_model.get_articulation("lift_frame_to_work_surface")

    ctx.check(
        "two telescoping leg stages are prismatic",
        leg_drive.articulation_type == ArticulationType.PRISMATIC
        and leg_follower.articulation_type == ArticulationType.PRISMATIC
        and leg_follower.mimic is not None,
        details=f"drive={leg_drive.articulation_type}, follower={leg_follower.articulation_type}, mimic={leg_follower.mimic}",
    )
    ctx.expect_overlap(
        leg0,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="column_0_front_wall",
        min_overlap=0.18,
        name="leg stage 0 remains inserted in outer column",
    )
    ctx.expect_overlap(
        leg1,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="column_1_front_wall",
        min_overlap=0.18,
        name="leg stage 1 remains inserted in outer column",
    )

    lift_rest = ctx.part_world_position(lift)
    with ctx.pose({leg_drive: 0.30}):
        lift_high = ctx.part_world_position(lift)
        ctx.expect_overlap(
            leg0,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="column_0_front_wall",
            min_overlap=0.10,
            name="extended leg stage 0 still retained",
        )
        ctx.expect_overlap(
            leg1,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="column_1_front_wall",
            min_overlap=0.10,
            name="extended leg stage 1 still retained",
        )
    ctx.check(
        "lifting frame rises with telescoping legs",
        lift_rest is not None and lift_high is not None and lift_high[2] > lift_rest[2] + 0.25,
        details=f"rest={lift_rest}, high={lift_high}",
    )

    level_aabb = ctx.part_element_world_aabb(top, elem="desktop_panel")
    with ctx.pose({tilt: 0.65}):
        tilted_aabb = ctx.part_element_world_aabb(top, elem="desktop_panel")
    ctx.check(
        "work surface tilts upward about rear hinge",
        level_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > level_aabb[1][2] + 0.30,
        details=f"level={level_aabb}, tilted={tilted_aabb}",
    )

    ctx.expect_contact(
        pod,
        top,
        elem_a="slim_control_strip",
        elem_b="desktop_panel",
        contact_tol=0.002,
        name="control strip is mounted under front lip",
    )

    for name in ("preset_button_0", "preset_button_1", "preset_button_2", "power_button"):
        button = object_model.get_part(name)
        joint = object_model.get_articulation(f"control_pod_to_{name}")
        ctx.expect_gap(
            pod,
            button,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="memory_bank_body",
            negative_elem="button_cap",
            name=f"{name} starts proud of control pod",
        )
        button_rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            button_pressed = ctx.part_world_position(button)
        ctx.check(
            f"{name} depresses independently",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1] > button_rest[1] + 0.004,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
