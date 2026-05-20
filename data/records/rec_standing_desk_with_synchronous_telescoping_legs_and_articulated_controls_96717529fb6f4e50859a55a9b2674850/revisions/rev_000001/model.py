from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    maple = model.material("pale_maple_top", rgba=(0.72, 0.53, 0.34, 1.0))
    black = model.material("satin_black_metal", rgba=(0.025, 0.027, 0.030, 1.0))
    dark = model.material("matte_control_plastic", rgba=(0.010, 0.011, 0.013, 1.0))
    button_gray = model.material("soft_gray_buttons", rgba=(0.16, 0.17, 0.18, 1.0))
    button_white = model.material("white_button_marks", rgba=(0.90, 0.90, 0.84, 1.0))
    power_green = model.material("muted_power_button", rgba=(0.08, 0.45, 0.20, 1.0))
    metal = model.material("brushed_inner_metal", rgba=(0.52, 0.55, 0.56, 1.0))

    base_frame = model.part("base_frame")
    # Two compact desk feet are tied together by a rear floor stretcher, so the
    # root reads as one supported standing-desk base rather than two islands.
    for x in (-0.32, 0.32):
        base_frame.visual(
            Box((0.09, 0.56, 0.048)),
            origin=Origin(xyz=(x, 0.0, 0.024)),
            material=black,
            name=f"foot_{0 if x < 0 else 1}",
        )
    base_frame.visual(
        Box((0.73, 0.038, 0.040)),
        origin=Origin(xyz=(0.0, 0.20, 0.068)),
        material=black,
        name="rear_floor_stretcher",
    )

    outer_width = 0.082
    outer_depth = 0.064
    wall = 0.008
    column_height = 0.560

    outer_columns = []
    for idx, x in enumerate((-0.32, 0.32)):
        column = model.part(f"outer_column_{idx}")
        outer_columns.append(column)
        column.visual(
            Box((0.13, 0.105, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=black,
            name="foot_plate",
        )
        column.visual(
            Box((wall, outer_depth, column_height)),
            origin=Origin(xyz=(-(outer_width - wall) / 2.0, 0.0, column_height / 2.0)),
            material=black,
            name="side_wall_0",
        )
        column.visual(
            Box((wall, outer_depth, column_height)),
            origin=Origin(xyz=((outer_width - wall) / 2.0, 0.0, column_height / 2.0)),
            material=black,
            name="side_wall_1",
        )
        column.visual(
            Box((outer_width, wall, column_height)),
            origin=Origin(xyz=(0.0, -(outer_depth - wall) / 2.0, column_height / 2.0)),
            material=black,
            name="front_wall",
        )
        column.visual(
            Box((outer_width, wall, column_height)),
            origin=Origin(xyz=(0.0, (outer_depth - wall) / 2.0, column_height / 2.0)),
            material=black,
            name="rear_wall",
        )
        collar_width = 0.102
        collar_depth = 0.084
        collar_wall = 0.012
        column.visual(
            Box((collar_wall, collar_depth, 0.026)),
            origin=Origin(xyz=(-(collar_width - collar_wall) / 2.0, 0.0, column_height - 0.004)),
            material=black,
            name="collar_side_0",
        )
        column.visual(
            Box((collar_wall, collar_depth, 0.026)),
            origin=Origin(xyz=((collar_width - collar_wall) / 2.0, 0.0, column_height - 0.004)),
            material=black,
            name="collar_side_1",
        )
        column.visual(
            Box((collar_width, collar_wall, 0.026)),
            origin=Origin(xyz=(0.0, -(collar_depth - collar_wall) / 2.0, column_height - 0.004)),
            material=black,
            name="collar_front",
        )
        column.visual(
            Box((collar_width, collar_wall, 0.026)),
            origin=Origin(xyz=(0.0, (collar_depth - collar_wall) / 2.0, column_height - 0.004)),
            material=black,
            name="collar_rear",
        )
        model.articulation(
            f"base_to_outer_column_{idx}",
            ArticulationType.FIXED,
            parent=base_frame,
            child=column,
            origin=Origin(xyz=(x, 0.0, 0.060)),
        )

    inner_stages = []
    for idx, column in enumerate(outer_columns):
        stage = model.part(f"inner_stage_{idx}")
        inner_stages.append(stage)
        stage.visual(
            Box((0.052, 0.040, 0.620)),
            origin=Origin(xyz=(0.0, 0.0, -0.235)),
            material=metal,
            name="inner_tube",
        )
        stage.visual(
            Box((0.064, 0.052, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.081)),
            material=metal,
            name="top_pad",
        )
        stage.visual(
            Box((0.024, 0.0045, 0.050)),
            origin=Origin(xyz=(0.0, -0.02175, -0.020)),
            material=button_gray,
            name="front_glide_pad",
        )
        stage.visual(
            Box((0.024, 0.0045, 0.050)),
            origin=Origin(xyz=(0.0, 0.02175, -0.020)),
            material=button_gray,
            name="rear_glide_pad",
        )

    lift_limits = MotionLimits(effort=260.0, velocity=0.12, lower=0.0, upper=0.38)
    master_lift = model.articulation(
        "outer_column_0_to_inner_stage_0",
        ArticulationType.PRISMATIC,
        parent=outer_columns[0],
        child=inner_stages[0],
        origin=Origin(xyz=(0.0, 0.0, column_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "outer_column_1_to_inner_stage_1",
        ArticulationType.PRISMATIC,
        parent=outer_columns[1],
        child=inner_stages[1],
        origin=Origin(xyz=(0.0, 0.0, column_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint=master_lift.name, multiplier=1.0, offset=0.0),
    )

    top_frame = model.part("top_frame")
    top_frame.visual(
        Box((0.84, 0.025, 0.026)),
        origin=Origin(xyz=(0.32, -0.17, 0.022)),
        material=black,
        name="front_rail",
    )
    top_frame.visual(
        Box((0.84, 0.025, 0.026)),
        origin=Origin(xyz=(0.32, 0.17, 0.022)),
        material=black,
        name="rear_rail",
    )
    for idx, x in enumerate((0.0, 0.64)):
        top_frame.visual(
            Box((0.030, 0.365, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.022)),
            material=black,
            name=f"crossmember_{idx}",
        )
        top_frame.visual(
            Box((0.120, 0.090, 0.016)),
            origin=Origin(xyz=(x, 0.0, 0.008)),
            material=black,
            name=f"saddle_plate_{idx}",
        )
    top_frame.visual(
        Box((0.28, 0.365, 0.018)),
        origin=Origin(xyz=(0.32, 0.0, 0.026)),
        material=black,
        name="center_brace",
    )
    model.articulation(
        "inner_stage_0_to_top_frame",
        ArticulationType.FIXED,
        parent=inner_stages[0],
        child=top_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
    )

    desktop = model.part("desktop")
    top_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(1.00, 0.48, 0.035, corner_segments=10),
            0.036,
            cap=True,
            center=True,
        ),
        "rounded_narrow_desktop",
    )
    desktop.visual(top_mesh, material=maple, name="rounded_top")
    model.articulation(
        "top_frame_to_desktop",
        ArticulationType.FIXED,
        parent=top_frame,
        child=desktop,
        origin=Origin(xyz=(0.32, 0.0, 0.053)),
    )

    control_pod = model.part("control_pod")
    pod_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.22, 0.065, 0.012, corner_segments=8),
            0.030,
            cap=True,
            center=True,
        ),
        "rounded_control_pod",
    )
    control_pod.visual(pod_mesh, material=dark, name="pod_body")
    control_pod.visual(
        Box((0.18, 0.034, 0.008)),
        origin=Origin(xyz=(0.0, 0.006, 0.019)),
        material=dark,
        name="top_mount_flange",
    )
    model.articulation(
        "desktop_to_control_pod",
        ArticulationType.FIXED,
        parent=desktop,
        child=control_pod,
        origin=Origin(xyz=(0.34, -0.255, -0.041)),
    )

    button_limits = MotionLimits(effort=6.0, velocity=0.05, lower=0.0, upper=0.004)
    for idx, x in enumerate((-0.035, 0.005, 0.045)):
        button = model.part(f"preset_button_{idx}")
        button.visual(
            Box((0.026, 0.006, 0.014)),
            origin=Origin(),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.012, 0.0012, 0.0022)),
            origin=Origin(xyz=(0.0, -0.0031, 0.0025)),
            material=button_white,
            name="button_mark",
        )
        model.articulation(
            f"control_pod_to_preset_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=button,
            origin=Origin(xyz=(x, -0.0355, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=button_limits,
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=power_green,
        name="button_cap",
    )
    power_button.visual(
        Box((0.0022, 0.0012, 0.010)),
        origin=Origin(xyz=(0.0, -0.0031, 0.0015)),
        material=button_white,
        name="power_mark",
    )
    model.articulation(
        "control_pod_to_power_button",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=power_button,
        origin=Origin(xyz=(-0.085, -0.0355, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=button_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_0 = object_model.get_part("outer_column_0")
    outer_1 = object_model.get_part("outer_column_1")
    inner_0 = object_model.get_part("inner_stage_0")
    inner_1 = object_model.get_part("inner_stage_1")
    top_frame = object_model.get_part("top_frame")
    desktop = object_model.get_part("desktop")
    control_pod = object_model.get_part("control_pod")

    lift_0 = object_model.get_articulation("outer_column_0_to_inner_stage_0")
    lift_1 = object_model.get_articulation("outer_column_1_to_inner_stage_1")

    ctx.check(
        "twin lift joints are prismatic",
        lift_0.articulation_type == ArticulationType.PRISMATIC
        and lift_1.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "second lift stage mimics first",
        lift_1.mimic is not None
        and lift_1.mimic.joint == lift_0.name
        and abs(lift_1.mimic.multiplier - 1.0) < 1e-9,
    )

    for idx, (inner, outer) in enumerate(((inner_0, outer_0), (inner_1, outer_1))):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="inner_tube",
            margin=0.001,
            name=f"inner stage {idx} stays centered inside outer column",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            min_overlap=0.50,
            elem_a="inner_tube",
            name=f"collapsed stage {idx} keeps deep sleeve insertion",
        )

    ctx.expect_gap(
        top_frame,
        outer_0,
        axis="z",
        min_gap=0.05,
        max_gap=0.10,
        name="top frame is visibly separate above first column",
    )
    ctx.expect_gap(
        top_frame,
        outer_1,
        axis="z",
        min_gap=0.05,
        max_gap=0.10,
        name="top frame is visibly separate above second column",
    )

    ctx.expect_gap(
        desktop,
        control_pod,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="rounded_top",
        negative_elem="top_mount_flange",
        name="control pod flange seats under desktop",
    )
    ctx.expect_overlap(
        control_pod,
        desktop,
        axes="x",
        min_overlap=0.15,
        name="control pod sits under one desktop corner",
    )
    ctx.expect_overlap(
        control_pod,
        desktop,
        axes="y",
        min_overlap=0.015,
        name="control pod tucks below the front edge",
    )

    rest_desktop_pos = ctx.part_world_position(desktop)
    rest_inner_0 = ctx.part_world_position(inner_0)
    rest_inner_1 = ctx.part_world_position(inner_1)
    with ctx.pose({lift_0: 0.38}):
        ctx.expect_within(
            inner_0,
            outer_0,
            axes="xy",
            inner_elem="inner_tube",
            margin=0.001,
            name="extended first stage remains centered",
        )
        ctx.expect_within(
            inner_1,
            outer_1,
            axes="xy",
            inner_elem="inner_tube",
            margin=0.001,
            name="extended second stage remains centered",
        )
        ctx.expect_overlap(
            inner_0,
            outer_0,
            axes="z",
            min_overlap=0.14,
            elem_a="inner_tube",
            name="extended first stage retains insertion",
        )
        ctx.expect_overlap(
            inner_1,
            outer_1,
            axes="z",
            min_overlap=0.14,
            elem_a="inner_tube",
            name="extended second stage retains insertion",
        )
        extended_desktop_pos = ctx.part_world_position(desktop)
        extended_inner_0 = ctx.part_world_position(inner_0)
        extended_inner_1 = ctx.part_world_position(inner_1)

    ctx.check(
        "desktop rises with telescoping legs",
        rest_desktop_pos is not None
        and extended_desktop_pos is not None
        and extended_desktop_pos[2] > rest_desktop_pos[2] + 0.35,
    )
    ctx.check(
        "both lift stages move as a synchronized pair",
        rest_inner_0 is not None
        and rest_inner_1 is not None
        and extended_inner_0 is not None
        and extended_inner_1 is not None
        and abs((extended_inner_0[2] - rest_inner_0[2]) - (extended_inner_1[2] - rest_inner_1[2])) < 1e-6,
    )

    button_joint_names = [
        "control_pod_to_preset_button_0",
        "control_pod_to_preset_button_1",
        "control_pod_to_preset_button_2",
        "control_pod_to_power_button",
    ]
    for joint_name in button_joint_names:
        joint = object_model.get_articulation(joint_name)
        button = object_model.get_part(joint.child)
        ctx.check(
            f"{joint.child} is an independent prismatic push control",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.motion_limits is not None
            and joint.motion_limits.upper == 0.004,
        )
        ctx.expect_gap(
            control_pod,
            button,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="pod_body",
            negative_elem="button_cap",
            name=f"{joint.child} rests proud of pod face",
        )
        rest_button_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.004}):
            depressed_button_pos = ctx.part_world_position(button)
            ctx.expect_gap(
                control_pod,
                button,
                axis="y",
                max_penetration=0.005,
                positive_elem="pod_body",
                negative_elem="button_cap",
                name=f"{joint.child} depresses into pod face",
            )
        ctx.check(
            f"{joint.child} moves inward when pressed",
            rest_button_pos is not None
            and depressed_button_pos is not None
            and depressed_button_pos[1] > rest_button_pos[1] + 0.003,
        )

    return ctx.report()


object_model = build_object_model()
