from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="photo_all_in_one_printer")

    body_color = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_color = model.material("trim_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    tray_color = model.material("tray_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    button_color = model.material("button_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    knob_color = model.material("knob_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    glass_color = model.material("glass_tint", rgba=(0.43, 0.59, 0.67, 0.35))
    accent_color = model.material("accent_black", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.54, 0.39, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_color,
        name="bottom_plate",
    )
    body.visual(
        Box((0.014, 0.39, 0.170)),
        origin=Origin(xyz=(-0.263, 0.0, 0.085)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((0.014, 0.39, 0.170)),
        origin=Origin(xyz=(0.263, 0.0, 0.085)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((0.512, 0.014, 0.170)),
        origin=Origin(xyz=(0.0, -0.188, 0.085)),
        material=body_color,
        name="rear_wall",
    )
    body.visual(
        Box((0.080, 0.014, 0.170)),
        origin=Origin(xyz=(-0.223, 0.188, 0.085)),
        material=body_color,
        name="front_left_tower",
    )
    body.visual(
        Box((0.080, 0.014, 0.170)),
        origin=Origin(xyz=(0.223, 0.188, 0.085)),
        material=body_color,
        name="front_right_tower",
    )
    body.visual(
        Box((0.366, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.188, 0.010)),
        material=body_color,
        name="lower_tray_sill",
    )
    body.visual(
        Box((0.366, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.188, 0.060)),
        material=body_color,
        name="tray_separator",
    )
    body.visual(
        Box((0.366, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.188, 0.103)),
        material=body_color,
        name="photo_separator",
    )
    body.visual(
        Box((0.366, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.188, 0.137)),
        material=body_color,
        name="output_lip",
    )
    body.visual(
        Box((0.512, 0.28, 0.012)),
        origin=Origin(xyz=(0.0, -0.030, 0.174)),
        material=body_color,
        name="top_deck",
    )
    body.visual(
        Box((0.190, 0.090, 0.012)),
        origin=Origin(xyz=(0.125, 0.132, 0.186)),
        material=body_color,
        name="control_console",
    )
    body.visual(
        Box((0.018, 0.280, 0.044)),
        origin=Origin(xyz=(-0.231, -0.035, 0.202)),
        material=body_color,
        name="scanner_left_rim",
    )
    body.visual(
        Box((0.018, 0.280, 0.044)),
        origin=Origin(xyz=(0.231, -0.035, 0.202)),
        material=body_color,
        name="scanner_right_rim",
    )
    body.visual(
        Box((0.444, 0.018, 0.044)),
        origin=Origin(xyz=(0.0, 0.096, 0.202)),
        material=body_color,
        name="scanner_front_rim",
    )
    body.visual(
        Box((0.444, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.166, 0.205)),
        material=body_color,
        name="scanner_rear_rim",
    )
    body.visual(
        Box((0.400, 0.236, 0.003)),
        origin=Origin(xyz=(0.0, -0.035, 0.1815)),
        material=glass_color,
        name="scanner_glass",
    )
    body.visual(
        Box((0.079, 0.220, 0.006)),
        origin=Origin(xyz=(-0.2165, 0.020, 0.030)),
        material=trim_color,
        name="main_rail_left",
    )
    body.visual(
        Box((0.079, 0.220, 0.006)),
        origin=Origin(xyz=(0.2165, 0.020, 0.030)),
        material=trim_color,
        name="main_rail_right",
    )
    body.visual(
        Box((0.138, 0.150, 0.004)),
        origin=Origin(xyz=(-0.187, 0.035, 0.078)),
        material=trim_color,
        name="photo_rail_left",
    )
    body.visual(
        Box((0.138, 0.150, 0.004)),
        origin=Origin(xyz=(0.187, 0.035, 0.078)),
        material=trim_color,
        name="photo_rail_right",
    )
    body.visual(
        Box((0.300, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.184, 0.126)),
        material=accent_color,
        name="output_slot_trim",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.488, 0.290, 0.022)),
        origin=Origin(xyz=(0.0, 0.145, 0.011)),
        material=body_color,
        name="panel",
    )
    lid.visual(
        Box((0.456, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.286, 0.007)),
        material=trim_color,
        name="front_skirt",
    )
    lid.visual(
        Box((0.008, 0.270, 0.014)),
        origin=Origin(xyz=(-0.240, 0.145, 0.007)),
        material=trim_color,
        name="left_skirt",
    )
    lid.visual(
        Box((0.008, 0.270, 0.014)),
        origin=Origin(xyz=(0.240, 0.145, 0.007)),
        material=trim_color,
        name="right_skirt",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.360),
        origin=Origin(xyz=(0.0, -0.007, 0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_color,
        name="hinge_barrel",
    )

    main_tray = model.part("main_tray")
    main_tray.visual(
        Box((0.016, 0.260, 0.004)),
        origin=Origin(xyz=(-0.186, -0.110, 0.002)),
        material=trim_color,
        name="runner_left",
    )
    main_tray.visual(
        Box((0.016, 0.260, 0.004)),
        origin=Origin(xyz=(0.186, -0.110, 0.002)),
        material=trim_color,
        name="runner_right",
    )
    main_tray.visual(
        Box((0.356, 0.300, 0.003)),
        origin=Origin(xyz=(0.0, -0.110, 0.0055)),
        material=tray_color,
        name="floor",
    )
    main_tray.visual(
        Box((0.006, 0.292, 0.016)),
        origin=Origin(xyz=(-0.175, -0.110, 0.012)),
        material=tray_color,
        name="left_wall",
    )
    main_tray.visual(
        Box((0.006, 0.292, 0.016)),
        origin=Origin(xyz=(0.175, -0.110, 0.012)),
        material=tray_color,
        name="right_wall",
    )
    main_tray.visual(
        Box((0.356, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.045, 0.012)),
        material=tray_color,
        name="front_wall",
    )
    main_tray.visual(
        Box((0.356, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.256, 0.010)),
        material=tray_color,
        name="rear_wall",
    )

    photo_tray = model.part("photo_tray")
    photo_tray.visual(
        Box((0.010, 0.180, 0.003)),
        origin=Origin(xyz=(-0.123, -0.065, 0.0015)),
        material=trim_color,
        name="runner_left",
    )
    photo_tray.visual(
        Box((0.010, 0.180, 0.003)),
        origin=Origin(xyz=(0.123, -0.065, 0.0015)),
        material=trim_color,
        name="runner_right",
    )
    photo_tray.visual(
        Box((0.236, 0.240, 0.003)),
        origin=Origin(xyz=(0.0, -0.090, 0.0045)),
        material=tray_color,
        name="floor",
    )
    photo_tray.visual(
        Box((0.006, 0.230, 0.010)),
        origin=Origin(xyz=(-0.115, -0.090, 0.008)),
        material=tray_color,
        name="left_wall",
    )
    photo_tray.visual(
        Box((0.006, 0.230, 0.010)),
        origin=Origin(xyz=(0.115, -0.090, 0.008)),
        material=tray_color,
        name="right_wall",
    )
    photo_tray.visual(
        Box((0.236, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.035, 0.009)),
        material=tray_color,
        name="front_wall",
    )
    photo_tray.visual(
        Box((0.236, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.206, 0.008)),
        material=tray_color,
        name="rear_wall",
    )

    output_door = model.part("output_door")
    output_door.visual(
        Box((0.220, 0.004, 0.032)),
        origin=Origin(xyz=(0.0, 0.002, -0.016)),
        material=tray_color,
        name="panel",
    )
    output_door.visual(
        Cylinder(radius=0.0035, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_color,
        name="hinge_rod",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_color,
        name="stem",
    )
    knob.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=knob_color,
        name="body",
    )
    knob.visual(
        Box((0.005, 0.018, 0.004)),
        origin=Origin(xyz=(0.012, 0.0, 0.024)),
        material=button_color,
        name="indicator",
    )

    button_positions = (
        ("button_0", 0.112),
        ("button_1", 0.146),
        ("button_2", 0.180),
    )
    for button_name, x_pos in button_positions:
        button = model.part(button_name)
        button.visual(
            Box((0.018, 0.012, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=button_color,
            name="pedestal",
        )
        button.visual(
            Box((0.026, 0.018, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, 0.0065)),
            material=button_color,
            name="cap",
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.132, 0.192)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=0.003,
            ),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.174, 0.225)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "body_to_main_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=main_tray,
        origin=Origin(xyz=(0.0, 0.150, 0.033)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.18,
            lower=0.0,
            upper=0.090,
        ),
    )
    model.articulation(
        "body_to_photo_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=photo_tray,
        origin=Origin(xyz=(0.0, 0.150, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.15,
            lower=0.0,
            upper=0.050,
        ),
    )
    model.articulation(
        "body_to_output_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=output_door,
        origin=Origin(xyz=(0.0, 0.195, 0.127)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.062, 0.132, 0.192)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=5.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    main_tray = object_model.get_part("main_tray")
    photo_tray = object_model.get_part("photo_tray")
    output_door = object_model.get_part("output_door")
    knob = object_model.get_part("knob")

    lid_joint = object_model.get_articulation("body_to_lid")
    main_tray_joint = object_model.get_articulation("body_to_main_tray")
    photo_tray_joint = object_model.get_articulation("body_to_photo_tray")
    door_joint = object_model.get_articulation("body_to_output_door")
    knob_joint = object_model.get_articulation("body_to_knob")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="front_skirt",
        negative_elem="scanner_front_rim",
        min_gap=0.0005,
        max_gap=0.003,
        name="lid sits just above the scanner bay rim when closed",
    )

    lid_closed_aabb = ctx.part_element_world_aabb(lid, elem="panel")
    with ctx.pose({lid_joint: 1.05}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="panel")
    ctx.check(
        "lid opens upward from the rear hinge",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.10,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    ctx.expect_gap(
        main_tray,
        body,
        axis="z",
        positive_elem="runner_left",
        negative_elem="main_rail_left",
        max_gap=0.0005,
        max_penetration=0.0,
        name="main tray runner rests on its rail",
    )
    ctx.expect_overlap(
        main_tray,
        body,
        axes="y",
        elem_a="runner_left",
        elem_b="main_rail_left",
        min_overlap=0.10,
        name="main tray remains inserted on the rail at rest",
    )
    main_tray_rest_pos = ctx.part_world_position(main_tray)
    with ctx.pose({main_tray_joint: 0.090}):
        ctx.expect_gap(
            main_tray,
            body,
            axis="z",
            positive_elem="runner_left",
            negative_elem="main_rail_left",
            max_gap=0.0005,
            max_penetration=0.0,
            name="main tray runner stays supported when extended",
        )
        ctx.expect_overlap(
            main_tray,
            body,
            axes="y",
            elem_a="runner_left",
            elem_b="main_rail_left",
            min_overlap=0.04,
            name="main tray keeps retained insertion at full extension",
        )
        main_tray_open_pos = ctx.part_world_position(main_tray)
    ctx.check(
        "main tray slides outward",
        main_tray_rest_pos is not None
        and main_tray_open_pos is not None
        and main_tray_open_pos[1] > main_tray_rest_pos[1] + 0.07,
        details=f"rest={main_tray_rest_pos}, open={main_tray_open_pos}",
    )

    ctx.expect_gap(
        photo_tray,
        body,
        axis="z",
        positive_elem="runner_left",
        negative_elem="photo_rail_left",
        max_gap=0.0005,
        max_penetration=0.0,
        name="photo tray runner rests on its upper rail",
    )
    ctx.expect_overlap(
        photo_tray,
        body,
        axes="y",
        elem_a="runner_left",
        elem_b="photo_rail_left",
        min_overlap=0.07,
        name="photo tray stays inserted on its rail at rest",
    )
    photo_tray_rest_pos = ctx.part_world_position(photo_tray)
    with ctx.pose({photo_tray_joint: 0.050}):
        ctx.expect_gap(
            photo_tray,
            body,
            axis="z",
            positive_elem="runner_left",
            negative_elem="photo_rail_left",
            max_gap=0.0005,
            max_penetration=0.0,
            name="photo tray runner stays supported when extended",
        )
        ctx.expect_overlap(
            photo_tray,
            body,
            axes="y",
            elem_a="runner_left",
            elem_b="photo_rail_left",
            min_overlap=0.03,
            name="photo tray keeps retained insertion at full extension",
        )
        photo_tray_open_pos = ctx.part_world_position(photo_tray)
    ctx.check(
        "photo tray slides outward",
        photo_tray_rest_pos is not None
        and photo_tray_open_pos is not None
        and photo_tray_open_pos[1] > photo_tray_rest_pos[1] + 0.04,
        details=f"rest={photo_tray_rest_pos}, open={photo_tray_open_pos}",
    )

    for button_name in ("button_0", "button_1", "button_2"):
        button = object_model.get_part(button_name)
        button_joint = object_model.get_articulation(f"body_to_{button_name}")
        ctx.expect_gap(
            button,
            body,
            axis="z",
            positive_elem="cap",
            negative_elem="control_console",
            min_gap=0.0025,
            max_gap=0.0035,
            name=f"{button_name} stands proud of the control console",
        )
        with ctx.pose({button_joint: 0.003}):
            ctx.expect_gap(
                button,
                body,
                axis="z",
                positive_elem="cap",
                negative_elem="control_console",
                max_gap=0.0005,
                max_penetration=0.0,
                name=f"{button_name} presses down to the console surface",
            )

    ctx.expect_gap(
        knob,
        body,
        axis="z",
        positive_elem="stem",
        negative_elem="control_console",
        max_gap=0.0005,
        max_penetration=0.0,
        name="control knob is mounted on the control console",
    )
    knob_limits = knob_joint.motion_limits
    ctx.check(
        "control knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={knob_limits}",
    )

    door_closed_aabb = ctx.part_element_world_aabb(output_door, elem="panel")
    with ctx.pose({door_joint: 1.10}):
        door_open_aabb = ctx.part_element_world_aabb(output_door, elem="panel")
    ctx.check(
        "output door folds forward and down",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.015
        and (door_open_aabb[1][1] - door_open_aabb[0][1]) > 0.020,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
