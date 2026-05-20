from __future__ import annotations

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
    model = ArticulatedObject(name="overbed_table")

    powder_steel = model.material("powder_steel", rgba=(0.81, 0.83, 0.85, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    laminate = model.material("laminate", rgba=(0.85, 0.80, 0.69, 1.0))
    edge_band = model.material("edge_band", rgba=(0.57, 0.45, 0.31, 1.0))
    knob_grey = model.material("knob_grey", rgba=(0.24, 0.25, 0.27, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.60, 0.05, 0.03)),
        origin=Origin(xyz=(0.05, -0.18, 0.045)),
        material=powder_steel,
        name="rail_0",
    )
    pedestal.visual(
        Box((0.60, 0.05, 0.03)),
        origin=Origin(xyz=(0.05, 0.18, 0.045)),
        material=powder_steel,
        name="rail_1",
    )
    pedestal.visual(
        Box((0.08, 0.42, 0.035)),
        origin=Origin(xyz=(-0.25, 0.0, 0.0475)),
        material=powder_steel,
        name="rear_crossbar",
    )
    for index, y_pos in enumerate((-0.18, 0.18)):
        pedestal.visual(
            Box((0.07, 0.07, 0.03)),
            origin=Origin(xyz=(-0.24, y_pos, 0.015)),
            material=dark_steel,
            name=f"rear_pad_{index}",
        )

    for index, y_pos in enumerate((-0.18, 0.18)):
        pedestal.visual(
            Box((0.04, 0.05, 0.024)),
            origin=Origin(xyz=(0.35, y_pos, 0.072)),
            material=powder_steel,
            name=f"caster_bridge_{index}",
        )
        pedestal.visual(
            Box((0.02, 0.004, 0.05)),
            origin=Origin(xyz=(0.38, y_pos - 0.014, 0.04)),
            material=dark_steel,
            name=f"caster_fork_{index}_inner",
        )
        pedestal.visual(
            Box((0.02, 0.004, 0.05)),
            origin=Origin(xyz=(0.38, y_pos + 0.014, 0.04)),
            material=dark_steel,
            name=f"caster_fork_{index}_outer",
        )

    pedestal.visual(
        Box((0.16, 0.10, 0.03)),
        origin=Origin(xyz=(-0.11, -0.18, 0.075)),
        material=powder_steel,
        name="column_mount",
    )

    sleeve_center = (-0.11, -0.18, 0.22)
    sleeve_height = 0.26
    sleeve_outer = 0.08
    sleeve_wall = 0.008
    pedestal.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_center[0], sleeve_center[1] - (sleeve_outer - sleeve_wall) / 2.0, sleeve_center[2])
        ),
        material=powder_steel,
        name="sleeve_front",
    )
    pedestal.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_center[0], sleeve_center[1] + (sleeve_outer - sleeve_wall) / 2.0, sleeve_center[2])
        ),
        material=powder_steel,
        name="sleeve_back",
    )
    pedestal.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_center[0] - (sleeve_outer - sleeve_wall) / 2.0, sleeve_center[1], sleeve_center[2])
        ),
        material=powder_steel,
        name="sleeve_left",
    )
    pedestal.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_center[0] + (sleeve_outer - sleeve_wall) / 2.0, sleeve_center[1], sleeve_center[2])
        ),
        material=powder_steel,
        name="sleeve_right",
    )

    collar_center = (-0.11, -0.18, 0.295)
    collar_height = 0.09
    collar_outer = 0.11
    collar_wall = 0.020
    pedestal.visual(
        Box((collar_outer, collar_wall, collar_height)),
        origin=Origin(
            xyz=(collar_center[0], collar_center[1] - (collar_outer - collar_wall) / 2.0, collar_center[2])
        ),
        material=powder_steel,
        name="collar_front",
    )
    pedestal.visual(
        Box((collar_outer, collar_wall, collar_height)),
        origin=Origin(
            xyz=(collar_center[0], collar_center[1] + (collar_outer - collar_wall) / 2.0, collar_center[2])
        ),
        material=powder_steel,
        name="collar_back",
    )
    pedestal.visual(
        Box((collar_wall, collar_outer, collar_height)),
        origin=Origin(
            xyz=(collar_center[0] - (collar_outer - collar_wall) / 2.0, collar_center[1], collar_center[2])
        ),
        material=powder_steel,
        name="collar_left",
    )
    pedestal.visual(
        Box((collar_wall, collar_outer, collar_height)),
        origin=Origin(
            xyz=(collar_center[0] + (collar_outer - collar_wall) / 2.0, collar_center[1], collar_center[2])
        ),
        material=powder_steel,
        name="collar_right",
    )
    pedestal.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(-0.11, -0.236, 0.295), rpy=(1.57079632679, 0.0, 0.0)),
        material=dark_steel,
        name="knob_boss",
    )

    upper_post = model.part("upper_post")
    upper_post.visual(
        Box((0.05, 0.05, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_steel,
        name="inner_post",
    )
    upper_post.visual(
        Box((0.040, 0.008, 0.16)),
        origin=Origin(xyz=(0.0, -0.028, -0.17)),
        material=powder_steel,
        name="guide_pad_0",
    )
    upper_post.visual(
        Box((0.040, 0.008, 0.16)),
        origin=Origin(xyz=(0.0, 0.028, -0.17)),
        material=powder_steel,
        name="guide_pad_1",
    )
    upper_post.visual(
        Box((0.08, 0.08, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=powder_steel,
        name="post_cap",
    )
    upper_post.visual(
        Box((0.06, 0.08, 0.08)),
        origin=Origin(xyz=(0.03, 0.0, 0.33)),
        material=powder_steel,
        name="head_block",
    )

    main_top = model.part("main_top")
    main_top.visual(
        Box((0.16, 0.10, 0.04)),
        origin=Origin(xyz=(0.08, 0.0, 0.02)),
        material=dark_steel,
        name="support_arm",
    )
    main_top.visual(
        Box((0.20, 0.12, 0.03)),
        origin=Origin(xyz=(0.10, 0.0, 0.025)),
        material=powder_steel,
        name="underframe",
    )
    main_top.visual(
        Box((0.78, 0.40, 0.028)),
        origin=Origin(xyz=(0.40, 0.0, 0.054)),
        material=edge_band,
        name="tray_core",
    )
    main_top.visual(
        Box((0.76, 0.38, 0.005)),
        origin=Origin(xyz=(0.40, 0.0, 0.0705)),
        material=laminate,
        name="tray_skin",
    )
    main_top.visual(
        Box((0.08, 0.04, 0.02)),
        origin=Origin(xyz=(0.48, 0.205, 0.03)),
        material=powder_steel,
        name="wing_bracket",
    )
    main_top.visual(
        Box((0.006, 0.020, 0.040)),
        origin=Origin(xyz=(0.464, 0.215, 0.045)),
        material=powder_steel,
        name="wing_rib_0",
    )
    main_top.visual(
        Box((0.006, 0.020, 0.040)),
        origin=Origin(xyz=(0.496, 0.215, 0.045)),
        material=powder_steel,
        name="wing_rib_1",
    )
    main_top.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.468, 0.235, 0.065), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_steel,
        name="wing_hinge_barrel_0",
    )
    main_top.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.492, 0.235, 0.065), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_steel,
        name="wing_hinge_barrel_1",
    )

    reading_wing = model.part("reading_wing")
    reading_wing.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_steel,
        name="wing_hinge_barrel",
    )
    reading_wing.visual(
        Box((0.008, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=powder_steel,
        name="wing_leaf",
    )
    reading_wing.visual(
        Box((0.18, 0.02, 0.025)),
        origin=Origin(xyz=(0.0, 0.045, 0.0125)),
        material=powder_steel,
        name="wing_strut",
    )
    reading_wing.visual(
        Box((0.24, 0.18, 0.022)),
        origin=Origin(xyz=(0.0, 0.095, 0.021)),
        material=edge_band,
        name="wing_panel",
    )
    reading_wing.visual(
        Box((0.22, 0.16, 0.004)),
        origin=Origin(xyz=(0.0, 0.095, 0.034)),
        material=laminate,
        name="wing_skin",
    )

    height_knob = model.part("height_knob")
    height_knob.visual(
        Cylinder(radius=0.0075, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    height_knob.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, -0.028, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=knob_grey,
        name="knob_core",
    )
    height_knob.visual(
        Box((0.036, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
        material=knob_grey,
        name="knob_lobe_a",
    )
    height_knob.visual(
        Box((0.014, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
        material=knob_grey,
        name="knob_lobe_b",
    )

    for index in range(2):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.030, length=0.020),
            origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
            material=powder_steel,
            name="hub",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
            material=dark_steel,
            name="axle_hub",
        )

    model.articulation(
        "post_lift",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=upper_post,
        origin=Origin(xyz=(-0.11, -0.18, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.20),
    )
    model.articulation(
        "top_mount",
        ArticulationType.FIXED,
        parent=upper_post,
        child=main_top,
        origin=Origin(xyz=(0.03, 0.0, 0.37)),
    )
    model.articulation(
        "wing_hinge",
        ArticulationType.REVOLUTE,
        parent=main_top,
        child=reading_wing,
        origin=Origin(xyz=(0.48, 0.235, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=height_knob,
        origin=Origin(xyz=(-0.11, -0.247, 0.295)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    for index, y_pos in enumerate((-0.18, 0.18)):
        model.articulation(
            f"caster_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=pedestal,
            child=f"caster_{index}",
            origin=Origin(xyz=(0.38, y_pos, 0.03)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    upper_post = object_model.get_part("upper_post")
    main_top = object_model.get_part("main_top")
    reading_wing = object_model.get_part("reading_wing")
    height_knob = object_model.get_part("height_knob")

    post_lift = object_model.get_articulation("post_lift")
    wing_hinge = object_model.get_articulation("wing_hinge")

    ctx.expect_gap(
        reading_wing,
        main_top,
        axis="y",
        min_gap=0.02,
        max_gap=0.07,
        positive_elem="wing_panel",
        negative_elem="tray_core",
        name="reading wing sits beside the tray",
    )
    ctx.expect_gap(
        pedestal,
        height_knob,
        axis="y",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="knob_boss",
        negative_elem="hub",
        name="height knob seats on the collar boss",
    )

    rest_top = ctx.part_world_position(main_top)
    lift_limits = post_lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({post_lift: lift_limits.upper}):
            ctx.expect_overlap(
                upper_post,
                pedestal,
                axes="z",
                elem_a="inner_post",
                elem_b="sleeve_front",
                min_overlap=0.045,
                name="sliding post retains insertion at full height",
            )
            raised_top = ctx.part_world_position(main_top)
        ctx.check(
            "main tray rises with the post",
            rest_top is not None and raised_top is not None and raised_top[2] > rest_top[2] + 0.18,
            details=f"rest={rest_top}, raised={raised_top}",
        )

    wing_limits = wing_hinge.motion_limits
    if wing_limits is not None and wing_limits.upper is not None:
        rest_wing = ctx.part_element_world_aabb(reading_wing, elem="wing_panel")
        with ctx.pose({wing_hinge: wing_limits.upper}):
            raised_wing = ctx.part_element_world_aabb(reading_wing, elem="wing_panel")
        ctx.check(
            "reading wing rotates upward",
            rest_wing is not None
            and raised_wing is not None
            and raised_wing[1][2] > rest_wing[1][2] + 0.12,
            details=f"rest={rest_wing}, raised={raised_wing}",
        )

    return ctx.report()


object_model = build_object_model()
