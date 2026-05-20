from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    wood = model.material("warm_bamboo_top", rgba=(0.72, 0.50, 0.28, 1.0))
    dark = model.material("satin_black_metal", rgba=(0.025, 0.027, 0.030, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.07, 0.075, 0.080, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.67, 0.69, 0.70, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    button_mat = model.material("soft_gray_buttons", rgba=(0.18, 0.19, 0.20, 1.0))

    base = model.part("base")

    # Floor feet and low stretcher make the two-column base a single supported
    # assembly.  The outer lifting columns are open rectangular sleeves built
    # from four connected walls so the moving stages can visibly slide inside.
    base.visual(
        Box((0.16, 0.56, 0.04)),
        origin=Origin(xyz=(-0.32, 0.0, 0.02)),
        material=dark,
        name="foot_0",
    )
    base.visual(
        Box((0.080, 0.006, 0.54)),
        origin=Origin(xyz=(-0.32, -0.027, 0.31)),
        material=dark,
        name="front_wall_0",
    )
    base.visual(
        Box((0.080, 0.006, 0.54)),
        origin=Origin(xyz=(-0.32, 0.027, 0.31)),
        material=dark,
        name="rear_wall_0",
    )
    base.visual(
        Box((0.006, 0.060, 0.54)),
        origin=Origin(xyz=(-0.357, 0.0, 0.31)),
        material=dark,
        name="side_wall_0_a",
    )
    base.visual(
        Box((0.006, 0.060, 0.54)),
        origin=Origin(xyz=(-0.283, 0.0, 0.31)),
        material=dark,
        name="side_wall_0_b",
    )
    base.visual(
        Box((0.110, 0.012, 0.010)),
        origin=Origin(xyz=(-0.32, -0.036, 0.585)),
        material=dark,
        name="front_lip_0",
    )
    base.visual(
        Box((0.110, 0.012, 0.010)),
        origin=Origin(xyz=(-0.32, 0.036, 0.585)),
        material=dark,
        name="rear_lip_0",
    )
    base.visual(
        Box((0.012, 0.090, 0.010)),
        origin=Origin(xyz=(-0.366, 0.0, 0.585)),
        material=dark,
        name="side_lip_0_a",
    )
    base.visual(
        Box((0.012, 0.090, 0.010)),
        origin=Origin(xyz=(-0.274, 0.0, 0.585)),
        material=dark,
        name="side_lip_0_b",
    )
    base.visual(
        Box((0.16, 0.56, 0.04)),
        origin=Origin(xyz=(0.32, 0.0, 0.02)),
        material=dark,
        name="foot_1",
    )
    base.visual(
        Box((0.080, 0.006, 0.54)),
        origin=Origin(xyz=(0.32, -0.027, 0.31)),
        material=dark,
        name="front_wall_1",
    )
    base.visual(
        Box((0.080, 0.006, 0.54)),
        origin=Origin(xyz=(0.32, 0.027, 0.31)),
        material=dark,
        name="rear_wall_1",
    )
    base.visual(
        Box((0.006, 0.060, 0.54)),
        origin=Origin(xyz=(0.283, 0.0, 0.31)),
        material=dark,
        name="side_wall_1_a",
    )
    base.visual(
        Box((0.006, 0.060, 0.54)),
        origin=Origin(xyz=(0.357, 0.0, 0.31)),
        material=dark,
        name="side_wall_1_b",
    )
    base.visual(
        Box((0.110, 0.012, 0.010)),
        origin=Origin(xyz=(0.32, -0.036, 0.585)),
        material=dark,
        name="front_lip_1",
    )
    base.visual(
        Box((0.110, 0.012, 0.010)),
        origin=Origin(xyz=(0.32, 0.036, 0.585)),
        material=dark,
        name="rear_lip_1",
    )
    base.visual(
        Box((0.012, 0.090, 0.010)),
        origin=Origin(xyz=(0.274, 0.0, 0.585)),
        material=dark,
        name="side_lip_1_a",
    )
    base.visual(
        Box((0.012, 0.090, 0.010)),
        origin=Origin(xyz=(0.366, 0.0, 0.585)),
        material=dark,
        name="side_lip_1_b",
    )
    base.visual(
        Box((0.70, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.235, 0.0575)),
        material=dark,
        name="rear_stretcher",
    )

    stage_0 = model.part("leg_stage_0")
    stage_0.visual(
        Box((0.052, 0.034, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=aluminum,
        name="inner_tube",
    )
    stage_0.visual(
        Box((0.11, 0.08, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.193)),
        material=aluminum,
        name="top_cap",
    )
    stage_0.visual(
        Box((0.008, 0.030, 0.045)),
        origin=Origin(xyz=(-0.030, 0.0, -0.390)),
        material=aluminum,
        name="side_glide_0",
    )
    stage_0.visual(
        Box((0.008, 0.030, 0.045)),
        origin=Origin(xyz=(0.030, 0.0, -0.390)),
        material=aluminum,
        name="side_glide_1",
    )

    stage_1 = model.part("leg_stage_1")
    stage_1.visual(
        Box((0.052, 0.034, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=aluminum,
        name="inner_tube",
    )
    stage_1.visual(
        Box((0.11, 0.08, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.193)),
        material=aluminum,
        name="top_cap",
    )
    stage_1.visual(
        Box((0.008, 0.030, 0.045)),
        origin=Origin(xyz=(-0.030, 0.0, -0.390)),
        material=aluminum,
        name="side_glide_0",
    )
    stage_1.visual(
        Box((0.008, 0.030, 0.045)),
        origin=Origin(xyz=(0.030, 0.0, -0.390)),
        material=aluminum,
        name="side_glide_1",
    )

    lift_0 = model.articulation(
        "base_to_leg_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_0,
        origin=Origin(xyz=(-0.32, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.08, lower=0.0, upper=0.36),
    )
    model.articulation(
        "base_to_leg_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_1,
        origin=Origin(xyz=(0.32, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.08, lower=0.0, upper=0.36),
        mimic=Mimic(joint="base_to_leg_stage_0", multiplier=1.0, offset=0.0),
    )

    top_frame = model.part("top_frame")
    top_frame.visual(
        Box((0.96, 0.46, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=wood,
        name="narrow_desktop",
    )
    top_frame.visual(
        Box((0.88, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.190, 0.020)),
        material=dark,
        name="front_rail",
    )
    top_frame.visual(
        Box((0.88, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.190, 0.020)),
        material=dark,
        name="rear_rail",
    )
    for x in (-0.425, 0.425):
        top_frame.visual(
            Box((0.035, 0.36, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.020)),
            material=dark,
            name=f"side_rail_{0 if x < 0 else 1}",
        )
    top_frame.visual(
        Box((0.76, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark,
        name="center_beam",
    )
    top_frame.visual(
        Box((0.15, 0.11, 0.010)),
        origin=Origin(xyz=(-0.32, 0.0, 0.005)),
        material=dark,
        name="mount_plate_0",
    )
    top_frame.visual(
        Box((0.025, 0.22, 0.047)),
        origin=Origin(xyz=(-0.245, -0.105, -0.020)),
        material=dark,
        name="guide_hanger_0",
    )
    top_frame.visual(
        Box((0.035, 0.28, 0.018)),
        origin=Origin(xyz=(-0.245, -0.100, -0.052)),
        material=dark,
        name="tray_guide_0",
    )
    top_frame.visual(
        Box((0.15, 0.11, 0.010)),
        origin=Origin(xyz=(0.32, 0.0, 0.005)),
        material=dark,
        name="mount_plate_1",
    )
    top_frame.visual(
        Box((0.025, 0.22, 0.047)),
        origin=Origin(xyz=(0.245, -0.105, -0.020)),
        material=dark,
        name="guide_hanger_1",
    )
    top_frame.visual(
        Box((0.035, 0.28, 0.018)),
        origin=Origin(xyz=(0.245, -0.100, -0.052)),
        material=dark,
        name="tray_guide_1",
    )

    model.articulation(
        "leg_stage_0_to_top_frame",
        ArticulationType.FIXED,
        parent=stage_0,
        child=top_frame,
        origin=Origin(xyz=(0.32, 0.0, 0.20)),
    )

    tray = model.part("keyboard_tray")
    tray.visual(
        Box((0.50, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="shallow_tray",
    )
    tray.visual(
        Box((0.50, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, -0.129, 0.002)),
        material=graphite,
        name="front_lip",
    )
    tray.visual(
        Box((0.028, 0.24, 0.014)),
        origin=Origin(xyz=(-0.245, 0.0, 0.015)),
        material=aluminum,
        name="slide_runner_0",
    )
    tray.visual(
        Box((0.028, 0.24, 0.014)),
        origin=Origin(xyz=(0.245, 0.0, 0.015)),
        material=aluminum,
        name="slide_runner_1",
    )

    model.articulation(
        "top_frame_to_keyboard_tray",
        ArticulationType.PRISMATIC,
        parent=top_frame,
        child=tray,
        origin=Origin(xyz=(0.0, -0.100, -0.083)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.18),
    )

    pod = model.part("control_pod")
    pod.visual(
        Box((0.12, 0.055, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=graphite,
        name="pod_case",
    )
    pod.visual(
        Box((0.10, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, -0.019)),
        material=dark,
        name="front_bezel",
    )

    model.articulation(
        "top_frame_to_control_pod",
        ArticulationType.FIXED,
        parent=top_frame,
        child=pod,
        origin=Origin(xyz=(0.390, -0.235, 0.008)),
    )

    for i, x in enumerate((-0.023, 0.023)):
        button = model.part(f"button_{i}")
        button.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=button_mat,
            name="round_cap",
        )
        button.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.001)),
            material=rubber,
            name="short_stem",
        )
        model.articulation(
            f"control_pod_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=pod,
            child=button,
            origin=Origin(xyz=(x, -0.004, -0.035)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=-0.004, upper=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stage_0 = object_model.get_part("leg_stage_0")
    stage_1 = object_model.get_part("leg_stage_1")
    top_frame = object_model.get_part("top_frame")
    tray = object_model.get_part("keyboard_tray")
    pod = object_model.get_part("control_pod")
    button_0 = object_model.get_part("button_0")
    lift = object_model.get_articulation("base_to_leg_stage_0")
    tray_slide = object_model.get_articulation("top_frame_to_keyboard_tray")

    # The moving leg stages are separate links inside the fixed outer columns,
    # with small glide pads touching the sleeve walls rather than a hidden
    # fused column/top-frame block.
    ctx.expect_contact(
        stage_0,
        base,
        elem_a="side_glide_0",
        elem_b="side_wall_0_a",
        name="stage 0 left glide bears on outer sleeve",
    )
    ctx.expect_contact(
        stage_1,
        base,
        elem_a="side_glide_1",
        elem_b="side_wall_1_b",
        name="stage 1 right glide bears on outer sleeve",
    )
    ctx.expect_gap(
        top_frame,
        stage_0,
        axis="z",
        positive_elem="mount_plate_0",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="top frame plate sits separately on stage cap",
    )
    ctx.expect_gap(
        top_frame,
        stage_1,
        axis="z",
        positive_elem="mount_plate_1",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="synchronized stage supports opposite frame plate",
    )

    rest_top = ctx.part_world_position(top_frame)
    with ctx.pose({lift: 0.36}):
        raised_top = ctx.part_world_position(top_frame)
        ctx.expect_contact(
            stage_0,
            base,
            elem_a="side_glide_0",
            elem_b="side_wall_0_a",
            name="stage 0 remains guided at full lift",
        )
        ctx.expect_contact(
            stage_1,
            base,
            elem_a="side_glide_1",
            elem_b="side_wall_1_b",
            name="stage 1 remains guided at full lift",
        )
        ctx.expect_gap(
            top_frame,
            stage_1,
            axis="z",
            positive_elem="mount_plate_1",
            negative_elem="top_cap",
            max_gap=0.001,
            max_penetration=0.0,
            name="mimic lift keeps opposite cap under the frame",
        )
    ctx.check(
        "lift raises desktop by standing-desk travel",
        rest_top is not None and raised_top is not None and raised_top[2] > rest_top[2] + 0.30,
        details=f"rest={rest_top}, raised={raised_top}",
    )

    # The keyboard tray runs on short side guides below the front rail and keeps
    # a retained insertion when pulled forward.
    ctx.expect_contact(
        tray,
        top_frame,
        elem_a="slide_runner_0",
        elem_b="tray_guide_0",
        name="tray runner rides in left guide",
    )
    ctx.expect_overlap(
        tray,
        top_frame,
        axes="y",
        elem_a="slide_runner_0",
        elem_b="tray_guide_0",
        min_overlap=0.20,
        name="closed tray is substantially captured in guide",
    )
    rest_tray = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.18}):
        extended_tray = ctx.part_world_position(tray)
        ctx.expect_contact(
            tray,
            top_frame,
            elem_a="slide_runner_0",
            elem_b="tray_guide_0",
            name="extended tray still rides on guide",
        )
        ctx.expect_overlap(
            tray,
            top_frame,
            axes="y",
            elem_a="slide_runner_0",
            elem_b="tray_guide_0",
            min_overlap=0.055,
            name="extended tray retains guide insertion",
        )
    ctx.check(
        "tray slides out toward the front edge",
        rest_tray is not None and extended_tray is not None and extended_tray[1] < rest_tray[1] - 0.15,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    ctx.expect_contact(
        button_0,
        pod,
        elem_a="short_stem",
        elem_b="pod_case",
        name="control button stem seats in pod",
    )

    return ctx.report()


object_model = build_object_model()
