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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_leg_standing_desk")

    wood = model.material("warm_maple_top", color=(0.72, 0.52, 0.32, 1.0))
    black = model.material("satin_black_metal", color=(0.015, 0.016, 0.018, 1.0))
    inner_metal = model.material("brushed_lift_stage", color=(0.62, 0.64, 0.66, 1.0))
    pod_plastic = model.material("matte_black_plastic", color=(0.005, 0.005, 0.006, 1.0))
    button_gray = model.material("soft_gray_buttons", color=(0.42, 0.43, 0.44, 1.0))
    red = model.material("red_power_button", color=(0.80, 0.05, 0.04, 1.0))
    glass = model.material("black_display_lens", color=(0.0, 0.0, 0.0, 1.0))

    # Fixed lower structure: two floor feet, two open rectangular outer columns,
    # and a rear stabilizing crossbar so the stationary frame is one supported part.
    base = model.part("base_frame")
    for x in (-0.50, 0.50):
        suffix = "0" if x < 0.0 else "1"
        base.visual(
            Box((0.16, 0.72, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=black,
            name=f"foot_{suffix}",
        )
        # Four walls make a hollow sleeve rather than a solid collision proxy.
        base.visual(
            Box((0.105, 0.012, 0.630)),
            origin=Origin(xyz=(x, -0.0465, 0.365)),
            material=black,
            name=f"outer_column_{suffix}_front_wall",
        )
        base.visual(
            Box((0.105, 0.012, 0.630)),
            origin=Origin(xyz=(x, 0.0465, 0.365)),
            material=black,
            name=f"outer_column_{suffix}_rear_wall",
        )
        base.visual(
            Box((0.012, 0.081, 0.630)),
            origin=Origin(xyz=(x - 0.0465, 0.0, 0.365)),
            material=black,
            name=f"outer_column_{suffix}_outer_wall",
        )
        base.visual(
            Box((0.012, 0.081, 0.630)),
            origin=Origin(xyz=(x + 0.0465, 0.0, 0.365)),
            material=black,
            name=f"outer_column_{suffix}_inner_wall",
        )
    base.visual(
        Box((1.08, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, 0.080, 0.165)),
        material=black,
        name="rear_floor_crossbar",
    )

    lift_stage_0 = model.part("lift_stage_0")
    lift_stage_0.visual(
        Box((0.064, 0.064, 0.650)),
        origin=Origin(xyz=(0.0, 0.0, -0.325)),
        material=inner_metal,
        name="inner_post",
    )
    for y, name in ((-0.03625, "front_glide"), (0.03625, "rear_glide")):
        lift_stage_0.visual(
            Box((0.040, 0.0085, 0.110)),
            origin=Origin(xyz=(0.0, y, -0.250)),
            material=black,
            name=name,
        )

    lift_stage_1 = model.part("lift_stage_1")
    lift_stage_1.visual(
        Box((0.064, 0.064, 0.650)),
        origin=Origin(xyz=(0.0, 0.0, -0.325)),
        material=inner_metal,
        name="inner_post",
    )
    for y, name in ((-0.03625, "front_glide"), (0.03625, "rear_glide")):
        lift_stage_1.visual(
            Box((0.040, 0.0085, 0.110)),
            origin=Origin(xyz=(0.0, y, -0.250)),
            material=black,
            name=name,
        )

    lift_limits = MotionLimits(effort=900.0, velocity=0.06, lower=0.0, upper=0.45)
    model.articulation(
        "base_to_lift_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_stage_0,
        origin=Origin(xyz=(-0.50, 0.0, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "base_to_lift_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_stage_1,
        origin=Origin(xyz=(0.50, 0.0, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic("base_to_lift_stage_0"),
    )

    desktop = model.part("desktop")
    desktop.visual(
        Box((1.40, 0.75, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=wood,
        name="rectangular_top",
    )
    # Steel under-frame that contacts the two lifting stages and reads as a
    # rectangular crossbeam frame beneath the wood top.
    desktop.visual(
        Box((0.055, 0.60, 0.020)),
        origin=Origin(xyz=(-0.50, 0.0, -0.035)),
        material=black,
        name="side_rail_0",
    )
    desktop.visual(
        Box((0.055, 0.60, 0.020)),
        origin=Origin(xyz=(0.50, 0.0, -0.035)),
        material=black,
        name="side_rail_1",
    )
    desktop.visual(
        Box((1.08, 0.045, 0.020)),
        origin=Origin(xyz=(0.0, -0.280, -0.035)),
        material=black,
        name="front_crossbeam",
    )
    desktop.visual(
        Box((1.08, 0.045, 0.020)),
        origin=Origin(xyz=(0.0, 0.280, -0.035)),
        material=black,
        name="rear_crossbeam",
    )
    model.articulation(
        "lift_stage_0_to_desktop",
        ArticulationType.FIXED,
        parent=lift_stage_0,
        child=desktop,
        origin=Origin(xyz=(0.50, 0.0, 0.045)),
    )

    control_bank = model.part("control_bank")
    control_bank.visual(
        Box((0.240, 0.090, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pod_plastic,
        name="pod_body",
    )
    control_bank.visual(
        Box((0.220, 0.035, 0.030)),
        origin=Origin(xyz=(0.0, 0.035, 0.031)),
        material=pod_plastic,
        name="mounting_neck",
    )
    control_bank.visual(
        Box((0.050, 0.026, 0.003)),
        origin=Origin(xyz=(-0.082, -0.012, -0.0175)),
        material=glass,
        name="display_window",
    )
    model.articulation(
        "desktop_to_control_bank",
        ArticulationType.FIXED,
        parent=desktop,
        child=control_bank,
        origin=Origin(xyz=(0.43, -0.405, -0.071)),
    )

    button_limits = MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.004)
    for index, x in enumerate((-0.025, 0.020, 0.065)):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Box((0.032, 0.024, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=button_gray,
            name="cap",
        )
        model.articulation(
            f"control_bank_to_preset_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_bank,
            child=button,
            origin=Origin(xyz=(x, -0.012, -0.016)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=button_limits,
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=red,
        name="cap",
    )
    model.articulation(
        "control_bank_to_power_button",
        ArticulationType.PRISMATIC,
        parent=control_bank,
        child=power_button,
        origin=Origin(xyz=(0.105, -0.012, -0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=button_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    lift_stage_0 = object_model.get_part("lift_stage_0")
    lift_stage_1 = object_model.get_part("lift_stage_1")
    desktop = object_model.get_part("desktop")
    control_bank = object_model.get_part("control_bank")
    lift = object_model.get_articulation("base_to_lift_stage_0")

    for q in (0.0, 0.45):
        with ctx.pose({lift: q}):
            p0 = ctx.part_world_position(lift_stage_0)
            p1 = ctx.part_world_position(lift_stage_1)
            ctx.check(
                f"lift stages remain height matched at q={q:.2f}",
                p0 is not None and p1 is not None and abs(p0[2] - p1[2]) < 0.001,
                details=f"stage_0={p0}, stage_1={p1}",
            )
            ctx.expect_contact(
                lift_stage_0,
                desktop,
                elem_a="inner_post",
                elem_b="side_rail_0",
                name=f"stage 0 supports desktop at q={q:.2f}",
            )
            ctx.expect_contact(
                lift_stage_1,
                desktop,
                elem_a="inner_post",
                elem_b="side_rail_1",
                name=f"stage 1 supports desktop at q={q:.2f}",
            )
            ctx.expect_overlap(
                lift_stage_0,
                base,
                axes="z",
                elem_a="inner_post",
                elem_b="outer_column_0_front_wall",
                min_overlap=0.14,
                name=f"stage 0 remains inserted in outer column at q={q:.2f}",
            )
            ctx.expect_overlap(
                lift_stage_1,
                base,
                axes="z",
                elem_a="inner_post",
                elem_b="outer_column_1_front_wall",
                min_overlap=0.14,
                name=f"stage 1 remains inserted in outer column at q={q:.2f}",
            )

    ctx.expect_contact(
        control_bank,
        desktop,
        elem_a="mounting_neck",
        elem_b="rectangular_top",
        name="control pod is mounted beneath the front edge",
    )
    ctx.expect_overlap(
        control_bank,
        desktop,
        axes="x",
        elem_a="pod_body",
        elem_b="rectangular_top",
        min_overlap=0.20,
        name="control pod is a real-width handset under the desktop",
    )

    for name in ("preset_button_0", "preset_button_1", "preset_button_2", "power_button"):
        part = object_model.get_part(name)
        joint = object_model.get_articulation(f"control_bank_to_{name}")
        ctx.expect_contact(
            part,
            control_bank,
            elem_a="cap",
            elem_b="pod_body",
            name=f"{name} starts proud of the control pod",
        )
        rest = ctx.part_world_position(part)
        with ctx.pose({joint: 0.004}):
            depressed = ctx.part_world_position(part)
        ctx.check(
            f"{name} depresses independently",
            rest is not None and depressed is not None and depressed[2] > rest[2] + 0.003,
            details=f"rest={rest}, depressed={depressed}",
        )

    return ctx.report()


object_model = build_object_model()
