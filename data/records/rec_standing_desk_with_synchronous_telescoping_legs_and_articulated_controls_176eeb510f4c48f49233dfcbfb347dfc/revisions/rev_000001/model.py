from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    wood = Material("warm_light_oak", rgba=(0.72, 0.52, 0.32, 1.0))
    black = Material("satin_black_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    dark = Material("dark_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    grey = Material("powder_coated_grey", rgba=(0.42, 0.45, 0.46, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    button_grey = Material("soft_grey_button", rgba=(0.58, 0.60, 0.61, 1.0))
    blue = Material("blue_memory_button", rgba=(0.08, 0.18, 0.55, 1.0))

    # Dimensions are in meters.  This is a compact home-office desk: a narrow
    # 1.10 m by 0.50 m top with a two-column sit/stand base.
    model.meta["description"] = (
        "Compact standing desk with a visibly separate top frame, twin "
        "synchronized telescoping legs, under-corner control pod, and four "
        "independent prismatic keypad buttons."
    )

    base = model.part("base")

    # Two floor feet plus a low cross-tie make the root a physically connected
    # base.  The feet are long in Y so the desk reads stable despite the narrow
    # top.
    for x, suffix in [(-0.38, "0"), (0.38, "1")]:
        base.visual(
            Box((0.18, 0.58, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.0175)),
            material=black,
            name=f"foot_{suffix}",
        )
        base.visual(
            Box((0.145, 0.040, 0.012)),
            origin=Origin(xyz=(x, -0.245, 0.006)),
            material=rubber,
            name=f"front_pad_{suffix}",
        )
        base.visual(
            Box((0.145, 0.040, 0.012)),
            origin=Origin(xyz=(x, 0.245, 0.006)),
            material=rubber,
            name=f"rear_pad_{suffix}",
        )

    base.visual(
        Box((0.82, 0.058, 0.060)),
        origin=Origin(xyz=(0.0, 0.075, 0.078)),
        material=black,
        name="lower_crossbar",
    )

    # Hollow rectangular outer columns.  Each column is four steel walls around
    # a clear central pocket so the moving stage can slide inside without being
    # represented as an impossible solid-on-solid intersection.
    outer_x = 0.112
    outer_y = 0.092
    wall = 0.012
    outer_h = 0.625
    col_center_z = 0.035 + outer_h / 2.0
    column_names = [
        (
            -0.38,
            "0",
            "outer_column_0_x_neg",
            "outer_column_0_x_pos",
            "outer_column_0_front",
            "outer_column_0_rear",
            "column_base_plate_0",
        ),
        (
            0.38,
            "1",
            "outer_column_1_x_neg",
            "outer_column_1_x_pos",
            "outer_column_1_front",
            "outer_column_1_rear",
            "column_base_plate_1",
        ),
    ]
    for x, suffix, x_neg_name, x_pos_name, front_name, rear_name, plate_name in column_names:
        base.visual(
            Box((wall, outer_y, outer_h)),
            origin=Origin(xyz=(x - outer_x / 2.0 + wall / 2.0, 0.0, col_center_z)),
            material=black,
            name=x_neg_name,
        )
        base.visual(
            Box((wall, outer_y, outer_h)),
            origin=Origin(xyz=(x + outer_x / 2.0 - wall / 2.0, 0.0, col_center_z)),
            material=black,
            name=x_pos_name,
        )
        base.visual(
            Box((outer_x, wall, outer_h)),
            origin=Origin(xyz=(x, -outer_y / 2.0 + wall / 2.0, col_center_z)),
            material=black,
            name=front_name,
        )
        base.visual(
            Box((outer_x, wall, outer_h)),
            origin=Origin(xyz=(x, outer_y / 2.0 - wall / 2.0, col_center_z)),
            material=black,
            name=rear_name,
        )
        base.visual(
            Box((0.145, 0.110, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.044)),
            material=black,
            name=plate_name,
        )

    # Sliding inner stages.  Their part frames sit at the top lips of the outer
    # columns; most of the tube extends downward into the sleeve, preserving
    # retained insertion at full standing height.
    for suffix in ["0", "1"]:
        leg = model.part(f"leg_stage_{suffix}")
        leg.visual(
            Box((0.062, 0.042, 0.620)),
            origin=Origin(xyz=(0.0, 0.0, -0.295)),
            material=grey,
            name="inner_tube",
        )
        leg.visual(
            Box((0.080, 0.060, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.0125)),
            material=grey,
            name="top_cap",
        )
        leg.visual(
            Box((0.050, 0.030, 0.055)),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=grey,
            name="neck_reinforcement",
        )
        leg.visual(
            Box((0.040, 0.013, 0.090)),
            origin=Origin(xyz=(0.0, -0.0275, -0.520)),
            material=dark,
            name="front_glide",
        )
        leg.visual(
            Box((0.040, 0.013, 0.090)),
            origin=Origin(xyz=(0.0, 0.0275, -0.520)),
            material=dark,
            name="rear_glide",
        )

    leg_0 = model.get_part("leg_stage_0")
    leg_1 = model.get_part("leg_stage_1")

    # The top frame is a separate moving part mounted to the master lifting
    # stage.  It deliberately reads as distinct black metal under the oak top,
    # rather than as a fused continuation of the lifting columns.
    top_frame = model.part("top_frame")
    top_frame.visual(
        Box((1.10, 0.50, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=wood,
        name="narrow_top",
    )
    top_frame.visual(
        Box((1.00, 0.026, 0.025)),
        origin=Origin(xyz=(0.0, -0.195, -0.030)),
        material=black,
        name="front_rail",
    )
    top_frame.visual(
        Box((1.00, 0.026, 0.025)),
        origin=Origin(xyz=(0.0, 0.195, -0.030)),
        material=black,
        name="rear_rail",
    )
    top_frame.visual(
        Box((0.028, 0.340, 0.025)),
        origin=Origin(xyz=(-0.485, 0.0, -0.030)),
        material=black,
        name="side_rail_0",
    )
    top_frame.visual(
        Box((0.028, 0.340, 0.025)),
        origin=Origin(xyz=(0.485, 0.0, -0.030)),
        material=black,
        name="side_rail_1",
    )
    top_frame.visual(
        Box((0.84, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, 0.090, -0.030)),
        material=black,
        name="center_spreader",
    )
    for x, plate_name in [(-0.38, "mount_plate_0"), (0.38, "mount_plate_1")]:
        top_frame.visual(
            Box((0.120, 0.085, 0.0275)),
            origin=Origin(xyz=(x, 0.0, -0.03125)),
            material=black,
            name=plate_name,
        )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.150, 0.050, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="handset_body",
    )
    control_pod.visual(
        Box((0.126, 0.004, 0.073)),
        origin=Origin(xyz=(0.0, -0.027, 0.0)),
        material=black,
        name="keypad_face",
    )
    control_pod.visual(
        Box((0.120, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, 0.0475)),
        material=dark,
        name="under_top_mount",
    )

    # Four independent push buttons in a 2x2 keypad face.  The child frames
    # are located at the button backs; the cap boxes protrude forward at rest
    # and move inward along +Y when pressed.
    button_specs = [
        ("button_0_0", -0.033, 0.019, button_grey),
        ("button_0_1", 0.033, 0.019, button_grey),
        ("button_1_0", -0.033, -0.019, blue),
        ("button_1_1", 0.033, -0.019, blue),
    ]
    for name, x, z, material in button_specs:
        button = model.part(name)
        button.visual(
            Box((0.027, 0.006, 0.017)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=material,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.002, 0.004)),
            origin=Origin(xyz=(0.0, -0.0065, 0.0)),
            material=black,
            name="recess_line",
        )
        model.articulation(
            f"control_pod_to_{name}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=button,
            origin=Origin(xyz=(x, -0.0285, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.04, lower=0.0, upper=0.006),
        )

    lift_limits = MotionLimits(effort=900.0, velocity=0.06, lower=0.0, upper=0.36)
    model.articulation(
        "base_to_leg_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=leg_0,
        origin=Origin(xyz=(-0.38, 0.0, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "base_to_leg_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=leg_1,
        origin=Origin(xyz=(0.38, 0.0, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic("base_to_leg_0", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "leg_0_to_top_frame",
        ArticulationType.FIXED,
        parent=leg_0,
        child=top_frame,
        origin=Origin(xyz=(0.38, 0.0, 0.0625)),
    )
    model.articulation(
        "top_frame_to_control_pod",
        ArticulationType.FIXED,
        parent=top_frame,
        child=control_pod,
        origin=Origin(xyz=(0.455, -0.265, -0.075)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    leg_0 = object_model.get_part("leg_stage_0")
    leg_1 = object_model.get_part("leg_stage_1")
    top_frame = object_model.get_part("top_frame")
    control_pod = object_model.get_part("control_pod")
    lift = object_model.get_articulation("base_to_leg_0")

    ctx.check(
        "twin leg sliders are synchronized",
        object_model.get_articulation("base_to_leg_1").mimic is not None,
        details="Second lifting column should mimic the primary prismatic lift joint.",
    )
    ctx.expect_overlap(
        leg_0,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_column_0_front",
        min_overlap=0.20,
        name="leg 0 remains inserted in outer column at rest",
    )
    ctx.expect_overlap(
        leg_1,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_column_1_front",
        min_overlap=0.20,
        name="leg 1 remains inserted in outer column at rest",
    )
    ctx.expect_gap(
        top_frame,
        leg_0,
        axis="z",
        positive_elem="mount_plate_0",
        negative_elem="top_cap",
        min_gap=0.0,
        max_gap=0.006,
        name="top frame sits just above leg 0 cap",
    )
    ctx.expect_gap(
        top_frame,
        leg_1,
        axis="z",
        positive_elem="mount_plate_1",
        negative_elem="top_cap",
        min_gap=0.0,
        max_gap=0.006,
        name="top frame sits just above leg 1 cap",
    )
    ctx.expect_contact(
        control_pod,
        top_frame,
        elem_a="under_top_mount",
        elem_b="narrow_top",
        contact_tol=0.0015,
        name="control pod mount contacts underside of desk top",
    )

    rest_top = ctx.part_world_position(top_frame)
    rest_leg_1 = ctx.part_world_position(leg_1)
    with ctx.pose({lift: 0.36}):
        raised_top = ctx.part_world_position(top_frame)
        raised_leg_1 = ctx.part_world_position(leg_1)
        ctx.expect_overlap(
            leg_0,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_column_0_front",
            min_overlap=0.12,
            name="leg 0 retains insertion when raised",
        )
        ctx.expect_overlap(
            leg_1,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_column_1_front",
            min_overlap=0.12,
            name="leg 1 retains insertion when raised",
        )
    ctx.check(
        "top frame raises with lifting stage",
        rest_top is not None and raised_top is not None and raised_top[2] > rest_top[2] + 0.30,
        details=f"rest={rest_top}, raised={raised_top}",
    )
    ctx.check(
        "mimic leg raises with primary leg",
        rest_leg_1 is not None
        and raised_leg_1 is not None
        and raised_leg_1[2] > rest_leg_1[2] + 0.30,
        details=f"rest={rest_leg_1}, raised={raised_leg_1}",
    )

    for name in ("button_0_0", "button_0_1", "button_1_0", "button_1_1"):
        button = object_model.get_part(name)
        joint = object_model.get_articulation(f"control_pod_to_{name}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{name} pushes inward independently",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.004,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
