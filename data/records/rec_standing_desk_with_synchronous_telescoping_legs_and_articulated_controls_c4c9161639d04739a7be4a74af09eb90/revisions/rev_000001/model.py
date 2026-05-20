from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    wood = model.material("warm_oak", rgba=(0.72, 0.49, 0.28, 1.0))
    dark_edge = model.material("dark_edge_banding", rgba=(0.18, 0.12, 0.08, 1.0))
    black_metal = model.material("satin_black_metal", rgba=(0.015, 0.017, 0.018, 1.0))
    inner_metal = model.material("brushed_inner_metal", rgba=(0.52, 0.55, 0.56, 1.0))
    panel_plastic = model.material("matte_black_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    display = model.material("blue_display_lens", rgba=(0.03, 0.14, 0.22, 1.0))
    button_mat = model.material("soft_gray_buttons", rgba=(0.17, 0.18, 0.19, 1.0))

    # Fixed lower T-frame: two long feet, a rear tie bar, and two hollow
    # rectangular outer columns.  The columns are built from wall strips so
    # the inner stages can slide inside a real clearance rather than a solid.
    base = model.part("base")
    leg_xs = (-0.36, 0.36)
    foot_size = (0.20, 0.64, 0.055)
    outer_w, outer_d, wall_t, outer_h = 0.096, 0.074, 0.009, 0.608
    outer_bottom = 0.052
    outer_center_z = outer_bottom + outer_h / 2.0
    for i, x in enumerate(leg_xs):
        base.visual(
            Box(foot_size),
            origin=Origin(xyz=(x, 0.0, foot_size[2] / 2.0)),
            material=black_metal,
            name=f"foot_{i}",
        )
        base.visual(
            Box((outer_w, wall_t, outer_h)),
            origin=Origin(xyz=(x, -outer_d / 2.0 + wall_t / 2.0, outer_center_z)),
            material=black_metal,
            name=f"column_{i}_front_wall",
        )
        base.visual(
            Box((outer_w, wall_t, outer_h)),
            origin=Origin(xyz=(x, outer_d / 2.0 - wall_t / 2.0, outer_center_z)),
            material=black_metal,
            name=f"column_{i}_rear_wall",
        )
        base.visual(
            Box((wall_t, outer_d, outer_h)),
            origin=Origin(xyz=(x - outer_w / 2.0 + wall_t / 2.0, 0.0, outer_center_z)),
            material=black_metal,
            name=f"column_{i}_outer_wall",
        )
        base.visual(
            Box((wall_t, outer_d, outer_h)),
            origin=Origin(xyz=(x + outer_w / 2.0 - wall_t / 2.0, 0.0, outer_center_z)),
            material=black_metal,
            name=f"column_{i}_inner_wall",
        )

    base.visual(
        Box((0.88, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, 0.265, 0.035)),
        material=black_metal,
        name="rear_tie_bar",
    )

    # Synchronized lifting stages.  Each silver rectangular stage has a long
    # hidden insertion length so it remains captured at standing height.
    stage_parts = []
    inner_size = (0.064, 0.044, 0.640)
    inner_center_z = -0.285
    for i in range(2):
        stage = model.part(f"inner_stage_{i}")
        stage.visual(
            Box(inner_size),
            origin=Origin(xyz=(0.0, 0.0, inner_center_z)),
            material=inner_metal,
            name="inner_tube",
        )
        # A dark glide cap at the sleeve mouth makes the nested sliding fit
        # legible without closing the hollow column.
        stage.visual(
            Box((0.074, 0.052, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=panel_plastic,
            name="glide_cap",
        )
        stage_parts.append(stage)

    lift_limits = MotionLimits(effort=900.0, velocity=0.08, lower=0.0, upper=0.42)
    lift_0 = model.articulation(
        "base_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_parts[0],
        origin=Origin(xyz=(leg_xs[0], 0.0, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "base_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_parts[1],
        origin=Origin(xyz=(leg_xs[1], 0.0, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint=lift_0.name),
    )

    # Moving desktop and upper T-frame carried by the two sliding stages.
    desktop = model.part("desktop")
    desktop.visual(
        Box((1.05, 0.42, 0.035)),
        origin=Origin(),
        material=wood,
        name="wood_top",
    )
    desktop.visual(
        Box((1.06, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, -0.216, 0.0)),
        material=dark_edge,
        name="front_edge",
    )
    desktop.visual(
        Box((1.06, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, 0.216, 0.0)),
        material=dark_edge,
        name="rear_edge",
    )
    desktop.visual(
        Box((0.90, 0.070, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material=black_metal,
        name="support_crossbar",
    )
    for i, x in enumerate(leg_xs):
        desktop.visual(
            Box((0.105, 0.330, 0.034)),
            origin=Origin(xyz=(x, 0.0, -0.0405)),
            material=black_metal,
            name=f"top_t_arm_{i}",
        )

    model.articulation(
        "stage_to_desktop",
        ArticulationType.FIXED,
        parent=stage_parts[0],
        child=desktop,
        # The desktop frame is at the wood top center at the low-height pose.
        origin=Origin(xyz=(0.36, 0.0, 0.0925)),
    )

    # Small memory-control panel hanging under the front edge.  The housing is
    # an open-front shell with separate socket strips around the button row.
    panel = model.part("control_panel")
    panel_w, panel_d, panel_h = 0.260, 0.069, 0.060
    front_y = -panel_d / 2.0
    back_y = panel_d / 2.0
    shell_t = 0.006
    panel.visual(
        Box((panel_w, shell_t, panel_h)),
        origin=Origin(xyz=(0.0, back_y - shell_t / 2.0, 0.0)),
        material=panel_plastic,
        name="back_wall",
    )
    panel.visual(
        Box((panel_w, panel_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_h / 2.0 - shell_t / 2.0)),
        material=panel_plastic,
        name="top_wall",
    )
    panel.visual(
        Box((panel_w, panel_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, -panel_h / 2.0 + shell_t / 2.0)),
        material=panel_plastic,
        name="bottom_wall",
    )
    panel.visual(
        Box((shell_t, panel_d, panel_h)),
        origin=Origin(xyz=(-panel_w / 2.0 + shell_t / 2.0, 0.0, 0.0)),
        material=panel_plastic,
        name="side_wall_0",
    )
    panel.visual(
        Box((shell_t, panel_d, panel_h)),
        origin=Origin(xyz=(panel_w / 2.0 - shell_t / 2.0, 0.0, 0.0)),
        material=panel_plastic,
        name="side_wall_1",
    )
    # Socket frame around the four memory buttons.
    socket_y = front_y + 0.002
    panel.visual(
        Box((0.174, 0.004, 0.006)),
        origin=Origin(xyz=(0.044, socket_y, 0.010)),
        material=panel_plastic,
        name="button_top_strip",
    )
    panel.visual(
        Box((0.174, 0.004, 0.006)),
        origin=Origin(xyz=(0.044, socket_y, -0.022)),
        material=panel_plastic,
        name="button_bottom_strip",
    )
    for j, x in enumerate((-0.045, -0.011, 0.023, 0.057, 0.091, 0.125)):
        panel.visual(
            Box((0.006, 0.004, 0.032)),
            origin=Origin(xyz=(x, socket_y, -0.006)),
            material=panel_plastic,
            name=f"button_separator_{j}",
        )
    panel.visual(
        Box((0.072, 0.004, 0.018)),
        origin=Origin(xyz=(-0.083, front_y - 0.001, 0.011)),
        material=display,
        name="display_lens",
    )

    model.articulation(
        "desktop_to_panel",
        ArticulationType.FIXED,
        parent=desktop,
        child=panel,
        origin=Origin(xyz=(0.0, -0.2325, -0.0475)),
    )

    button_xs = (-0.028, 0.006, 0.040, 0.074)
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.020, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=button_mat,
            name="cap",
        )
        button.visual(
            # The hidden plunger fills the rectangular guide opening exactly:
            # its side and top/bottom faces ride against the fixed socket
            # strips while the visible cap remains smaller and proud.
            Box((0.028, 0.020, 0.026)),
            origin=Origin(xyz=(0.0, 0.010, 0.0)),
            material=button_mat,
            name="plunger",
        )
        model.articulation(
            f"panel_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=Origin(xyz=(x, front_y, -0.006)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    desktop = object_model.get_part("desktop")
    panel = object_model.get_part("control_panel")
    stage_0 = object_model.get_part("inner_stage_0")
    stage_1 = object_model.get_part("inner_stage_1")
    lift = object_model.get_articulation("base_to_stage_0")

    # Both lifting tubes support the moving top frame at the low pose.
    ctx.expect_contact(
        stage_0,
        desktop,
        elem_a="inner_tube",
        elem_b="support_crossbar",
        contact_tol=1e-5,
        name="stage 0 carries support crossbar",
    )
    ctx.expect_contact(
        stage_1,
        desktop,
        elem_a="inner_tube",
        elem_b="support_crossbar",
        contact_tol=1e-5,
        name="stage 1 carries support crossbar",
    )
    ctx.expect_overlap(
        stage_0,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="column_0_front_wall",
        min_overlap=0.50,
        name="stage 0 deeply inserted when lowered",
    )
    ctx.expect_overlap(
        stage_1,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="column_1_front_wall",
        min_overlap=0.50,
        name="stage 1 deeply inserted when lowered",
    )

    lowered_pos = ctx.part_world_position(desktop)
    with ctx.pose({lift: 0.42}):
        raised_pos = ctx.part_world_position(desktop)
        ctx.expect_overlap(
            stage_0,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="column_0_front_wall",
            min_overlap=0.16,
            name="stage 0 remains captured when raised",
        )
        ctx.expect_overlap(
            stage_1,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="column_1_front_wall",
            min_overlap=0.16,
            name="stage 1 remains captured when raised",
        )
    ctx.check(
        "desktop raises on lifting columns",
        lowered_pos is not None
        and raised_pos is not None
        and raised_pos[2] > lowered_pos[2] + 0.40,
        details=f"lowered={lowered_pos}, raised={raised_pos}",
    )

    # Memory buttons stay in their socket row and move inward along +Y.
    for i in range(4):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"panel_to_button_{i}")
        ctx.expect_within(
            button,
            panel,
            axes="xz",
            margin=0.002,
            name=f"button {i} is framed by the panel",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed_pos = ctx.part_world_position(button)
            ctx.expect_within(
                button,
                panel,
                axes="xz",
                margin=0.002,
                name=f"button {i} stays in its socket when pressed",
            )
        ctx.check(
            f"button {i} plunges inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.005,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
