from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_monitor")

    # Overall proportions: a small office monitor, deliberately thicker than an
    # ultrathin entertainment panel so the rear support and tilt barrel read as
    # mechanically credible.
    display_w = 0.55
    display_h = 0.34
    display_t = 0.055

    charcoal = model.material("matte_charcoal", rgba=(0.025, 0.028, 0.030, 1.0))
    dark_plastic = model.material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    soft_black = model.material("soft_black", rgba=(0.005, 0.006, 0.007, 1.0))
    glass = model.material("dark_glass", rgba=(0.02, 0.04, 0.055, 1.0))
    button_rubber = model.material("button_rubber", rgba=(0.012, 0.013, 0.014, 1.0))
    power_mark = model.material("power_mark", rgba=(0.62, 0.95, 0.68, 1.0))

    # Fixed rounded pedestal base with a low bearing collar on top.
    base = model.part("base")
    base_footprint = ExtrudeGeometry.from_z0(
        superellipse_profile(0.32, 0.22, exponent=2.75, segments=72),
        0.032,
        cap=True,
    )
    base.visual(
        mesh_from_geometry(base_footprint, "rounded_pedestal_base"),
        material=dark_plastic,
        name="pedestal_shell",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=charcoal,
        name="swivel_socket",
    )

    # Rotating stand body: a turntable disk plus a U-channel sleeve that the
    # height-adjustable neck slides through.
    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=charcoal,
        name="turntable_disk",
    )
    swivel.visual(
        Box((0.080, 0.012, 0.200)),
        origin=Origin(xyz=(0.0, 0.012, 0.112)),
        material=charcoal,
        name="sleeve_back",
    )
    for x, visual_name in ((-0.034, "sleeve_side_0"), (0.034, "sleeve_side_1")):
        swivel.visual(
            Box((0.012, 0.040, 0.200)),
            origin=Origin(xyz=(x, -0.002, 0.112)),
            material=charcoal,
            name=visual_name,
        )
    swivel.visual(
        Box((0.092, 0.050, 0.025)),
        origin=Origin(xyz=(0.0, -0.002, 0.024)),
        material=charcoal,
        name="sleeve_foot",
    )

    # Sliding inner neck and the tilt-head yoke.  The inner member extends down
    # into the sleeve so height adjustment retains insertion at full travel.
    neck = model.part("neck")
    neck.visual(
        Box((0.034, 0.016, 0.300)),
        origin=Origin(xyz=(0.0, -0.002, -0.005)),
        material=dark_plastic,
        name="inner_member",
    )
    neck.visual(
        Box((0.062, 0.032, 0.045)),
        origin=Origin(xyz=(0.0, 0.014, 0.125)),
        material=dark_plastic,
        name="head_block",
    )
    neck.visual(
        Cylinder(radius=0.006, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.175), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="head_pin",
    )
    neck.visual(
        Box((0.150, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.018, 0.136)),
        material=dark_plastic,
        name="head_bridge",
    )
    for x, visual_name in ((-0.072, "yoke_plate_0"), (0.072, "yoke_plate_1")):
        neck.visual(
            Box((0.014, 0.040, 0.065)),
            origin=Origin(xyz=(x, 0.012, 0.175)),
            material=dark_plastic,
            name=visual_name,
        )

    # Tilting display shell.  The shell and bezel are rounded meshes, but the
    # mass is boxy and thick enough to look like a business monitor housing.
    display = model.part("display")
    shell_mesh = ExtrudeGeometry(
        rounded_rect_profile(display_w, display_h, 0.020, corner_segments=10),
        display_t,
        cap=True,
        center=True,
    )
    shell_mesh.rotate_x(math.pi / 2.0)
    display.visual(
        mesh_from_geometry(shell_mesh, "display_shell"),
        origin=Origin(xyz=(0.0, -0.0375, 0.0)),
        material=charcoal,
        name="display_shell",
    )

    bezel_mesh = BezelGeometry(
        opening_size=(0.492, 0.286),
        outer_size=(display_w, display_h),
        depth=0.010,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.010,
        outer_corner_radius=0.020,
        face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.0015),
        center=True,
    )
    bezel_mesh.rotate_x(math.pi / 2.0)
    display.visual(
        mesh_from_geometry(bezel_mesh, "front_bezel"),
        origin=Origin(xyz=(0.0, -0.067, 0.0)),
        material=soft_black,
        name="front_bezel",
    )
    display.visual(
        Box((0.494, 0.003, 0.288)),
        origin=Origin(xyz=(0.0, -0.0735, 0.002)),
        material=glass,
        name="screen_glass",
    )
    display.visual(
        Cylinder(radius=0.024, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="display_hinge_barrel",
    )
    display.visual(
        Box((0.130, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, 0.040)),
        material=charcoal,
        name="rear_hinge_boss",
    )
    for idx, x in enumerate((-0.16, -0.09, -0.02, 0.05, 0.12)):
        display.visual(
            Box((0.044, 0.003, 0.006)),
            origin=Origin(xyz=(x, -0.011, 0.085)),
            material=soft_black,
            name=f"rear_vent_{idx}",
        )

    # Underside controls: a distinct rocker and three independent menu buttons.
    rocker = model.part("power_rocker")
    rocker.visual(
        Box((0.044, 0.017, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=button_rubber,
        name="rocker_cap",
    )
    rocker.visual(
        Box((0.018, 0.002, 0.001)),
        origin=Origin(xyz=(-0.006, -0.0088, -0.0082)),
        material=power_mark,
        name="power_glyph",
    )

    menu_buttons = []
    for idx in range(3):
        button = model.part(f"menu_button_{idx}")
        button.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=button_rubber,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0022, length=0.0008),
            origin=Origin(xyz=(0.0, 0.0, -0.0059)),
            material=dark_plastic,
            name="button_dot",
        )
        menu_buttons.append(button)

    # Primary mechanisms.
    model.articulation(
        "base_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "sleeve_to_neck",
        ArticulationType.PRISMATIC,
        parent=swivel,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.212)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=0.10),
    )
    model.articulation(
        "neck_to_display",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.35, upper=0.20),
    )
    model.articulation(
        "display_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=display,
        child=rocker,
        origin=Origin(xyz=(-0.145, -0.055, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.7, velocity=3.0, lower=-0.25, upper=0.25),
    )
    for button, x in zip(menu_buttons, (-0.070, -0.035, 0.000)):
        model.articulation(
            f"display_to_{button.name}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x, -0.055, -0.170)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    swivel = object_model.get_part("swivel")
    neck = object_model.get_part("neck")
    display = object_model.get_part("display")
    rocker = object_model.get_part("power_rocker")
    menu_0 = object_model.get_part("menu_button_0")
    menu_1 = object_model.get_part("menu_button_1")
    menu_2 = object_model.get_part("menu_button_2")

    swivel_joint = object_model.get_articulation("base_to_swivel")
    slide_joint = object_model.get_articulation("sleeve_to_neck")
    tilt_joint = object_model.get_articulation("neck_to_display")
    rocker_joint = object_model.get_articulation("display_to_power_rocker")
    button_joints = [
        object_model.get_articulation("display_to_menu_button_0"),
        object_model.get_articulation("display_to_menu_button_1"),
        object_model.get_articulation("display_to_menu_button_2"),
    ]

    # The pin/barrel hinge and underside controls intentionally seat into
    # simplified solid housing proxies.
    ctx.allow_overlap(
        neck,
        display,
        elem_a="head_pin",
        elem_b="display_hinge_barrel",
        reason="The stand head pin is intentionally captured inside the display hinge barrel.",
    )
    for button in (menu_0, menu_1, menu_2):
        ctx.allow_overlap(
            display,
            button,
            elem_a="display_shell",
            elem_b="button_cap",
            reason="The button cap is represented as depressing into a shallow underside pocket in the bezel proxy.",
        )
    ctx.allow_overlap(
        display,
        rocker,
        elem_a="display_shell",
        elem_b="rocker_cap",
        reason="The rocker pivots into a shallow underside recess represented by the solid display shell.",
    )

    ctx.expect_contact(base, swivel, elem_a="swivel_socket", elem_b="turntable_disk", contact_tol=0.001)
    ctx.expect_overlap(neck, display, axes="x", elem_a="head_pin", elem_b="display_hinge_barrel", min_overlap=0.090)
    ctx.expect_within(neck, display, axes="yz", elem_a="head_pin", elem_b="display_hinge_barrel", margin=0.001)

    ctx.expect_within(neck, swivel, axes="xy", inner_elem="inner_member", margin=0.006)
    ctx.expect_overlap(neck, swivel, axes="z", elem_a="inner_member", min_overlap=0.070)
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        ctx.expect_within(
            neck,
            swivel,
            axes="xy",
            inner_elem="inner_member",
            margin=0.006,
            name="raised neck stays centered in sleeve",
        )
        ctx.expect_overlap(
            neck,
            swivel,
            axes="z",
            elem_a="inner_member",
            min_overlap=0.050,
            name="raised neck remains inserted",
        )

    ctx.expect_contact(display, rocker, elem_a="display_shell", elem_b="rocker_cap", contact_tol=0.0015)
    for idx, button in enumerate((menu_0, menu_1, menu_2)):
        ctx.expect_contact(
            display,
            button,
            elem_a="display_shell",
            elem_b="button_cap",
            contact_tol=0.0015,
            name=f"menu button {idx} touches underside bezel",
        )
    ctx.expect_origin_distance(rocker, menu_0, axes="x", min_dist=0.055, name="rocker separated from menu buttons")
    ctx.expect_origin_distance(menu_0, menu_1, axes="x", min_dist=0.025, max_dist=0.050)
    ctx.expect_origin_distance(menu_1, menu_2, axes="x", min_dist=0.025, max_dist=0.050)

    # Prompt-specific mechanism checks: slide raises the screen, tilt rotates
    # about a horizontal axis, and controls move independently.
    rest_display_pos = ctx.part_world_position(display)
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        raised_display_pos = ctx.part_world_position(display)
    ctx.check(
        "neck slide raises display",
        rest_display_pos is not None
        and raised_display_pos is not None
        and raised_display_pos[2] > rest_display_pos[2] + 0.09,
        details=f"rest={rest_display_pos}, raised={raised_display_pos}",
    )

    ctx.check(
        "swivel is continuous vertical",
        swivel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel_joint.articulation_type}, axis={swivel_joint.axis}",
    )
    ctx.check(
        "tilt hinge is horizontal",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE and abs(tuple(tilt_joint.axis)[0]) == 1.0,
        details=f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}",
    )

    rest_button_positions = [ctx.part_world_position(button) for button in (menu_0, menu_1, menu_2)]
    for idx, (button, joint) in enumerate(zip((menu_0, menu_1, menu_2), button_joints)):
        with ctx.pose({joint: joint.motion_limits.upper}):
            depressed_pos = ctx.part_world_position(button)
            ctx.expect_gap(
                display,
                button,
                axis="z",
                positive_elem="display_shell",
                negative_elem="button_cap",
                max_penetration=0.006,
                name=f"menu button {idx} depresses into bezel",
            )
        rest_pos = rest_button_positions[idx]
        ctx.check(
            f"menu button {idx} moves upward",
            rest_pos is not None and depressed_pos is not None and depressed_pos[2] > rest_pos[2] + 0.003,
            details=f"rest={rest_pos}, depressed={depressed_pos}",
        )

    with ctx.pose({rocker_joint: rocker_joint.motion_limits.upper}):
        ctx.expect_gap(
            display,
            rocker,
            axis="z",
            positive_elem="display_shell",
            negative_elem="rocker_cap",
            max_penetration=0.008,
            name="rocker pivots into underside recess",
        )

    return ctx.report()


object_model = build_object_model()
