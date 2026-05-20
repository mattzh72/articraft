from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="battle_mixer")

    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_black = model.material("panel_black", rgba=(0.07, 0.08, 0.09, 1.0))
    slot_black = model.material("slot_black", rgba=(0.03, 0.03, 0.04, 1.0))
    cap_gray = model.material("cap_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    switch_metal = model.material("switch_metal", rgba=(0.75, 0.76, 0.79, 1.0))
    switch_tip = model.material("switch_tip", rgba=(0.80, 0.16, 0.16, 1.0))

    def add_box(
        part,
        *,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        name: str,
    ) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    chassis = model.part("chassis")

    # Bottom pan.
    add_box(
        chassis,
        size=(0.240, 0.240, 0.010),
        xyz=(0.0, 0.0, 0.005),
        material=body_dark,
        name="bottom_pan",
    )

    # Structural deck blocks. Together they leave real openings for the sliders.
    housing_blocks = (
        ("upper_band_body", (0.240, 0.028, 0.032), (0.0, 0.106, 0.026)),
        ("mid_band_body", (0.240, 0.055, 0.032), (0.0, -0.040, 0.026)),
        ("front_band_body", (0.240, 0.037, 0.032), (0.0, -0.1015, 0.026)),
        ("left_side_body", (0.048, 0.105, 0.032), (-0.096, 0.040, 0.026)),
        ("center_body", (0.116, 0.105, 0.032), (0.0, 0.040, 0.026)),
        ("right_side_body", (0.048, 0.105, 0.032), (0.096, 0.040, 0.026)),
        ("cross_left_body", (0.065, 0.016, 0.032), (-0.0875, -0.075, 0.026)),
        ("cross_right_body", (0.065, 0.016, 0.032), (0.0875, -0.075, 0.026)),
    )
    for name, size, xyz in housing_blocks:
        add_box(chassis, size=size, xyz=xyz, material=body_dark, name=name)

    # Thin black faceplate skin on the top deck.
    faceplate_tiles = (
        ("upper_band_panel", (0.240, 0.028, 0.002), (0.0, 0.106, 0.041)),
        ("mid_band_panel", (0.240, 0.055, 0.002), (0.0, -0.040, 0.041)),
        ("front_band_panel", (0.240, 0.037, 0.002), (0.0, -0.1015, 0.041)),
        ("left_side_panel", (0.048, 0.105, 0.002), (-0.096, 0.040, 0.041)),
        ("center_panel", (0.116, 0.105, 0.002), (0.0, 0.040, 0.041)),
        ("right_side_panel", (0.048, 0.105, 0.002), (0.096, 0.040, 0.041)),
        ("cross_left_panel", (0.065, 0.016, 0.002), (-0.0875, -0.075, 0.041)),
        ("cross_right_panel", (0.065, 0.016, 0.002), (0.0875, -0.075, 0.041)),
    )
    for name, size, xyz in faceplate_tiles:
        add_box(chassis, size=size, xyz=xyz, material=panel_black, name=name)

    # Dark slot floors so the channels read as recessed tracks rather than paint.
    add_box(
        chassis,
        size=(0.014, 0.105, 0.002),
        xyz=(-0.065, 0.040, 0.011),
        material=slot_black,
        name="left_slot_floor",
    )
    add_box(
        chassis,
        size=(0.014, 0.105, 0.002),
        xyz=(0.065, 0.040, 0.011),
        material=slot_black,
        name="right_slot_floor",
    )
    add_box(
        chassis,
        size=(0.110, 0.016, 0.002),
        xyz=(0.0, -0.075, 0.011),
        material=slot_black,
        name="cross_slot_floor",
    )

    # Simple printed lane accents for the two mixer channels.
    add_box(
        chassis,
        size=(0.034, 0.118, 0.001),
        xyz=(-0.065, 0.040, 0.0415),
        material=panel_black,
        name="left_channel_strip",
    )
    add_box(
        chassis,
        size=(0.034, 0.118, 0.001),
        xyz=(0.065, 0.040, 0.0415),
        material=panel_black,
        name="right_channel_strip",
    )

    # Hamster switch cradle.
    add_box(
        chassis,
        size=(0.016, 0.014, 0.002),
        xyz=(0.086, -0.075, 0.041),
        material=panel_black,
        name="switch_mount_pad",
    )
    add_box(
        chassis,
        size=(0.004, 0.006, 0.010),
        xyz=(0.086, -0.081, 0.047),
        material=body_dark,
        name="switch_left_cheek",
    )
    add_box(
        chassis,
        size=(0.004, 0.006, 0.010),
        xyz=(0.086, -0.069, 0.047),
        material=body_dark,
        name="switch_right_cheek",
    )

    left_channel_fader = model.part("left_channel_fader")
    add_box(
        left_channel_fader,
        size=(0.006, 0.010, 0.030),
        xyz=(0.0, 0.0, 0.027),
        material=knob_dark,
        name="slider_stem",
    )
    add_box(
        left_channel_fader,
        size=(0.018, 0.015, 0.010),
        xyz=(0.0, 0.0, 0.047),
        material=cap_gray,
        name="fader_cap",
    )
    add_box(
        left_channel_fader,
        size=(0.010, 0.011, 0.004),
        xyz=(0.0, 0.0, 0.054),
        material=knob_dark,
        name="cap_ridge",
    )

    right_channel_fader = model.part("right_channel_fader")
    add_box(
        right_channel_fader,
        size=(0.006, 0.010, 0.030),
        xyz=(0.0, 0.0, 0.027),
        material=knob_dark,
        name="slider_stem",
    )
    add_box(
        right_channel_fader,
        size=(0.018, 0.015, 0.010),
        xyz=(0.0, 0.0, 0.047),
        material=cap_gray,
        name="fader_cap",
    )
    add_box(
        right_channel_fader,
        size=(0.010, 0.011, 0.004),
        xyz=(0.0, 0.0, 0.054),
        material=knob_dark,
        name="cap_ridge",
    )

    crossfader = model.part("crossfader")
    add_box(
        crossfader,
        size=(0.014, 0.006, 0.030),
        xyz=(0.0, 0.0, 0.027),
        material=knob_dark,
        name="slider_stem",
    )
    add_box(
        crossfader,
        size=(0.030, 0.015, 0.012),
        xyz=(0.0, 0.0, 0.048),
        material=cap_gray,
        name="crossfader_cap",
    )
    add_box(
        crossfader,
        size=(0.016, 0.011, 0.004),
        xyz=(0.0, 0.0, 0.056),
        material=knob_dark,
        name="crossfader_grip",
    )

    hamster_toggle = model.part("hamster_toggle")
    hamster_toggle.visual(
        Cylinder(radius=0.0025, length=0.006),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=switch_metal,
        name="switch_barrel",
    )
    add_box(
        hamster_toggle,
        size=(0.003, 0.003, 0.016),
        xyz=(0.0, 0.0, 0.010),
        material=switch_metal,
        name="lever_stem",
    )
    add_box(
        hamster_toggle,
        size=(0.006, 0.0035, 0.006),
        xyz=(0.0, 0.0, 0.019),
        material=switch_tip,
        name="lever_tip",
    )

    model.articulation(
        "chassis_to_left_channel_fader",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=left_channel_fader,
        origin=Origin(xyz=(-0.065, -0.005, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.30,
            lower=0.0,
            upper=0.090,
        ),
    )
    model.articulation(
        "chassis_to_right_channel_fader",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=right_channel_fader,
        origin=Origin(xyz=(0.065, -0.005, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.30,
            lower=0.0,
            upper=0.090,
        ),
    )
    model.articulation(
        "chassis_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=crossfader,
        origin=Origin(xyz=(-0.046, -0.075, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.40,
            lower=0.0,
            upper=0.092,
        ),
    )
    model.articulation(
        "chassis_to_hamster_toggle",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=hamster_toggle,
        origin=Origin(xyz=(0.086, -0.075, 0.047)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    left_fader = object_model.get_part("left_channel_fader")
    right_fader = object_model.get_part("right_channel_fader")
    crossfader = object_model.get_part("crossfader")
    hamster_toggle = object_model.get_part("hamster_toggle")

    left_joint = object_model.get_articulation("chassis_to_left_channel_fader")
    right_joint = object_model.get_articulation("chassis_to_right_channel_fader")
    cross_joint = object_model.get_articulation("chassis_to_crossfader")
    toggle_joint = object_model.get_articulation("chassis_to_hamster_toggle")

    left_slot_floor = chassis.get_visual("left_slot_floor")
    right_slot_floor = chassis.get_visual("right_slot_floor")
    cross_slot_floor = chassis.get_visual("cross_slot_floor")
    switch_left_cheek = chassis.get_visual("switch_left_cheek")
    switch_right_cheek = chassis.get_visual("switch_right_cheek")

    left_slider_stem = left_fader.get_visual("slider_stem")
    right_slider_stem = right_fader.get_visual("slider_stem")
    cross_slider_stem = crossfader.get_visual("slider_stem")
    switch_barrel = hamster_toggle.get_visual("switch_barrel")
    lever_tip = hamster_toggle.get_visual("lever_tip")

    ctx.expect_within(
        left_fader,
        chassis,
        axes="xy",
        inner_elem=left_slider_stem,
        outer_elem=left_slot_floor,
        name="left fader stem stays within left slot",
    )
    ctx.expect_contact(
        left_fader,
        chassis,
        elem_a=left_slider_stem,
        elem_b=left_slot_floor,
        name="left fader stem is supported by slot floor",
    )

    ctx.expect_within(
        right_fader,
        chassis,
        axes="xy",
        inner_elem=right_slider_stem,
        outer_elem=right_slot_floor,
        name="right fader stem stays within right slot",
    )
    ctx.expect_contact(
        right_fader,
        chassis,
        elem_a=right_slider_stem,
        elem_b=right_slot_floor,
        name="right fader stem is supported by slot floor",
    )

    ctx.expect_within(
        crossfader,
        chassis,
        axes="xy",
        inner_elem=cross_slider_stem,
        outer_elem=cross_slot_floor,
        name="crossfader stem stays within cross slot",
    )
    ctx.expect_contact(
        crossfader,
        chassis,
        elem_a=cross_slider_stem,
        elem_b=cross_slot_floor,
        name="crossfader stem is supported by slot floor",
    )

    ctx.expect_contact(
        hamster_toggle,
        chassis,
        elem_a=switch_barrel,
        elem_b=switch_left_cheek,
        name="hamster toggle barrel seats against left cheek",
    )
    ctx.expect_contact(
        hamster_toggle,
        chassis,
        elem_a=switch_barrel,
        elem_b=switch_right_cheek,
        name="hamster toggle barrel seats against right cheek",
    )

    left_rest = ctx.part_world_position(left_fader)
    with ctx.pose({left_joint: left_joint.motion_limits.upper}):
        left_top = ctx.part_world_position(left_fader)
        ctx.expect_within(
            left_fader,
            chassis,
            axes="xy",
            inner_elem=left_slider_stem,
            outer_elem=left_slot_floor,
            name="left fader remains guided at top travel",
        )
        ctx.expect_contact(
            left_fader,
            chassis,
            elem_a=left_slider_stem,
            elem_b=left_slot_floor,
            name="left fader remains supported at top travel",
        )
    ctx.check(
        "left fader slides upward",
        left_rest is not None and left_top is not None and left_top[1] > left_rest[1] + 0.08,
        details=f"rest={left_rest}, top={left_top}",
    )

    right_rest = ctx.part_world_position(right_fader)
    with ctx.pose({right_joint: right_joint.motion_limits.upper}):
        right_top = ctx.part_world_position(right_fader)
        ctx.expect_within(
            right_fader,
            chassis,
            axes="xy",
            inner_elem=right_slider_stem,
            outer_elem=right_slot_floor,
            name="right fader remains guided at top travel",
        )
        ctx.expect_contact(
            right_fader,
            chassis,
            elem_a=right_slider_stem,
            elem_b=right_slot_floor,
            name="right fader remains supported at top travel",
        )
    ctx.check(
        "right fader slides upward",
        right_rest is not None and right_top is not None and right_top[1] > right_rest[1] + 0.08,
        details=f"rest={right_rest}, top={right_top}",
    )

    cross_rest = ctx.part_world_position(crossfader)
    with ctx.pose({cross_joint: cross_joint.motion_limits.upper}):
        cross_right = ctx.part_world_position(crossfader)
        ctx.expect_within(
            crossfader,
            chassis,
            axes="xy",
            inner_elem=cross_slider_stem,
            outer_elem=cross_slot_floor,
            name="crossfader remains guided at full right travel",
        )
        ctx.expect_contact(
            crossfader,
            chassis,
            elem_a=cross_slider_stem,
            elem_b=cross_slot_floor,
            name="crossfader remains supported at full right travel",
        )
    ctx.check(
        "crossfader slides to the right",
        cross_rest is not None and cross_right is not None and cross_right[0] > cross_rest[0] + 0.08,
        details=f"rest={cross_rest}, right={cross_right}",
    )

    def aabb_center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    with ctx.pose({toggle_joint: toggle_joint.motion_limits.lower}):
        toggle_left = ctx.part_element_world_aabb(hamster_toggle, elem=lever_tip)
    with ctx.pose({toggle_joint: toggle_joint.motion_limits.upper}):
        toggle_right = ctx.part_element_world_aabb(hamster_toggle, elem=lever_tip)
    toggle_left_x = aabb_center_x(toggle_left)
    toggle_right_x = aabb_center_x(toggle_right)
    ctx.check(
        "hamster toggle flips laterally",
        toggle_left_x is not None
        and toggle_right_x is not None
        and toggle_right_x > toggle_left_x + 0.01,
        details=f"left={toggle_left_x}, right={toggle_right_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
