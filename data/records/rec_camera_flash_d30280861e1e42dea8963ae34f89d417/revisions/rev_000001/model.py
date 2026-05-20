from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A simple consumer-electronics rounded rectangle solid."""

    solid = cq.Workplane("XY").box(size[0], size[1], size[2])
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_speedlight_flash")

    matte_black = model.material("matte_black", rgba=(0.012, 0.012, 0.014, 1.0))
    satin_black = model.material("satin_black", rgba=(0.045, 0.045, 0.050, 1.0))
    rubber = model.material("rubber_black", rgba=(0.010, 0.010, 0.012, 1.0))
    dark_panel = model.material("smoked_panel", rgba=(0.020, 0.024, 0.030, 1.0))
    button_mat = model.material("button_gray", rgba=(0.14, 0.15, 0.16, 1.0))
    button_blue = model.material("mode_blue", rgba=(0.05, 0.16, 0.34, 1.0))
    diffuser = model.material("frosted_diffuser", rgba=(0.78, 0.86, 0.92, 0.62))
    reflector = model.material("soft_reflector", rgba=(0.95, 0.96, 0.90, 0.82))
    metal = model.material("brushed_metal", rgba=(0.55, 0.55, 0.52, 1.0))

    body = model.part("body")

    lower_body = _rounded_box((0.078, 0.052, 0.090), 0.006).translate((0.0, 0.0, 0.053))
    upper_step = _rounded_box((0.064, 0.044, 0.040), 0.005).translate((0.0, 0.0, 0.113))
    top_plate = _rounded_box((0.054, 0.038, 0.012), 0.004).translate((0.0, 0.0, 0.138))
    body_shell = lower_body.union(upper_step).union(top_plate)
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell", tolerance=0.0006),
        material=matte_black,
        name="body_shell",
    )

    # Stepped battery door and molded panel breaks on the front face.
    body.visual(
        Box((0.062, 0.0022, 0.055)),
        origin=Origin(xyz=(0.0, -0.0271, 0.046)),
        material=dark_panel,
        name="battery_door",
    )
    body.visual(
        Box((0.068, 0.0030, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0268, 0.093)),
        material=satin_black,
        name="body_step_line",
    )
    body.visual(
        Box((0.030, 0.0030, 0.014)),
        origin=Origin(xyz=(0.0, -0.0222, 0.108)),
        material=dark_panel,
        name="ready_window",
    )

    # Right-side control strip: the individual buttons below are articulated
    # child parts whose inner faces rest on this raised rubberized strip.
    body.visual(
        Box((0.0050, 0.020, 0.070)),
        origin=Origin(xyz=(0.0415, 0.001, 0.065)),
        material=rubber,
        name="control_strip",
    )
    body.visual(
        Box((0.0052, 0.016, 0.011)),
        origin=Origin(xyz=(0.0418, 0.001, 0.105)),
        material=dark_panel,
        name="control_display",
    )

    # Hot-shoe and shoe lock stack under the body.
    body.visual(
        Box((0.030, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=matte_black,
        name="shoe_pedestal",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=satin_black,
        name="lock_wheel",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=metal,
        name="shoe_screw",
    )
    body.visual(
        Box((0.050, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.008, 0.038, 0.006)),
        origin=Origin(xyz=(-0.025, 0.0, -0.010)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.008, 0.038, 0.006)),
        origin=Origin(xyz=(0.025, 0.0, -0.010)),
        material=metal,
        name="shoe_rail_1",
    )
    body.visual(
        Box((0.044, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.019, -0.010)),
        material=metal,
        name="shoe_stop",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_black,
        name="turntable",
    )
    swivel.visual(
        Box((0.040, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_black,
        name="swivel_neck",
    )
    swivel.visual(
        Box((0.120, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, 0.018)),
        material=satin_black,
        name="yoke_bridge",
    )
    swivel.visual(
        Box((0.010, 0.020, 0.050)),
        origin=Origin(xyz=(-0.065, 0.0, 0.039)),
        material=satin_black,
        name="yoke_arm_0",
    )
    swivel.visual(
        Box((0.010, 0.020, 0.050)),
        origin=Origin(xyz=(0.065, 0.0, 0.039)),
        material=satin_black,
        name="yoke_arm_1",
    )

    head = model.part("head")
    head_shell = _rounded_box((0.100, 0.056, 0.048), 0.006)
    head.visual(
        mesh_from_cadquery(head_shell, "head_shell", tolerance=0.0006),
        origin=Origin(xyz=(0.0, -0.011, 0.003)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.084, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, -0.041, 0.003)),
        material=diffuser,
        name="front_lens",
    )
    head.visual(
        Box((0.074, 0.002, 0.022)),
        origin=Origin(xyz=(0.0, -0.0432, 0.0025)),
        material=reflector,
        name="flash_tube_window",
    )
    head.visual(
        Box((0.094, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, -0.042, 0.024)),
        material=satin_black,
        name="lens_top_bezel",
    )
    head.visual(
        Box((0.094, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, -0.042, -0.018)),
        material=satin_black,
        name="lens_bottom_bezel",
    )
    head.visual(
        Box((0.005, 0.006, 0.040)),
        origin=Origin(xyz=(-0.045, -0.042, 0.003)),
        material=satin_black,
        name="lens_side_bezel_0",
    )
    head.visual(
        Box((0.005, 0.006, 0.040)),
        origin=Origin(xyz=(0.045, -0.042, 0.003)),
        material=satin_black,
        name="lens_side_bezel_1",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_pin_0",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_pin_1",
    )

    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.144)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.6, lower=-0.15, upper=1.55),
    )

    # Four separate prismatic mode buttons on the side strip.  Positive travel
    # moves inward along -X, as a real push button would compress into the body.
    button_positions = [
        ("mode_button_0", 0.084, button_blue),
        ("mode_button_1", 0.068, button_mat),
        ("mode_button_2", 0.052, button_mat),
        ("mode_button_3", 0.036, button_mat),
    ]
    for name, z_pos, material in button_positions:
        button = model.part(name)
        button.visual(
            Box((0.006, 0.014, 0.010)),
            origin=Origin(),
            material=material,
            name="cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.047, 0.001, z_pos)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")
    swivel_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

    ctx.expect_contact(
        swivel,
        body,
        elem_a="turntable",
        elem_b="body_shell",
        contact_tol=0.001,
        name="swivel turntable seats on body top",
    )
    ctx.expect_contact(
        head,
        swivel,
        elem_a="pivot_pin_1",
        elem_b="yoke_arm_1",
        contact_tol=0.001,
        name="head pivot is carried by yoke",
    )

    for i in range(4):
        button = object_model.get_part(f"mode_button_{i}")
        joint = object_model.get_articulation(f"body_to_mode_button_{i}")
        ctx.expect_gap(
            button,
            body,
            axis="x",
            positive_elem="cap",
            negative_elem="control_strip",
            max_gap=0.001,
            max_penetration=0.0002,
            name=f"mode button {i} rests on control strip",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="yz",
            elem_a="cap",
            elem_b="control_strip",
            min_overlap=0.006,
            name=f"mode button {i} is within side strip footprint",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.004}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"mode button {i} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[0] < rest_pos[0] - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    closed_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({tilt_joint: 0.9}):
        tilted_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "tilt joint aims lamp upward",
        closed_lens is not None
        and tilted_lens is not None
        and (tilted_lens[0][2] + tilted_lens[1][2]) > (closed_lens[0][2] + closed_lens[1][2]) + 0.035,
        details=f"closed={closed_lens}, tilted={tilted_lens}",
    )

    front_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({swivel_joint: 1.0}):
        swiveled_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "swivel joint yaws lamp around vertical axis",
        front_lens is not None
        and swiveled_lens is not None
        and abs((swiveled_lens[0][0] + swiveled_lens[1][0]) - (front_lens[0][0] + front_lens[1][0])) > 0.045,
        details=f"front={front_lens}, swiveled={swiveled_lens}",
    )

    return ctx.report()


object_model = build_object_model()
